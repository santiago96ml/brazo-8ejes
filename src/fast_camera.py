import threading
import time
from typing import Any

import cv2
import numpy as np

from config.env_performance import apply_performance_env, configure_opencv

# Apply performance settings before heavy imports are used
apply_performance_env()
configure_opencv()


class FastUSBCamera:
    def __init__(
        self,
        index: int = 0,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        fourcc: str = "MJPG",
        backend: Any = "auto",
        exposure: float | None = None,
        auto_exposure: float | None = None,
        auto_wb: int | None = None,
        wb_temperature: int | None = None,
        saturation: int | None = None,
        brightness: int | None = None,
        contrast: int | None = None,
        gain: int | None = None,
        sharpness: int | None = None,
        capture: Any | None = None,
        warmup_frames: int = 8,
    ):
        self.index = index
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self.backend = backend
        self.exposure = exposure
        self.auto_exposure = auto_exposure
        self.auto_wb = auto_wb
        self.wb_temperature = wb_temperature
        self.saturation = saturation
        self.brightness = brightness
        self.contrast = contrast
        self.gain = gain
        self.sharpness = sharpness
        self._capture_injected = capture
        self._warmup_remaining = max(0, int(warmup_frames))
        self._resolved_backend = self._resolve_backend(backend, width, height)

        self._cap = None
        self._thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self._latest_frame = None
        self._latest_ts_ns = 0
        self._frames = 0
        self._dropped = 0
        self._consumer_reads = 0
        self._consecutive_failures = 0

        self._intervals = []
        self._last_frame_time_ns = 0
        self._measured_fps = 0.0

        self._actual_width = width
        self._actual_height = height
        self._actual_fourcc = fourcc

        self._running = False

    @staticmethod
    def _resolve_backend(backend: Any, width: int, height: int) -> int:
        """Pick the capture backend.

        Measured on this PC with the Trust USB camera (2026-10-30):
        - DSHOW at 1280x720 negotiates YUY2 and delivers 8.1 FPS on USB 2.0.
        - MSMF at 1280x720/1920x1080 negotiates MJPG and delivers ~30 FPS.
        - DSHOW at 640x480 reaches 30 FPS.
        So "auto" prefers MSMF for 720p and above, DSHOW for smaller frames.
        """
        if isinstance(backend, int):
            return backend
        key = str(backend).strip().lower()
        if key in ("msmf", "mediafoundation", "cap_msmf"):
            return cv2.CAP_MSMF
        if key in ("dshow", "directshow", "cap_dshow"):
            return cv2.CAP_DSHOW
        return cv2.CAP_MSMF if (width >= 1280 or height >= 720) else cv2.CAP_DSHOW

    def _configure(self, cap) -> None:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.auto_exposure is not None:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, self.auto_exposure)
        if self.exposure is not None:
            cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        # UVC colour controls: best-effort, some MSMF drivers ignore them.
        if self.auto_wb is not None:
            cap.set(cv2.CAP_PROP_AUTO_WB, self.auto_wb)
        if self.wb_temperature is not None:
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, self.wb_temperature)
        if self.saturation is not None:
            cap.set(cv2.CAP_PROP_SATURATION, self.saturation)
        if self.brightness is not None:
            cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
        if self.contrast is not None:
            cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)
        if self.gain is not None:
            cap.set(cv2.CAP_PROP_GAIN, self.gain)
        if self.sharpness is not None:
            cap.set(cv2.CAP_PROP_SHARPNESS, self.sharpness)

    def _open_capture(self):
        candidates = [self._resolved_backend]
        fallback = cv2.CAP_DSHOW if self._resolved_backend == cv2.CAP_MSMF else cv2.CAP_MSMF
        candidates.append(fallback)
        last_error = None
        for backend in candidates:
            cap = cv2.VideoCapture(self.index, backend)
            if not cap.isOpened():
                cap.release()
                last_error = f"backend {backend}"
                continue
            self._configure(cap)
            self._resolved_backend = backend
            return cap
        raise RuntimeError(
            f"Cannot open camera index {self.index} (tried {last_error}). "
            "Check that no other application is using it."
        )

    def start(self) -> "FastUSBCamera":
        if self._running:
            return self

        if self._capture_injected is not None:
            self._cap = self._capture_injected
        else:
            self._cap = self._open_capture()

        self._actual_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._actual_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc_val = int(self._cap.get(cv2.CAP_PROP_FOURCC))
        self._actual_fourcc = "".join([chr((fourcc_val >> 8 * i) & 0xFF) for i in range(4)]).strip("\x00")

        self._stop_event.clear()
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        return self

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            if self._cap is None:
                time.sleep(0.005)
                continue
            ret, frame = self._cap.read()
            if not ret or frame is None:
                self._consecutive_failures += 1
                if self._consecutive_failures >= 100:
                    break
                time.sleep(0.005)
                continue
            self._consecutive_failures = 0
            if self._warmup_remaining > 0:
                # MSMF returns black frames during the first few reads; never publish them.
                self._warmup_remaining -= 1
                continue
            now_ns = time.perf_counter_ns()
            if self._last_frame_time_ns > 0:
                interval = now_ns - self._last_frame_time_ns
                self._intervals.append(interval)
                if len(self._intervals) > 30:
                    self._intervals.pop(0)
                if self._intervals:
                    avg_interval = sum(self._intervals) / len(self._intervals)
                    if avg_interval > 0:
                        self._measured_fps = 1e9 / avg_interval
            self._last_frame_time_ns = now_ns

            with self._lock:
                if self._latest_frame is not None:
                    self._dropped += 1
                self._latest_frame = frame
                self._latest_ts_ns = now_ns
                self._frames += 1

    def stop(self) -> None:
        if not self._running and self._thread is None:
            return
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        if self._cap is not None and self._capture_injected is None:
            self._cap.release()
        elif self._cap is not None and self._capture_injected is not None:
            pass
        self._cap = None
        self._running = False

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def actual_width(self) -> int:
        return self._actual_width

    @property
    def actual_height(self) -> int:
        return self._actual_height

    def get_latest_frame(self) -> tuple[np.ndarray | None, int, float]:
        with self._lock:
            frame = self._latest_frame
            ts_ns = self._latest_ts_ns
            fps = self._measured_fps
        if frame is None:
            return None, ts_ns, fps
        return frame, ts_ns, fps

    def read(self) -> tuple[bool, np.ndarray | None]:
        with self._lock:
            frame = self._latest_frame
            if frame is None:
                return False, None
            self._consumer_reads += 1
            return True, frame

    def stats(self) -> dict:
        return {
            "frames": self._frames,
            "dropped": self._dropped,
            "fps": self._measured_fps,
            "width": self._actual_width,
            "height": self._actual_height,
            "fourcc": self._actual_fourcc,
            "exposure": self.exposure,
        }

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        return False
