"""Live monitoring HUD for the 6-DOF arm vision pipeline (S14)."""
from __future__ import annotations

import argparse
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
SRC_DIR = os.path.join(REPO_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

# Performance env before cv2/numpy
try:
    from config.env_performance import apply_performance_env, configure_opencv

    apply_performance_env()
except Exception:
    try:
        from env_performance import apply_performance_env, configure_opencv  # type: ignore

        apply_performance_env()
    except Exception:
        pass

import numpy as np  # noqa: E402

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore

# Defensive module imports
try:
    from src.fast_camera import FastUSBCamera  # type: ignore
except Exception:
    try:
        from fast_camera import FastUSBCamera  # type: ignore
    except Exception:
        FastUSBCamera = None  # type: ignore

try:
    from src.arm_vision_analyzer import ArmVisionAnalyzer  # type: ignore
except Exception:
    try:
        from arm_vision_analyzer import ArmVisionAnalyzer  # type: ignore
    except Exception:
        ArmVisionAnalyzer = None  # type: ignore

try:
    from src.dataset_recorder import DatasetRecorder  # type: ignore
except Exception:
    try:
        from dataset_recorder import DatasetRecorder  # type: ignore
    except Exception:
        DatasetRecorder = None  # type: ignore

try:
    from src.agent_controller import AgentController  # type: ignore
except Exception:
    try:
        from agent_controller import AgentController  # type: ignore
    except Exception:
        AgentController = None  # type: ignore


def _instantiate(cls, *args, **kwargs):
    if cls is None:
        return None
    if isinstance(cls, type):
        try:
            return cls(*args, **kwargs)
        except Exception:
            try:
                return cls()
            except Exception:
                return None
    return cls


def _build_camera(camera_index, width, height, fps):
    if FastUSBCamera is not None:
        for kwargs in (
            {"index": camera_index, "width": width, "height": height, "fps": fps},
            {"camera_index": camera_index, "width": width, "height": height, "fps": fps},
        ):
            try:
                return FastUSBCamera(**kwargs)
            except Exception:
                continue
        try:
            return FastUSBCamera(camera_index, width, height, fps)
        except Exception:
            try:
                return FastUSBCamera(camera_index)
            except Exception:
                pass
    # fallback using cv2
    return _FallbackCamera(camera_index, width, height, fps)


class _FallbackCamera:
    def __init__(self, index=0, width=1280, height=720, fps=30):
        self.index = index
        self.width = width
        self.height = height
        self.fps = fps
        self._cap = None
        try:
            self._cap = cv2.VideoCapture(index, cv2.CAP_DSHOW) if cv2 is not None else None
            if self._cap is not None:
                try:
                    self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                    self._cap.set(cv2.CAP_PROP_FPS, fps)
                except Exception:
                    pass
        except Exception:
            self._cap = None
        self._last = None
        self._ts = 0
        self._fps = float(fps)

    def get_latest_frame(self):
        if self._cap is None:
            return None, 0, 0.0
        try:
            ok, frame = self._cap.read()
        except Exception:
            return None, 0, 0.0
        if not ok or frame is None:
            return None, 0, 0.0
        self._ts = int(time.time() * 1e9)
        self._last = frame
        return frame, self._ts, self._fps

    def stop(self):
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass


def _build_analyzer(mode):
    if ArmVisionAnalyzer is None:
        return None
    for kwargs in ({"mode": mode}, {}):
        try:
            return ArmVisionAnalyzer(**kwargs)
        except Exception:
            continue
    try:
        return ArmVisionAnalyzer(mode)
    except Exception:
        return None


def _oe_text(controller):
    if controller is None:
        return "OE = UNKNOWN", (200, 200, 200)
    try:
        st = getattr(controller, "state", None)
        emergency = False
        if st is not None:
            if hasattr(st, "state") and isinstance(st.state, dict):
                emergency = bool(st.state.get("emergency_stop", False))
            elif isinstance(st, dict):
                emergency = bool(st.get("emergency_stop", False))
        if emergency:
            return "OE = HIGH [SAFE]", (0, 165, 255)
        return "OE = LOW [ACTIVE]", (0, 255, 0)
    except Exception:
        return "OE = UNKNOWN", (200, 200, 200)


def _draw_text(img, text, pos, color=(255, 255, 255), scale=0.6, thick=1):
    if cv2 is None:
        return
    try:
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)
    except Exception:
        pass


def _draw_skeleton(img, analysis):
    if cv2 is None or not isinstance(analysis, dict):
        return
    # skeleton may be list of points or list of segments
    skel = analysis.get("skeleton")
    if isinstance(skel, (list, tuple)):
        pts = []
        for item in skel:
            if isinstance(item, (list, tuple)) and len(item) >= 2 and not isinstance(item[0], (list, tuple)):
                # point
                pts.append((int(item[0]), int(item[1])))
            elif isinstance(item, (list, tuple)) and len(item) == 2 and isinstance(item[0], (list, tuple)):
                # segment
                try:
                    a = (int(item[0][0]), int(item[0][1]))
                    b = (int(item[1][0]), int(item[1][1]))
                    cv2.line(img, a, b, (255, 255, 0), 2)
                except Exception:
                    pass
        if len(pts) >= 2:
            for i in range(len(pts) - 1):
                try:
                    cv2.line(img, pts[i], pts[i + 1], (255, 255, 0), 2)
                except Exception:
                    pass
    # keypoints
    kp = analysis.get("keypoints_px")
    if isinstance(kp, dict):
        for name, pt in kp.items():
            xy = None
            if isinstance(pt, (list, tuple)) and len(pt) >= 2:
                xy = (int(pt[0]), int(pt[1]))
            elif isinstance(pt, dict):
                try:
                    xy = (int(pt["x"]), int(pt["y"]))
                except Exception:
                    xy = None
            if xy is not None:
                try:
                    cv2.circle(img, xy, 4, (0, 255, 255), -1)
                    cv2.putText(img, str(name), (xy[0] + 5, xy[1] - 5),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
                except Exception:
                    pass
    elif isinstance(kp, (list, tuple)):
        for i, pt in enumerate(kp):
            if isinstance(pt, (list, tuple)) and len(pt) >= 2:
                try:
                    xy = (int(pt[0]), int(pt[1]))
                    cv2.circle(img, xy, 4, (0, 255, 255), -1)
                    cv2.putText(img, str(i), (xy[0] + 5, xy[1] - 5),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
                except Exception:
                    pass


def run_hud(
    camera_index: int = 0,
    width: int = 1280,
    height: int = 720,
    fps: int = 30,
    mode: str = "hybrid",
    camera=None,
    analyzer=None,
    controller=None,
    recorder=None,
    window_name: str = "ARM 6-DOF VISION",
    max_frames: int | None = None,
    show: bool = True,
    analyze_every: int = 2,
) -> dict:
    # instantiate classes if passed
    if isinstance(camera, type):
        camera = camera()
    if isinstance(analyzer, type):
        analyzer = analyzer()
    if isinstance(recorder, type):
        recorder = recorder()
    if isinstance(controller, type):
        controller = controller()

    if camera is None:
        camera = _build_camera(camera_index, width, height, fps)
        if camera is not None and hasattr(camera, "start") and not getattr(camera, "is_running", False):
            try:
                camera.start()
            except Exception as exc:
                return {"frames": 0, "saved": 0, "last_analysis": {"error": str(exc)},
                        "stopped_by": "error"}
    if analyzer is None:
        analyzer = _build_analyzer(mode)
    if recorder is None and DatasetRecorder is not None:
        try:
            recorder = DatasetRecorder(root="dataset")
        except Exception:
            recorder = None

    frames = 0
    saved = 0
    last_analysis = None
    stopped_by = None
    _none_streak = 0

    try:
        if show and cv2 is not None:
            try:
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            except Exception:
                pass

        while True:
            if max_frames is not None and frames >= max_frames:
                stopped_by = "max_frames"
                break

            try:
                frame, ts_ns, fps_real = camera.get_latest_frame()
            except Exception:
                frame, ts_ns, fps_real = None, 0, 0.0

            if frame is None:
                _none_streak += 1
                if _none_streak > 3000:
                    stopped_by = "error"
                    break
                time.sleep(0.002)
                continue
            _none_streak = 0

            # analyze (decimated: analysis costs ~15-20 ms, the stream is 30 FPS)
            analysis = None
            err = None
            if analyze_every > 1 and last_analysis is not None and (frames % analyze_every) != 0:
                analysis = last_analysis
            else:
                try:
                    if analyzer is not None:
                        analysis = analyzer.analyze(frame)
                    else:
                        analysis = {"mode_used": "none", "confidence": 0.0, "latency_ms": 0.0}
                except Exception as e:
                    err = str(e)
                    analysis = {"mode_used": "error", "confidence": 0.0, "latency_ms": 0.0, "error": err}

            last_analysis = analysis
            frames += 1

            if show and cv2 is not None:
                try:
                    disp = frame.copy() if hasattr(frame, "copy") else frame
                except Exception:
                    disp = frame
                try:
                    _draw_skeleton(disp, analysis if isinstance(analysis, dict) else {})
                except Exception:
                    pass

                # text overlays
                thetas = None
                if isinstance(analysis, dict):
                    thetas = analysis.get("thetas_deg")
                y = 25
                if isinstance(thetas, (list, tuple)):
                    for i, t in enumerate(thetas[:6], start=1):
                        try:
                            _draw_text(disp, "theta%d = %.1f" % (i, float(t)), (10, y))
                        except Exception:
                            pass
                        y += 20

                cart = None
                if isinstance(analysis, dict):
                    cart = analysis.get("cartesian_mm")
                if isinstance(cart, dict):
                    try:
                        x = cart.get("x", cart.get("X"))
                        yy = cart.get("y", cart.get("Y"))
                        z = cart.get("z", cart.get("Z"))
                        _draw_text(disp, "X=%.1f Y=%.1f Z=%.1f mm" % (float(x), float(yy), float(z)), (10, y))
                    except Exception:
                        _draw_text(disp, "X Y Z = --", (10, y))
                else:
                    _draw_text(disp, "X Y Z = --", (10, y))
                y += 20

                oe_txt, oe_color = _oe_text(controller)
                _draw_text(disp, oe_txt, (10, y), color=oe_color)
                y += 20

                # top-left FPS & latency & mode & confidence
                try:
                    fps_val = float(fps_real) if fps_real else 0.0
                except Exception:
                    fps_val = 0.0
                lat = 0.0
                mode_used = mode
                conf = 0.0
                if isinstance(analysis, dict):
                    try:
                        lat = float(analysis.get("latency_ms", 0.0))
                    except Exception:
                        lat = 0.0
                    mode_used = str(analysis.get("mode_used", mode))
                    try:
                        conf = float(analysis.get("confidence", 0.0))
                    except Exception:
                        conf = 0.0
                _draw_text(disp, "FPS %.1f  lat %.1f ms" % (fps_val, lat), (10, 15), scale=0.5)
                _draw_text(disp, "mode=%s conf=%.2f" % (mode_used, conf), (10, 32), scale=0.5)

                if err is not None:
                    _draw_text(disp, "ERROR: %s" % err, (10, 50), color=(0, 0, 255))

                try:
                    cv2.imshow(window_name, disp)
                except Exception:
                    pass

                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key in (ord("q"), ord("Q")):
                    stopped_by = "q"
                    break
                if key == 32:  # SPACE
                    try:
                        if controller is not None:
                            controller.emergency_stop()
                    except Exception:
                        pass
                    try:
                        if controller is not None and hasattr(controller, "transport"):
                            controller.transport.send_line("S")
                    except Exception:
                        pass
                elif key in (ord("z"), ord("Z")):
                    try:
                        if controller is not None and hasattr(controller, "zero_jump_boot_sync"):
                            controller.zero_jump_boot_sync()
                    except Exception:
                        try:
                            if controller is not None and hasattr(controller, "state"):
                                st = controller.state
                                if hasattr(st, "update_from_thetas") and isinstance(thetas, (list, tuple)):
                                    st.update_from_thetas(list(thetas))
                        except Exception:
                            pass
                elif key in (ord("g"), ord("G")):
                    try:
                        if recorder is not None:
                            r = recorder.save_sample(frame, analysis)
                            if isinstance(r, dict) and r.get("saved"):
                                saved += 1
                    except Exception:
                        pass
            else:
                # headless: still process keys? no GUI -> no keys
                pass

            # headless key handling not applicable
    except KeyboardInterrupt:
        stopped_by = "q"
    except Exception as e:
        stopped_by = "error"
        # store error in last_analysis
        if not isinstance(last_analysis, dict):
            last_analysis = {}
        last_analysis = dict(last_analysis) if isinstance(last_analysis, dict) else {}
        last_analysis["error"] = str(e)
    finally:
        try:
            if camera is not None and hasattr(camera, "stop"):
                camera.stop()
        except Exception:
            pass
        if show and cv2 is not None:
            try:
                cv2.destroyWindow(window_name)
            except Exception:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass

    if stopped_by is None:
        stopped_by = "max_frames" if (max_frames is not None and frames >= max_frames) else "q"

    return {
        "frames": frames,
        "saved": saved,
        "last_analysis": last_analysis,
        "stopped_by": stopped_by,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="python -m src.live_vision_hud")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--mode", default="hybrid")
    parser.add_argument("--port", default=None)
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args(argv)

    controller = None
    sim = bool(args.sim) or (args.port is None)
    if AgentController is not None:
        try:
            controller = AgentController(sim=sim, port=args.port)
        except Exception:
            controller = None

    try:
        configure_opencv()
    except Exception:
        pass

    res = run_hud(
        camera_index=args.camera,
        width=args.width,
        height=args.height,
        fps=args.fps,
        mode=args.mode,
        camera=None,
        analyzer=None,
        controller=controller,
        recorder=None,
        window_name="ARM 6-DOF VISION",
        max_frames=args.max_frames,
        show=not args.headless,
    )
    try:
        import json

        print(json.dumps({"frames": res["frames"], "saved": res["saved"], "stopped_by": res["stopped_by"]}))
    except Exception:
        pass
    if controller is not None and hasattr(controller, "transport"):
        try:
            controller.transport.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
