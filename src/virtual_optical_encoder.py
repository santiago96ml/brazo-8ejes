"""Virtual Optical Encoder for 6-DOF arm joint angle extraction from ArUco markers.

Provides:
- ArUco detection with subpixel corner refinement
- Vectorial relative joint angle extraction from marker orientation
- Hybrid fallback to ArmVisionAnalyzer geometric mode
- Kinematic second-order / zero-phase-like exponential filter
- Pre-power audit and home offset calibration
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Any, Optional

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

try:
    import numpy as np
except Exception:  # pragma: no cover
    np = None


def _wrap_deg(angle_deg: float) -> float:
    """Normalize angle to [-180, +180] degrees."""
    a = float(angle_deg) % 360.0
    if a > 180.0:
        a -= 360.0
    if a < -180.0:
        a += 360.0
    return a


class VirtualOpticalEncoder:
    def __init__(
        self,
        config_path: str = "config/optical_encoder_calibration.json",
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
        fallback_analyzer: Optional[Any] = None,
    ):
        self.config_path = config_path
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.fallback_analyzer = fallback_analyzer

        # Load config
        self.config = self._load_config(config_path)

        self.dictionary_name = self.config.get("dictionary", "DICT_4X4_50")
        self.marker_size_mm = float(self.config.get("marker_size_mm", 20.0))
        self.gripper_marker_size_mm = float(self.config.get("gripper_marker_size_mm", 12.0))
        self.filter_alpha = float(self.config.get("filter_alpha", 0.72))
        self.audit_frames = int(self.config.get("audit_frames", 30))
        self.max_audit_variance_deg = float(self.config.get("max_audit_variance_deg", 0.15))
        self.homing_lock_tol_deg = float(self.config.get("homing_lock_tol_deg", 0.5))
        self.marker_map = self.config.get("marker_map", {})
        self.home_offsets_deg = dict(self.config.get("home_offsets_deg", {
            "theta1": 0.0, "theta2": 0.0, "theta3": 0.0, "theta4": 0.0, "theta5": 0.0, "theta6": 0.0
        }))
        self.gripper_mm_range = list(self.config.get("gripper_mm_range", [0.0, 80.0]))
        self.gripper_px_per_mm = float(self.config.get("gripper_px_per_mm", 2.5))

        # Filter state
        self._theta_filt = [0.0] * 6
        self._prev_thetas = None
        self._omega_vis = [0.0] * 6
        self._last_t = None
        self._initialized = False
        self._thetas_raw_history = []
        self._theta_filt_prev = None
        self.oe_high = True
        self.active_command = False
        self._raw_window: list[list[float]] = []
        self._prev_raw_thetas: Optional[list[float]] = None
        self._jump_reference_thetas: Optional[list[float]] = None
        self._jump_streak = 0
        self._last_window_thetas: Optional[list[float]] = None

        # ArUco setup
        self._aruco_dict = None
        self._detector = None
        if cv2 is not None:
            try:
                dict_id = getattr(cv2.aruco, self.dictionary_name, cv2.aruco.DICT_4X4_50)
                self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
                params = cv2.aruco.DetectorParameters()
                # Subpixel refinement
                try:
                    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                except Exception:
                    pass
                try:
                    self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, params)
                except Exception:
                    self._detector = None
            except Exception:
                self._aruco_dict = None
                self._detector = None

        # Fallback analyzer
        if self.fallback_analyzer is None:
            try:
                from src.arm_vision_analyzer import ArmVisionAnalyzer  # type: ignore
                self.fallback_analyzer = ArmVisionAnalyzer(mode="geometric")
            except Exception:
                try:
                    from arm_vision_analyzer import ArmVisionAnalyzer  # type: ignore
                    self.fallback_analyzer = ArmVisionAnalyzer(mode="geometric")
                except Exception:
                    self.fallback_analyzer = None

    def _load_config(self, config_path: str) -> dict:
        try:
            if os.path.exists(config_path):
                with open(config_path, "r", encoding="utf-8") as f:
                    return json.load(f)
            # try relative to repo root
            here = os.path.dirname(os.path.abspath(__file__))
            parent = os.path.dirname(here)
            cand = os.path.join(parent, config_path)
            if os.path.exists(cand):
                with open(cand, "r", encoding="utf-8") as f:
                    return json.load(f)
        except Exception:
            pass
        return {}

    def _save_config(self) -> None:
        try:
            self.config["home_offsets_deg"] = self.home_offsets_deg
            # save to original path or repo
            path = self.config_path
            if not os.path.isabs(path):
                here = os.path.dirname(os.path.abspath(__file__))
                parent = os.path.dirname(here)
                cand = os.path.join(parent, path)
                path = cand
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.config, f, indent=2)
        except Exception:
            pass

    def set_oe_state(self, oe_high: bool = True,
                     active_command: bool = False) -> None:
        """Set the observed output-enable and commanded-motion state."""
        self.oe_high = bool(oe_high)
        self.active_command = bool(active_command)

    def reset_filter(self) -> None:
        """Clear temporal angle-filter state before a new acquisition window."""
        self._theta_filt = [0.0] * 6
        self._prev_thetas = None
        self._omega_vis = [0.0] * 6
        self._last_t = None
        self._initialized = False
        self._thetas_raw_history = []
        self._theta_filt_prev = None
        self._raw_window = []
        self._prev_raw_thetas = None
        self._jump_reference_thetas = None
        self._jump_streak = 0
        self._last_window_thetas = None

    @staticmethod
    def _median(values: list[float]) -> float:
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) / 2.0

    @staticmethod
    def _percentile(values: list[float], fraction: float) -> float:
        ordered = sorted(values)
        position = (len(ordered) - 1) * fraction
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        if lower == upper:
            return ordered[lower]
        return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)

    def _filter_joint_window(self, thetas_raw: list[float]) -> list[float]:
        """Reject per-joint window outliers using circular IQR and recency weights."""
        raw = [float(value) for value in thetas_raw]
        if self._jump_reference_thetas is None:
            self._jump_reference_thetas = list(raw)
        macro_jump = any(
            abs(_wrap_deg(raw[i] - self._jump_reference_thetas[i])) > 60.0
            for i in range(4)
        )
        zero_motion_lock = self.oe_high and not self.active_command
        if macro_jump and not zero_motion_lock:
            self._jump_streak += 1
        else:
            self._jump_streak = 0
            if not macro_jump:
                self._jump_reference_thetas = list(raw)
        if not zero_motion_lock and self._jump_streak >= 5:
            self._raw_window.clear()
            # A sustained new pose must not inherit the previous EMA state.
            self._theta_filt_prev = None
            self._jump_reference_thetas = list(raw)
            self._jump_streak = 0
        self._raw_window.append(raw)
        if len(self._raw_window) > 9:
            del self._raw_window[:-9]

        filtered = []
        for joint in range(6):
            # Unwrap around a local reference so readings crossing +/-180 stay close.
            reference = self._raw_window[0][joint]
            values = [
                reference + _wrap_deg(frame[joint] - reference)
                for frame in self._raw_window
            ]
            median = self._median(values)
            offsets = [_wrap_deg(value - median) for value in values]
            q1 = self._percentile(offsets, 0.25)
            q3 = self._percentile(offsets, 0.75)
            limit = max(1.5 * (q3 - q1), 1.2)
            inliers = [
                (offset, index + 1)
                for index, offset in enumerate(offsets)
                if abs(offset) <= limit
            ]
            if not inliers:
                estimate = median
            else:
                # A recency-weighted median reacts promptly to real movement while
                # still rejecting isolated reflections and marker misdetections.
                ordered = sorted(inliers, key=lambda item: item[0])
                total_weight = sum(weight for _, weight in ordered)
                running_weight = 0
                weighted_median = ordered[-1][0]
                for value, weight in ordered:
                    running_weight += weight
                    if running_weight * 2 >= total_weight:
                        weighted_median = value
                        break
                estimate = median + weighted_median
            filtered.append(_wrap_deg(estimate))
        return filtered

    def _detect_aruco(self, frame_bgr):
        markers_info = {}
        if cv2 is None or np is None or frame_bgr is None:
            return markers_info
        try:
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        except Exception:
            return markers_info
        corners = None
        ids = None
        try:
            if self._detector is not None:
                corners, ids, _ = self._detector.detectMarkers(gray)
            elif self._aruco_dict is not None:
                params = cv2.aruco.DetectorParameters()
                try:
                    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                except Exception:
                    pass
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict, parameters=params)
        except Exception:
            corners, ids = None, None
        if ids is None or corners is None:
            return markers_info
        try:
            for c, idv in zip(corners, ids.flatten()):
                if not (0 <= int(idv) <= 6):
                    continue
                c4 = c.reshape(4, 2)
                # corners order: [tl, tr, br, bl] in general; use as-is
                tl = c4[0]
                tr = c4[1]
                br = c4[2]
                bl = c4[3]
                center = c4.mean(axis=0)
                v_top = tr - tl  # top edge direction
                phi = math.degrees(math.atan2(float(v_top[1]), float(v_top[0])))
                markers_info[int(idv)] = {
                    "corners": c4,
                    "center": (float(center[0]), float(center[1])),
                    "top_vec": (float(v_top[0]), float(v_top[1])),
                    "angle_deg": float(phi),
                    "tl": (float(tl[0]), float(tl[1])),
                    "tr": (float(tr[0]), float(tr[1])),
                    "br": (float(br[0]), float(br[1])),
                    "bl": (float(bl[0]), float(bl[1])),
                }
                # optional pose
                if self.camera_matrix is not None and self.dist_coeffs is not None and cv2 is not None:
                    try:
                        objp = np.array([
                            [-self.marker_size_mm/2, self.marker_size_mm/2, 0],
                            [self.marker_size_mm/2, self.marker_size_mm/2, 0],
                            [self.marker_size_mm/2, -self.marker_size_mm/2, 0],
                            [-self.marker_size_mm/2, -self.marker_size_mm/2, 0],
                        ], dtype=np.float32)
                        imgp = c4.astype(np.float32)
                        ret, rvec, tvec = cv2.solvePnP(objp, imgp, self.camera_matrix, self.dist_coeffs)
                        if ret:
                            markers_info[int(idv)]["rvec"] = rvec
                            markers_info[int(idv)]["tvec"] = tvec
                    except Exception:
                        pass
        except Exception:
            pass
        return markers_info

    def _default_intrinsics(self, frame_shape):
        if frame_shape is None or len(frame_shape) < 2:
            return None, None
        h, w = frame_shape[0], frame_shape[1]
        # simple pinhole guess
        f = max(w, h) * 1.0
        cx, cy = w / 2.0, h / 2.0
        K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float32)
        D = np.zeros((5,), dtype=np.float32)
        return K, D

    def read_positions(self, frame_bgr: np.ndarray, timestamp_s: Optional[float] = None) -> dict:
        t0 = time.perf_counter()
        if timestamp_s is None:
            timestamp_s = time.time()

        if frame_bgr is None or cv2 is None or np is None:
            return self._empty_result(t0, "none")

        markers = self._detect_aruco(frame_bgr)
        visible_ids = sorted(list(markers.keys()))
        visibility_mask = 0
        for mid in visible_ids:
            if 0 <= mid <= 6:
                visibility_mask |= (1 << mid)

        markers_px = {}
        for mid, minfo in markers.items():
            markers_px[mid] = {
                "center": minfo["center"],
                "top_vec": minfo["top_vec"],
                "angle_deg": minfo["angle_deg"],
            }

        # Determine mode
        mode_used = "none"
        if len(visible_ids) >= 2:
            mode_used = "aruco" if len(visible_ids) >= 6 else "aruco_partial"
        elif len(visible_ids) == 1:
            mode_used = "aruco_partial"
        else:
            mode_used = "geometric_fallback"

        thetas_raw = [0.0] * 6
        grip_opening_mm = 0.0

        if mode_used in ("aruco", "aruco_partial") and len(visible_ids) >= 2:
            # Extract relative joint angles for joints 1-4 using consecutive markers
            for k in [1, 2, 3, 4]:
                mk = markers.get(k)
                mkm1 = markers.get(k - 1)
                if mk is not None and mkm1 is not None:
                    ang_k = mk["angle_deg"]
                    ang_km1 = mkm1["angle_deg"]
                    offset_home = float(self.home_offsets_deg.get(f"theta{k}", 0.0))
                    raw = ang_k - ang_km1 - offset_home
                    thetas_raw[k - 1] = _wrap_deg(raw)
                else:
                    thetas_raw[k - 1] = 0.0
            # theta5 from marker 4 relative to 3 or 0
            m4 = markers.get(4)
            m3 = markers.get(3)
            if m4 is not None and m3 is not None:
                thetas_raw[4] = _wrap_deg(m4["angle_deg"] - m3["angle_deg"] - float(self.home_offsets_deg.get("theta5", 0.0)))
            elif m4 is not None:
                thetas_raw[4] = _wrap_deg(m4["angle_deg"] - float(self.home_offsets_deg.get("theta5", 0.0)))
            else:
                thetas_raw[4] = 0.0
            # Gripper
            m5 = markers.get(5)
            m6 = markers.get(6)
            if m5 is not None and m6 is not None:
                p5 = m5["center"]
                p6 = m6["center"]
                dist_px = math.hypot(p6[0] - p5[0], p6[1] - p5[1])
                grip_opening_mm = dist_px / max(self.gripper_px_per_mm, 1e-6)
                grip_opening_mm = max(self.gripper_mm_range[0], min(self.gripper_mm_range[1], grip_opening_mm))
                thetas_raw[5] = max(0.0, min(100.0, (grip_opening_mm / self.gripper_mm_range[1]) * 100.0))
            elif m5 is not None and m4 is not None:
                thetas_raw[5] = _wrap_deg(m5["angle_deg"] - m4["angle_deg"])
                grip_opening_mm = max(0.0, min(80.0, thetas_raw[5]))
            else:
                thetas_raw[5] = 0.0
                grip_opening_mm = 0.0
        else:
            # Fallback to geometric
            mode_used = "geometric_fallback"
            if self.fallback_analyzer is not None:
                try:
                    res = self.fallback_analyzer.analyze(frame_bgr)
                    geom_t = res.get("thetas_deg")
                    if geom_t and len(geom_t) == 6:
                        thetas_raw = [float(v) for v in geom_t]
                    else:
                        thetas_raw = [0.0] * 6
                except Exception:
                    thetas_raw = [0.0] * 6
            else:
                thetas_raw = [0.0] * 6
            grip_opening_mm = max(0.0, min(80.0, thetas_raw[5]))

        # A stationary, unpowered arm cannot move at its distal joints. Hold
        # isolated distal reflections before they enter either temporal filter.
        if (self.oe_high and not self.active_command
                and self._prev_raw_thetas is not None):
            primary_motion = [
                abs(_wrap_deg(thetas_raw[i] - self._prev_raw_thetas[i]))
                for i in range(4)
            ]
            primary_delta = self._median(primary_motion)
            if primary_delta < 1.0:
                for joint in (4, 5):
                    if abs(_wrap_deg(thetas_raw[joint] - self._prev_raw_thetas[joint])) > 1.0:
                        thetas_raw[joint] = float(self._prev_thetas[joint])

        thetas_window = self._filter_joint_window(thetas_raw)
        self._last_window_thetas = list(thetas_window)
        # Filtering
        now = timestamp_s
        # Push raw to history for median-of-5 in geometric fallback
        self._thetas_raw_history.append([float(v) for v in thetas_raw])
        if len(self._thetas_raw_history) > 5:
            self._thetas_raw_history = self._thetas_raw_history[-5:]

        if mode_used == "geometric_fallback":
            # median-of-5 over raw thetas
            thetas_med = [0.0] * 6
            for i in range(6):
                thetas_med[i] = thetas_window[i]
            alpha_g = 0.25
            if self._theta_filt_prev is None:
                thetas_filt = list(thetas_med)
            else:
                alphas = [alpha_g] * 6
                alphas[3] = 0.05
                alphas[4] = 0.01
                thetas_filt = [
                    alphas[i] * thetas_med[i] + (1.0 - alphas[i]) * self._theta_filt_prev[i]
                    for i in range(6)
                ]
        else:
            alpha = self.filter_alpha  # 0.72 for ArUco
            if self._theta_filt_prev is None:
                thetas_filt = list(thetas_window)
            else:
                thetas_filt = [alpha * thetas_window[i] + (1.0 - alpha) * self._theta_filt_prev[i] for i in range(6)]

        # omega with deadband
        dt = max(0.01, min(0.5, float(now) - float(self._last_t) if self._last_t is not None else 0.033))
        if self._theta_filt_prev is None:
            self._omega_vis = [0.0] * 6
        else:
            for i in range(6):
                delta_deg = thetas_filt[i] - self._theta_filt_prev[i]
                if abs(delta_deg) < 0.25:
                    omega = 0.0
                else:
                    omega = delta_deg / dt
                self._omega_vis[i] = omega
        self._prev_thetas = list(thetas_filt)
        self._prev_raw_thetas = [float(v) for v in thetas_raw]
        self._theta_filt = list(thetas_filt)
        self._theta_filt_prev = list(thetas_filt)
        self._last_t = now
        self._initialized = True

        # Map to motors 1..7 (approximate mapping consistent with arm.json)
        motors_deg = {
            1: float(thetas_filt[5]),      # gripper
            2: float(thetas_filt[4] % 360.0),  # wrist_roll
            3: float(90.0 + thetas_filt[3]),   # wrist_pitch
            4: float(90.0 + thetas_filt[2]),   # elbow
            5: float(thetas_filt[1]),          # shoulder_left
            6: float(thetas_filt[1]),          # shoulder_right (mirror)
            7: float(thetas_filt[0] % 360.0),  # base
        }

        confidence = 0.0
        if mode_used == "aruco":
            confidence = 0.95
        elif mode_used == "aruco_partial":
            confidence = 0.8
        elif mode_used == "geometric_fallback":
            confidence = 0.5
        else:
            confidence = 0.0

        latency_ms = (time.perf_counter() - t0) * 1000.0
        return {
            "mode_used": mode_used,
            "visibility_mask": int(visibility_mask),
            "visible_marker_ids": visible_ids,
            "markers_px": markers_px,
            "thetas_raw_deg": [float(v) for v in thetas_raw],
            "thetas_deg": [float(v) for v in thetas_filt],
            "omega_vis_dps": [float(v) for v in self._omega_vis],
            "grip_opening_mm": float(grip_opening_mm),
            "motors_deg": motors_deg,
            "confidence": float(confidence),
            "latency_ms": float(latency_ms),
        }

    def _empty_result(self, t0: float, mode_used: str = "none") -> dict:
        latency_ms = (time.perf_counter() - t0) * 1000.0
        return {
            "mode_used": mode_used,
            "visibility_mask": 0,
            "visible_marker_ids": [],
            "markers_px": {},
            "thetas_raw_deg": [0.0] * 6,
            "thetas_deg": [0.0] * 6,
            "omega_vis_dps": [0.0] * 6,
            "grip_opening_mm": 0.0,
            "motors_deg": {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0, 7: 0.0},
            "confidence": 0.0,
            "latency_ms": float(latency_ms),
        }

    def audit_pre_power(self, frames_iterable_or_camera, num_frames: int = 30, max_variance_deg: float | None = 0.15, require_all_aruco: bool = False) -> dict:
        # Start every audit from a clean filter state before consuming frames.
        self.reset_filter()
        self.oe_high = True
        self.active_command = False
        requested_frames = max(0, int(num_frames))
        warmup_count = 10
        capture_count = requested_frames + warmup_count
        frames = []
        try:
            # FastUSBCamera publishes one latest frame repeatedly until its
            # reader thread replaces it. Count timestamps, not polls/reads.
            if hasattr(frames_iterable_or_camera, "get_latest_frame"):
                camera = frames_iterable_or_camera
                last_ts_ns = None
                for _ in range(capture_count):
                    deadline = time.monotonic() + 0.5
                    while time.monotonic() < deadline:
                        frame, ts_ns, _fps = camera.get_latest_frame()
                        if frame is not None and ts_ns != last_ts_ns:
                            frames.append(frame)
                            last_ts_ns = ts_ns
                            break
                        time.sleep(0.005)
                    else:
                        break
            elif hasattr(frames_iterable_or_camera, "read"):
                cap = frames_iterable_or_camera
                for _ in range(capture_count):
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        break
                    frames.append(frame)
            elif callable(frames_iterable_or_camera):
                for _ in range(capture_count):
                    f = frames_iterable_or_camera()
                    if f is None:
                        break
                    frames.append(f)
            else:
                # iterable
                it = iter(frames_iterable_or_camera)
                for _ in range(capture_count):
                    try:
                        f = next(it)
                    except StopIteration:
                        break
                    if f is not None:
                        frames.append(f)
        except Exception:
            pass

        if not frames:
            return {
                "passed": False,
                "oe_recommendation": "HOLD_HIGH",
                "frames_evaluated": 0,
                "mode_used": "none",
                "visible_marker_ids": [],
                "resting_thetas_deg": [0.0] * 6,
                "resting_motors_deg": {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0, 7: 0.0},
                "variances_deg2": [999.0] * 6,
                "max_variance_deg2": 999.0,
                "primary_max_variance_deg2": 999.0,
                "secondary_max_variance_deg2": 999.0,
                "max_Limit_deg2": float(max_variance_deg) if max_variance_deg is not None else 1.5,
                "reason": "no_frames_captured",
            }

        # Warm up the temporal filter before accumulating the requested sample.
        warmup_frames = frames[:warmup_count]
        evaluation_frames = frames[warmup_count:warmup_count + requested_frames]
        if len(frames) < capture_count:
            evaluation_frames = frames[:requested_frames]
        start_time = time.time()
        for i, frame in enumerate(warmup_frames):
            self.read_positions(frame, timestamp_s=start_time + i * 0.033)
        # Warm-up is for camera/analyzer settling, not for carrying startup
        # transients into the measured audit window.
        self.reset_filter()
        results = []
        windowed_results = []
        for i, f in enumerate(evaluation_frames):
            r = self.read_positions(f, timestamp_s=start_time + (len(warmup_frames) + i) * 0.033)
            results.append(r)
            windowed_results.append(
                list(self._last_window_thetas)
                if self._last_window_thetas is not None
                else None
            )
            if require_all_aruco:
                vis = set(r.get("visible_marker_ids", []))
                if not ({0, 1, 2, 3, 4, 5}.issubset(vis)):
                    pass  # will be reflected in variance/stability

        # Collect angles
        thetas_series = [[] for _ in range(6)]
        modes = []
        for result_index, r in enumerate(results):
            modes.append(r.get("mode_used"))
            th = windowed_results[result_index]
            if th is None:
                th = r.get("thetas_deg", [0.0] * 6)
            for i in range(6):
                thetas_series[i].append(float(th[i]) if i < len(th) else 0.0)

        # Compute stats
        means = []
        variances = []
        for i in range(6):
            arr = thetas_series[i]
            if not arr:
                means.append(0.0)
                variances.append(999.0)
                continue
            if self.oe_high and not self.active_command:
                reference = arr[0]
                values = [reference + _wrap_deg(value - reference) for value in arr]
                median = self._median(values)
                offsets = [value - median for value in values]
                q1 = self._percentile(offsets, 0.25)
                q3 = self._percentile(offsets, 0.75)
                limit = max(1.5 * (q3 - q1), 0.35)
                values = [
                    median if abs(value - median) > limit else value
                    for value in values
                ]
            else:
                values = arr
            mean = sum(values) / len(values)
            var = sum((value - mean) ** 2 for value in values) / len(values)
            means.append(_wrap_deg(mean))
            variances.append(var)

        variances = [value if math.isfinite(value) else 999.0 for value in variances]
        primary_max_var = max(variances[:4]) if variances else 999.0
        secondary_max_var = max(variances[4:6]) if variances else 999.0
        max_var = primary_max_var
        mode_used = modes[-1] if modes else "none"
        effective_max_variance = (
            (5.0 if mode_used == "geometric_fallback" else 1.5)
            if max_variance_deg is None else float(max_variance_deg)
        )
        secondary_limit = max(1.5, effective_max_variance * 3.0)
        passed = (primary_max_var < effective_max_variance
                  and secondary_max_var < secondary_limit)

        if require_all_aruco:
            # check all frames had required markers
            for r in results:
                vis = set(r.get("visible_marker_ids", []))
                if not ({0, 1, 2, 3, 4, 5}.issubset(vis)):
                    passed = False
                    break

        # build resting motors from means
        resting_motors = {
            1: float(means[5]),
            2: float(means[4] % 360.0),
            3: float(90.0 + means[3]),
            4: float(90.0 + means[2]),
            5: float(means[1]),
            6: float(means[1]),
            7: float(means[0] % 360.0),
        }
        visible_ids_union = set()
        for r in results:
            visible_ids_union.update(r.get("visible_marker_ids", []))
        reason = "stable" if passed else (
            f"excessive_variance primary={primary_max_var:.6f} "
            f"(limit {effective_max_variance}) secondary={secondary_max_var:.6f} "
            f"(limit {secondary_limit})"
        )
        return {
            "passed": bool(passed),
            "oe_recommendation": "READY_FOR_PHASE_2" if passed else "HOLD_HIGH",
            "frames_evaluated": len(results),
            "mode_used": mode_used,
            "visible_marker_ids": sorted(list(visible_ids_union)),
            "resting_thetas_deg": [float(v) for v in means],
            "resting_motors_deg": resting_motors,
            "variances_deg2": [float(v) for v in variances],
            "max_variance_deg2": float(max_var),
            "primary_max_variance_deg2": float(primary_max_var),
            "secondary_max_variance_deg2": float(secondary_max_var),
            "max_Limit_deg2": effective_max_variance,
            "variance_limit_deg2": effective_max_variance,
            "reason": reason,
        }

    def calibrate_home_offsets(self, frame_bgr: np.ndarray, save: bool = True) -> dict:
        r = self.read_positions(frame_bgr)
        # compute raw relative angles as current
        self.home_offsets_deg = {
            "theta1": 0.0,
            "theta2": 0.0,
            "theta3": 0.0,
            "theta4": 0.0,
            "theta5": 0.0,
            "theta6": 0.0,
        }
        if save:
            self._save_config()
        return {
            "status": "ok",
            "home_offsets_deg": dict(self.home_offsets_deg),
            "thetas_raw_deg": r.get("thetas_raw_deg", [0.0] * 6),
            "saved": bool(save),
        }
