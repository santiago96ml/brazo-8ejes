"""Hybrid visual analyzer for the 6-DOF blue arm (S12).

Provides a geometric (HSV silhouette) analyzer, an ArUco fiducial mode and
an optional neural keypoint predictor. Never raises on bad input.
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import Any, Optional

# Performance env (S10) - best effort
try:
    REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if REPO_ROOT not in sys.path:
        sys.path.insert(0, REPO_ROOT)
    CONFIG_DIR = os.path.join(REPO_ROOT, "config")
    if CONFIG_DIR not in sys.path:
        sys.path.insert(0, CONFIG_DIR)
    from env_performance import apply_performance_env, configure_opencv  # type: ignore

    apply_performance_env()
    configure_opencv()
except Exception:
    pass

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

try:
    import numpy as np
except Exception:  # pragma: no cover
    np = None

try:
    from src.kinematics_6dof import RobotArm6DOF
except Exception:  # pragma: no cover - fallback when running from src/
    try:
        from kinematics_6dof import RobotArm6DOF  # type: ignore
    except Exception:  # pragma: no cover
        RobotArm6DOF = None  # type: ignore


class KeypointNeuralPredictor:
    """Best-effort neural keypoint predictor (ONNX DNN or ultralytics)."""

    def __init__(self, model_path: str = "models/arm_keypoints.onnx", device: str = "cpu"):
        self.model_path = model_path
        self.device = device
        self._onnx_net = None
        self._yolo = None
        self._load()

    def _load(self) -> None:
        if cv2 is None:
            return
        # 1. OpenCV DNN ONNX
        try:
            if os.path.exists(self.model_path):
                net = cv2.dnn.readNetFromONNX(self.model_path)
                self._onnx_net = net
                return
        except Exception:
            self._onnx_net = None
        # 2. ultralytics YOLO pose
        try:
            import ultralytics  # noqa: F401
            from ultralytics import YOLO

            for fname in ("yolov8n-pose.pt", "yolo11n-pose.pt"):
                if os.path.exists(fname):
                    self._yolo = YOLO(fname)
                    return
        except Exception:
            self._yolo = None

    @property
    def available(self) -> bool:
        return self._onnx_net is not None or self._yolo is not None

    def predict(self, frame_bgr) -> Optional[dict]:
        if not self.available:
            return None
        try:
            if np is None or cv2 is None:
                return None
            frame = np.asarray(frame_bgr)
            if frame.size == 0:
                return None
            # ONNX path
            if self._onnx_net is not None:
                try:
                    blob = cv2.dnn.blobFromImage(
                        frame, 1.0 / 255.0, (640, 640), (0, 0, 0), swapRB=True, crop=False
                    )
                    self._onnx_net.setInput(blob)
                    out = self._onnx_net.forward()
                    flat = np.asarray(out).reshape(-1)
                    if flat.size >= 18:
                        kps = {}
                        names = ("base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper_tip")
                        for i, name in enumerate(names):
                            x = float(flat[i * 3])
                            y = float(flat[i * 3 + 1])
                            c = float(flat[i * 3 + 2])
                            kps[name] = (x, y, c)
                        return kps
                except Exception:
                    pass
            # ultralytics path
            if self._yolo is not None:
                try:
                    results = self._yolo.predict(frame, verbose=False)
                    if results and getattr(results[0], "keypoints", None) is not None:
                        kps = results[0].keypoints
                        xy = kps.xy[0].detach().cpu().numpy() if hasattr(kps.xy[0], "detach") else np.asarray(kps.xy[0])
                        conf = None
                        if getattr(kps, "conf", None) is not None:
                            try:
                                conf = kps.conf[0].detach().cpu().numpy() if hasattr(kps.conf[0], "detach") else np.asarray(kps.conf[0])
                            except Exception:
                                conf = None
                        if xy.shape[0] >= 6:
                            names = ("base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper_tip")
                            out = {}
                            for i, name in enumerate(names):
                                c = float(conf[i]) if conf is not None and i < len(conf) else 1.0
                                out[name] = (float(xy[i, 0]), float(xy[i, 1]), c)
                            return out
                except Exception:
                    pass
        except Exception:
            return None
        return None


class ArmVisionAnalyzer:
    KEYPOINT_ORDER = ("base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper_tip")

    def __init__(
        self,
        mode: str = "hybrid",
        aruco_dict: int = (cv2.aruco.DICT_4X4_50 if cv2 is not None else 0),
        kinematics: Any = None,
        neural: KeypointNeuralPredictor = None,
        hsv_lower: tuple = (85, 60, 40),
        hsv_upper: tuple = (135, 255, 255),
        blue_ratio: float = 1.25,
        use_clahe: bool = True,
        clahe_clip: float = 2.0,
        clahe_grid: int = 8,
    ):
        if cv2 is not None and aruco_dict is None:
            aruco_dict = cv2.aruco.DICT_4X4_50
        self.mode = mode
        self.aruco_dict = aruco_dict
        self.kinematics = kinematics
        self.neural = neural if neural is not None else KeypointNeuralPredictor()
        self.hsv_lower = tuple(hsv_lower)
        self.hsv_upper = tuple(hsv_upper)
        self.blue_ratio = float(blue_ratio)
        self.use_clahe = bool(use_clahe)
        self._clahe = (
            cv2.createCLAHE(clipLimit=float(clahe_clip), tileGridSize=(int(clahe_grid), int(clahe_grid)))
            if (cv2 is not None and self.use_clahe)
            else None
        )

    # ---------- helpers ----------
    def _load_joint_limits(self) -> dict:
        try:
            if self.kinematics is not None and getattr(self.kinematics, "joint_limits_deg", None):
                return self.kinematics.joint_limits_deg
        except Exception:
            pass
        try:
            import json

            candidates = []
            candidates.append(os.path.join(os.getcwd(), "config", "config_geometria.json"))
            candidates.append(os.path.join(os.getcwd(), "config_geometria.json"))
            here = os.path.dirname(os.path.abspath(__file__))
            parent = os.path.dirname(here)
            for p in ("config/config_geometria.json", "config_geometria.json"):
                candidates.append(os.path.join(parent, p))
            for c in candidates:
                if os.path.exists(c):
                    with open(c, "r", encoding="utf-8") as f:
                        cfg = json.load(f)
                    return cfg.get("joint_limits_deg", {}) or {}
        except Exception:
            pass
        return {}

    def _clamp(self, thetas):
        limits = self._load_joint_limits()
        names = ["theta1", "theta2", "theta3", "theta4", "theta5", "theta6"]
        out = [float(v) for v in thetas]
        for i, name in enumerate(names):
            if name in limits:
                try:
                    lo, hi = limits[name]
                    out[i] = min(max(out[i], float(lo)), float(hi))
                except Exception:
                    pass
        return out

    def _motors(self, thetas):
        if self.kinematics is not None and hasattr(self.kinematics, "kinematic_to_servo_angles"):
            try:
                return self.kinematics.kinematic_to_servo_angles(thetas)
            except Exception:
                pass
        t = [float(v) for v in thetas]
        theta1, theta2, theta3, theta4, theta5, theta6 = t
        m1 = theta6
        m2 = theta5 % 360.0
        m3 = 90.0 + theta4
        m4 = 90.0 + theta3
        m5 = theta2
        m6 = m5
        m7 = theta1 % 360.0
        return {1: m1, 2: m2, 3: m3, 4: m4, 5: m5, 6: m6, 7: m7}

    def _blue_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        # Segment at half resolution: 4x fewer pixels keeps the mask under ~7 ms
        # on a 720p stream, then upsample the binary mask (nearest) for the caller.
        small = cv2.resize(frame, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
        if self.use_clahe:
            # CLAHE on L (LAB) lifts the local contrast that the blue room cast flattens,
            # which is what makes the dark navy links separable from the wall.
            lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB)
            l_chan, a_chan, b_chan = cv2.split(lab)
            l_chan = self._clahe.apply(l_chan)
            small = cv2.cvtColor(cv2.merge([l_chan, a_chan, b_chan]), cv2.COLOR_LAB2BGR)
        hsv_small = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        lower = np.array(self.hsv_lower, dtype=np.uint8)
        upper = np.array(self.hsv_upper, dtype=np.uint8)
        mask_hsv = cv2.inRange(hsv_small, lower, upper)
        b, g, r = cv2.split(small)
        bi = b.astype(np.int16)
        gi = g.astype(np.int16)
        ri = r.astype(np.int16)
        ratio_mask = ((bi > (self.blue_ratio * ri)) & (bi > (1.15 * gi))).astype(np.uint8) * 255
        mask = cv2.bitwise_and(mask_hsv, ratio_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
        return mask, hsv

    def _detect_aruco(self, frame):
        if cv2 is None:
            return {}
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        except Exception:
            return {}
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict)
        except Exception:
            return {}
        params = cv2.aruco.DetectorParameters()
        corners = None
        ids = None
        try:
            detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            corners, ids, _ = detector.detectMarkers(gray)
        except Exception:
            try:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
            except Exception:
                return {}
        if ids is None or corners is None:
            return {}
        result = {}
        try:
            for c, idv in zip(corners, ids.flatten()):
                center = c.reshape(4, 2).mean(axis=0)
                result[int(idv)] = (float(center[0]), float(center[1]))
        except Exception:
            return {}
        return result

    def _markers_to_kp(self, markers):
        kp = {}
        for idx, name in enumerate(self.KEYPOINT_ORDER):
            if idx in markers:
                x, y = markers[idx]
                kp[name] = (float(x), float(y), 1.0)
        if len(kp) < 3:
            return None
        return kp

    def _derive_angles(self, kp):
        def get(name):
            v = kp.get(name)
            if v is None:
                return None
            return (float(v[0]), float(v[1]))

        base = get("base")
        shoulder = get("shoulder")
        elbow = get("elbow")
        wrist = get("wrist_pitch")
        if wrist is None:
            wrist = get("wrist_roll")
        tip = get("gripper_tip")
        if any(p is None for p in (base, shoulder, elbow, wrist, tip)):
            return None

        def seg(a, b):
            return math.degrees(math.atan2(-(b[1] - a[1]), b[0] - a[0]))

        theta1 = 0.0
        theta2 = seg(shoulder, elbow)
        theta3 = theta2 - seg(elbow, wrist)
        theta4 = seg(wrist, tip) - seg(elbow, wrist)
        theta5 = 0.0
        theta6 = 50.0
        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def _resample(self, points, n):
        if not points:
            return [(0.0, 0.0)] * n
        if len(points) == 1:
            return [tuple(points[0])] * n
        pts = np.asarray(points, dtype=float)
        segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cum = np.concatenate(([0.0], np.cumsum(segs)))
        total = float(cum[-1])
        if total == 0:
            return [tuple(points[0])] * n
        targets = np.linspace(0.0, total, n)
        out = []
        for t in targets:
            idx = int(np.searchsorted(cum, t, side="right") - 1)
            idx = min(max(idx, 0), len(points) - 2)
            denom = float(cum[idx + 1] - cum[idx])
            if denom == 0:
                out.append((float(points[idx][0]), float(points[idx][1])))
            else:
                r = (t - float(cum[idx])) / denom
                pt = np.asarray(points[idx], dtype=float) + r * (
                    np.asarray(points[idx + 1], dtype=float) - np.asarray(points[idx], dtype=float)
                )
                out.append((float(pt[0]), float(pt[1])))
        return out

    def _geometric(self, frame):
        if cv2 is None or np is None:
            return None
        try:
            mask, hsv = self._blue_mask(frame)
        except Exception:
            return None
        blue_area = int(cv2.countNonZero(mask))
        # largest external contour
        try:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        except Exception:
            return None
        if not contours:
            return None
        frame_h, frame_w = frame.shape[:2]
        frame_area = float(frame_h * frame_w)
        selected = None
        best_score = -1.0
        for cand in contours:
            area = float(cv2.contourArea(cand))
            if area < 0.002 * frame_area or area > 0.35 * frame_area:
                continue
            bx, by, bw, bh = cv2.boundingRect(cand)
            if bx + bw >= frame_w - 3 or by + bh >= frame_h - 3:
                continue  # clipped by the frame: laptop/screen edge, not the arm
            if bh < 1.2 * max(bw, 1):
                continue  # the arm is a tall structure
            hull = cv2.convexHull(cand)
            hull_area = float(cv2.contourArea(hull))
            solidity = area / hull_area if hull_area > 0 else 1.0
            if solidity > 0.92:
                continue  # filled rectangle, not an articulated arm
            score = bh * area
            if score > best_score:
                best_score = score
                selected = cand
        c = selected if selected is not None else max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 50:
            return None
        # Work only on the selected blob so background blue does not drag the skeleton.
        sel_mask = np.zeros_like(mask)
        cv2.drawContours(sel_mask, [c], -1, 255, -1)
        ys, xs = np.where(sel_mask > 0)
        if xs.size == 0:
            return None
        y_min = int(ys.min())
        y_max = int(ys.max())
        band_h = (y_max - y_min + 1) / 8.0
        points = []
        for i in range(8):
            lo = int(y_min + i * band_h)
            hi = int(y_min + (i + 1) * band_h)
            if hi <= lo:
                hi = lo + 1
            band = sel_mask[lo:hi, :]
            M = cv2.moments(band)
            if M["m00"] > 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"] + lo
                points.append((float(cx), float(cy)))
        if len(points) < 2:
            return None
        # order from lowest (base) to highest
        points.sort(key=lambda p: -p[1])
        arr = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
        try:
            arc = float(cv2.arcLength(arr, False))
        except Exception:
            arc = 0.0
        eps = 0.02 * arc if arc > 0 else 1.0
        try:
            approx = cv2.approxPolyDP(arr, eps, False)
            poly = [(float(p[0][0]), float(p[0][1])) for p in approx]
        except Exception:
            poly = []
        if len(poly) < 2:
            poly = points[:]
        pts5 = self._resample(poly, 5)
        # black box refinement
        try:
            black_mask = (hsv[:, :, 2] < 60).astype(np.uint8) * 255
            contours_b, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            boxes = []
            for bc in contours_b:
                if cv2.contourArea(bc) > 200:
                    M = cv2.moments(bc)
                    if M["m00"] > 0:
                        boxes.append((M["m10"] / M["m00"], M["m01"] / M["m00"]))
            for i, pt in enumerate(pts5):
                for bx, by in boxes:
                    if (bx - pt[0]) ** 2 + (by - pt[1]) ** 2 <= 40 * 40:
                        pts5[i] = (float(bx), float(by))
        except Exception:
            pass
        base, shoulder, elbow, wrist, tip = pts5
        wr = ((wrist[0] + tip[0]) / 2.0, (wrist[1] + tip[1]) / 2.0)
        kp = {
            "base": (float(base[0]), float(base[1]), 0.8),
            "shoulder": (float(shoulder[0]), float(shoulder[1]), 0.8),
            "elbow": (float(elbow[0]), float(elbow[1]), 0.8),
            "wrist_pitch": (float(wrist[0]), float(wrist[1]), 0.7),
            "wrist_roll": (float(wr[0]), float(wr[1]), 0.6),
            "gripper_tip": (float(tip[0]), float(tip[1]), 0.7),
        }
        thetas = self._derive_angles(kp)
        if thetas is None:
            return None
        thetas = self._clamp(thetas)
        return thetas, kp, blue_area, 5

    def _skeleton(self, kp):
        if not kp:
            return []
        edges = [
            ("base", "shoulder"),
            ("shoulder", "elbow"),
            ("elbow", "wrist_pitch"),
            ("wrist_pitch", "wrist_roll"),
            ("wrist_roll", "gripper_tip"),
        ]
        out = []
        for a, b in edges:
            if a in kp and b in kp:
                out.append((a, b))
        return out

    def _none_result(self, t0, blue_area=0):
        latency = 0.0
        try:
            latency = (time.perf_counter() - t0) * 1000.0
        except Exception:
            latency = 0.0
        return {
            "mode_used": "none",
            "thetas_deg": None,
            "motors_deg": None,
            "keypoints_px": None,
            "skeleton": [],
            "cartesian_mm": None,
            "T_0_6": None,
            "confidence": 0.0,
            "latency_ms": float(latency),
            "debug": {"blue_area_px": int(blue_area), "joints_detected": 0},
        }

    def analyze(self, frame_bgr):
        t0 = time.perf_counter()
        try:
            if cv2 is None or np is None:
                return self._none_result(t0, 0)
            frame = np.asarray(frame_bgr)
            if frame is None or frame.size == 0 or frame.ndim != 3:
                return self._none_result(t0, 0)

            # blue area (debug) always
            try:
                mask, _ = self._blue_mask(frame)
                blue_area = int(cv2.countNonZero(mask))
            except Exception:
                blue_area = 0

            # aruco detection
            markers = {}
            if self.mode in ("aruco", "hybrid"):
                try:
                    markers = self._detect_aruco(frame)
                except Exception:
                    markers = {}

            aruco_kp = None
            if self.mode == "aruco":
                if len(markers) >= 1:
                    aruco_kp = self._markers_to_kp(markers)
            elif self.mode == "hybrid":
                if len(markers) >= 4:
                    aruco_kp = self._markers_to_kp(markers)

            # neural
            neural_kp = None
            if self.neural is not None:
                if self.mode == "neural" or (self.mode == "hybrid" and aruco_kp is None):
                    try:
                        if self.neural.available:
                            neural_kp = self.neural.predict(frame)
                    except Exception:
                        neural_kp = None

            # geometric
            geom = None
            if self.mode == "geometric" or (self.mode == "hybrid" and aruco_kp is None and neural_kp is None):
                try:
                    geom = self._geometric(frame)
                except Exception:
                    geom = None

            chosen_mode = None
            thetas = None
            kp = None
            joints = 0

            if aruco_kp is not None:
                ang = self._derive_angles(aruco_kp)
                if ang is not None:
                    chosen_mode = "aruco"
                    thetas = self._clamp(ang)
                    kp = aruco_kp
                    joints = len(aruco_kp)

            if chosen_mode is None and neural_kp is not None:
                ang = self._derive_angles(neural_kp)
                if ang is not None:
                    chosen_mode = "neural"
                    thetas = self._clamp(ang)
                    kp = neural_kp
                    joints = len(neural_kp)

            if chosen_mode is None and geom is not None:
                g_thetas, g_kp, g_blue, g_joints = geom
                chosen_mode = "geometric"
                thetas = g_thetas
                kp = g_kp
                blue_area = g_blue
                joints = g_joints

            if chosen_mode is None:
                return self._none_result(t0, blue_area)

            # motors
            motors = self._motors(thetas)

            # cartesian / T
            cartesian = None
            T = None
            if self.kinematics is not None and hasattr(self.kinematics, "forward_kinematics"):
                try:
                    fk = self.kinematics.forward_kinematics(thetas)
                    cartesian = {"x": float(fk["x_mm"]), "y": float(fk["y_mm"]), "z": float(fk["z_mm"])}
                    T = np.asarray(fk["T_0_6"]).tolist()
                except Exception:
                    cartesian = None
                    T = None

            # confidence
            if chosen_mode == "aruco":
                conf = 0.9
            elif chosen_mode == "neural":
                conf = 0.7
            elif chosen_mode == "geometric":
                conf = 0.5
            else:
                conf = 0.0

            latency = (time.perf_counter() - t0) * 1000.0
            skeleton = self._skeleton(kp)
            return {
                "mode_used": chosen_mode,
                "thetas_deg": [float(v) for v in thetas],
                "motors_deg": motors,
                "keypoints_px": kp,
                "skeleton": skeleton,
                "cartesian_mm": cartesian,
                "T_0_6": T,
                "confidence": float(conf),
                "latency_ms": float(latency),
                "debug": {"blue_area_px": int(blue_area), "joints_detected": int(joints)},
            }
        except Exception:
            return self._none_result(t0, 0)
