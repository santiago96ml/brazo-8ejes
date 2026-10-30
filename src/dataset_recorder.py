"""Dataset auto-recorder for the 6-DOF arm vision pipeline (S13).

Saves images + YOLO-keypoint labels + JSON sidecars in a sequential,
zero-padded layout under ``<root>/``.
"""
from __future__ import annotations

import json
import os
import re
import time
from typing import Any, Dict, List, Optional, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in __import__("sys").path:
    __import__("sys").path.insert(0, REPO_ROOT)

try:
    import cv2
except Exception:  # pragma: no cover - cv2 is expected
    cv2 = None  # type: ignore

import numpy as np  # noqa: E402

DEFAULT_KEYPOINT_ORDER = (
    "base",
    "shoulder",
    "elbow",
    "wrist",
    "gripper",
    "tcp",
)


def _to_xy(point: Any) -> Optional[Tuple[float, float]]:
    if point is None:
        return None
    if isinstance(point, (list, tuple)):
        if len(point) >= 2:
            try:
                return float(point[0]), float(point[1])
            except (TypeError, ValueError):
                return None
        return None
    if isinstance(point, dict):
        if "x" in point and "y" in point:
            try:
                return float(point["x"]), float(point["y"])
            except (TypeError, ValueError):
                return None
        # fallback: first two values
        vals = list(point.values())
        if len(vals) >= 2:
            try:
                return float(vals[0]), float(vals[1])
            except (TypeError, ValueError):
                return None
    return None



def _to_serializable(v):
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (list, tuple)):
        return [_to_serializable(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _to_serializable(val) for k, val in v.items()}
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (np.floating,)):
        return float(v)
    return v

class DatasetRecorder:
    def __init__(
        self,
        root: str = "dataset",
        min_interval_s: float = 0.0,
        image_format: str = "jpg",
        jpeg_quality: int = 92,
        class_id: int = 0,
        keypoint_order: tuple = DEFAULT_KEYPOINT_ORDER,
    ):
        self.root = os.path.abspath(root) if not os.path.isabs(root) else root
        # keep relative root as given when relative? Tests pass temp paths (abs). Fine.
        self.root = root
        self.min_interval_s = float(min_interval_s)
        self.image_format = str(image_format).lower().lstrip(".")
        self.jpeg_quality = int(jpeg_quality)
        self.class_id = int(class_id)
        self.keypoint_order = tuple(keypoint_order)
        self._last_save_time = 0.0
        self._next_index = self._scan_next_index()

    # ------------------------------------------------------------------
    def _images_dir(self) -> str:
        return os.path.join(self.root, "images")

    def _labels_dir(self) -> str:
        return os.path.join(self.root, "labels")

    def _scan_next_index(self) -> int:
        d = self._images_dir()
        mx = 0
        if os.path.isdir(d):
            for name in os.listdir(d):
                m = re.match(r"^(\d+)\.", name)
                if m:
                    try:
                        mx = max(mx, int(m.group(1)))
                    except ValueError:
                        pass
        # also scan labels for safety
        ld = self._labels_dir()
        if os.path.isdir(ld):
            for name in os.listdir(ld):
                m = re.match(r"^(\d+)\.", name)
                if m:
                    try:
                        mx = max(mx, int(m.group(1)))
                    except ValueError:
                        pass
        return mx + 1

    def _ensure_dirs(self) -> None:
        os.makedirs(self._images_dir(), exist_ok=True)
        os.makedirs(self._labels_dir(), exist_ok=True)
        self._write_manifest_if_missing()

    def _write_manifest_if_missing(self) -> None:
        manifest = os.path.join(self.root, "dataset.yaml")
        if os.path.exists(manifest):
            return
        try:
            os.makedirs(self.root, exist_ok=True)
            with open(manifest, "w", encoding="utf-8") as f:
                f.write("path: %s\n" % self.root)
                f.write("count: %d\n" % self.count())
                f.write("keypoint_order: %s\n" % list(self.keypoint_order))
                f.write("class_names: [arm]\n")
        except Exception:
            pass

    def count(self) -> int:
        d = self._images_dir()
        if not os.path.isdir(d):
            return 0
        n = 0
        for name in os.listdir(d):
            if re.match(r"^\d+\." + re.escape(self.image_format) + r"$", name):
                n += 1
        if n == 0:
            # count any image-like files
            for name in os.listdir(d):
                if re.match(r"^\d+\.", name):
                    n += 1
                    break
        # simpler: count indexed image files
        n = 0
        if os.path.isdir(d):
            for name in os.listdir(d):
                if re.match(r"^\d+\.", name):
                    n += 1
        return n

    def summary(self) -> dict:
        return {
            "root": self.root,
            "count": self.count(),
            "next_index": self._next_index,
            "keypoint_order": list(self.keypoint_order),
        }

    # ------------------------------------------------------------------
    def _extract_points(
        self, analysis: dict, frame_shape: Tuple[int, int]
    ) -> List[Optional[Tuple[float, float]]]:
        h, w = frame_shape[:2]
        pts: List[Optional[Tuple[float, float]]] = [None] * len(self.keypoint_order)
        kp = None
        if isinstance(analysis, dict):
            kp = analysis.get("keypoints_px")
            if kp is None:
                debug = analysis.get("debug")
                if isinstance(debug, dict):
                    kp = debug.get("keypoints_px")
        n = len(self.keypoint_order)

        if isinstance(kp, (list, tuple)):
            for i in range(min(n, len(kp))):
                pts[i] = _to_xy(kp[i])
            return pts

        if isinstance(kp, dict):
            values = list(kp.values())
            for i, name in enumerate(self.keypoint_order):
                cand = None
                if name in kp:
                    cand = kp[name]
                elif str(name) in kp:
                    cand = kp[str(name)]
                elif str(i) in kp:
                    cand = kp[str(i)]
                elif i in kp:
                    cand = kp[i]
                else:
                    # fallback by insertion order
                    if i < len(values):
                        cand = values[i]
                pts[i] = _to_xy(cand)
            return pts

        # maybe keypoints under a different key
        if isinstance(analysis, dict):
            for key in ("keypoints", "points", "joints"):
                kp2 = analysis.get(key)
                if isinstance(kp2, (list, tuple)):
                    for i in range(min(n, len(kp2))):
                        pts[i] = _to_xy(kp2[i])
                    return pts
        return pts

    def save_sample(
        self,
        frame_bgr,
        analysis: dict,
        thetas_deg: list | None = None,
        motors_deg: dict | None = None,
    ) -> dict:
        # throttle
        now = time.time()
        if self.min_interval_s > 0 and (now - self._last_save_time) < self.min_interval_s:
            return {"saved": False, "reason": "throttled"}

        if cv2 is None:
            return {"saved": False, "reason": "cv2 unavailable"}

        try:
            if frame_bgr is None:
                raise ValueError("frame is None")
            frame = np.asarray(frame_bgr)
            if frame.ndim < 2:
                raise ValueError("invalid frame")
            h, w = frame.shape[:2]
            if h <= 0 or w <= 0:
                raise ValueError("empty frame")

            self._ensure_dirs()
            idx = self._next_index
            stem = "%06d" % idx
            ext = self.image_format
            img_name = "%s.%s" % (stem, ext)
            label_name = "%s.txt" % stem
            json_name = "%s.json" % stem
            img_path = os.path.join(self._images_dir(), img_name)
            label_path = os.path.join(self._labels_dir(), label_name)
            json_path = os.path.join(self._labels_dir(), json_name)

            # write image
            params = []
            if ext in ("jpg", "jpeg"):
                params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)]
            ok = cv2.imwrite(img_path, frame, params)
            if not ok:
                raise IOError("cv2.imwrite failed")

            # keypoints
            pts = self._extract_points(analysis if isinstance(analysis, dict) else {}, (h, w))
            confidence = 0.5
            if isinstance(analysis, dict):
                try:
                    confidence = float(analysis.get("confidence", 0.5))
                except (TypeError, ValueError):
                    confidence = 0.5

            visible = []  # list of (x,y) normalized
            norm_pts = []  # (nx,ny,v) or None
            for p in pts:
                if p is None:
                    norm_pts.append(None)
                    continue
                x, y = p
                v = 2 if confidence >= 0.5 else 1
                nx = min(1.0, max(0.0, x / float(w)))
                ny = min(1.0, max(0.0, y / float(h)))
                norm_pts.append((nx, ny, v))
                visible.append((x, y))

            # bounding box from visible keypoints, expanded 10px, clamped
            if visible:
                xs = [p[0] for p in visible]
                ys = [p[1] for p in visible]
                x_min = max(0.0, min(xs) - 10.0)
                x_max = min(float(w), max(xs) + 10.0)
                y_min = max(0.0, min(ys) - 10.0)
                y_max = min(float(h), max(ys) + 10.0)
                cx = (x_min + x_max) / 2.0 / float(w)
                cy = (y_min + y_max) / 2.0 / float(h)
                bw = (x_max - x_min) / float(w)
                bh = (y_max - y_min) / float(h)
            else:
                cx, cy, bw, bh = 0.5, 0.5, 0.0, 0.0
            cx = min(1.0, max(0.0, cx))
            cy = min(1.0, max(0.0, cy))
            bw = min(1.0, max(0.0, bw))
            bh = min(1.0, max(0.0, bh))

            # build label tokens
            tokens = [str(self.class_id), repr(cx), repr(cy), repr(bw), repr(bh)]
            # Use clean float formatting
            tokens = [str(self.class_id)]
            tokens.append(_fmt(cx))
            tokens.append(_fmt(cy))
            tokens.append(_fmt(bw))
            tokens.append(_fmt(bh))
            for np_ in norm_pts:
                if np_ is None:
                    tokens.append("0.0")
                    tokens.append("0.0")
                    tokens.append("0")
                else:
                    nx, ny, v = np_
                    tokens.append(_fmt(nx))
                    tokens.append(_fmt(ny))
                    tokens.append(str(v))

            with open(label_path, "w", encoding="utf-8") as f:
                f.write(" ".join(tokens) + "\n")

            # thetas / motors
            if thetas_deg is None and isinstance(analysis, dict):
                thetas_deg = analysis.get("thetas_deg")
            if motors_deg is None and isinstance(analysis, dict):
                motors_deg = analysis.get("motors_deg")
            T_0_6 = None
            cartesian_mm = None
            source_mode = ""
            timestamp_ns = int(time.time() * 1e9)
            if isinstance(analysis, dict):
                T_0_6 = analysis.get("T_0_6")
                cartesian_mm = analysis.get("cartesian_mm")
                source_mode = str(analysis.get("mode_used", analysis.get("source_mode", "")))
                if "timestamp_ns" in analysis:
                    try:
                        timestamp_ns = int(analysis["timestamp_ns"])
                    except (TypeError, ValueError):
                        pass

            sidecar = {
                "frame": img_name,
                "thetas_deg": _to_serializable(thetas_deg),
                "motors_deg": _to_serializable(motors_deg),
                "T_0_6": _to_serializable(T_0_6),
                "cartesian_mm": _to_serializable(cartesian_mm),
                "timestamp_ns": int(timestamp_ns),
                "source_mode": source_mode,
            }
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(sidecar, f)

            self._last_save_time = now
            self._next_index = idx + 1

            return {
                "saved": True,
                "image": img_path,
                "label": label_path,
                "json": json_path,
                "index": idx,
            }
        except Exception as e:
            return {"saved": False, "reason": str(e)}


def _fmt(v: float) -> str:
    # compact float
    s = repr(float(v))
    return s
