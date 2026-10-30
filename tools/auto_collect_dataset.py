"""Automated dataset collection for the arm vision pipeline.

Drives the arm through safe poses, captures frames, runs the hybrid analyzer,
optionally inpaints ArUco markers, and saves pairs via DatasetRecorder.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
SRC_DIR = os.path.join(REPO_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

try:
    import cv2
    import numpy as np
except Exception:  # pragma: no cover
    cv2 = None
    np = None

from src.fast_camera import FastUSBCamera  # noqa: E402
from src.arm_vision_analyzer import ArmVisionAnalyzer  # noqa: E402
from src.dataset_recorder import DatasetRecorder  # noqa: E402
from src.agent_controller import AgentController  # noqa: E402


def generate_poses(count: int, seed: int = 42) -> list:
    """Deterministic seeded random walk inside conservative Cartesian bounds."""
    rng = random.Random(seed)
    poses = []
    r = 220.0
    theta = 0.0
    z = 400.0
    pitch = 55.0
    for _ in range(int(count)):
        r += rng.uniform(-25.0, 25.0)
        r = min(320.0, max(120.0, r))
        theta += rng.uniform(-0.25, 0.25)
        z += rng.uniform(-30.0, 30.0)
        z = min(520.0, max(200.0, z))
        pitch += rng.uniform(-6.0, 6.0)
        pitch = min(80.0, max(30.0, pitch))
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        poses.append((float(x), float(y), float(z), float(pitch)))
    return poses


def _has_six_keypoints(analysis: dict) -> bool:
    if not isinstance(analysis, dict):
        return False
    kp = analysis.get("keypoints_px")
    if isinstance(kp, (list, tuple, dict)):
        try:
            return len(kp) >= 6
        except Exception:
            return False
    return False


def inpaint_markers(frame):
    if cv2 is None or np is None or frame is None:
        return frame
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    except Exception:
        return frame
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    except Exception:
        return frame
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
            corners, ids = None, None
    if ids is None or corners is None:
        return frame
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    for c in corners:
        try:
            pts = c.reshape(4, 2).astype("int32")
            cv2.fillConvexPoly(mask, pts, 255)
        except Exception:
            continue
    if cv2.countNonZero(mask) == 0:
        return frame
    try:
        return cv2.inpaint(frame, mask, 3, cv2.INPAINT_TELEA)
    except Exception:
        return frame


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="auto_collect_dataset")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--out", default="dataset")
    parser.add_argument("--inpaint", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--port", default=None)
    parser.add_argument("--settle-ms", type=int, default=900)
    args = parser.parse_args(argv)

    poses = generate_poses(args.count)

    if args.dry_run:
        print(json.dumps({"poses": poses, "count": len(poses)}, indent=2))
        return 0

    camera = FastUSBCamera(index=args.camera)
    camera.start()
    analyzer = ArmVisionAnalyzer(mode="hybrid")
    recorder = DatasetRecorder(root=args.out)
    sim = args.port is None
    controller = AgentController(sim=sim, port=args.port)

    saved = 0
    skipped = 0
    aborted = False
    try:
        for idx, (x, y, z, pitch) in enumerate(poses):
            try:
                controller.move_to_xyz(x, y, z, pitch_deg=pitch)
            except Exception as e:
                print("pose %d move failed: %s" % (idx, e))
                skipped += 1
                continue
            time.sleep(max(0, args.settle_ms) / 1000.0)

            frame, _ts, _fps = camera.get_latest_frame()
            if frame is None:
                time.sleep(0.2)
                frame, _ts, _fps = camera.get_latest_frame()
            if frame is None:
                print("pose %d: no frame" % idx)
                skipped += 1
                continue

            analysis = analyzer.analyze(frame)
            if not _has_six_keypoints(analysis):
                print("pose %d: skipped (markers<4)" % idx)
                skipped += 1
                continue

            work_frame = frame
            if args.inpaint:
                work_frame = inpaint_markers(frame)

            res = recorder.save_sample(
                work_frame,
                analysis,
                thetas_deg=analysis.get("thetas_deg"),
                motors_deg=analysis.get("motors_deg"),
            )
            if res.get("saved"):
                saved += 1
                print("pose %d: saved (%d/%d)" % (idx, saved, args.count))
            else:
                skipped += 1
                print("pose %d: skipped (%s)" % (idx, res.get("reason")))
    except KeyboardInterrupt:
        aborted = True
    finally:
        try:
            controller.emergency_stop()
        except Exception:
            pass
        try:
            camera.stop()
        except Exception:
            pass
        summary = {
            "saved": saved,
            "skipped": skipped,
            "total": len(poses),
            "aborted": aborted,
        }
        print(json.dumps(summary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
