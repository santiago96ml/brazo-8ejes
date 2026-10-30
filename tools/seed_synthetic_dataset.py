#!/usr/bin/env python3
"""Seed a tiny SYNTHETIC dataset to smoke-test the YOLO pose pipeline.

This is NOT training data for the real arm. It draws a simple articulated arm in
random poses with six known keypoints, writes images/labels through
``DatasetRecorder``, and exists so ``tools/train_arm_keypoints.py`` can be proven
end-to-end without the physical robot or ArUco markers.

Usage:
  python tools/seed_synthetic_dataset.py --count 12 --out dataset_smoke
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from config import env_performance  # noqa: E402

env_performance.apply_performance_env()

import cv2  # noqa: E402

from src.dataset_recorder import DatasetRecorder  # noqa: E402

NAMES = ["base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper_tip"]


def draw_pose(rng: np.random.Generator, width: int, height: int):
    frame = np.full((height, width, 3), 235, dtype=np.uint8)
    base = np.array([width * 0.5, height * 0.88], dtype=np.float64)
    angles = [90.0]
    for _ in range(4):
        angles.append(angles[-1] + rng.uniform(-38.0, 38.0))
    lengths = [0.16, 0.22, 0.20, 0.14, 0.12]
    points = [base]
    for angle, length in zip(angles, lengths):
        rad = math.radians(angle)
        nxt = points[-1] + np.array([math.cos(rad), -math.sin(rad)]) * length * height
        points.append(nxt)
    pts = [(float(p[0]), float(p[1])) for p in points]
    for i in range(len(pts) - 1):
        cv2.line(frame, (int(pts[i][0]), int(pts[i][1])),
                 (int(pts[i + 1][0]), int(pts[i + 1][1])), (150, 70, 20), 16)
    for i, p in enumerate(pts[1:5]):
        cv2.rectangle(frame, (int(p[0]) - 12, int(p[1]) - 12),
                      (int(p[0]) + 12, int(p[1]) + 12), (25, 25, 25), -1)
    keypoints = {name: (x, y, 1.0) for name, (x, y) in zip(NAMES, pts[1:])}
    skeleton = [[NAMES[i], NAMES[i + 1]] for i in range(len(NAMES) - 1)]
    analysis = {
        "mode_used": "synthetic",
        "keypoints_px": keypoints,
        "skeleton": skeleton,
        "thetas_deg": [0.0, 90.0, 0.0, 0.0, 0.0, 50.0],
        "motors_deg": {1: 50.0, 2: 0.0, 3: 90.0, 4: 90.0, 5: 90.0, 6: 90.0, 7: 0.0},
        "cartesian_mm": {"x": 0.0, "y": 0.0, "z": 570.0},
        "T_0_6": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 570.0], [0.0, 0.0, 0.0, 1.0]],
        "confidence": 1.0,
        "latency_ms": 0.0,
    }
    return frame, analysis


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=12)
    parser.add_argument("--out", default="dataset_smoke")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    out = os.path.join(REPO_ROOT, args.out)
    recorder = DatasetRecorder(root=out)
    rng = np.random.default_rng(args.seed)
    saved = 0
    for _ in range(args.count):
        frame, analysis = draw_pose(rng, args.width, args.height)
        result = recorder.save_sample(frame, analysis)
        if result.get("saved"):
            saved += 1
    print("saved", saved, "samples to", out)
    return 0 if saved == args.count else 1


if __name__ == "__main__":
    raise SystemExit(main())
