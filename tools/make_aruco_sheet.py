#!/usr/bin/env python3
"""Generate a printable ArUco sheet for the 6 arm joints (DICT_4X4_50, IDs 0..5).

Print at 100% scale on A4, cut each marker with a 10 mm white border, and glue:
  id 0 -> base plate       id 1 -> shoulder
  id 2 -> elbow            id 3 -> wrist pitch
  id 4 -> wrist roll       id 5 -> gripper tip

Marker size is written under each square; keep the physical size consistent with
`MARKER_SIZE_MM` in the analyzer calibration if you later use tvec/rvec.
"""
from __future__ import annotations

import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from config import env_performance  # noqa: E402

env_performance.apply_performance_env()

import cv2  # noqa: E402
import numpy as np  # noqa: E402

LABELS = ["0 BASE", "1 SHOULDER", "2 ELBOW", "3 WRIST PITCH", "4 WRIST ROLL", "5 GRIPPER"]


def make_sheet(marker_px: int = 320, margin_px: int = 60) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    cell = marker_px + 2 * margin_px
    cols = 3
    rows = 2
    sheet = np.full((rows * cell + 80, cols * cell, 3), 255, dtype=np.uint8)
    for idx in range(6):
        marker = cv2.aruco.generateImageMarker(dictionary, idx, marker_px)
        marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        marker_bgr = cv2.copyMakeBorder(
            marker_bgr, margin_px, margin_px, margin_px, margin_px,
            cv2.BORDER_CONSTANT, value=(255, 255, 255),
        )
        r = idx // cols
        c = idx % cols
        y = r * cell
        x = c * cell
        sheet[y:y + cell, x:x + cell] = marker_bgr
        cv2.putText(sheet, LABELS[idx], (x + 20, y + cell - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
    header = "ARM 6-DOF ARUCO SHEET - DICT_4X4_50 - PRINT AT 100%"
    banner = np.full((80, sheet.shape[1], 3), 255, dtype=np.uint8)
    cv2.putText(banner, header, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
    return np.vstack([banner, sheet])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="tools/aruco_sheet.png")
    parser.add_argument("--marker-px", type=int, default=320)
    args = parser.parse_args()
    sheet = make_sheet(marker_px=args.marker_px)
    out = os.path.join(REPO_ROOT, args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    cv2.imwrite(out, sheet)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
