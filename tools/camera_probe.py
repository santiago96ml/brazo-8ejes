#!/usr/bin/env python3
"""Probe camera indices on Windows (DirectShow + MJPG) and save one preview per index.

The NVIDIA virtual camera and DSHOW write noisy logs to file descriptor 2, so the
probe temporarily silences fd 2 while opening devices. Preview JPEGs are written to
``tools/camera_previews/index_<n>.jpg`` for visual identification of the USB camera.
"""
from __future__ import annotations

import argparse
import contextlib
import json
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from config import env_performance  # noqa: E402  (must be imported before cv2)

env_performance.apply_performance_env()

import cv2  # noqa: E402
import numpy as np  # noqa: E402

env_performance.configure_opencv()


@contextlib.contextmanager
def silence_fd2(enabled: bool = True):
    """Temporarily redirect C-level stderr to NUL (OpenCV/DSHOW/NVIDIA noise)."""
    if not enabled or os.name != "nt":
        yield
        return
    saved = os.dup(2)
    nul = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(nul, 2)
        yield
    finally:
        os.dup2(saved, 2)
        os.close(nul)
        os.close(saved)


def probe_index(index: int, width: int, height: int, fps: int) -> dict:
    result = {
        "index": index,
        "opened": False,
        "width": 0,
        "height": 0,
        "fps": 0.0,
        "fourcc": "",
        "backend": "",
        "mean_brightness": 0.0,
        "frames_read": 0,
        "open_ms": 0.0,
        "preview": None,
    }
    started = time.perf_counter()
    with silence_fd2():
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            result["open_ms"] = (time.perf_counter() - started) * 1000.0
            return result
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        frames = []
        for _ in range(6):
            ok, frame = cap.read()
            if ok and frame is not None:
                frames.append(frame)
        fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc = "".join(chr((fourcc_int >> (8 * i)) & 0xFF) for i in range(4))
        result.update(
            opened=True,
            width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            fps=float(cap.get(cv2.CAP_PROP_FPS)),
            fourcc=fourcc.strip("\x00"),
            backend=cap.getBackendName(),
            frames_read=len(frames),
        )
        cap.release()
        result["open_ms"] = (time.perf_counter() - started) * 1000.0
        if frames:
            frame = frames[-1]
            result["mean_brightness"] = float(np.mean(frame))
            out_dir = os.path.join(REPO_ROOT, "tools", "camera_previews")
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(out_dir, "index_%d.jpg" % index)
            cv2.imwrite(path, frame)
            result["preview"] = os.path.relpath(path, REPO_ROOT).replace("\\", "/")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-index", type=int, default=5)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    devices = [probe_index(i, args.width, args.height, args.fps) for i in range(args.max_index + 1)]
    print(json.dumps({"devices": devices}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
