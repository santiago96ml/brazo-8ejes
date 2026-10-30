#!/usr/bin/env python3
"""Benchmark FastUSBCamera: real FPS and end-to-end frame latency.

Usage:
  python tools/bench_camera.py --index 0 --frames 100
  python tools/bench_camera.py --index 0 --frames 100 --width 1920 --height 1080

Latency here is (read wall clock - capture timestamp), i.e. how old the frame is
when the consumer gets it. With a 30 FPS sensor the floor is one frame period
(~33 ms); lower values mean the reader thread is keeping up without a backlog.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from config import env_performance  # noqa: E402

env_performance.apply_performance_env()

from src.fast_camera import FastUSBCamera  # noqa: E402


def percentile(values, q):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((q / 100.0) * (len(ordered) - 1)))))
    return ordered[index]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--exposure", type=float, default=None)
    args = parser.parse_args()

    camera = FastUSBCamera(
        index=args.index,
        width=args.width,
        height=args.height,
        fps=args.fps,
        exposure=args.exposure,
    )
    camera.start()
    try:
        for _ in range(args.warmup):
            camera.get_latest_frame()
            time.sleep(0.01)

        latencies_ms = []
        capture_stamps = []
        duplicates = 0
        started = time.perf_counter()
        collected = 0
        last_ts = -1
        deadline = started + 30.0
        while collected < args.frames and time.perf_counter() < deadline:
            frame, ts_ns, fps_real = camera.get_latest_frame()
            if frame is None or ts_ns == last_ts:
                duplicates += 1
                time.sleep(0.001)
                continue
            last_ts = ts_ns
            now_ns = time.perf_counter_ns()
            latencies_ms.append((now_ns - ts_ns) / 1e6)
            capture_stamps.append(ts_ns)
            collected += 1
        elapsed = time.perf_counter() - started

        intervals_ms = [
            (capture_stamps[i] - capture_stamps[i - 1]) / 1e6
            for i in range(1, len(capture_stamps))
        ]
        capture_fps = 0.0
        if intervals_ms:
            median_interval = statistics.median(intervals_ms)
            capture_fps = 1000.0 / median_interval if median_interval > 0 else 0.0

        report = {
            "index": args.index,
            "requested": {"width": args.width, "height": args.height, "fps": args.fps},
            "actual": {
                "width": camera.actual_width,
                "height": camera.actual_height,
                "fps_reported": round(camera.stats().get("fps", 0.0), 2),
                "fourcc": camera.stats().get("fourcc", "").strip("\x00") or "n/a (MSMF)",
                "backend": str(camera._resolved_backend),
            },
            "unique_frames": collected,
            "duplicate_reads": duplicates,
            "elapsed_s": round(elapsed, 3),
            "consumer_read_fps": round((collected + duplicates) / elapsed, 2) if elapsed > 0 else 0.0,
            "capture_fps_median": round(capture_fps, 2),
            "latency_ms": {
                "p50": round(percentile(latencies_ms, 50), 2),
                "p95": round(percentile(latencies_ms, 95), 2),
                "max": round(max(latencies_ms), 2) if latencies_ms else 0.0,
                "min": round(min(latencies_ms), 2) if latencies_ms else 0.0,
            },
            "stats": camera.stats(),
            "verdict": "OK" if collected >= args.frames else "INSUFFICIENT_FRAMES",
        }
    finally:
        camera.stop()

    print(json.dumps(report, indent=2))
    return 0 if report["verdict"] == "OK" else 1


if __name__ == "__main__":
    raise SystemExit(main())
