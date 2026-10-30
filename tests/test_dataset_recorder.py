"""Tests for DatasetRecorder and live_vision_hud (S13-S14)."""
import json
import os
import sys
import tempfile
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
SRC_DIR = os.path.join(REPO_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from src.dataset_recorder import DatasetRecorder  # noqa: E402
from src.live_vision_hud import run_hud  # noqa: E402


def _make_frame(w=1280, h=720):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = (40, 80, 120)
    return img


def _make_analysis():
    kps = {
        "base": [100, 200],
        "shoulder": [300, 400],
        "elbow": [500, 300],
        "wrist": [700, 250],
        "gripper": [900, 200],
        "tcp": [1100, 150],
    }
    return {
        "mode_used": "hybrid",
        "thetas_deg": [0.0, 90.0, 0.0, 0.0, 0.0, 50.0],
        "motors_deg": {"1": 50.0, "2": 0.0},
        "keypoints_px": kps,
        "skeleton": list(kps.values()),
        "cartesian_mm": {"x": 10.0, "y": 20.0, "z": 30.0},
        "T_0_6": [[1.0, 0, 0, 10], [0, 1, 0, 20], [0, 0, 1, 30], [0, 0, 0, 1]],
        "confidence": 0.9,
        "latency_ms": 12.3,
        "timestamp_ns": 123456789,
    }


def test_save_sample_creates_files():
    with tempfile.TemporaryDirectory() as tmp:
        rec = DatasetRecorder(root=tmp)
        frame = _make_frame()
        analysis = _make_analysis()
        res = rec.save_sample(frame, analysis)
        assert res["saved"] is True
        assert os.path.exists(res["image"])
        assert os.path.exists(res["label"])
        assert os.path.exists(res["json"])
        with open(res["label"]) as f:
            toks = f.read().split()
        assert len(toks) == 23
        # coordinates (cx,cy,w,h and each x,y) must be in [0,1]; v flags may be 0/1/2
        coord_indices = [1, 2, 3, 4] + [5 + 3 * i for i in range(6)] + [6 + 3 * i for i in range(6)]
        for i in coord_indices:
            v = float(toks[i])
            assert 0.0 - 1e-9 <= v <= 1.0 + 1e-9
        with open(res["json"]) as f:
            side = json.load(f)
        assert "thetas_deg" in side
        assert side["thetas_deg"] == [0.0, 90.0, 0.0, 0.0, 0.0, 50.0]


def test_sequence_continues():
    with tempfile.TemporaryDirectory() as tmp:
        rec1 = DatasetRecorder(root=tmp)
        rec1.save_sample(_make_frame(), _make_analysis())
        rec2 = DatasetRecorder(root=tmp)
        res2 = rec2.save_sample(_make_frame(), _make_analysis())
        assert res2["index"] == 2
        assert res2["saved"] is True


def test_throttle():
    with tempfile.TemporaryDirectory() as tmp:
        rec = DatasetRecorder(root=tmp, min_interval_s=10.0)
        r1 = rec.save_sample(_make_frame(), _make_analysis())
        assert r1["saved"] is True
        r2 = rec.save_sample(_make_frame(), _make_analysis())
        assert r2["saved"] is False


def test_bad_frame():
    with tempfile.TemporaryDirectory() as tmp:
        rec = DatasetRecorder(root=tmp)
        r = rec.save_sample(None, _make_analysis())
        assert r["saved"] is False
        assert "reason" in r


class FakeCamera:
    def __init__(self):
        self.n = 0

    def get_latest_frame(self):
        if self.n >= 3:
            return None, 0, 0.0
        self.n += 1
        return _make_frame(320, 240), int(time.time() * 1e9), 30.0

    def stop(self):
        pass


class FakeAnalyzer:
    def analyze(self, frame):
        return _make_analysis()


def test_hud_headless():
    with tempfile.TemporaryDirectory() as tmp:
        rec = DatasetRecorder(root=os.path.join(tmp, "dataset"))
        res = run_hud(show=False, max_frames=3, camera=FakeCamera, analyzer=FakeAnalyzer, recorder=rec)
        assert res["frames"] == 3
        assert "saved" in res
        assert res["stopped_by"] in ("max_frames", "q")


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__, "-v"]))
