import json
import os
import queue
import sys
import tempfile
import unittest

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
SRC_DIR = os.path.join(REPO_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from src.visual_pulse_calibrator import VisualPulseCalibrator


class _FakeTransport:
    def __init__(self):
        self.sent = []

    def send_line(self, line):
        self.sent.append(line)

    def close(self):
        pass


class _FakeCamera:
    def __init__(self, moving=True):
        self.moving = moving
        self.frame_count = 0

    def get_latest_frame(self):
        h, w = 240, 320
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        # blue background
        frame[:] = (100, 50, 20)
        # moving blue dot
        x = 50 + (self.frame_count % 40)
        y = 120
        cv2 = None
        try:
            import cv2
            cv2.circle(frame, (x, y), 5, (255, 200, 100), -1)
        except Exception:
            # simple fill
            for dy in range(-3, 4):
                for dx in range(-3, 4):
                    if 0 <= y + dy < h and 0 <= x + dx < w:
                        frame[y + dy, x + dx] = (255, 200, 100)
        self.frame_count += 1
        return frame, self.frame_count * 1000000, 30.0

    def read(self):
        f, t, fps = self.get_latest_frame()
        return True, f

    def start(self):
        return self

    def stop(self):
        pass


class _FakeAnalyzer:
    def __init__(self):
        self.t = 0

    def analyze(self, frame):
        self.t += 1
        # keypoints moving deterministically
        kps = {}
        base_x = 40 + self.t % 10
        kps["base"] = (base_x, 200, 0.9)
        kps["shoulder"] = (base_x + 30, 150, 0.9)
        kps["elbow"] = (base_x + 60, 100, 0.9)
        kps["wrist_pitch"] = (base_x + 80, 80, 0.8)
        kps["wrist_roll"] = (base_x + 90, 75, 0.8)
        kps["gripper_tip"] = (base_x + 100, 70, 0.8)
        thetas = [0.0, 90.0 + self.t * 0.1, 0.0, 0.0, 0.0, 50.0]
        return {"keypoints_px": kps, "thetas_deg": thetas, "mode_used": "aruco", "confidence": 0.9, "cartesian_mm": {"x": 0, "y": 0, "z": 570}}


class TestVisualPulseCalibrator(unittest.TestCase):
    def test_zero_sync_order(self):
        cam = _FakeCamera()
        ana = _FakeAnalyzer()
        ctrl = type("C", (), {})()
        ctrl.transport = _FakeTransport()
        ctrl.state = type("S", (), {"update_from_thetas": lambda self, t: None})()
        cal = VisualPulseCalibrator(camera=cam, analyzer=ana, controller=ctrl, dry_run=False)
        res = cal.zero_sync()
        sent = ctrl.transport.sent
        self.assertTrue(len(sent) >= 3)
        self.assertEqual(sent[0], "S")
        # find Z and R
        z_idx = -1
        r_idx = -1
        for i, s in enumerate(sent):
            if str(s).startswith("Z:"):
                z_idx = i
            if str(s).strip() == "R":
                r_idx = i
        self.assertTrue(z_idx > 0)
        self.assertTrue(r_idx > z_idx)
        self.assertEqual(res["preloaded"], True)

    def test_measure_stop_point_picks_smallest_motion(self):
        cam = _FakeCamera()
        ana = _FakeAnalyzer()
        ctrl = type("C", (), {})()
        ctrl.transport = _FakeTransport()
        ctrl.state = type("S", (), {"update_from_thetas": lambda self, t: None})()
        cal = VisualPulseCalibrator(camera=cam, analyzer=ana, controller=ctrl, dry_run=False, candidates=(304, 307, 310))
        res = cal.measure_stop_point(1)
        self.assertIn("stop_pulse", res)
        self.assertIn("displacement_px_s", res)
        # should pick one of candidates
        self.assertIn(res["stop_pulse"], (304, 307, 310))

    def test_measure_speed_positive(self):
        cam = _FakeCamera()
        ana = _FakeAnalyzer()
        ctrl = type("C", (), {})()
        ctrl.transport = _FakeTransport()
        ctrl.state = type("S", (), {"update_from_thetas": lambda self, t: None})()
        cal = VisualPulseCalibrator(camera=cam, analyzer=ana, controller=ctrl, dry_run=False, burst_ms=200)
        res = cal.measure_speed(1, delta_pwm=40, burst_ms=200)
        self.assertIn("deg_per_ms_pos", res)
        self.assertIn("deg_per_ms_neg", res)
        self.assertGreater(res["deg_per_ms_pos"], 0)
        self.assertGreater(res["deg_per_ms_neg"], 0)

    def test_calibrate_motor_emits_C_and_E(self):
        cam = _FakeCamera()
        ana = _FakeAnalyzer()
        ctrl = type("C", (), {})()
        ctrl.transport = _FakeTransport()
        ctrl.state = type("S", (), {"update_from_thetas": lambda self, t: None})()
        cal = VisualPulseCalibrator(camera=cam, analyzer=ana, controller=ctrl, dry_run=False)
        res = cal.calibrate_motor(1)
        sent = " ".join(str(s) for s in ctrl.transport.sent)
        self.assertTrue(any(str(s).startswith("C1:") for s in ctrl.transport.sent))
        self.assertTrue(any(str(s).strip() == "E" for s in ctrl.transport.sent))

    def test_dry_run_sends_nothing(self):
        cam = _FakeCamera()
        ana = _FakeAnalyzer()
        ctrl = type("C", (), {})()
        ctrl.transport = _FakeTransport()
        ctrl.state = type("S", (), {"update_from_thetas": lambda self, t: None})()
        cal = VisualPulseCalibrator(camera=cam, analyzer=ana, controller=ctrl, dry_run=True)
        cal.zero_sync()
        self.assertEqual(len(ctrl.transport.sent), 0)


if __name__ == "__main__":
    unittest.main()
