import time
import unittest

import numpy as np

from src.fast_camera import FastUSBCamera


class FakeCapture:
    def __init__(self, frames=None):
        self.frames = frames if frames is not None else []
        self.index = 0
        self.released = False
        self.props = {}

    def isOpened(self):
        return True

    def set(self, prop, value):
        self.props[prop] = value
        return True

    def get(self, prop):
        if prop == 3:  # CAP_PROP_FRAME_WIDTH
            return 1280
        if prop == 4:  # CAP_PROP_FRAME_HEIGHT
            return 720
        if prop == 6:  # CAP_PROP_FOURCC
            return ord("M") | (ord("J") << 8) | (ord("P") << 16) | (ord("G") << 24)
        return self.props.get(prop, 0)

    def read(self):
        if self.index < len(self.frames):
            frame = self.frames[self.index]
            self.index += 1
            return True, frame
        # Return last frame or empty
        if self.frames:
            return True, self.frames[-1]
        return False, None

    def release(self):
        self.released = True


class TestFastUSBCamera(unittest.TestCase):
    def make_frame(self, val=0):
        return np.full((720, 1280, 3), val, dtype=np.uint8)

    def test_start_stop_lifecycle(self):
        cap = FakeCapture([self.make_frame(1)])
        cam = FastUSBCamera(capture=cap)
        self.assertFalse(cam.is_running)
        cam.start()
        self.assertTrue(cam.is_running)
        time.sleep(0.05)
        cam.stop()
        self.assertFalse(cam.is_running)

    def test_get_latest_frame_returns_frame_and_monotonic_timestamps(self):
        frames = [self.make_frame(i) for i in range(5)]
        cap = FakeCapture(frames)
        cam = FastUSBCamera(capture=cap)
        cam.start()
        time.sleep(0.1)
        frame, ts1, fps = cam.get_latest_frame()
        self.assertIsNotNone(frame)
        time.sleep(0.01)
        frame2, ts2, fps2 = cam.get_latest_frame()
        self.assertIsNotNone(frame2)
        self.assertGreaterEqual(ts2, ts1)
        self.assertIsInstance(fps, float)
        cam.stop()

    def test_slow_consumer_gets_newest_frame_no_backlog(self):
        frames = [self.make_frame(i) for i in range(60)]
        cap = FakeCapture(frames)
        cam = FastUSBCamera(capture=cap)
        cam.start()
        time.sleep(0.2)
        frame, ts, fps = cam.get_latest_frame()
        self.assertIsNotNone(frame)
        # Should get the newest frame (last value)
        self.assertTrue(np.all(frame == 59))
        cam.stop()

    def test_fps_computation_positive_float(self):
        frames = [self.make_frame(i) for i in range(30)]
        cap = FakeCapture(frames)
        cam = FastUSBCamera(capture=cap)
        cam.start()
        time.sleep(0.15)
        _, _, fps = cam.get_latest_frame()
        self.assertIsInstance(fps, float)
        self.assertGreater(fps, 0)
        cam.stop()

    def test_stats_keys_exist(self):
        frames = [self.make_frame(1)]
        cap = FakeCapture(frames)
        cam = FastUSBCamera(capture=cap)
        cam.start()
        time.sleep(0.05)
        stats = cam.stats()
        for key in ["frames", "dropped", "fps", "width", "height", "fourcc", "exposure"]:
            self.assertIn(key, stats)
        cam.stop()

    def test_stop_idempotent(self):
        frames = [self.make_frame(1)]
        cap = FakeCapture(frames)
        cam = FastUSBCamera(capture=cap)
        cam.start()
        time.sleep(0.05)
        cam.stop()
        cam.stop()
        self.assertFalse(cam.is_running)


if __name__ == "__main__":
    unittest.main()
