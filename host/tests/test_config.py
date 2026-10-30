import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from armctl import config as C


class TestConfig(unittest.TestCase):
    def test_load(self):
        cfg = C.load_config(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "config", "arm.json"))
        self.assertEqual(len(cfg["joints"]), 8)
        self.assertEqual(cfg["joints"][0]["name"], "gripper")
        self.assertEqual(cfg["joints"][0]["motor"], 1)

    def test_joint_lookup(self):
        cfg = C.load_config()
        self.assertEqual(C.joint_index(cfg, "base"), 6)
        self.assertEqual(C.joint_index(cfg, 7), 6)
        j = C.get_joint(cfg, "gripper")
        self.assertEqual(j["motor"], 1)

    def test_poses(self):
        poses = C.load_poses()
        self.assertIn("home", poses)
        self.assertIn("ready", poses)


if __name__ == "__main__":
    unittest.main()