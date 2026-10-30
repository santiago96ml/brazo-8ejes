import os
import sys
import unittest
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from armctl.client import ArmController
from armctl.transport import SimTransport
from armctl import config as C
from armctl import protocol as P


class TestSimRoundtrip(unittest.TestCase):
    def setUp(self):
        cfg = C.load_config()
        self.cfg = cfg
        self.transport = SimTransport(cfg)
        self.ctrl = ArmController(self.transport, watchdog_ms=0, send_enable_on_connect=True)

    def tearDown(self):
        self.ctrl.close()

    def test_status(self):
        self.ctrl.send_line(P.cmd_state())
        r = self.ctrl.wait_ack("STATE", 2.0)
        self.assertIsNotNone(r)
        self.assertEqual(r["status"], "STATE")
        self.assertEqual(len(r["ch"]), 8)

    def test_move(self):
        idx = C.joint_index(self.cfg, "base")
        self.ctrl.send_line(P.cmd_move(self.cfg["joints"][idx].get("motor", idx+1), 45))
        r = self.ctrl.wait_ack("M_OK", 2.0)
        self.assertIsNotNone(r)
        self.assertEqual(r["motor"], 7)

    def test_estop(self):
        self.ctrl.send_line(P.cmd_stop())
        r = self.ctrl.wait_ack("EMERGENCY_STOP", 2.0)
        self.assertIsNotNone(r)

    def test_home(self):
        self.ctrl.send_line(P.cmd_home())
        r = self.ctrl.wait_ack("HOME", 2.0)
        self.assertIsNotNone(r)


if __name__ == "__main__":
    unittest.main()