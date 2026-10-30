import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from armctl.client import ArmController
from armctl.transport import SimTransport
from armctl import config as C
from armctl import protocol as P


class TestClosePolicy(unittest.TestCase):
    def _drain_transport(self, transport):
        items = []
        while True:
            try:
                items.append(transport.rx_queue.get_nowait())
            except Exception:
                break
        return items

    def test_default_no_disable_on_close(self):
        cfg = C.load_config()
        transport = SimTransport(cfg)
        ctrl = ArmController(transport, watchdog_ms=0, send_enable_on_connect=False)
        ctrl.send_line(P.cmd_move(7, 100))
        ctrl.wait_ack("M_OK", 2.0)
        self._drain_transport(transport)
        ctrl.close()
        items = self._drain_transport(transport)
        texts = [str(i) for i in items]
        self.assertFalse(any("ALL_NEUTRAL" in t for t in texts))
        self.assertFalse(any("EMERGENCY_STOP" in t for t in texts))

    def test_disable_enqueues_fs(self):
        cfg = C.load_config()
        transport = SimTransport(cfg)
        ctrl = ArmController(transport, watchdog_ms=0, send_enable_on_connect=False, disable_on_exit=True)
        ctrl.send_line(P.cmd_move(7, 100))
        ctrl.wait_ack("M_OK", 2.0)
        self._drain_transport(transport)
        ctrl.close()
        items = self._drain_transport(transport)
        texts = [str(i) for i in items]
        self.assertTrue(any("ALL_NEUTRAL" in t for t in texts))
        self.assertTrue(any("EMERGENCY_STOP" in t for t in texts))


if __name__ == "__main__":
    unittest.main()