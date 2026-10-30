import os
import sys
import time
import queue
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from armctl.client import ArmController


class FakeTransport:
    def __init__(self):
        self.rx_queue = queue.Queue()
        self.sent = []
        self._drop_first = True

    def send_line(self, line):
        self.sent.append(line)
        if self._drop_first:
            self._drop_first = False
            return  # drop first command
        # reply on retry
        self.rx_queue.put('{"ack":"M_OK","motor":7}')

    def close(self):
        pass


class TestRetry(unittest.TestCase):
    def test_retry_first_command(self):
        t = FakeTransport()
        ctrl = ArmController(t, watchdog_ms=0, send_enable_on_connect=False)
        ctrl.send_line("M7:100")
        r = ctrl.wait_ack("M_OK", 0.5)
        self.assertIsNotNone(r)
        self.assertEqual(len(t.sent), 2)


if __name__ == "__main__":
    unittest.main()