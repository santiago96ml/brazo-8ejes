import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from armctl import protocol as P


class TestProtocol(unittest.TestCase):
    def test_move(self):
        self.assertEqual(P.cmd_move(1, 40), "M1:40")
        self.assertEqual(P.cmd_move(1, 90, 500), "M1:90:500")

    def test_spin(self):
        self.assertEqual(P.cmd_spin(7, 30, 500), "T7:30:500")
        with self.assertRaises(ValueError):
            P.cmd_spin(7, 200, 500)

    def test_multi(self):
        self.assertEqual(P.cmd_multi([(1, 10), (7, 20)]), "X1:10,7:20")

    def test_parse(self):
        r = P.parse_line('{"ack":"M_OK","n":1,"deg":90}')
        self.assertEqual(r["ack"], "M_OK")

    def test_joint_state_n(self):
        r = P.parse_line('{"status":"STATE","ch":[{"c":0,"n":1,"m":"P","d":90,"p":307,"tg":90,"mv":0},{"c":6}]}')
        states = P.parse_state(r)
        self.assertEqual(states[0].motor, 1)
        self.assertEqual(states[1].motor, 7)

    def test_is_ack(self):
        r = {"ack": "M_OK"}
        self.assertTrue(P.is_ack(r, "M_OK"))

    def test_all_builders(self):
        self.assertEqual(P.cmd_info(), "I")
        self.assertEqual(P.cmd_stop(), "S")
        self.assertEqual(P.cmd_home(), "H")
        self.assertEqual(P.cmd_state(), "Q")

    def test_unknown_name(self):
        import armctl.config as C
        cfg = C.load_config()
        with self.assertRaises(ValueError):
            C.joint_index(cfg, "el")


if __name__ == "__main__":
    unittest.main()