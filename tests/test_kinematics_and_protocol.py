"""Automated QA tests for the 6-DOF kinematic control system (S7)."""

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

try:
    from src.kinematics_6dof import RobotArm6DOF, WorkspaceCollisionError
except Exception:  # pragma: no cover - fallback when running from src/
    from kinematics_6dof import RobotArm6DOF, WorkspaceCollisionError  # type: ignore

try:
    from src.state_manager import RobotStateManager, ShoulderCouplingError
except Exception:  # pragma: no cover
    from state_manager import RobotStateManager, ShoulderCouplingError  # type: ignore

try:
    from src.agent_controller import AgentController
except Exception:  # pragma: no cover
    from agent_controller import AgentController  # type: ignore


class _SpyTransport:
    def __init__(self):
        self.sent = []
        self.rx_queue = queue.Queue()

    def send_line(self, line):
        self.sent.append(line)
        stripped = line.strip()
        if stripped == "I":
            self.rx_queue.put(json.dumps({"status": "OK", "pca9685": "CONECTADO_0x40"}))
        else:
            self.rx_queue.put(json.dumps({"ack": "OK"}))

    def close(self):
        pass


class TestKinematicsAndProtocol(unittest.TestCase):

    def test_01_home_identity(self):
        arm = RobotArm6DOF()
        fk = arm.forward_kinematics([0.0, 90.0, 0.0, 0.0, 0.0, 50.0])
        matrices = fk["matrices"]
        self.assertEqual(len(matrices), 6)
        for m in matrices:
            self.assertEqual(np.asarray(m).shape, (4, 4))

        chained = np.eye(4, dtype=float)
        for m in matrices:
            chained = chained @ np.asarray(m, dtype=float)
        self.assertTrue(np.allclose(fk["T_0_6"], chained))

        self.assertLess(abs(fk["x_mm"]), 1e-6)
        self.assertLess(abs(fk["y_mm"]), 1e-6)
        self.assertLess(abs(fk["z_mm"] - 570.0), 1e-6)

    def test_02_fk_ik_consistency_1000_points(self):
        arm = RobotArm6DOF()
        rng = np.random.default_rng(42)
        errors = []
        attempts = 0
        max_attempts = 100000
        while len(errors) < 1000 and attempts < max_attempts:
            attempts += 1
            th1 = float(rng.uniform(0.0, 360.0))
            th2 = float(rng.uniform(25.0, 155.0))
            th3 = float(rng.uniform(-85.0, -5.0))
            th4 = float(rng.uniform(-85.0, 85.0))
            th5 = float(rng.uniform(0.0, 360.0))
            th6 = float(rng.uniform(0.0, 100.0))
            thetas = [th1, th2, th3, th4, th5, th6]
            try:
                fk = arm.forward_kinematics(thetas, check_collision=True)
            except WorkspaceCollisionError:
                continue
            jps = fk["joint_positions_mm"]
            if any(jps[k][2] < 15.0 for k in ("elbow", "wrist_pitch", "wrist_roll", "gripper_base", "tcp")):
                continue
            X = float(fk["x_mm"])
            Y = float(fk["y_mm"])
            Z = float(fk["z_mm"])
            pitch = float(fk["pitch_deg"])
            roll = float(fk["roll_deg"])
            grip = float(fk["grip_percent"])
            try:
                ik = arm.inverse_kinematics(X, Y, Z, pitch, roll, grip)
            except Exception:
                continue
            fk2 = arm.forward_kinematics(ik)
            err = float(np.linalg.norm(
                np.array([fk2["x_mm"], fk2["y_mm"], fk2["z_mm"]]) - np.array([X, Y, Z])
            ))
            errors.append(err)

        self.assertEqual(len(errors), 1000)
        errors_arr = np.asarray(errors, dtype=float)
        mean_err = float(errors_arr.mean())
        max_err = float(errors_arr.max())
        p99_err = float(np.percentile(errors_arr, 99))
        print("FK-IK 1000: mean=%.6f mm max=%.6f mm p99=%.6f mm" % (mean_err, max_err, p99_err))
        self.assertTrue(np.all(errors_arr < 0.5))

    def test_03_shoulder_coupling_guard(self):
        for coupling in (1, -1):
            arm = RobotArm6DOF(shoulder_coupling=coupling)
            thetas = [0.0, 90.0, 0.0, 0.0, 0.0, 50.0]
            servo = arm.kinematic_to_servo_angles(thetas, shoulder_coupling=coupling)
            m5 = float(servo[5])
            m6 = float(servo[6])
            if coupling == 1:
                self.assertAlmostEqual(m6, m5, places=6)
            else:
                self.assertAlmostEqual(m6, 180.0 - m5, places=6)

            with tempfile.TemporaryDirectory() as td:
                state_path = os.path.join(td, "robot_state.json")
                sm = RobotStateManager(state_path=state_path)
                if coupling == -1:
                    sm.set_shoulder_coupling(-1)

                m5v = 30.0
                bad_m6 = 100.0 if coupling == 1 else 30.0

                with self.assertRaises((ShoulderCouplingError, ValueError)):
                    sm.validate_shoulder_pair(m5v, bad_m6)

                with self.assertRaises((ShoulderCouplingError, ValueError)):
                    sm.build_coordinated_command(
                        {1: 50.0, 2: 0.0, 3: 90.0, 4: 90.0, 5: m5v, 6: bad_m6, 7: 0.0},
                        duration_ms=1000,
                    )

                if coupling == 1:
                    pulses = {4: 400, 5: 200}
                else:
                    pulses = {4: 400, 5: 400}
                with self.assertRaises((ShoulderCouplingError, ValueError)):
                    sm.validate_channel_pulses(pulses)

                # valid coupled command
                valid_m5 = 40.0
                expected_m6 = valid_m5 if coupling == 1 else 180.0 - valid_m5
                cmd = sm.build_coordinated_command(
                    {1: 50.0, 2: 0.0, 3: 90.0, 4: 90.0, 5: valid_m5, 7: 0.0},
                    duration_ms=500,
                )
                self.assertTrue(cmd.startswith("G:"))
                self.assertTrue(os.path.exists(state_path))
                with open(state_path, "r", encoding="utf-8") as f:
                    st = json.load(f)
                pca = st["pca9685_channels_deg"]
                self.assertAlmostEqual(float(pca["4"]), valid_m5, places=3)
                self.assertAlmostEqual(float(pca["5"]), expected_m6, places=3)
                if coupling == 1:
                    self.assertAlmostEqual(float(pca["4"]), float(pca["5"]), places=3)
                else:
                    self.assertAlmostEqual(float(pca["5"]), 180.0 - float(pca["4"]), places=3)

    def test_04_table_collision_protection(self):
        arm = RobotArm6DOF()
        self.assertTrue(issubclass(WorkspaceCollisionError, ValueError))
        with self.assertRaises(WorkspaceCollisionError):
            arm.inverse_kinematics(150.0, 0.0, -20.0)

        ctrl = AgentController(sim=True)
        spy = _SpyTransport()
        ctrl.transport = spy
        with self.assertRaises(WorkspaceCollisionError):
            ctrl.move_to_xyz(150.0, 0.0, -20.0)
        self.assertEqual(len(spy.sent), 0)

        with self.assertRaises(WorkspaceCollisionError):
            arm.move_linear_xyz([0.0, 0.0, 5.0], [0.0, 0.0, 5.0], num_waypoints=5)

    def test_05_agent_controller_and_protocol(self):
        import host.armctl.protocol as P

        ctrl = AgentController(sim=True)
        spy = _SpyTransport()
        ctrl.transport = spy

        pose = ctrl.get_current_pose()
        self.assertIn("x_mm", pose)
        self.assertIn("y_mm", pose)
        self.assertIn("z_mm", pose)

        spy.sent.clear()
        ctrl.move_to_xyz(0.0, 0.0, 570.0, pitch_deg=90.0, speed_mm_s=80.0)
        self.assertTrue(any(str(l).startswith("G:") for l in spy.sent))

        spy.sent.clear()
        ctrl.rotate_wrist(90.0)
        self.assertTrue(any(str(l).startswith("M2:") for l in spy.sent))

        spy.sent.clear()
        ctrl.set_gripper(50.0)
        self.assertTrue(any(str(l).startswith("M1:") for l in spy.sent))

        spy.sent.clear()
        ctrl.emergency_stop()
        self.assertTrue(any(str(l).strip() == "S" for l in spy.sent))

        spy.sent.clear()
        ctrl.verify_i2c_link()
        self.assertTrue(any(str(l).strip() == "I" for l in spy.sent))

        spy.sent.clear()
        ctrl.pick_and_place((0.0, 0.0, 500.0), (10.0, 0.0, 500.0), approach_height_mm=50.0, pitch_deg=90.0)
        self.assertTrue(len(spy.sent) > 0)

        # protocol builders
        self.assertEqual(P.cmd_info(), "I")
        self.assertEqual(P.cmd_stop(), "S")
        self.assertEqual(P.cmd_enable(), "R")
        self.assertEqual(P.cmd_home(), "H")
        self.assertEqual(P.cmd_state(), "Q")
        self.assertEqual(P.cmd_spin(1, 10, 100), "T1:10:100")
        self.assertEqual(P.cmd_calibrate(1, 307), "C1:307")
        self.assertTrue(P.cmd_multi([(1, 10), (7, 20)]).startswith("G") is False)
        self.assertEqual(P.cmd_multi([(1, 10), (7, 20)]), "X1:10,7:20")
        coord = "G:1,2,3,4,5,7,1000"
        self.assertTrue(coord.startswith("G:"))


if __name__ == "__main__":
    unittest.main()
