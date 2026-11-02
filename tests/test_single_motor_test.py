import unittest

from tools.single_motor_test import run_single_motor_test


class FakeCamera:
    def __init__(self):
        self.running = False
        self.stopped = False
        self.timestamp = 0

    def start(self):
        self.running = True
        return self

    def stop(self):
        self.running = False
        self.stopped = True

    def get_latest_frame(self):
        self.timestamp += 1
        return object(), self.timestamp, 30.0

    def stats(self):
        return {"fps": 30.0}


class FakeEncoder:
    def __init__(self, audit_variance=0.1):
        self.audit_variance = audit_variance
        self.audit_calls = 0
        self.angle = 30.0
        self.oe_high = True

    def audit_pre_power(self, camera, num_frames, max_variance_deg):
        self.audit_calls += 1
        return {
            "passed": self.audit_variance < max_variance_deg,
            "frames_evaluated": num_frames,
            "mode_used": "aruco",
            "visible_marker_ids": [0, 1, 2, 3, 4, 5],
            "resting_thetas_deg": [0.0, 90.0, 0.0, 0.0, 0.0, self.angle],
            "resting_motors_deg": {1: self.angle, 2: 0.0, 3: 90.0, 4: 90.0, 5: 90.0, 6: 90.0, 7: 0.0},
            "variances_deg2": [self.audit_variance] * 6,
            "max_variance_deg2": self.audit_variance,
        }

    def read_positions(self, frame):
        return {
            "thetas_deg": [0.0, 90.0, 0.0, 0.0, 0.0, self.angle],
            "motors_deg": {1: self.angle, 2: 0.0, 3: 90.0, 4: 90.0, 5: 90.0, 6: 90.0, 7: 0.0},
        }

    def set_oe_state(self, oe_high=True, active_command=False):
        self.oe_high = oe_high

class FakeTransport:
    def __init__(self, encoder=None, probe_motion=True):
        self.commands = []
        self.encoder = encoder
        self.probe_motion = probe_motion

    def send_line(self, command):
        self.commands.append(command)
        if command.startswith("T") and command.endswith(":40") and self.encoder and self.probe_motion:
            self.encoder.angle += 0.5
        elif command.startswith("T") and command.endswith(":400") and self.encoder:
            self.encoder.angle += 6.0


class TestSingleMotorTest(unittest.TestCase):
    def run_test(self, **kwargs):
        camera = FakeCamera()
        encoder = kwargs.pop("encoder", FakeEncoder())
        transport = kwargs.pop("transport", FakeTransport(encoder))
        result = run_single_motor_test(
            motor=1,
            frames=3,
            delta_deg=8.0,
            burst_ms=400,
            probe_ms=40,
            camera=camera,
            encoder=encoder,
            transport=transport,
            **kwargs,
        )
        return result, camera, encoder, transport

    def test_dry_run_sends_nothing_and_never_enables_oe(self):
        result, camera, _encoder, transport = self.run_test(dry_run=True)

        self.assertEqual(result["status"], "OK")
        self.assertEqual(result["oe_sequence"], [])
        self.assertEqual(transport.commands, [])
        self.assertTrue(camera.stopped)

    def test_failed_audit_aborts_without_enabling_oe(self):
        encoder = FakeEncoder(audit_variance=9.0)
        result, _camera, _encoder, transport = self.run_test(encoder=encoder)

        self.assertEqual(result["status"], "ABORT_AUDIT")
        self.assertNotIn("R", transport.commands)

    def test_unverified_probe_aborts_and_leaves_stop_as_last_command(self):
        encoder = FakeEncoder(audit_variance=0.5)
        transport = FakeTransport(encoder, probe_motion=False)
        result, _camera, _encoder, transport = self.run_test(encoder=encoder, transport=transport)

        self.assertEqual(result["status"], "ABORT_PROBE")
        self.assertEqual(transport.commands[-1], "S")

    def test_probe_runs_after_enable_not_before(self):
        encoder = FakeEncoder(audit_variance=0.5)
        transport = FakeTransport(encoder, probe_motion=True)
        result, _camera, _encoder, transport = self.run_test(encoder=encoder, transport=transport)

        self.assertEqual(result["status"], "OK")
        self.assertIn("R", transport.commands)
        r_index = transport.commands.index("R")
        probe_indexes = [i for i, c in enumerate(transport.commands) if c.startswith("T1:") and c != "T1:0:1"]
        self.assertTrue(probe_indexes, "expected at least one probe/burst pulse")
        self.assertLess(r_index, probe_indexes[0], "OE must be enabled before any timed pulse")

    def test_happy_path_measures_motion_and_stops(self):
        encoder = FakeEncoder(audit_variance=0.5)
        result, _camera, _encoder, transport = self.run_test(encoder=encoder)

        self.assertEqual(result["status"], "OK")
        self.assertTrue(result["direction_ok"])
        self.assertAlmostEqual(result["measured_delta_deg"], 6.0, delta=0.3)
        self.assertEqual(transport.commands[-1], "S")


if __name__ == "__main__":
    unittest.main()
