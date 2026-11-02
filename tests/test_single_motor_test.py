import unittest
import math
from unittest.mock import call, patch

from src.virtual_optical_encoder import VirtualOpticalEncoder
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
        self.gripper_width_px = 50.0
        self.oe_high = True

    def audit_pre_power(self, camera, num_frames, max_variance_deg):
        self.audit_calls += 1
        limit = 1.5 if max_variance_deg is None else max_variance_deg
        return {
            "passed": self.audit_variance < limit,
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
            "mode_used": "aruco",
            "gripper_width_px": self.gripper_width_px,
            "gripper_open_percent": self.gripper_width_px / 120.0 * 100.0,
            "thetas_deg": [0.0, 90.0, 0.0, 0.0, 0.0, self.angle],
            "motors_deg": {1: self.angle, 2: 0.0, 3: 90.0, 4: 90.0, 5: 90.0, 6: 90.0, 7: 0.0},
        }

    def set_oe_state(self, oe_high=True, active_command=False):
        self.oe_high = oe_high


class SequenceAuditEncoder(FakeEncoder):
    def __init__(self, variances):
        super().__init__()
        self.variances = list(variances)

    def audit_pre_power(self, camera, num_frames, max_variance_deg):
        variance = self.variances[self.audit_calls]
        self.audit_calls += 1
        limit = 1.5 if max_variance_deg is None else max_variance_deg
        return {
            "passed": variance < limit,
            "frames_evaluated": num_frames,
            "mode_used": "aruco",
            "visible_marker_ids": [0, 1, 2, 3, 4, 5],
            "variances_deg2": [variance] * 6,
            "max_variance_deg2": variance,
        }


class ModeVarianceEncoder(FakeEncoder):
    def __init__(self, mode, variance):
        super().__init__(audit_variance=variance)
        self.mode = mode

    def audit_pre_power(self, camera, num_frames, max_variance_deg):
        self.audit_calls += 1
        mode_limit = (
            5.0 if self.mode == "geometric_fallback" else 1.5
        ) if max_variance_deg is None else max_variance_deg
        variance = self.audit_variance
        return {
            "passed": variance < mode_limit,
            "frames_evaluated": num_frames,
            "mode_used": self.mode,
            "visible_marker_ids": [0, 1, 2, 3, 4, 5],
            "variances_deg2": [variance] * 6,
            "max_variance_deg2": variance,
        }


class GeometricFallbackEncoder(FakeEncoder):
    def audit_pre_power(self, camera, num_frames, max_variance_deg):
        result = super().audit_pre_power(camera, num_frames, max_variance_deg)
        result["mode_used"] = "geometric_fallback"
        return result

    def read_positions(self, frame):
        result = super().read_positions(frame)
        result["mode_used"] = "geometric_fallback"
        return result

class FakeTransport:
    def __init__(self, encoder=None, probe_motion=True, gripper_motion=True):
        self.commands = []
        self.encoder = encoder
        self.probe_motion = probe_motion
        self.gripper_motion = gripper_motion

    def send_line(self, command):
        self.commands.append(command)
        if command.startswith("T") and command.endswith(":40") and self.encoder and self.probe_motion:
            self.encoder.angle += 0.5
        elif command.startswith("T") and command.endswith(":400") and self.encoder:
            self.encoder.angle += 6.0
            if self.gripper_motion:
                self.encoder.gripper_width_px += 40.0


class TestSingleMotorTest(unittest.TestCase):
    def run_test(self, **kwargs):
        camera = FakeCamera()
        motor = kwargs.pop("motor", 1)
        encoder = kwargs.pop("encoder", FakeEncoder())
        transport = kwargs.pop("transport", FakeTransport(encoder))
        result = run_single_motor_test(
            motor=motor,
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
        result, camera, encoder, transport = self.run_test(dry_run=True)

        self.assertEqual(result["status"], "OK")
        self.assertEqual(result["oe_sequence"], [])
        self.assertEqual(transport.commands, [])
        self.assertTrue(encoder.oe_high)
        self.assertTrue(camera.stopped)

    def test_retries_failed_port_open_once_after_fresh_autodetection(self):
        camera = FakeCamera()
        encoder = FakeEncoder()
        transport = FakeTransport(encoder)
        with patch("host.armctl.transport.resolve_serial_port", return_value=("COM4", False)), \
             patch("host.armctl.transport.detect_auto_serial_port", return_value="COM8"), \
             patch("tools.single_motor_test._open_transport", side_effect=[FileNotFoundError("COM4"), transport]) as open_port:
            result = run_single_motor_test(
                motor=1, port="COM4", frames=3, camera=camera, encoder=encoder,
            )

        self.assertEqual(open_port.call_args_list, [call("COM4"), call("COM8")])
        self.assertEqual(result["port_requested"], "COM4")
        self.assertEqual(result["port_used"], "COM8")
        self.assertTrue(result["port_autodetected"])
        self.assertEqual(result["status"], "OK")

    def test_warms_camera_and_retries_empty_audits_until_three_valid_passes(self):
        clock = {"now": 0.0}

        class WarmingCamera(FakeCamera):
            empty_audits_remaining = 2

            def get_latest_frame(self):
                if clock["now"] >= 2.0 and self.empty_audits_remaining:
                    self.empty_audits_remaining -= 1
                    self.timestamp += 1
                    return None, self.timestamp, 30.0
                return super().get_latest_frame()

        class EmptyAuditEncoder(FakeEncoder):
            def audit_pre_power(self, camera, num_frames, max_variance_deg):
                frame, _timestamp, _fps = camera.get_latest_frame()
                if frame is None:
                    self.audit_calls += 1
                    return {
                        "passed": False,
                        "frames_evaluated": 0,
                        "max_variance_deg2": 999.0,
                        "variances_deg2": [999.0] * 6,
                        "mode_used": "none",
                        "visible_marker_ids": [],
                    }
                return super().audit_pre_power(camera, num_frames, max_variance_deg)

        def monotonic():
            return clock["now"]

        def sleep(duration):
            clock["now"] += duration

        camera = WarmingCamera()
        encoder = EmptyAuditEncoder()
        with patch("tools.single_motor_test.time.monotonic", side_effect=monotonic), \
             patch("tools.single_motor_test.time.sleep", side_effect=sleep), \
             patch("host.armctl.transport.resolve_serial_port", return_value=("COM8", True)):
            result = run_single_motor_test(
                motor=1, port="auto", frames=3, camera=camera, encoder=encoder,
                dry_run=True,
            )

        self.assertEqual(encoder.audit_calls, 5)
        self.assertEqual(result["decision_gate"]["consecutive_passes"], 3)
        self.assertEqual(len(result["decision_gate"]["audit_results"]), 3)
        self.assertEqual(result["audit"]["frames_evaluated"], [0, 0, 3, 3, 3])
        self.assertEqual(result["audit_attempts"], 5)
        self.assertEqual(result["port_used"], "COM8")
        self.assertEqual(result["status"], "OK")
        self.assertEqual(result["oe_sequence"], [])

    def test_sliding_audit_window_recovers_after_initial_failure(self):
        encoder = SequenceAuditEncoder([9.0, 0.1, 0.1, 0.1])
        result, _camera, _encoder, transport = self.run_test(
            encoder=encoder, max_audit_attempts=6,
        )

        self.assertEqual(result["status"], "OK")
        self.assertEqual(result["audit_attempts"], 4)
        self.assertEqual(result["decision_gate"]["consecutive_passes"], 3)
        self.assertEqual(
            [item["passed"] for item in result["audit"]["results"]],
            [False, True, True, True],
        )
        self.assertIn("R", transport.commands)

    def test_sliding_audit_window_aborts_after_all_attempts_fail(self):
        encoder = SequenceAuditEncoder([9.0] * 6)
        result, _camera, _encoder, transport = self.run_test(
            encoder=encoder, max_audit_attempts=6,
        )

        self.assertEqual(result["status"], "ABORT_AUDIT")
        self.assertEqual(result["audit_attempts"], 6)
        self.assertEqual(
            result["audit"]["reason"], "three_consecutive_passes_not_reached",
        )
        self.assertEqual(result["decision_gate"]["consecutive_passes"], 0)
        self.assertNotIn("R", transport.commands)

    def _run_mode_limit_test(self, mode, variance, **options):
        clock = {"now": 0.0}

        def monotonic():
            return clock["now"]

        def sleep(duration):
            clock["now"] += duration

        encoder = ModeVarianceEncoder(mode, variance)
        camera = FakeCamera()
        transport = FakeTransport(encoder)
        with patch("tools.single_motor_test.time.monotonic", side_effect=monotonic), \
             patch("tools.single_motor_test.time.sleep", side_effect=sleep), \
             patch("host.armctl.transport.resolve_serial_port", return_value=("COM8", True)):
            result = run_single_motor_test(
                motor=1, port="auto", frames=3, max_audit_attempts=3,
                camera=camera, encoder=encoder, transport=transport, dry_run=True,
                **options,
            )
        return result, transport

    def test_auto_geometric_limit_accepts_variance_three(self):
        result, _transport = self._run_mode_limit_test("geometric_fallback", 3.0)

        self.assertEqual(result["status"], "OK")
        self.assertEqual(result["variance_limit_deg2"], 5.0)
        self.assertEqual(result["variance_limit_source"], "auto_geometric")

    def test_auto_aruco_limit_rejects_variance_three(self):
        result, transport = self._run_mode_limit_test("aruco", 3.0)

        self.assertEqual(result["status"], "OK")
        self.assertEqual(result["variance_limit_deg2"], 1.5)
        self.assertEqual(result["variance_limit_source"], "auto_aruco")
        self.assertFalse(result["audit"]["passed"])
        self.assertNotIn("R", transport.commands)

    def test_cli_variance_override_and_strict_limit_sources(self):
        cli_result, _transport = self._run_mode_limit_test(
            "geometric_fallback", 3.0, max_variance=3.5,
        )
        strict_result, _transport = self._run_mode_limit_test(
            "geometric_fallback", 3.0, strict=True,
        )

        self.assertEqual(cli_result["status"], "OK")
        self.assertEqual(cli_result["variance_limit_deg2"], 3.5)
        self.assertEqual(cli_result["variance_limit_source"], "cli")
        self.assertEqual(strict_result["status"], "OK")
        self.assertEqual(strict_result["variance_limit_deg2"], 1.5)
        self.assertEqual(strict_result["variance_limit_source"], "strict")
        self.assertFalse(strict_result["audit"]["passed"])

    def test_encoder_auto_limit_uses_detected_mode(self):
        def run_audit(mode):
            encoder = VirtualOpticalEncoder()

            def read_positions(frame, timestamp_s=None):
                encoder._last_window_thetas = None
                angle = math.sqrt(3.0) * (1 if int(frame) % 2 else -1)
                return {
                    "mode_used": mode,
                    "thetas_deg": [angle, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "visible_marker_ids": [],
                }

            with patch.object(encoder, "read_positions", side_effect=read_positions):
                return encoder.audit_pre_power(
                    list(range(40)), num_frames=30, max_variance_deg=None,
                )

        geometric = run_audit("geometric_fallback")
        aruco = run_audit("aruco")

        self.assertAlmostEqual(geometric["max_variance_deg2"], 3.0)
        self.assertTrue(geometric["passed"])
        self.assertEqual(geometric["variance_limit_deg2"], 5.0)
        self.assertAlmostEqual(aruco["max_variance_deg2"], 3.0)
        self.assertFalse(aruco["passed"])
        self.assertEqual(aruco["variance_limit_deg2"], 1.5)

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
        self.assertIsNone(result["measured_delta_deg"])
        self.assertAlmostEqual(result["measured_proxy"]["delta_px"], 40.0, delta=0.3)
        self.assertEqual(transport.commands[-1], "S")

    def test_gripper_proxy_change_reports_ok_with_pixel_delta(self):
        result, _camera, _encoder, _transport = self.run_test(motor=1)

        self.assertEqual(result["status"], "OK")
        self.assertIsNone(result["measured_delta_deg"])
        self.assertEqual(result["measured_proxy"]["delta_px"], 40.0)

    def test_unchanged_gripper_proxy_is_commanded_unverified(self):
        camera = FakeCamera()
        encoder = FakeEncoder()
        transport = FakeTransport(encoder, gripper_motion=False)
        result = run_single_motor_test(
            motor=1, frames=3, camera=camera, encoder=encoder, transport=transport,
        )

        self.assertEqual(result["status"], "COMMANDED_UNVERIFIED")
        self.assertEqual(result["reason"], "vision_cannot_observe_joint")
        self.assertEqual(result["measured_proxy"]["delta_px"], 0.0)

    def test_observable_motor_four_without_motion_reports_no_motion(self):
        result, _camera, _encoder, _transport = self.run_test(
            motor=4, transport=FakeTransport(FakeEncoder(), probe_motion=False),
        )

        self.assertEqual(result["status"], "NO_MOTION")

    def test_unchanged_motor_two_geometric_fallback_is_commanded_unverified(self):
        encoder = GeometricFallbackEncoder()
        camera = FakeCamera()
        transport = FakeTransport(encoder)
        result = run_single_motor_test(
            motor=2, frames=3, camera=camera, encoder=encoder, transport=transport,
        )

        self.assertEqual(result["status"], "COMMANDED_UNVERIFIED")
        self.assertEqual(result["reason"], "vision_cannot_observe_joint")


if __name__ == "__main__":
    unittest.main()
