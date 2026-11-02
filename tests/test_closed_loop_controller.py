from __future__ import annotations

import json
import queue
import sys
import time
from types import SimpleNamespace

from src.pulse_odometry import PulseOdometry
from src.closed_loop_controller import ClosedLoopArmController


class FakeCamera:
    def __init__(self):
        self.started = False
        self.frame_number = 0

    def start(self):
        self.started = True
        return self

    def stop(self):
        self.started = False

    def get_latest_frame(self):
        self.frame_number += 1
        return object(), self.frame_number, 30.0


class FakeArm:
    def __init__(self):
        self.odometry = PulseOdometry()
        self.thetas = [15.0, 90.0, 0.0, 0.0, 0.0, 50.0]

    def apply_pulse(self, motor, pulse):
        motors = self.odometry.thetas_to_motors(self.thetas)
        measured_deg = self.odometry.pulse_to_deg(motor, pulse)
        if motor == 6:
            motors[5] = measured_deg
        else:
            motors[motor] = measured_deg
        self.thetas = self.odometry.motors_to_thetas(motors)


class FakeEncoder:
    def __init__(self, arm, audit_passes=True, audit_sequence=None):
        self.arm = arm
        self.audit_passes = audit_passes
        self.mode_used = "aruco"
        self.thetas_override = None
        self.audit_sequence = list(audit_sequence or [])
        self.audit_calls = []

    def audit_pre_power(self, _camera, **kwargs):
        self.audit_calls.append(kwargs)
        if self.audit_sequence:
            return self.audit_sequence.pop(0)
        variance = 0.01 if self.audit_passes else 9.0
        return {
            "passed": self.audit_passes,
            "mode_used": "aruco" if self.audit_passes else "geometric_fallback",
            "resting_thetas_deg": list(self.arm.thetas),
            "frames_evaluated": 30,
            "max_variance_deg2": variance,
            "primary_max_variance_deg2": variance,
            "secondary_max_variance_deg2": 0.0,
        }

    def read_positions(self, _frame):
        motors = self.arm.odometry.thetas_to_motors(self.arm.thetas)
        return {
            "mode_used": self.mode_used,
            "thetas_deg": (
                list(self.thetas_override)
                if self.thetas_override is not None else list(self.arm.thetas)
            ),
            "motors_deg": motors,
            "omega_vis_dps": [0.0] * 6,
        }


class FakeTransport:
    def __init__(self, arm):
        self.arm = arm
        self.commands = []
        self.closed = False

    def send_line(self, command):
        self.commands.append(command)
        if command.startswith("P"):
            motor_text, pulse_text = command[1:].split(":", 1)
            self.arm.apply_pulse(int(motor_text), int(pulse_text))

    def close(self):
        self.closed = True


def make_controller(*, audit_passes=True, audit_sequence=None, **kwargs):
    arm = FakeArm()
    camera = FakeCamera()
    transport = FakeTransport(arm)
    controller = ClosedLoopArmController(
        camera=camera,
        encoder=FakeEncoder(arm, audit_passes=audit_passes, audit_sequence=audit_sequence),
        transport=transport,
        control_hz=kwargs.pop("control_hz", 200.0),
        **kwargs,
    )
    return controller, arm, transport


def test_connect_refuses_to_enable_when_pre_power_audit_fails():
    controller, _arm, transport = make_controller(audit_passes=False)

    result = controller.connect()

    assert not result["ok"]
    assert "R" not in transport.commands
    assert transport.commands[-1] == "S"
    assert result["reason"] == "three_consecutive_passes_not_reached"
    assert len(result["audits"]) == 6


def _audit(mode, variance, passed=True):
    return {
        "passed": passed,
        "mode_used": mode,
        "resting_thetas_deg": [15.0, 90.0, 0.0, 0.0, 0.0, 50.0],
        "frames_evaluated": 30,
        "max_variance_deg2": variance,
        "primary_max_variance_deg2": variance,
        "secondary_max_variance_deg2": 0.0,
    }


def test_connect_sliding_audit_window_recovers_after_first_failure():
    sequence = [
        _audit("geometric_fallback", 9.0, passed=False),
        _audit("geometric_fallback", 0.33),
        _audit("geometric_fallback", 0.4),
        _audit("geometric_fallback", 0.5),
    ]
    controller, _arm, transport = make_controller(audit_sequence=sequence)

    result = controller.connect()

    assert result["ok"] is True
    assert len(result["audits"]) == 4
    assert result["variance_limit_deg2"] == 5.0
    assert [call["max_variance_deg"] for call in controller.encoder.audit_calls] == [
        5.0, 5.0, 5.0, 5.0
    ]
    assert "R" in transport.commands


def test_connect_fails_after_six_audit_failures():
    sequence = [_audit("geometric_fallback", 9.0, passed=False) for _ in range(6)]
    controller, _arm, transport = make_controller(audit_sequence=sequence)

    result = controller.connect()

    assert result["ok"] is False
    assert result["reason"] == "three_consecutive_passes_not_reached"
    assert len(result["audits"]) == 6
    assert result["gate"]["consecutive_passes"] == 0
    assert "R" not in transport.commands
    assert transport.commands[-1] == "S"


def test_aruco_audit_above_mode_limit_is_rejected():
    controller, _arm, transport = make_controller(
        audit_sequence=[_audit("aruco", 3.0)],
        max_audit_attempts=1,
    )

    result = controller.connect()

    assert result["ok"] is False
    assert result["audits"][0]["variance_limit_deg2"] == 1.5
    assert controller.encoder.audit_calls[0]["max_variance_deg"] == 5.0
    assert "R" not in transport.commands


def test_connect_closes_when_fresh_valid_vision_is_unavailable():
    controller, _arm, transport = make_controller()
    controller.encoder.thetas_override = [0.0, 90.0, 0.0, 0.0, 0.0, 50.0]

    result = controller.connect()

    assert result["ok"] is False
    assert result["reason"] == "vision_unavailable"
    assert "R" not in transport.commands
    assert transport.commands[-1] == "S"
    assert transport.closed


def test_move_to_thetas_converges_on_simulated_two_joint_arm():
    controller, arm, _transport = make_controller()
    assert controller.connect()["ok"]

    result = controller.move_to_thetas(
        [0.0, 100.0, 10.0, 0.0, 0.0, 50.0],
        joint_mask=[1, 2],
        tolerance_deg=2.5,
        timeout_s=3.0,
    )

    assert result["ok"] is True, result
    assert abs(arm.thetas[1] - 100.0) <= 2.5
    assert abs(arm.thetas[2] - 10.0) <= 2.5


def test_slew_limit_is_respected_between_motion_cycles():
    controller, _arm, transport = make_controller(max_slew_counts_per_cycle=4)
    assert controller.connect()["ok"]
    start = len(transport.commands)

    controller.move_to_thetas(
        [0.0, 120.0, 25.0, 0.0, 0.0, 50.0],
        joint_mask=[1, 2],
        tolerance_deg=2.5,
        timeout_s=0.25,
    )

    previous = {}
    for command in transport.commands[start:]:
        if not command.startswith("P"):
            continue
        motor_text, pulse_text = command[1:].split(":", 1)
        motor = int(motor_text)
        pulse = int(pulse_text)
        if motor in previous:
            assert abs(pulse - previous[motor]) <= 4
        previous[motor] = pulse


def test_timeout_aborts_and_sends_emergency_stop():
    controller, _arm, transport = make_controller(max_slew_counts_per_cycle=1)
    assert controller.connect()["ok"]

    result = controller.move_to_thetas(
        [0.0, 110.0, 0.0, 0.0, 0.0, 50.0],
        joint_mask=[1],
        timeout_s=0.03,
    )

    assert result["ok"] is False
    assert result["reason"] == "timeout"
    assert transport.commands[-1] == "S"


def test_stop_is_idempotent_and_always_sends_stop_command():
    controller, _arm, transport = make_controller()

    controller.stop()
    controller.stop()

    assert transport.commands[-2:] == ["S", "S"]


def test_read_state_rejects_missing_vision_and_default_pose_sentinel():
    controller, _arm, _transport = make_controller()
    encoder = controller.encoder
    encoder.mode_used = "none"
    none_state = controller.read_state(wait_fresh=False)
    assert none_state["valid"] is False
    assert none_state["source"] == "none"

    encoder.mode_used = "aruco"
    encoder.thetas_override = [0.0, 90.0, 0.0, 0.0, 0.0, 50.0]
    sentinel_state = controller.read_state(wait_fresh=False)
    assert sentinel_state["valid"] is False


def test_invalid_pose_refuses_both_move_apis_without_pulse_commands():
    controller, _arm, transport = make_controller()
    assert controller.connect()["ok"]
    controller.encoder.mode_used = "none"
    command_start = len(transport.commands)

    move = controller.move_to_thetas([20.0, 90.0, 0.0, 0.0, 0.0, 50.0], joint_mask=[0])
    assert move["ok"] is False
    assert move["reason"] == "invalid_start_pose"
    assert not any(command.startswith(("P", "T")) for command in transport.commands[command_start:])

    result = controller.move_joint(7, 5.0)
    assert result["ok"] is False
    assert result["reason"] == "invalid_start_pose"
    assert not any(command.startswith(("P", "T")) for command in transport.commands[command_start:])


def test_return_to_start_converges_on_simulated_arm():
    controller, arm, _transport = make_controller()
    initial_thetas = list(arm.thetas)
    assert controller.connect()["ok"]
    assert controller.start_thetas_deg == initial_thetas

    moved = list(arm.thetas)
    moved[1] = 100.0
    assert controller.move_to_thetas(moved, joint_mask=[1], timeout_s=3.0)["ok"]

    result = controller.return_to_start(timeout_s=3.0)

    assert result["ok"] is True
    assert abs(arm.thetas[1] - controller.start_thetas_deg[1]) <= controller.tolerance_deg


class FirmwareStateTransport(FakeTransport):
    def __init__(self, arm, coupling=1, pulses=None):
        super().__init__(arm)
        self.rx_queue = queue.Queue()
        self.coupling = coupling
        self.pulses = pulses if pulses is not None else {
            motor: int(round(arm.odometry.deg_to_pulse(motor, angle)))
            for motor, angle in arm.odometry.thetas_to_motors(arm.thetas).items()
        }

    def send_line(self, command):
        self.commands.append(command)
        if command == "Q":
            self.rx_queue.put(json.dumps({
                "status": "STATE",
                "oe": 1,
                "shoulder_coupling": self.coupling,
                "motors": [
                    {"m": motor, "pulse": pulse}
                    for motor, pulse in self.pulses.items()
                ],
            }))
            return
        if command.startswith("P"):
            motor_text, pulse_text = command[1:].split(":", 1)
            self.pulses[int(motor_text)] = int(pulse_text)


def test_apply_pulse_delta_clamps_and_mirrors_motor_six():
    arm = FakeArm()
    odometry = arm.odometry
    transport = FirmwareStateTransport(
        arm,
        coupling=-1,
        pulses={5: 490, 6: 120, 1: 307, 2: 307, 3: 307, 4: 307, 7: 307},
    )
    controller = ClosedLoopArmController(transport=transport)

    pulse = controller.apply_pulse_delta(6, 500)

    assert pulse == int(odometry._calib[6]["pulse_max"])
    assert transport.pulses[6] == int(odometry._calib[6]["pulse_max"])
    assert transport.pulses[5] == int(odometry._calib[5]["pulse_min"])
    assert transport.commands[-2:] == [
        f"P6:{int(odometry._calib[6]['pulse_max'])}",
        f"P5:{int(odometry._calib[5]['pulse_min'])}",
    ]


def test_apply_pulse_delta_mirrors_motor_six_with_positive_coupling():
    arm = FakeArm()
    transport = FirmwareStateTransport(
        arm,
        coupling=1,
        pulses={5: 200, 6: 300, 1: 307, 2: 307, 3: 307, 4: 307, 7: 307},
    )
    controller = ClosedLoopArmController(transport=transport)

    controller.apply_pulse_delta(6, 25)

    assert transport.pulses[6] == 325
    assert transport.pulses[5] == 225
    assert transport.commands[-2:] == ["P6:325", "P5:225"]


def test_move_to_xyz_openloop_refuses_unreachable_target_without_pulses():
    controller, _arm, transport = make_controller()
    result = controller.move_to_xyz_openloop(150, 0, 0)

    assert result == {"ok": False, "reason": "unreachable"}
    assert not any(command.startswith("P") for command in transport.commands)


def test_openloop_step_reports_excessive_vision_drift(monkeypatch):
    controller, _arm, _transport = make_controller()
    states = [
        {"valid": True, "thetas_deg": [0, 90, 0, 0, 0, 50]},
        {"valid": True, "thetas_deg": [20, 90, 0, 0, 0, 50]},
    ]
    controller.read_state = lambda **_kwargs: states.pop(0)
    controller.apply_pulse_delta = lambda *_args: 330
    monkeypatch.setattr("src.closed_loop_controller.time.sleep", lambda _seconds: None)

    result = controller.move_delta_openloop(0, 10)

    assert result["ok"] is False
    assert result["reason"] == "drift"
    assert result["verified"] is True
    assert result["measured_deg"] == 20
    assert result["drift_deg"] == 10


def test_autonomous_demo_restores_start_after_step_three_failure(monkeypatch, capsys):
    class FailingController:
        def __init__(self, **_kwargs):
            self.return_calls = 0
            self.delta_calls = 0

        def connect(self):
            return {"ok": True}

        def read_state(self, **_kwargs):
            return {"valid": True, "thetas_deg": [20.0, 90.0, 0.0, 0.0, 0.0, 50.0]}

        def move_delta_openloop(self, *_args, **_kwargs):
            self.delta_calls += 1
            return {"ok": False, "reason": "simulated_failure"}

        def move_joint(self, *_args, **_kwargs):
            return {"ok": True}

        def return_to_start(self, timeout_s=6.0):
            self.return_calls += 1
            assert timeout_s <= 6.0
            return {"ok": True, "final_error_deg": [0.0] * 6}

        def stop(self):
            pass

        def close(self):
            pass

    controller = FailingController()
    module = SimpleNamespace(ClosedLoopArmController=lambda **_kwargs: controller)
    monkeypatch.setitem(sys.modules, "src.closed_loop_controller", module)

    from tools import autonomous_demo

    status = autonomous_demo.main([])
    report = json.loads(capsys.readouterr().out.strip().splitlines()[-1])

    assert status == 1
    assert report["status"] == "ABORT_STEP_3"
    assert report["return_to_start"]["ok"] is True
    assert controller.delta_calls == 1
