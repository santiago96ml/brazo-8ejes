from __future__ import annotations

import time

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
        self.thetas = [0.0, 90.0, 0.0, 0.0, 0.0, 50.0]

    def apply_pulse(self, motor, pulse):
        motors = self.odometry.thetas_to_motors(self.thetas)
        measured_deg = self.odometry.pulse_to_deg(motor, pulse)
        if motor == 6:
            motors[5] = measured_deg
        else:
            motors[motor] = measured_deg
        self.thetas = self.odometry.motors_to_thetas(motors)


class FakeEncoder:
    def __init__(self, arm, audit_passes=True):
        self.arm = arm
        self.audit_passes = audit_passes

    def audit_pre_power(self, _camera, **_kwargs):
        return {
            "passed": self.audit_passes,
            "mode_used": "aruco",
            "resting_thetas_deg": list(self.arm.thetas),
            "max_variance_deg2": 0.01,
        }

    def read_positions(self, _frame):
        motors = self.arm.odometry.thetas_to_motors(self.arm.thetas)
        return {
            "mode_used": "aruco",
            "thetas_deg": list(self.arm.thetas),
            "motors_deg": motors,
            "omega_vis_dps": [0.0] * 6,
        }


class FakeTransport:
    def __init__(self, arm):
        self.arm = arm
        self.commands = []

    def send_line(self, command):
        self.commands.append(command)
        if command.startswith("P"):
            motor_text, pulse_text = command[1:].split(":", 1)
            self.arm.apply_pulse(int(motor_text), int(pulse_text))

    def close(self):
        pass


def make_controller(*, audit_passes=True, **kwargs):
    arm = FakeArm()
    camera = FakeCamera()
    transport = FakeTransport(arm)
    controller = ClosedLoopArmController(
        camera=camera,
        encoder=FakeEncoder(arm, audit_passes=audit_passes),
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
