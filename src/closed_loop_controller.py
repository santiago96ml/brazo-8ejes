"""Visual closed-loop joint controller for the 6-DOF arm."""

from __future__ import annotations

import math
import time
from typing import Any

try:
    from .pid_pwm_controller import (
        DecisionGate,
        JointPIDConfig,
        NonlinearPIDController,
    )
    from .pulse_odometry import PulseOdometry
except ImportError:  # pragma: no cover - supports direct module execution
    from pid_pwm_controller import DecisionGate, JointPIDConfig, NonlinearPIDController
    from pulse_odometry import PulseOdometry


_JOINTS = (
    # theta, motor, name, kp, ki, kd, deadband, maximum offset
    (0, 7, "base", 0.85, 0.04, 0.22, 6, 35),
    (1, 5, "shoulder", 1.25, 0.08, 0.38, 8, 42),
    (2, 4, "elbow", 1.05, 0.06, 0.30, 7, 38),
    (3, 3, "wrist_pitch", 0.75, 0.03, 0.18, 6, 30),
    (4, 2, "wrist_roll", 0.75, 0.03, 0.18, 6, 30),
    (5, 1, "gripper", 0.90, 0.02, 0.10, 5, 25),
)
_MOTOR_TO_THETA = {7: 0, 5: 1, 6: 1, 4: 2, 3: 3, 2: 4, 1: 5}
_DEFAULT_THETAS = [0.0, 90.0, 0.0, 0.0, 0.0, 50.0]


def _wrap_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


class ClosedLoopArmController:
    """Drive the arm from visual joint angles using the ASCII pulse protocol."""

    def __init__(
        self,
        camera=None,
        encoder=None,
        transport=None,
        port: str = "auto",
        pid_gains: dict | None = None,
        control_hz: float = 20.0,
        tolerance_deg: float = 2.5,
        max_iterations: int = 400,
        max_slew_counts_per_cycle: int = 12,
    ):
        if float(control_hz) <= 0:
            raise ValueError("control_hz must be greater than zero")
        if int(max_iterations) <= 0:
            raise ValueError("max_iterations must be greater than zero")
        if int(max_slew_counts_per_cycle) <= 0:
            raise ValueError("max_slew_counts_per_cycle must be greater than zero")

        self.camera = camera
        self.encoder = encoder
        self.transport = transport
        self.port = port
        self.pid_gains = pid_gains or {}
        self.control_hz = float(control_hz)
        self.tolerance_deg = float(tolerance_deg)
        self.max_iterations = int(max_iterations)
        self.max_slew_counts_per_cycle = int(max_slew_counts_per_cycle)
        self.odometry = PulseOdometry()
        self._controllers = self._build_controllers()
        self._decision_gate = DecisionGate(
            max_audit_variance_deg2=5.0,
            required_consecutive_passes=3,
        )
        self._connected = False
        self._enabled = False
        self._owns_transport = False
        self._camera_started = False
        self._vision_none_since: float | None = None
        self._last_mode_used = "none"
        self._last_omega = [0.0] * 6
        self._last_frame_timestamp = None
        self.start_thetas_deg: list[float] | None = None
        self._last_sent_pulses = {
            motor: int(round(float(self.odometry._calib[motor]["pulse_stop"])))
            for motor in range(1, 8)
        }

    def _build_controllers(self) -> dict[int, NonlinearPIDController]:
        controllers = {}
        for theta_index, motor, name, kp, ki, kd, dead, maximum in _JOINTS:
            calibration = self.odometry._calib[motor]
            overrides = self.pid_gains.get(theta_index)
            if overrides is None:
                overrides = self.pid_gains.get(str(theta_index))
            if overrides is None:
                overrides = self.pid_gains.get(f"theta{theta_index + 1}")
            if overrides is None:
                overrides = self.pid_gains.get(name)
            overrides = overrides if isinstance(overrides, dict) else {}
            config = JointPIDConfig(
                name=name,
                channel=motor - 1,
                kp=float(overrides.get("kp", kp)),
                ki=float(overrides.get("ki", ki)),
                kd=float(overrides.get("kd", kd)),
                u_stop=int(round(float(calibration["pulse_stop"]))),
                du_dead=int(overrides.get("du_dead", dead)),
                du_max=int(overrides.get("du_max", maximum)),
                tol_deg=float(overrides.get("tol_deg", 0.4)),
                u_min=int(round(float(calibration["pulse_min"]))),
                u_max=int(round(float(calibration["pulse_max"]))),
                gravity_mode="none",
            )
            controllers[theta_index] = NonlinearPIDController(config)
        return controllers

    def _ensure_components(self) -> None:
        if self.camera is None or isinstance(self.camera, int):
            try:
                from .fast_camera import FastUSBCamera
            except ImportError:  # pragma: no cover
                from fast_camera import FastUSBCamera
            index = 0 if self.camera is None else self.camera
            self.camera = FastUSBCamera(index=index)
        if self.encoder is None:
            try:
                from .virtual_optical_encoder import VirtualOpticalEncoder
            except ImportError:  # pragma: no cover
                from virtual_optical_encoder import VirtualOpticalEncoder
            self.encoder = VirtualOpticalEncoder()
        if self.transport is None:
            try:
                from host.armctl.transport import SerialTransport, resolve_serial_port
            except ImportError as exc:  # pragma: no cover - hardware-only path
                raise RuntimeError("serial transport is unavailable") from exc
            resolved, _detected = resolve_serial_port(self.port)
            if not resolved or resolved == "auto":
                raise RuntimeError("no Arduino serial port was found")
            self.transport = SerialTransport(resolved, 115200)
            self._owns_transport = True
            self.port = resolved

    def _send(self, command: str) -> None:
        if self.transport is None:
            raise RuntimeError("serial transport is not connected")
        self.transport.send_line(command)

    def _send_pulse(self, motor: int, pulse: int) -> None:
        self._send(f"P{motor}:{int(pulse)}")
        self._last_sent_pulses[motor] = int(pulse)

    def _preload_pulses(self, thetas: list[float]) -> None:
        motors = self.odometry.thetas_to_motors(thetas)
        for motor in range(1, 8):
            pulse = self.odometry.deg_to_pulse(motor, motors[motor])
            self._send_pulse(motor, pulse)

    def connect(self) -> dict:
        """Open I/O, pass three stable audits, preload the pose, then enable."""
        try:
            self._ensure_components()
            self._send("S")
            start = getattr(self.camera, "start", None)
            if callable(start):
                start()
                self._camera_started = True

            self._decision_gate.reset()
            audits = []
            for _ in range(self._decision_gate.required_consecutive_passes):
                audit = self.encoder.audit_pre_power(
                    self.camera,
                    num_frames=30,
                    max_variance_deg=None,
                )
                audits.append(audit)
                gate = self._decision_gate.record_audit(audit)
                if not gate["can_enable_oe"] and not audit.get("passed", False):
                    self.stop()
                    return {
                        "ok": False,
                        "reason": audit.get("reason", "pre_power_audit_failed"),
                        "audits": audits,
                        "gate": gate,
                    }

            if not gate["can_enable_oe"]:
                self.stop()
                return {
                    "ok": False,
                    "reason": "decision_gate_denied",
                    "audits": audits,
                    "gate": gate,
                }

            latest = self._get_frame()
            self._last_frame_timestamp = latest[1] if latest[0] is not None else None
            state = self.read_state(wait_fresh=True, timeout_s=1.5)
            if not state["valid"]:
                self.stop()
                self.close()
                return {"ok": False, "reason": "vision_unavailable", "audits": audits}
            self.start_thetas_deg = list(state["thetas_deg"])
            self._preload_pulses(self.start_thetas_deg)
            self._send("R")
            self._connected = True
            self._enabled = True
            latest = self._get_frame()
            self._last_frame_timestamp = latest[1] if latest[0] is not None else None
            final_state = self.read_state(wait_fresh=True, timeout_s=1.5)
            if not final_state["valid"]:
                self.stop()
                self.close()
                return {"ok": False, "reason": "vision_unavailable", "audits": audits}
            return {
                "ok": True,
                "reason": "enabled",
                "audits": audits,
                "gate": gate,
                "state": final_state,
            }
        except KeyboardInterrupt:
            self.stop()
            return {"ok": False, "reason": "keyboard_interrupt"}
        except Exception as exc:
            self.stop()
            return {"ok": False, "reason": str(exc)}

    def _get_frame(self):
        if self.camera is None:
            return None, None
        getter = getattr(self.camera, "get_latest_frame", None)
        if callable(getter):
            value = getter()
            if isinstance(value, tuple):
                frame = value[0] if value else None
                timestamp = value[1] if len(value) > 1 else None
                return frame, timestamp
            return value, time.monotonic_ns()
        reader = getattr(self.camera, "read", None)
        if callable(reader):
            value = reader()
            if isinstance(value, tuple) and len(value) == 2:
                return (value[1] if value[0] else None), time.monotonic_ns()
            return value, time.monotonic_ns()
        return None, None

    def _state_from_result(self, result: dict, timestamp: float) -> dict:
        if not isinstance(result, dict):
            result = {"mode_used": "none"}
        mode = str(result.get("mode_used", "none"))
        now = time.monotonic()
        if mode == "none":
            if self._vision_none_since is None:
                self._vision_none_since = now
        else:
            self._vision_none_since = None
        self._last_mode_used = mode

        raw_thetas = result.get("thetas_deg")
        thetas: list[float] = []
        if isinstance(raw_thetas, (list, tuple)):
            try:
                thetas = [float(value) for value in raw_thetas[:6]]
            except (TypeError, ValueError):
                thetas = []
        valid = (
            mode != "none"
            and len(thetas) == 6
            and all(math.isfinite(value) for value in thetas)
            and not all(abs(value - sentinel) <= 0.01
                        for value, sentinel in zip(thetas, _DEFAULT_THETAS))
        )

        motors = result.get("motors_deg")
        if not valid:
            motors = {}
        elif not isinstance(motors, dict):
            motors = self.odometry.thetas_to_motors(thetas)
        else:
            motors = {int(motor): float(value) for motor, value in motors.items()}
        omega = result.get("omega_vis_dps", [0.0] * 6)
        try:
            self._last_omega = [float(value) for value in omega[:6]]
        except (TypeError, ValueError):
            self._last_omega = [0.0] * 6
        if len(self._last_omega) < 6:
            self._last_omega = [0.0] * 6

        return {
            "thetas_deg": thetas,
            "motors_deg": motors,
            "pulses": dict(self._last_sent_pulses),
            "timestamp": timestamp,
            "valid": bool(valid),
            "source": str(result.get("source") or mode),
        }

    def read_state(self, wait_fresh: bool = True, timeout_s: float = 0.6) -> dict:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        last_state = None
        while True:
            frame, frame_timestamp = self._get_frame()
            if frame is not None:
                fresh = (
                    not wait_fresh
                    or frame_timestamp is None
                    or frame_timestamp != self._last_frame_timestamp
                )
                if fresh:
                    if frame_timestamp is not None:
                        self._last_frame_timestamp = frame_timestamp
                    try:
                        try:
                            result = self.encoder.read_positions(frame, timestamp_s=time.time())
                        except TypeError:
                            result = self.encoder.read_positions(frame)
                    except Exception:
                        result = {"mode_used": "none"}
                    last_state = self._state_from_result(result, time.time())
                    if last_state["valid"] or not wait_fresh:
                        return last_state
            elif not wait_fresh:
                return self._state_from_result({"mode_used": "none"}, time.time())

            remaining = deadline - time.monotonic()
            if not wait_fresh or remaining <= 0:
                return last_state or self._state_from_result(
                    {"mode_used": "none"}, time.time()
                )
            time.sleep(min(0.01, remaining))

    @staticmethod
    def _normalize_joint_mask(joint_mask) -> list[int]:
        if joint_mask is None:
            return list(range(6))
        values = list(joint_mask)
        if len(values) == 6 and all(isinstance(value, bool) for value in values):
            selected = [index for index, enabled in enumerate(values) if enabled]
        else:
            selected = [int(value) for value in values]
        if any(index < 0 or index > 5 for index in selected):
            raise ValueError("joint_mask entries must be theta indices 0..5")
        return sorted(set(selected))

    def _target_pulse(self, motor: int, offset: float) -> int:
        calibration = self.odometry._calib[motor]
        stop = int(round(float(calibration["pulse_stop"])))
        pulse = stop + int(round(offset))
        minimum = int(round(float(calibration["pulse_min"])))
        maximum = int(round(float(calibration["pulse_max"])))
        return max(minimum, min(maximum, pulse))

    def _actuate_cycle(self, state: dict, targets: list[float], errors: list[float],
                       selected: list[int], dt: float) -> None:
        desired: dict[int, int] = {}
        target_motors = self.odometry.thetas_to_motors(targets)
        for index in selected:
            controller = self._controllers[index]
            theta = state["thetas_deg"][index]
            # Pass the wrapped reference because the PID computes its own error.
            reference = theta + errors[index]
            pid_pulse = controller.compute(
                theta_ref=reference,
                theta_vis=theta,
                omega_vis=self._last_omega[index],
                dt=dt,
                theta2=state["thetas_deg"][1],
                theta3=state["thetas_deg"][2],
            )
            motor = _JOINTS[index][1]
            signed_deadband = (
                math.copysign(controller.config.du_dead, errors[index])
                if abs(errors[index]) > controller.config.tol_deg else 0.0
            )
            # Treat PID output as a correction around the calibrated position
            # pulse; cancel its deadband step so the position can lock precisely.
            correction = pid_pulse - controller.config.u_stop - signed_deadband
            target_position = self.odometry.deg_to_pulse(motor, target_motors[motor])
            target_stop = int(round(float(self.odometry._calib[motor]["pulse_stop"])))
            desired[motor] = self._target_pulse(
                motor, target_position - target_stop - correction
            )
            if index == 1:
                coupling = int(getattr(self.odometry, "shoulder_coupling", 1))
                slave_correction = correction if coupling == 1 else -correction
                slave_position = self.odometry.deg_to_pulse(6, target_motors[6])
                slave_stop = int(round(float(self.odometry._calib[6]["pulse_stop"])))
                desired[6] = self._target_pulse(
                    6, slave_position - slave_stop - slave_correction
                )

        for motor, target in desired.items():
            previous = self._last_sent_pulses[motor]
            delta = max(
                -self.max_slew_counts_per_cycle,
                min(self.max_slew_counts_per_cycle, target - previous),
            )
            pulse = previous + delta
            if pulse != previous:
                self._send_pulse(motor, pulse)

    def _result(self, ok: bool, reason: str, iterations: int, start: float,
                errors: list[float]) -> dict:
        return {
            "ok": bool(ok),
            "reason": reason,
            "iterations": int(iterations),
            "final_error_deg": [float(value) for value in errors],
            "duration_s": max(0.0, time.monotonic() - start),
            "pulses": dict(self._last_sent_pulses),
        }

    def move_to_thetas(self, target_thetas_deg, tolerance_deg=None,
                       timeout_s: float = 10.0, joint_mask=None) -> dict:
        start = time.monotonic()
        errors = [0.0] * 6
        iterations = 0
        if not self._connected or not self._enabled:
            return self._result(False, "not_connected", iterations, start, errors)
        try:
            targets = [float(value) for value in target_thetas_deg]
            if len(targets) < 6 or not all(math.isfinite(value) for value in targets[:6]):
                raise ValueError("target_thetas_deg must contain six finite angles")
            targets = targets[:6]
            selected = self._normalize_joint_mask(joint_mask)
            if not selected:
                return self._result(True, "within_tolerance", 0, start, errors)
            tolerance = self.tolerance_deg if tolerance_deg is None else float(tolerance_deg)
            if tolerance < 0 or not math.isfinite(tolerance):
                raise ValueError("tolerance_deg must be a finite nonnegative value")
            timeout = max(0.0, float(timeout_s))
            period = 1.0 / self.control_hz
            for index in selected:
                self._controllers[index].reset()

            while iterations < self.max_iterations:
                cycle_start = time.monotonic()
                state = self.read_state()
                if not state["valid"]:
                    self.stop()
                    return self._result(False, "invalid_start_pose", iterations, start, errors)
                if (self._vision_none_since is not None
                        and cycle_start - self._vision_none_since > 0.5):
                    self.stop()
                    return self._result(False, "vision_lost", iterations, start, errors)
                for index in selected:
                    errors[index] = _wrap_deg(targets[index] - state["thetas_deg"][index])
                if any(abs(errors[index]) > 60.0 for index in selected):
                    self.stop()
                    return self._result(False, "joint_error_exceeds_60_deg", iterations, start, errors)
                elapsed = cycle_start - start
                if elapsed >= timeout:
                    self.stop()
                    return self._result(False, "timeout", iterations, start, errors)
                if all(abs(errors[index]) <= tolerance for index in selected):
                    return self._result(True, "within_tolerance", iterations, start, errors)

                dt = period if iterations == 0 else max(0.001, cycle_start - previous_cycle)
                self._actuate_cycle(state, targets, errors, selected, dt)
                iterations += 1
                previous_cycle = cycle_start
                remaining = period - (time.monotonic() - cycle_start)
                if remaining > 0:
                    time.sleep(remaining)

            self.stop()
            return self._result(False, "max_iterations", iterations, start, errors)
        except KeyboardInterrupt:
            self.stop()
            return self._result(False, "keyboard_interrupt", iterations, start, errors)
        except Exception as exc:
            self.stop()
            return self._result(False, str(exc), iterations, start, errors)

    def move_joint(self, motor: int, delta_deg: float, timeout_s: float = 6.0) -> dict:
        if int(motor) not in _MOTOR_TO_THETA:
            start = time.monotonic()
            return self._result(False, "unsupported_motor", 0, start, [0.0] * 6)
        state = self.read_state()
        if not state["valid"]:
            start = time.monotonic()
            self.stop()
            return self._result(False, "invalid_start_pose", 0, start, [0.0] * 6)
        index = _MOTOR_TO_THETA[int(motor)]
        targets = list(state["thetas_deg"])
        targets[index] += float(delta_deg)
        return self.move_to_thetas(
            targets,
            timeout_s=timeout_s,
            joint_mask=[index],
        )

    def return_to_start(self, timeout_s: float = 6.0) -> dict:
        """Safely re-enable from the measured pose and return to the connect pose."""
        started = time.monotonic()
        timeout = max(0.0, float(timeout_s))
        deadline = started + timeout
        errors = [0.0] * 6
        if self.start_thetas_deg is None:
            return self._result(False, "invalid_start_pose", 0, started, errors)
        if not self._connected or self.transport is None:
            return self._result(False, "not_connected", 0, started, errors)
        if not self._enabled:
            state = self.read_state(
                wait_fresh=True,
                timeout_s=min(0.6, max(0.0, deadline - time.monotonic())),
            )
            if not state["valid"]:
                return self._result(False, "invalid_start_pose", 0, started, errors)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return self._result(False, "timeout", 0, started, errors)
            self._preload_pulses(state["thetas_deg"])
            self._send("R")
            self._enabled = True
        return self.move_to_thetas(
            self.start_thetas_deg,
            timeout_s=max(0.0, deadline - time.monotonic()),
        )

    def hold(self, seconds: float) -> dict:
        start = time.monotonic()
        errors = [0.0] * 6
        iterations = 0
        if not self._connected or not self._enabled:
            return self._result(False, "not_connected", iterations, start, errors)
        deadline = start + max(0.0, float(seconds))
        period = 1.0 / self.control_hz
        try:
            while time.monotonic() < deadline:
                cycle_start = time.monotonic()
                self.read_state()
                iterations += 1
                if (self._vision_none_since is not None
                        and cycle_start - self._vision_none_since > 0.5):
                    self.stop()
                    return self._result(False, "vision_lost", iterations, start, errors)
                remaining = min(period, deadline - time.monotonic())
                if remaining > 0:
                    time.sleep(remaining)
            return self._result(True, "held", iterations, start, errors)
        except KeyboardInterrupt:
            self.stop()
            return self._result(False, "keyboard_interrupt", iterations, start, errors)

    def stop(self) -> dict:
        """Always attempt the firmware emergency-stop command."""
        self._enabled = False
        try:
            self._send("S")
            return {"ok": True, "command": "S"}
        except Exception as exc:
            return {"ok": False, "command": "S", "reason": str(exc)}

    def close(self) -> None:
        self.stop()
        self._connected = False
        if self._camera_started and self.camera is not None:
            try:
                self.camera.stop()
            except Exception:
                pass
            self._camera_started = False
        if self.transport is not None:
            try:
                self.transport.close()
            except Exception:
                pass
        self.transport = None
