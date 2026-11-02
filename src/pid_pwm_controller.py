"""Discrete nonlinear PID-PWM controller, 20-byte binary frame codec and
safe visual homing state machine for the 8-axis arm (Sections 4.2, 4.3, 5.2).

Pure Python, no I/O. ASCII-only.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Union

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

TOL_DEG_DEFAULT = 0.4
U_STOP_DEFAULT = 307
U_REG_MIN = 205
U_REG_MAX = 410

FLAG_OE_DISABLE = 0x01
FLAG_EMERGENCY_STOP = 0x02
FLAG_HOMING_MODE = 0x04

PHASE_0_INHIBITED = "PHASE_0_INHIBITED"
PHASE_1_OPTICAL_AUDIT = "PHASE_1_OPTICAL_AUDIT"
PHASE_2_NEUTRAL_PRELOAD = "PHASE_2_NEUTRAL_PRELOAD"
PHASE_3_SEQUENTIAL_TOP_DOWN = "PHASE_3_SEQUENTIAL_TOP_DOWN"
PHASE_4_HOMING_LOCKED = "PHASE_4_HOMING_LOCKED"


# ---------------------------------------------------------------------------
# PID configuration
# ---------------------------------------------------------------------------


@dataclass
class JointPIDConfig:
    """Per-joint tuning for the nonlinear discrete PID law."""

    name: str = "joint"
    channel: int = 0
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    kg: float = 0.0
    kg_coupled: float = 0.0
    u_stop: int = U_STOP_DEFAULT
    du_dead: int = 0
    du_max: int = 0
    tol_deg: float = TOL_DEG_DEFAULT
    u_min: int = U_REG_MIN
    u_max: int = U_REG_MAX
    gravity_mode: str = "none"  # "none" | "shoulder" | "elbow"
    shoulder_coupling: int = 1


def _mk(name, channel, kp, ki, kd, kg, kg_coupled, u_stop, du_dead, du_max,
        gravity_mode, **kw):
    return JointPIDConfig(
        name=name, channel=channel, kp=kp, ki=ki, kd=kd, kg=kg,
        kg_coupled=kg_coupled, u_stop=u_stop, du_dead=du_dead, du_max=du_max,
        gravity_mode=gravity_mode, **kw)


DEFAULT_JOINT_CONFIGS: Dict[str, JointPIDConfig] = {
    "J1_base": _mk("J1_base", 6, 0.85, 0.04, 0.22, 0.0, 0.0, 307, 6, 35,
                   "none"),
    "J2_shoulder": _mk("J2_shoulder", 4, 1.25, 0.08, 0.38, 4.5, 2.8, 307, 8,
                       42, "shoulder"),
    "J3_elbow": _mk("J3_elbow", 3, 1.05, 0.06, 0.30, 2.8, 0.0, 307, 7, 38,
                    "elbow"),
    "J4_wrist_pitch": _mk("J4_wrist_pitch", 2, 0.75, 0.03, 0.18, 0.0, 0.0,
                          307, 6, 30, "none"),
    "J5_wrist_roll": _mk("J5_wrist_roll", 1, 0.75, 0.03, 0.18, 0.0, 0.0, 307,
                         6, 30, "none"),
    "J6_gripper": _mk("J6_gripper", 0, 0.90, 0.02, 0.10, 0.0, 0.0, 307, 5,
                      25, "none"),
}


# ---------------------------------------------------------------------------
# Nonlinear discrete PID controller
# ---------------------------------------------------------------------------


class NonlinearPIDController:
    """Implements the Section 4.3 nonlinear discrete PID law."""

    def __init__(self, config: JointPIDConfig):
        self.config = config
        self.integral: float = 0.0
        self.last_error: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.last_error = 0.0

    @staticmethod
    def sign_error(e: float, tol: float) -> int:
        if e > tol:
            return 1
        if e < -tol:
            return -1
        return 0

    def gravity_feedforward(self, theta2: float, theta3: float,
                            theta4: float = 0.0,
                            mode: Optional[str] = None) -> float:
        mode = mode or self.config.gravity_mode
        if mode == "shoulder":
            kg2 = self.config.kg
            kg3 = self.config.kg_coupled if self.config.kg_coupled else self.config.kg
            return (kg2 * math.sin(math.radians(theta2))
                    + kg3 * math.sin(math.radians(theta2 + theta3)))
        if mode == "elbow":
            return self.config.kg * math.sin(math.radians(theta2 + theta3))
        return 0.0

    def compute(self, theta_ref: float, theta_vis: float,
                omega_vis: float = 0.0, dt: float = 0.0,
                theta2: float = 0.0, theta3: float = 0.0,
                theta4: float = 0.0,
                gravity_mode: Optional[str] = None,
                shoulder_coupling: Optional[int] = None) -> float:
        cfg = self.config
        tol = cfg.tol_deg
        e = float(theta_ref) - float(theta_vis)

        if abs(e) <= tol:
            # Inside lock window: silent motor, freeze integral.
            s = 0
        else:
            s = 1 if e > tol else -1
            if dt and dt > 0:
                self.integral += e * dt

        # Anti-windup: back-calculate integral clamp contribution.
        grav = self.gravity_feedforward(theta2, theta3, theta4,
                                       mode=gravity_mode)

        offset = (s * cfg.du_dead
                  + cfg.kp * e
                  + cfg.ki * self.integral
                  + cfg.kd * (-float(omega_vis))
                  + grav)

        # Clamp total offset.
        offset = max(-cfg.du_max, min(cfg.du_max, offset))

        u = cfg.u_stop + offset
        # Clamp final register.
        u = max(cfg.u_min, min(cfg.u_max, u))

        if abs(e) <= tol:
            # Force exact stop pulse inside lock window.
            u = float(cfg.u_stop)

        self.last_error = e
        return float(u)

    def update(self, *args, **kwargs) -> float:
        return self.compute(*args, **kwargs)


# ---------------------------------------------------------------------------
# Shoulder coupling helper
# ---------------------------------------------------------------------------


def apply_shoulder_coupling(u_ch4: float, u_stop_ch4: int = U_STOP_DEFAULT,
                            u_stop_ch5: int = U_STOP_DEFAULT,
                            coupling: int = 1) -> float:
    delta = float(u_ch4) - float(u_stop_ch4)
    if coupling == 1:
        return float(u_stop_ch5) + delta
    return float(u_stop_ch5) - delta


# ---------------------------------------------------------------------------
# 20-byte binary frame codec
# ---------------------------------------------------------------------------


def crc8_xor(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc & 0xFF


def crc8_polynomial(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF


def crc8(data: bytes, poly: int = 0x00) -> int:
    if poly == 0x00:
        return crc8_xor(data)
    return crc8_polynomial(data, poly=poly)


def pack_binary_frame(pulses_ch0_to_ch7: Sequence[int],
                      oe_enabled: bool = False,
                      estop: bool = False,
                      homing: bool = False) -> bytes:
    if len(pulses_ch0_to_ch7) != 8:
        raise ValueError("pulses_ch0_to_ch7 must contain 8 values")
    clamped = []
    for p in pulses_ch0_to_ch7:
        v = int(round(float(p)))
        v = max(U_REG_MIN, min(U_REG_MAX, v))
        clamped.append(v)
    flags = 0
    if not oe_enabled:
        flags |= FLAG_OE_DISABLE
    if estop:
        flags |= FLAG_EMERGENCY_STOP
    if homing:
        flags |= FLAG_HOMING_MODE
    # crc input is bytes 2..18 = flags + 8 uint16
    crc = crc8_xor(struct.pack("<B8H", flags, *clamped))
    frame = struct.pack("<BB B 8H B", 0xAA, 0x55, flags, *clamped, crc)
    return frame


def unpack_binary_frame(frame: bytes) -> Dict[str, Any]:
    if not isinstance(frame, (bytes, bytearray)):
        raise ValueError("frame must be bytes")
    if len(frame) != 20:
        raise ValueError("frame must be 20 bytes")
    if frame[0] != 0xAA or frame[1] != 0x55:
        raise ValueError("invalid frame header")
    flags = frame[2]
    pulses = list(struct.unpack("<8H", frame[3:19]))
    crc_calc = crc8_xor(frame[2:19])
    if frame[19] != crc_calc:
        raise ValueError("CRC mismatch")
    return {
        "header": (frame[0], frame[1]),
        "flags": flags,
        "pulses": pulses,
        "pulses_ch0_to_ch7": pulses,
        "channels": pulses,
        "crc": frame[19],
        "crc_ok": True,
        "oe_enabled": not bool(flags & FLAG_OE_DISABLE),
        "oe_disabled": bool(flags & FLAG_OE_DISABLE),
        "estop": bool(flags & FLAG_EMERGENCY_STOP),
        "emergency_stop": bool(flags & FLAG_EMERGENCY_STOP),
        "homing": bool(flags & FLAG_HOMING_MODE),
        "homing_mode": bool(flags & FLAG_HOMING_MODE),
    }


# ---------------------------------------------------------------------------
# Safe visual homing state machine
# ---------------------------------------------------------------------------


class DecisionGate:
    """Require stable audits and provide a bounded optical probe pulse."""

    def __init__(self, max_audit_variance_deg2: float = 1.5,
                 required_consecutive_passes: int = 3,
                 probe_uncertainty_threshold_deg2: float = 0.15,
                 probe_duration_ms: int = 40,
                 probe_step_deg: float = 1.0):
        self.max_audit_variance_deg2 = float(max_audit_variance_deg2)
        self.required_consecutive_passes = max(1, int(required_consecutive_passes))
        self.probe_uncertainty_threshold_deg2 = float(probe_uncertainty_threshold_deg2)
        self.probe_duration_ms = int(probe_duration_ms)
        self.probe_step_deg = float(probe_step_deg)
        self.consecutive_passes = 0

    def record_audit(self, audit_or_frame_result: dict) -> dict:
        passed = audit_or_frame_result.get("passed")
        if passed is None:
            variance = audit_or_frame_result.get("max_variance_deg2")
            passed = (variance is not None
                      and float(variance) < self.max_audit_variance_deg2)
        if bool(passed):
            self.consecutive_passes += 1
        else:
            self.consecutive_passes = 0
        can_enable_oe = self.consecutive_passes >= self.required_consecutive_passes
        return {
            "can_enable_oe": can_enable_oe,
            "consecutive_passes": self.consecutive_passes,
            "oe_command": "R" if can_enable_oe else "S",
        }

    def reset(self) -> None:
        self.consecutive_passes = 0

    def needs_probe_pulse(self, joint_variance_deg2: float) -> bool:
        variance = float(joint_variance_deg2)
        return (self.probe_uncertainty_threshold_deg2 <= variance
                < self.max_audit_variance_deg2)

    def build_probe_pulse(self, motor: int, direction: int = 1,
                          pulse_odometry: Optional[Any] = None,
                          current_deg: float = 90.0) -> dict:
        signed_direction = 1 if direction >= 0 else -1
        odometry = pulse_odometry
        if odometry is None:
            try:
                from src.pulse_odometry import PulseOdometry
            except ImportError:
                from pulse_odometry import PulseOdometry  # type: ignore
            odometry = PulseOdometry()

        pulse = None
        offset = None
        try:
            current_pulse = int(odometry.deg_to_pulse(motor, float(current_deg)))
            target_position_pulse = int(odometry.deg_to_pulse(
                motor, float(current_deg) + signed_direction * self.probe_step_deg
            ))
            calibration = getattr(odometry, "_calib", {}).get(motor, {})
            stop = int(round(float(calibration.get("pulse_stop", U_STOP_DEFAULT))))
            offset = target_position_pulse - current_pulse
            offset = max(U_REG_MIN - stop, min(U_REG_MAX - stop, offset))
            pulse = stop + offset
        except (AttributeError, KeyError, TypeError, ValueError):
            offset = signed_direction * 6
            pulse = U_STOP_DEFAULT + offset
        command = f"T{motor}:{offset}:{self.probe_duration_ms}"
        return {
            "motor": motor,
            "direction": signed_direction,
            "duration_ms": self.probe_duration_ms,
            "step_deg": self.probe_step_deg,
            "pulse": pulse,
            "offset": offset,
            "command": command,
        }

    def verify_probe_response(self, theta_before_deg: float,
                              theta_after_deg: float,
                              expected_direction: int,
                              min_delta_deg: float = 0.15) -> dict:
        delta = float(theta_after_deg) - float(theta_before_deg)
        verified = delta * expected_direction >= min_delta_deg
        return {
            "verified": verified,
            "delta_deg": delta,
            "action": "PROCEED_FULL_PID" if verified else "ABORT_HOLD_HIGH",
        }


class SafeVisualHomingMachine:
    PHASE_0_INHIBITED = PHASE_0_INHIBITED
    PHASE_1_OPTICAL_AUDIT = PHASE_1_OPTICAL_AUDIT
    PHASE_2_NEUTRAL_PRELOAD = PHASE_2_NEUTRAL_PRELOAD
    PHASE_3_SEQUENTIAL_TOP_DOWN = PHASE_3_SEQUENTIAL_TOP_DOWN
    PHASE_4_HOMING_LOCKED = PHASE_4_HOMING_LOCKED

    def __init__(self, u_stop: int = U_STOP_DEFAULT, num_channels: int = 8,
                 frame_threshold: int = 30, variance_threshold: float = 0.15,
                 lock_tolerance: float = 0.5, lock_duration_s: float = 0.5):
        self.u_stop = u_stop
        self.num_channels = num_channels
        self.frame_threshold = frame_threshold
        self.variance_threshold = variance_threshold
        self.lock_tolerance = lock_tolerance
        self.lock_duration_s = lock_duration_s
        self.decision_gate = DecisionGate()

        self.phase: str = self.PHASE_0_INHIBITED
        self.oe_enabled: bool = False
        self.pulses: List[int] = [u_stop] * num_channels
        self.valid_frame_streak: int = 0
        self.homing_locked: bool = False
        self.lock_timer_s: float = 0.0
        self.bringup_order: List[Any] = [0, 1, 2, 3, [4, 5], 6]
        self.current_joint_index: int = 0
        self._update_active()

    # ---- properties ----
    @property
    def outputs_enabled(self) -> bool:
        return self.oe_enabled

    @outputs_enabled.setter
    def outputs_enabled(self, v: bool) -> None:
        self.oe_enabled = bool(v)

    @property
    def current_joint(self) -> Any:
        if self.current_joint_index < len(self.bringup_order):
            return self.bringup_order[self.current_joint_index]
        return None

    @property
    def active_channels(self) -> List[int]:
        item = self.current_joint
        if isinstance(item, list):
            return list(item)
        if item is None:
            return []
        return [item]

    @property
    def active_motor(self) -> Optional[int]:
        chs = self.active_channels
        if not chs:
            return None
        return chs[0] + 1

    def _update_active(self) -> None:
        pass

    # ---- phase 0 ----
    def inhibit(self) -> None:
        self.phase = self.PHASE_0_INHIBITED
        self.oe_enabled = False
        self.pulses = [self.u_stop] * self.num_channels
        self.homing_locked = False
        self.decision_gate.reset()

    # ---- phase 1 ----
    def start_optical_audit(self) -> None:
        if self.phase == self.PHASE_0_INHIBITED:
            self.phase = self.PHASE_1_OPTICAL_AUDIT
            self.valid_frame_streak = 0
            self.decision_gate.reset()

    def begin_optical_audit(self) -> None:
        self.start_optical_audit()

    def begin(self) -> None:
        self.start_optical_audit()

    def start(self) -> None:
        self.start_optical_audit()

    # ---- phase 1 feed ----
    def feed_frame(self, variance: Optional[float] = None,
                   valid: bool = True,
                   camera_variance: Optional[float] = None) -> None:
        if variance is None:
            variance = camera_variance
        if self.phase == self.PHASE_0_INHIBITED:
            self.start_optical_audit()
        if self.phase != self.PHASE_1_OPTICAL_AUDIT:
            return
        self.decision_gate.record_audit({
            "passed": bool(valid and variance is not None
                            and variance < self.decision_gate.max_audit_variance_deg2)
        })
        if valid and variance is not None and variance < self.variance_threshold:
            self.valid_frame_streak += 1
        else:
            if variance is not None:
                self.valid_frame_streak = 0
        if self.valid_frame_streak >= self.frame_threshold:
            self.neutral_preload()

    def feed_camera_frame(self, *a, **kw) -> None:
        self.feed_frame(*a, **kw)

    def on_camera_frame(self, *a, **kw) -> None:
        self.feed_frame(*a, **kw)

    def camera_frame(self, *a, **kw) -> None:
        self.feed_frame(*a, **kw)

    # ---- phase 2 ----
    def neutral_preload(self) -> None:
        self.pulses = [self.u_stop] * self.num_channels
        self.oe_enabled = (
            self.decision_gate.consecutive_passes
            >= self.decision_gate.required_consecutive_passes
        )
        # Manual phase transitions remain possible, but can never energize
        # outputs without the consecutive audit approvals.
        if self.phase in (self.PHASE_1_OPTICAL_AUDIT, self.PHASE_0_INHIBITED):
            self.phase = self.PHASE_2_NEUTRAL_PRELOAD

    def begin_neutral_preload(self) -> None:
        self.neutral_preload()

    # ---- phase 3 ----
    def begin_bringup(self) -> None:
        if self.phase in (self.PHASE_2_NEUTRAL_PRELOAD,
                          self.PHASE_1_OPTICAL_AUDIT):
            self.phase = self.PHASE_3_SEQUENTIAL_TOP_DOWN
            self.current_joint_index = 0

    def start_sequential_bringup(self) -> None:
        self.begin_bringup()

    def start_phase3(self) -> None:
        self.begin_bringup()

    def advance_joint(self) -> None:
        if self.phase != self.PHASE_3_SEQUENTIAL_TOP_DOWN:
            return
        if self.current_joint_index < len(self.bringup_order) - 1:
            self.current_joint_index += 1

    def next_joint(self) -> None:
        self.advance_joint()

    def next_phase(self) -> None:
        # manual advance through phases
        order = [self.PHASE_0_INHIBITED, self.PHASE_1_OPTICAL_AUDIT,
                 self.PHASE_2_NEUTRAL_PRELOAD,
                 self.PHASE_3_SEQUENTIAL_TOP_DOWN,
                 self.PHASE_4_HOMING_LOCKED]
        idx = order.index(self.phase) if self.phase in order else 0
        if idx < len(order) - 1:
            nxt = order[idx + 1]
            if nxt == self.PHASE_1_OPTICAL_AUDIT:
                self.start_optical_audit()
            elif nxt == self.PHASE_2_NEUTRAL_PRELOAD:
                self.neutral_preload()
            elif nxt == self.PHASE_3_SEQUENTIAL_TOP_DOWN:
                self.begin_bringup()
            elif nxt == self.PHASE_4_HOMING_LOCKED:
                self.homing_locked = True
                self.phase = self.PHASE_4_HOMING_LOCKED

    # ---- phase 4 ----
    def update_lock(self, errors: Optional[Sequence[float]] = None,
                    dt: float = 0.0,
                    max_error: Optional[float] = None) -> None:
        if errors is not None and max_error is None:
            try:
                max_error = max(abs(float(x)) for x in errors)
            except (TypeError, ValueError):
                max_error = 0.0
        if max_error is None:
            return
        if self.phase != self.PHASE_3_SEQUENTIAL_TOP_DOWN:
            if max_error <= self.lock_tolerance:
                self.lock_timer_s += dt
            else:
                self.lock_timer_s = 0.0
            return
        if max_error <= self.lock_tolerance:
            self.lock_timer_s += dt
            if self.lock_timer_s >= self.lock_duration_s:
                self.homing_locked = True
                self.phase = self.PHASE_4_HOMING_LOCKED
        else:
            self.lock_timer_s = 0.0

    # ---- generic update ----
    def update(self, camera_variance: Optional[float] = None,
               variance: Optional[float] = None,
               valid_frame: bool = True,
               dt: float = 0.0,
               angles: Optional[Sequence[float]] = None,
               errors: Optional[Sequence[float]] = None,
               theta_errors: Optional[Sequence[float]] = None,
               max_error: Optional[float] = None,
               start: bool = False,
               **kwargs) -> str:
        if start:
            if self.phase == self.PHASE_0_INHIBITED:
                self.start_optical_audit()
        v = variance if variance is not None else camera_variance
        if v is not None:
            self.feed_frame(v, valid=valid_frame)
        if self.phase == self.PHASE_2_NEUTRAL_PRELOAD:
            # auto-advance to bringup on next update after preload
            if angles is not None or dt > 0 or kwargs.get("bringup", False):
                self.begin_bringup()
        errs = errors if errors is not None else theta_errors
        if errs is not None or max_error is not None:
            self.update_lock(errors=errs, dt=dt, max_error=max_error)
        return self.phase

    def tick(self, *a, **kw) -> str:
        return self.update(*a, **kw)

    def step(self, *a, **kw) -> str:
        return self.update(*a, **kw)


# Backwards-friendly aliases
JointConfig = JointPIDConfig
PIDController = NonlinearPIDController
HomingMachine = SafeVisualHomingMachine


if __name__ == "__main__":
    # Self-check
    import math as _m

    # deadband jump + lock
    cfg = DEFAULT_JOINT_CONFIGS["J6_gripper"]
    ctrl = NonlinearPIDController(cfg)
    out = ctrl.compute(theta_ref=10.0, theta_vis=0.0, omega_vis=0.0, dt=0.02)
    assert out != cfg.u_stop, out
    ctrl2 = NonlinearPIDController(cfg)
    out2 = ctrl2.compute(theta_ref=0.0, theta_vis=0.0, omega_vis=0.0, dt=0.02)
    assert out2 == cfg.u_stop, out2

    # gravity feedforward shoulder
    scfg = DEFAULT_JOINT_CONFIGS["J2_shoulder"]
    sctrl = NonlinearPIDController(scfg)
    g = sctrl.gravity_feedforward(theta2=90.0, theta3=0.0)
    assert abs(g - (4.5 * _m.sin(_m.radians(90.0)) + 2.8 * _m.sin(_m.radians(90.0)))) < 1e-9, g

    # binary frame
    frame = pack_binary_frame([307] * 8, oe_enabled=False, estop=False, homing=True)
    assert len(frame) == 20, len(frame)
    assert frame[0] == 0xAA and frame[1] == 0x55
    info = unpack_binary_frame(frame)
    assert info["pulses"] == [307] * 8
    assert info["homing"] is True
    # CRC tamper
    bad = bytearray(frame)
    bad[5] ^= 0xFF
    try:
        unpack_binary_frame(bytes(bad))
        raise AssertionError("expected ValueError")
    except ValueError:
        pass

    # state machine transitions
    m = SafeVisualHomingMachine()
    assert m.phase == PHASE_0_INHIBITED
    m.start_optical_audit()
    assert m.phase == PHASE_1_OPTICAL_AUDIT
    for _ in range(30):
        m.feed_frame(0.05)
    assert m.phase == PHASE_2_NEUTRAL_PRELOAD, m.phase
    assert m.oe_enabled is True
    m.begin_bringup()
    assert m.phase == PHASE_3_SEQUENTIAL_TOP_DOWN
    assert m.active_channels == [0]
    # shoulder pair is index 4 in bringup order
    while m.current_joint_index < 4:
        m.advance_joint()
    assert m.active_channels == [4, 5], m.active_channels
    m.update_lock(errors=[0.1, -0.2], dt=0.5, max_error=None)
    # need 500ms continuous; single 0.5s enough
    assert m.homing_locked is True
    assert m.phase == PHASE_4_HOMING_LOCKED

    # shoulder coupling
    assert apply_shoulder_coupling(315, 307, 307, 1) == 315
    assert apply_shoulder_coupling(315, 307, 307, -1) == 299

    print("self-check OK")
