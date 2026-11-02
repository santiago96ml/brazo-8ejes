#!/usr/bin/env python
"""Safely bring up and optically test one arm motor at a time."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Any

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from src.fast_camera import FastUSBCamera
from src.pid_pwm_controller import DecisionGate
from src.pulse_odometry import PulseOdometry
from src.virtual_optical_encoder import VirtualOpticalEncoder


_MOTOR_JOINTS = {
    1: (5, "theta6_gripper"),
    2: (4, "theta5_wrist_roll"),
    3: (3, "theta4_wrist_pitch"),
    4: (2, "theta3_elbow"),
    5: (1, "theta2_shoulder"),
    6: (1, "theta2_shoulder"),
    7: (0, "theta1_base"),
}


def _open_transport(port: str):
    from host.armctl.transport import SerialTransport

    return SerialTransport(port, 115200)


def _joint_angle(reading: dict, motor: int) -> float:
    if motor not in _MOTOR_JOINTS:
        raise ValueError("Motor 8 (AUX) has no optical joint angle and cannot be verified safely")
    index, _name = _MOTOR_JOINTS[motor]
    motors = reading.get("motors_deg", {})
    channel_motor = 5 if motor == 6 else motor
    if channel_motor in motors:
        return float(motors[channel_motor])
    if str(channel_motor) in motors:
        return float(motors[str(channel_motor)])
    thetas = reading.get("thetas_deg", [])
    if len(thetas) > index:
        return float(thetas[index])
    raise ValueError(f"Optical reading does not contain an angle for motor {motor}")


def _wrap_delta(delta: float) -> float:
    return (float(delta) + 180.0) % 360.0 - 180.0


def _read_frame(camera: Any, timeout_s: float = 5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame, timestamp, _fps = camera.get_latest_frame()
        if frame is not None:
            return frame, timestamp
        time.sleep(0.005)
    raise RuntimeError("Camera did not publish a frame")


def _read_angle(camera: Any, encoder: Any, motor: int) -> tuple[float, Any]:
    frame, _timestamp = _read_frame(camera)
    reading = encoder.read_positions(frame)
    return _joint_angle(reading, motor), reading


def _wait_for_new_angle(camera: Any, encoder: Any, motor: int, previous_timestamp: Any,
                        timeout_s: float = 2.0) -> tuple[float, Any, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame, timestamp, _fps = camera.get_latest_frame()
        if frame is not None and timestamp != previous_timestamp:
            reading = encoder.read_positions(frame)
            return _joint_angle(reading, motor), timestamp, reading
        time.sleep(0.005)
    raise RuntimeError("Camera did not publish a new frame")


def _measure_response(camera: Any, encoder: Any, motor: int, baseline_deg: float,
                      burst_ms: int) -> float:
    fps = 30.0
    try:
        fps = max(1.0, float(camera.stats().get("fps") or 30.0))
    except Exception:
        pass
    sample_count = max(2, int(math.ceil((int(burst_ms) + 300) * fps / 1000.0)))
    last_timestamp = None
    angles = []
    deadline = time.monotonic() + max(2.0, (int(burst_ms) + 300) / 1000.0 + 2.0)
    while len(angles) < sample_count and time.monotonic() < deadline:
        frame, timestamp, _frame_fps = camera.get_latest_frame()
        if frame is not None and timestamp != last_timestamp:
            reading = encoder.read_positions(frame)
            angles.append(_joint_angle(reading, motor))
            last_timestamp = timestamp
        else:
            time.sleep(0.005)
    if not angles:
        raise RuntimeError("Camera produced no response frames")
    measured = angles[-1] - baseline_deg
    if motor in (2, 7):
        measured = _wrap_delta(measured)
    return float(measured)


def _save_snapshot(frame: Any, path: str | None) -> str | None:
    if not path or frame is None:
        return None
    try:
        import cv2

        parent = os.path.dirname(os.path.abspath(path))
        os.makedirs(parent, exist_ok=True)
        if cv2.imwrite(path, frame):
            return path
    except Exception:
        pass
    return None


def run_single_motor_test(
    *,
    motor: int = 1,
    port: str = "COM4",
    camera_index: int = 0,
    frames: int = 30,
    delta_deg: float = 8.0,
    burst_ms: int = 400,
    probe_ms: int = 40,
    dry_run: bool = False,
    snapshot: str | None = None,
    camera: Any = None,
    encoder: Any = None,
    transport: Any = None,
    odometry: Any = None,
) -> dict:
    """Run one optical-gated motor test; injectable devices keep tests offline."""
    motor = int(motor)
    if motor < 1 or motor > 8:
        raise ValueError("Motor must be 1..8")
    if motor not in _MOTOR_JOINTS:
        raise ValueError("Motor 8 (AUX) has no optical joint angle and cannot be verified safely")
    if int(frames) < 1:
        raise ValueError("frames must be at least 1")
    if int(burst_ms) < 1 or int(probe_ms) < 1:
        raise ValueError("pulse durations must be positive")

    encoder = encoder or VirtualOpticalEncoder()
    odometry = odometry or PulseOdometry()
    camera = camera or FastUSBCamera(index=int(camera_index), width=1280, height=720, fps=30)
    owns_transport = transport is None and not dry_run
    if owns_transport:
        transport = _open_transport(port)

    decision_gate = DecisionGate(
        max_audit_variance_deg2=1.5,
        required_consecutive_passes=3,
        probe_duration_ms=int(probe_ms),
    )
    oe_sequence: list[str] = []
    move_attempted = False
    brake_sent = False
    last_frame = None

    def send(command: str) -> None:
        if dry_run:
            return
        if transport is None:
            raise RuntimeError("Serial transport is unavailable")
        transport.send_line(command)
        if command == "S":
            oe_sequence.append("HIGH")
            encoder.set_oe_state(oe_high=True, active_command=False)
        elif command == "R":
            oe_sequence.append("LOW")
            encoder.set_oe_state(oe_high=False, active_command=True)

    result = {
        "status": "ABORT_AUDIT",
        "motor": motor,
        "audit": {},
        "decision_gate": {},
        "baseline": {},
        "commanded": {"offset": 0, "ms": int(burst_ms)},
        "measured_delta_deg": None,
        "direction_ok": False,
        "oe_sequence": oe_sequence,
        "snapshot": None,
    }
    try:
        # Establish OE=HIGH on the controller before opening or sampling the camera.
        if not dry_run:
            send("S")
        camera.start()

        audits = []
        gate_results = []
        for _ in range(3):
            try:
                audit = encoder.audit_pre_power(
                    camera, num_frames=int(frames), max_variance_deg=1.5
                )
            except Exception as exc:
                audit = {
                    "passed": False,
                    "max_variance_deg2": 999.0,
                    "variances_deg2": [999.0] * 6,
                    "reason": f"error:{exc}",
                    "frames_evaluated": 0,
                    "mode_used": "none",
                    "visible_marker_ids": [],
                }
            audits.append(audit)
            gate_results.append(decision_gate.record_audit(audit))
        last_audit = audits[-1]
        max_variance = float(last_audit.get("max_variance_deg2", 999.0))
        result["audit"] = {
            "passed": all(bool(item.get("passed")) for item in audits),
            "max_variance_deg2": max_variance,
            "frames_evaluated": [int(item.get("frames_evaluated", 0)) for item in audits],
            "mode_used": last_audit.get("mode_used", "none"),
            "visible_marker_ids": last_audit.get("visible_marker_ids", []),
            "reason": last_audit.get("reason"),
            "results": [
                {
                    "passed": bool(item.get("passed")),
                    "max_variance_deg2": float(item.get("max_variance_deg2", 999.0)),
                }
                for item in audits
            ],
        }
        can_enable = bool(gate_results[-1].get("can_enable_oe"))
        variances = last_audit.get("variances_deg2", [max_variance] * 6)
        joint_index, joint_name = _MOTOR_JOINTS[motor]
        joint_variance = float(variances[joint_index]) if len(variances) > joint_index else max_variance
        needs_probe = decision_gate.needs_probe_pulse(joint_variance)
        result["decision_gate"] = {
            "can_enable_oe": can_enable,
            "consecutive_passes": decision_gate.consecutive_passes,
            "required_consecutive_passes": 3,
            "needs_probe_pulse": needs_probe,
            "joint_variance_deg2": joint_variance,
            "audit_results": gate_results,
        }
        if not can_enable:
            result["status"] = "ABORT_AUDIT"
            return result

        last_frame, last_timestamp = _read_frame(camera)
        # Keep the full optical reading in the baseline report for diagnosis.
        baseline_reading = encoder.read_positions(last_frame)
        baseline_deg = _joint_angle(baseline_reading, motor)
        result["baseline"] = {
            "joint": joint_name,
            "angle_deg": baseline_deg,
            "thetas_deg": baseline_reading.get("thetas_deg", []),
            "motors_deg": baseline_reading.get("motors_deg", {}),
        }
        result["snapshot"] = _save_snapshot(last_frame, snapshot)

        # The probe pulse only produces motion with the outputs enabled, so OE must be
        # lowered BEFORE probing. The 3-consecutive-pass gate above already authorized it.
        if not dry_run:
            send("R")
            time.sleep(0.150)
        encoder.set_oe_state(oe_high=False, active_command=True)

        probe = None
        if needs_probe:
            probe_motor = 5 if motor == 6 else motor
            probe = decision_gate.build_probe_pulse(
                probe_motor, direction=1 if float(delta_deg) >= 0 else -1,
                pulse_odometry=odometry, current_deg=baseline_deg,
            )
            if dry_run:
                result["planned_probe"] = {
                    "offset": int(probe["offset"]), "ms": int(probe_ms),
                    "command": probe["command"],
                }
            else:
                send(probe["command"])
                time.sleep(int(probe_ms) / 1000.0)
                after_probe, probe_timestamp, _probe_reading = _wait_for_new_angle(
                    camera, encoder, motor, last_timestamp
                )
                probe_check = decision_gate.verify_probe_response(
                    baseline_deg, after_probe, expected_direction=probe["direction"]
                )
                result["decision_gate"]["probe_response"] = probe_check
                encoder.set_oe_state(oe_high=True, active_command=False)
                if not probe_check.get("verified", False):
                    result["status"] = "ABORT_PROBE"
                    return result
                baseline_deg = after_probe
                last_timestamp = probe_timestamp
                result["baseline"]["pre_probe_angle_deg"] = result["baseline"]["angle_deg"]
                result["baseline"]["angle_deg"] = baseline_deg

        command_motor = 5 if motor == 6 else motor
        target_deg = baseline_deg + float(delta_deg)
        offset = int(odometry.deg_to_pulse(command_motor, target_deg)) - int(
            odometry.deg_to_pulse(command_motor, baseline_deg)
        )
        offset = max(-150, min(150, offset))
        command = f"T{command_motor}:{offset}:{int(burst_ms)}"
        result["commanded"] = {"motor": command_motor, "offset": offset, "ms": int(burst_ms)}
        if dry_run:
            planned = []
            if probe is not None:
                planned.append(probe["command"])
            planned.extend(["R", command, f"T{command_motor}:0:1", "S"])
            result["planned_commands"] = planned
            result["status"] = "OK"
            return result

        send(command)
        move_attempted = True
        measured_delta = _measure_response(camera, encoder, motor, baseline_deg, int(burst_ms))
        result["measured_delta_deg"] = measured_delta
        result["direction_ok"] = bool(
            measured_delta > 0 if float(delta_deg) >= 0 else measured_delta < 0
        )
        result["status"] = "NO_MOTION" if abs(measured_delta) < 0.3 else "OK"
        send(f"T{command_motor}:0:1")
        brake_sent = True
        return result
    finally:
        if move_attempted and not brake_sent and not dry_run:
            try:
                send(f"T{5 if motor == 6 else motor}:0:1")
            except Exception:
                pass
        if not dry_run and transport is not None:
            try:
                send("S")
            except Exception:
                pass
        try:
            camera.stop()
        except Exception:
            pass
        if owns_transport and transport is not None:
            try:
                transport.close()
            except Exception:
                pass


def _exit_code(result: dict) -> int:
    return {"OK": 0, "NO_MOTION": 4, "ABORT_AUDIT": 2, "ABORT_PROBE": 3}.get(
        result.get("status"), 1
    )


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return 999.0
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Safely test one motor with an optical audit")
    parser.add_argument("--motor", type=int, choices=range(1, 9), default=1)
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--delta-deg", type=float, default=8.0)
    parser.add_argument("--burst-ms", type=int, default=400)
    parser.add_argument("--probe-ms", type=int, default=40)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument(
        "--snapshot", nargs="?", const="tools/camera_previews/single_motor_test.jpg",
        default=None, help="Save an optical frame (optionally provide an output path)",
    )
    parser.add_argument("--sequence", action="store_true", help="Test motors 1, 2, 3, 4, 5, and 7")
    args = parser.parse_args(argv)
    motors = [1, 2, 3, 4, 5, 7] if args.sequence else [args.motor]
    exit_code = 0
    for motor in motors:
        snapshot = args.snapshot
        if snapshot and args.sequence:
            root, ext = os.path.splitext(snapshot)
            snapshot = f"{root}_motor{motor}{ext or '.jpg'}"
        try:
            result = run_single_motor_test(
                motor=motor, port=args.port, camera_index=args.camera,
                frames=args.frames, delta_deg=args.delta_deg,
                burst_ms=args.burst_ms, probe_ms=args.probe_ms,
                dry_run=args.dry_run, snapshot=snapshot,
            )
        except Exception as exc:
            result = {
                "status": "ABORT_AUDIT", "motor": motor,
                "audit": {"reason": f"error:{exc}"},
                "decision_gate": {}, "baseline": {},
                "commanded": {"offset": 0, "ms": args.burst_ms},
                "measured_delta_deg": None, "direction_ok": False,
                "oe_sequence": [], "snapshot": None,
            }
            exit_code = max(exit_code, 2)
        if args.json or args.sequence:
            print(json.dumps(_json_safe(result), sort_keys=True))
        else:
            print(f"Motor {motor}: {result['status']}")
            print(json.dumps(_json_safe(result), sort_keys=True, indent=2))
        exit_code = max(exit_code, _exit_code(result))
        if result.get("status") in ("ABORT_AUDIT", "ABORT_PROBE", "NO_MOTION"):
            break
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
