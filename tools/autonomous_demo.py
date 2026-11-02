#!/usr/bin/env python
"""Supervised autonomous demo for the 6-DOF arm."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Any, Dict, List

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

STEP_NAMES = (
    "connect",
    "read_start_pose",
    "base_sweep",
    "shoulder_lift",
    "elbow",
    "gripper_salute",
    "move_to_xyz",
    "return_to_start",
)


def _wrap_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def _log_step(report: Dict[str, Any], number: int, name: str,
              result: Dict[str, Any], value: Any = 0.0) -> str:
    status = str(result.get("status", ""))
    if status not in ("ok", "drift", "fail"):
        status = "ok" if result.get("ok", False) else "fail"
    print(f"STEP {number}/{len(STEP_NAMES)} {name} {status} {value}")
    report["steps"].append({
        "name": name,
        "ok": bool(result.get("ok", False)),
        "status": status,
        "commanded_deg": result.get("commanded_deg"),
        "measured_deg": result.get("measured_deg"),
        "drift_deg": result.get("drift_deg"),
        **{key: result[key] for key in ("target_mm", "measured_mm", "error_mm")
           if key in result},
        **({"moves": result["moves"]} if "moves" in result else {}),
    })
    return status


def _move_delta(controller, mode: str, joint_index: int, delta_deg: float,
                settle_s: float = 0.9) -> Dict[str, Any]:
    if mode == "open-loop":
        return controller.move_delta_openloop(
            joint_index, delta_deg, settle_s=settle_s
        )

    before = controller.read_state(wait_fresh=True, timeout_s=1.5)
    if not before.get("valid"):
        return {
            "ok": False,
            "reason": "invalid_start_pose",
            "commanded_deg": float(delta_deg),
            "measured_deg": None,
            "drift_deg": None,
        }
    target = list(before["thetas_deg"])
    target[joint_index] += float(delta_deg)
    result = controller.move_to_thetas(target, joint_mask=[joint_index])
    after = controller.read_state(wait_fresh=True, timeout_s=1.5)
    measured = None
    drift = None
    if after.get("valid"):
        measured = _wrap_deg(
            float(after["thetas_deg"][joint_index])
            - float(before["thetas_deg"][joint_index])
        )
        drift = _wrap_deg(measured - float(delta_deg))
    return {
        **result,
        "reason": result.get("reason", "closed_loop"),
        "verified": measured is not None,
        "commanded_deg": float(delta_deg),
        "measured_deg": measured,
        "drift_deg": drift,
    }


def _run_moves(controller, mode: str, moves: List[tuple[int, float]],
               settle_s: float = 0.9) -> Dict[str, Any]:
    results = []
    tolerated_drift = False
    for joint_index, delta in moves:
        result = _move_delta(controller, mode, joint_index, delta, settle_s)
        result["joint_index"] = joint_index
        results.append(result)
        if not result.get("ok"):
            if result.get("reason") == "drift":
                tolerated_drift = True
                continue
            return {
                "ok": False,
                "reason": result.get("reason", "move_failed"),
                "status": "fail",
                "commanded_deg": [item.get("commanded_deg") for item in results],
                "measured_deg": [item.get("measured_deg") for item in results],
                "drift_deg": [item.get("drift_deg") for item in results],
                "moves": results,
            }
    return {
        "ok": True,
        "reason": "drift" if tolerated_drift else "completed",
        "status": "drift" if tolerated_drift else "ok",
        "commanded_deg": [item.get("commanded_deg") for item in results],
        "measured_deg": [item.get("measured_deg") for item in results],
        "drift_deg": [item.get("drift_deg") for item in results],
        "moves": results,
    }


def _summarize_moves(results: List[Dict[str, Any]]) -> Dict[str, Any]:
    hard_failure = next((move for move in results
                         if not move.get("ok") and move.get("reason") != "drift"), None)
    drift = any(move.get("reason") == "drift" for move in results)
    return {
        "ok": hard_failure is None,
        "reason": hard_failure.get("reason") if hard_failure else (
            "drift" if drift else "completed"
        ),
        "status": "fail" if hard_failure else ("drift" if drift else "ok"),
        "commanded_deg": [move.get("commanded_deg") for move in results],
        "measured_deg": [move.get("measured_deg") for move in results],
        "drift_deg": [move.get("drift_deg") for move in results],
        "moves": results,
    }


def _retry_firmware_read(controller, mode: str, number: int, name: str,
                         result: Dict[str, Any], prior_pulse_ack: bool = False
                         ) -> Dict[str, Any]:
    if result.get("ok") or result.get("reason") != "firmware STATE response timed out":
        return result
    moves = list(result.get("moves", []))
    if not moves:
        return result
    failed = moves[-1]
    acknowledged = prior_pulse_ack or failed.get("pulse_acknowledged", False) or any(
        move.get("pulse_acknowledged", False) for move in moves[:-1]
    )
    joint_index = failed.get("joint_index")
    if not acknowledged or joint_index is None:
        return result

    print(f"STEP {number}/{len(STEP_NAMES)} {name} retry_firmware_read")
    retry = _move_delta(
        controller,
        mode,
        int(joint_index),
        float(failed["commanded_deg"]),
    )
    retry["joint_index"] = int(joint_index)
    return {**result, **_summarize_moves(moves[:-1] + [retry])}


def _move_absolute(controller, mode: str, joint_index: int, target_deg: float,
                   settle_s: float = 0.9) -> Dict[str, Any]:
    state = controller.read_state(wait_fresh=True, timeout_s=1.5)
    if not state.get("valid"):
        return {
            "ok": False,
            "reason": "invalid_start_pose",
            "commanded_deg": None,
            "measured_deg": None,
            "drift_deg": None,
        }
    delta = _wrap_deg(float(target_deg) - float(state["thetas_deg"][joint_index]))
    result = _move_delta(controller, mode, joint_index, delta, settle_s)
    result["joint_index"] = joint_index
    return result


def _has_pulse_acknowledgement(result: Dict[str, Any]) -> bool:
    if result.get("pulse_acknowledged"):
        return True
    return any(move.get("pulse_acknowledged", False)
               for move in result.get("moves", []) if isinstance(move, dict))


def _move_to_xyz(controller, mode: str) -> Dict[str, Any]:
    xyz = (150.0, 0.0, 400.0)
    if mode == "open-loop":
        return controller.move_to_xyz_openloop(
            *xyz, pitch_deg=45.0, grip_percent=60.0
        )

    try:
        from src.kinematics_6dof import RobotArm6DOF, WorkspaceCollisionError

        arm = RobotArm6DOF()
        target = arm.inverse_kinematics(
            *xyz, pitch_deg=45.0, grip_percent=60.0
        )
    except WorkspaceCollisionError:
        return {"ok": False, "reason": "unreachable"}
    except Exception as exc:
        return {"ok": False, "reason": str(exc)}
    result = controller.move_to_thetas(target)
    state = controller.read_state(wait_fresh=True, timeout_s=1.5)
    if state.get("valid"):
        measured = [float(value) for value in arm.forward_kinematics(
            state["thetas_deg"]
        )["tcp_mm"]]
        error = math.sqrt(sum((actual - wanted) ** 2
                              for actual, wanted in zip(measured, xyz)))
    else:
        measured = None
        error = None
    return {
        **result,
        "target_mm": list(xyz),
        "measured_mm": measured,
        "error_mm": error,
    }


def _return_to_start(controller, mode: str,
                     start: List[float]) -> Dict[str, Any]:
    if mode == "closed":
        result = controller.move_to_thetas(start)
        state = controller.read_state(wait_fresh=True, timeout_s=1.5)
        errors = (
            [_wrap_deg(target - measured)
             for target, measured in zip(start, state["thetas_deg"])]
            if state.get("valid") else [None] * 6
        )
        current = state.get("thetas_deg", start)
        return {
            **result,
            "ok": bool(result.get("ok")) and all(
                error is not None and abs(error) <= 8.0 for error in errors
            ),
            "final_error_deg": errors,
            "commanded_deg": [_wrap_deg(target - actual)
                              for target, actual in zip(start, current)],
            "measured_deg": None,
            "drift_deg": None,
        }

    state = controller.read_state(wait_fresh=True, timeout_s=1.5)
    if not state.get("valid"):
        return {"ok": False, "reason": "invalid_start_pose", "final_error_deg": []}
    moves = []
    for index, target in enumerate(start):
        delta = _wrap_deg(float(target) - float(state["thetas_deg"][index]))
        if abs(delta) > 0.05:
            moves.append((index, delta))
    moved = _run_moves(controller, mode, moves)
    final_state = controller.read_state(wait_fresh=True, timeout_s=1.5)
    if not final_state.get("valid"):
        return {
            **moved,
            "ok": False,
            "reason": "vision_unavailable",
            "final_error_deg": [],
        }
    errors = [_wrap_deg(target - actual)
              for target, actual in zip(start, final_state["thetas_deg"])]
    return {
        **moved,
        "ok": all(abs(error) <= 8.0 for error in errors),
        "final_error_deg": errors,
        "commanded_deg": moved.get("commanded_deg", []),
        "measured_deg": moved.get("measured_deg", []),
        "drift_deg": moved.get("drift_deg", []),
    }


def _attempt_return_after_abort(report: Dict[str, Any], controller,
                                mode: str, start: List[float]) -> None:
    try:
        result = _return_to_start(controller, mode, start)
    except Exception as exc:
        result = {"ok": False, "reason": str(exc), "final_error_deg": []}
    result["status"] = "ok" if result.get("ok") else "fail"
    _log_step(report, 8, STEP_NAMES[7], result,
              max((abs(value) for value in result.get("final_error_deg", [])
                   if value is not None), default=0.0))
    report["return_to_start"] = {
        "ok": bool(result.get("ok")),
        "final_error_deg": result.get("final_error_deg", []),
    }


class _DryRunController:
    """No-I/O controller used to exercise and report the demo sequence."""

    def __init__(self):
        self.thetas = [0.0, 90.0, 0.0, 0.0, 0.0, 50.0]

    def connect(self):
        return {"ok": True, "reason": "dry_run"}

    def read_state(self, **_kwargs):
        return {
            "valid": True,
            "source": "dry_run",
            "thetas_deg": list(self.thetas),
        }

    def move_delta_openloop(self, joint_index, delta_deg, **_kwargs):
        self.thetas[joint_index] += float(delta_deg)
        return {
            "ok": True,
            "reason": "unverified",
            "verified": False,
            "commanded_deg": delta_deg,
            "measured_deg": None,
            "drift_deg": None,
        }

    def move_to_xyz_openloop(self, x, y, z, **_kwargs):
        return {
            "ok": True,
            "target_mm": [x, y, z],
            "measured_mm": [x, y, z],
            "error_mm": 0.0,
            "thetas": [0.0, 90.0, 0.0, 0.0, 0.0, 50.0],
        }

    def stop(self):
        pass

    def close(self):
        pass


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Autonomous demo")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--control", choices=("open-loop", "closed"),
                        default="open-loop",
                        help="Control strategy: firmware pulse targets (default) or visual PID")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    report: Dict[str, Any] = {
        "status": "OK",
        "control": args.control,
        "steps": [],
        "start_thetas": [],
        "final_thetas": [],
        "max_error_deg": 0.0,
        "duration_s": 0.0,
    }
    started = time.time()
    controller = None
    start = None
    return_attempted = False
    pulse_ack_seen = False
    active_step = 1
    try:
        if args.dry_run:
            controller = _DryRunController()
        else:
            try:
                from src.closed_loop_controller import ClosedLoopArmController
            except Exception as exc:
                print(f"Error: ClosedLoopArmController not available: {exc}", flush=True)
                return 1
            control_mode = "open_loop" if args.control == "open-loop" else "closed_loop"
            controller = ClosedLoopArmController(
                camera=args.camera,
                port=args.port,
                control_mode=control_mode,
            )

        active_step = 1
        connected = controller.connect()
        connect_ok = bool(connected.get("ok", connected.get("status") == "OK"))
        if not connect_ok:
            report["connect_failure"] = {
                "reason": connected.get("reason", "unknown"),
                "audits_count": len(connected.get("audits", [])),
                "gate": connected.get("gate"),
            }
        _log_step(report, 1, STEP_NAMES[0], {"ok": connect_ok})
        if not connect_ok:
            raise RuntimeError("connect_failed")

        active_step = 2
        state = controller.read_state(wait_fresh=True, timeout_s=1.5)
        if not state.get("valid"):
            _log_step(report, 2, STEP_NAMES[1], {"ok": False})
            raise RuntimeError("invalid_start_pose")
        start = [float(value) for value in state["thetas_deg"][:6]]
        report["start_thetas"] = start
        _log_step(report, 2, STEP_NAMES[1], {"ok": True})

        mode = args.control
        active_step = 3
        result = _run_moves(controller, mode, [(0, 12.0), (0, -12.0)])
        result = _retry_firmware_read(
            controller, mode, 3, STEP_NAMES[2], result, pulse_ack_seen
        )
        _log_step(report, 3, STEP_NAMES[2], result)
        if not result["ok"]:
            raise RuntimeError(result.get("reason", "base_sweep_failed"))
        pulse_ack_seen = pulse_ack_seen or _has_pulse_acknowledgement(result)

        active_step = 4
        result = _run_moves(controller, mode, [(1, 10.0), (1, -10.0)])
        result = _retry_firmware_read(
            controller, mode, 4, STEP_NAMES[3], result, pulse_ack_seen
        )
        _log_step(report, 4, STEP_NAMES[3], result)
        if not result["ok"]:
            raise RuntimeError(result.get("reason", "shoulder_lift_failed"))
        pulse_ack_seen = pulse_ack_seen or _has_pulse_acknowledgement(result)

        active_step = 5
        result = _run_moves(controller, mode, [(2, 10.0), (2, -10.0)])
        result = _retry_firmware_read(
            controller, mode, 5, STEP_NAMES[4], result, pulse_ack_seen
        )
        _log_step(report, 5, STEP_NAMES[4], result)
        if not result["ok"]:
            raise RuntimeError(result.get("reason", "elbow_failed"))
        pulse_ack_seen = pulse_ack_seen or _has_pulse_acknowledgement(result)

        active_step = 6
        salute_results = []
        for target in (100.0, 0.0, 50.0):
            move = _move_absolute(controller, mode, 5, target)
            salute_results.append(move)
            if not move.get("ok") and move.get("reason") != "drift":
                result = _summarize_moves(salute_results)
                result = _retry_firmware_read(
                    controller, mode, 6, STEP_NAMES[5], result,
                    pulse_ack_seen or _has_pulse_acknowledgement(
                        {"moves": salute_results[:-1]}
                    ),
                )
                salute_results = result.get("moves", salute_results)
                if result.get("ok"):
                    continue
                break
        result = _summarize_moves(salute_results)
        if len(salute_results) != 3 and result.get("ok"):
            result["ok"] = False
            result["status"] = "fail"
            result["reason"] = "incomplete_gripper_salute"
        _log_step(report, 6, STEP_NAMES[5], result)
        if not result["ok"]:
            raise RuntimeError(result.get("reason", "gripper_salute_failed"))
        pulse_ack_seen = pulse_ack_seen or _has_pulse_acknowledgement(result)

        active_step = 7
        result = _move_to_xyz(controller, mode)
        result = _retry_firmware_read(
            controller, mode, 7, STEP_NAMES[6], result, pulse_ack_seen
        )
        result["status"] = "ok" if result.get("ok") else "fail"
        _log_step(report, 7, STEP_NAMES[6], result, result.get("error_mm", 0.0))
        if not result.get("ok"):
            raise RuntimeError(result.get("reason", "move_to_xyz_failed"))
        pulse_ack_seen = pulse_ack_seen or _has_pulse_acknowledgement(result)

        active_step = 8
        return_attempted = True
        result = _return_to_start(controller, mode, start)
        result = _retry_firmware_read(
            controller, mode, 8, STEP_NAMES[7], result, pulse_ack_seen
        )
        result["status"] = "ok" if result.get("ok") else "fail"
        _log_step(report, 8, STEP_NAMES[7], result,
                  max((abs(value) for value in result.get("final_error_deg", [])
                       if value is not None), default=0.0))
        if not result.get("ok"):
            raise RuntimeError(result.get("reason", "return_to_start_failed"))

        final_state = controller.read_state(wait_fresh=True, timeout_s=1.5)
        report["final_thetas"] = (
            [float(value) for value in final_state["thetas_deg"][:6]]
            if final_state.get("valid") else []
        )
        report["max_error_deg"] = max(
            (abs(value) for value in result.get("final_error_deg", [])
             if value is not None),
            default=0.0,
        )
        report["status"] = "OK"
        report["duration_s"] = time.time() - started
        print(json.dumps(report))
        return 0
    except KeyboardInterrupt:
        report["status"] = f"ABORT_STEP_{active_step}"
        if start is not None and not return_attempted:
            return_attempted = True
            _attempt_return_after_abort(report, controller, args.control, start)
        report["duration_s"] = time.time() - started
        print(json.dumps(report))
        return 1
    except Exception as exc:
        report["status"] = f"ABORT_STEP_{active_step}"
        report["error"] = str(exc)
        if start is not None and not return_attempted:
            return_attempted = True
            _attempt_return_after_abort(report, controller, args.control, start)
        report["duration_s"] = time.time() - started
        print(json.dumps(report))
        return 1
    finally:
        if controller is not None:
            try:
                controller.stop()
            except Exception:
                pass
            try:
                controller.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
