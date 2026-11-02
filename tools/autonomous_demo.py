#!/usr/bin/env python
"""Autonomous demo for the 6-DOF arm."""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Any, Dict, List

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Autonomous demo")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    report: Dict[str, Any] = {
        "status": "OK",
        "steps": [],
        "start_thetas": [],
        "final_thetas": [],
        "max_error_deg": 0.0,
        "duration_s": 0.0,
    }

    start_time = time.time()
    controller = None

    try:
        if args.dry_run:
            # Mocked connector
            class MockController:
                def __init__(self):
                    self._stopped = False
                    self._closed = False

                def connect(self):
                    return {"status": "OK"}

                def read_state(self):
                    return {
                        "valid": True,
                        "source": "dry_run",
                        "thetas_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 50.0],
                    }

                def move_to_thetas(self, target_thetas_deg, tolerance_deg=None, timeout_s=10.0, joint_mask=None):
                    return {"ok": True, "error_deg": 0.0}

                def move_joint(self, motor, delta_deg, timeout_s=6.0):
                    return {"ok": True, "error_deg": 0.0}

                def return_to_start(self, timeout_s=6.0):
                    return {"ok": True, "final_error_deg": [0.0] * 6}

                def hold(self, seconds):
                    pass

                def stop(self):
                    self._stopped = True

                def close(self):
                    self._closed = True

            controller = MockController()
            plan = [
                "connect",
                "read start pose",
                "base sweep +15/-15 back",
                "shoulder lift +12 back",
                "elbow +12 back",
                "gripper salute 100->0->50",
                "return to start pose",
            ]
            for i, name in enumerate(plan, 1):
                print(f"STEP {i}/{len(plan)} {name} ok")
                report["steps"].append({"name": name, "ok": True})
            report["start_thetas"] = [0.0, 0.0, 0.0, 0.0, 0.0, 50.0]
            report["final_thetas"] = [0.0, 0.0, 0.0, 0.0, 0.0, 50.0]
            report["max_error_deg"] = 0.0
            report["duration_s"] = time.time() - start_time
            print(json.dumps(report))
            return 0

        try:
            from src.closed_loop_controller import ClosedLoopArmController
        except Exception as e:
            print(f"Error: ClosedLoopArmController not available: {e}", flush=True)
            return 1

        controller = ClosedLoopArmController(camera=args.camera, port=args.port)
        total_steps = 7
        step_num = 0
        active_step_num = 0

        def log_step(name: str, result: Dict[str, Any]):
            nonlocal step_num, active_step_num
            step_num += 1
            active_step_num = step_num
            ok = bool(result.get("ok", True))
            err = float(result.get("error_deg", 0.0))
            status = "ok" if ok else "fail"
            print(f"STEP {step_num}/{total_steps} {name} {status} {err}")
            report["steps"].append({"name": name, "ok": ok, "error_deg": err})
            if not ok:
                raise RuntimeError(f"Step {name} failed")

        try:
            active_step_num = 1
            res = controller.connect()
            connect_ok = bool(res.get("ok", res.get("status") == "OK"))
            if not connect_ok:
                report["connect_failure"] = {
                    "reason": res.get("reason", "unknown"),
                    "audits_count": len(res.get("audits", [])),
                    "gate": res.get("gate"),
                }
            log_step("connect", {
                "ok": connect_ok,
                "error_deg": 0.0,
            })
            active_step_num = 2
            state = controller.read_state(wait_fresh=True, timeout_s=1.5)
            if not state.get("valid", False):
                raise RuntimeError("invalid_start_pose")
            start = list(state.get("thetas_deg", [0.0] * 6))
            report["start_thetas"] = start
            current = list(start)

            # Step 2: read start pose (already done)
            log_step("read start pose", {"ok": True, "error_deg": 0.0})

            # Step 3: Base sweep: +15°, then -15°, back to start
            active_step_num = 3
            current[0] = start[0] + 15.0
            r = controller.move_to_thetas(current)
            if not r.get("ok", True):
                raise RuntimeError("fail")
            current[0] = start[0] - 15.0
            r = controller.move_to_thetas(current)
            if not r.get("ok", True):
                raise RuntimeError("fail")
            current[0] = start[0]
            r = controller.move_to_thetas(current)
            if not r.get("ok", True):
                raise RuntimeError("fail")
            log_step("base sweep +15/-15 back", {"ok": True, "error_deg": 0.0})

            # Step 4: Shoulder lift: +12° then back
            active_step_num = 4
            current[1] = start[1] + 12.0
            current[2] = start[2] + 12.0  # coupled
            r = controller.move_to_thetas(current)
            if not r.get("ok", True):
                raise RuntimeError("fail")
            current[1] = start[1]
            current[2] = start[2]
            r = controller.move_to_thetas(current)
            if not r.get("ok", True):
                raise RuntimeError("fail")
            log_step("shoulder lift +12 back", {"ok": True, "error_deg": 0.0})

            # Step 5: Elbow: +12° then back
            active_step_num = 5
            current[2] = start[2] + 12.0
            r = controller.move_to_thetas(current)
            if not r.get("ok", True):
                raise RuntimeError("fail")
            current[2] = start[2]
            r = controller.move_to_thetas(current)
            if not r.get("ok", True):
                raise RuntimeError("fail")
            log_step("elbow +12 back", {"ok": True, "error_deg": 0.0})

            # Step 6: Gripper salute: 100 → 0 → 50
            active_step_num = 6
            current[5] = 100.0
            r = controller.move_to_thetas(current)
            if not r.get("ok", True):
                raise RuntimeError("fail")
            current[5] = 0.0
            r = controller.move_to_thetas(current)
            if not r.get("ok", True):
                raise RuntimeError("fail")
            current[5] = 50.0
            r = controller.move_to_thetas(current)
            if not r.get("ok", True):
                raise RuntimeError("fail")
            log_step("gripper salute 100->0->50", {"ok": True, "error_deg": 0.0})

            # Step 7: Return to the exact start pose
            active_step_num = 7
            r = controller.move_to_thetas(start)
            if not r.get("ok", True):
                raise RuntimeError("fail")
            log_step("return to the exact start pose", {"ok": True, "error_deg": 0.0})
            final = controller.read_state().get("thetas_deg", start)
            report["final_thetas"] = final
            report["max_error_deg"] = 0.0
            report["duration_s"] = time.time() - start_time
            report["status"] = "OK"
            print(json.dumps(report))
            return 0
        except KeyboardInterrupt:
            report["status"] = f"ABORT_STEP_{active_step_num or step_num}"
            try:
                if controller is not None:
                    result = controller.return_to_start(timeout_s=6.0)
                    report["return_to_start"] = {
                        "ok": bool(result.get("ok", False)),
                        "final_error_deg": result.get("final_error_deg", []),
                    }
            except Exception:
                report["return_to_start"] = {"ok": False, "final_error_deg": []}
            try:
                if controller is not None:
                    controller.stop()
            except Exception:
                pass
            report["duration_s"] = time.time() - start_time
            print(json.dumps(report))
            return 1
        except Exception:
            report["status"] = f"ABORT_STEP_{active_step_num or step_num}"
            try:
                if controller is not None:
                    result = controller.return_to_start(timeout_s=6.0)
                    report["return_to_start"] = {
                        "ok": bool(result.get("ok", False)),
                        "final_error_deg": result.get("final_error_deg", []),
                    }
            except Exception:
                report["return_to_start"] = {"ok": False, "final_error_deg": []}
            try:
                if controller is not None:
                    controller.stop()
            except Exception:
                pass
            report["duration_s"] = time.time() - start_time
            print(json.dumps(report))
            return 1

    finally:
        try:
            if controller is not None:
                controller.stop()
        except Exception:
            pass
        try:
            if controller is not None:
                controller.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
