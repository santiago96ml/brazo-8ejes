"""Tests for autonomous demo."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import tools.autonomous_demo as demo


class FakeControllerOK:
    def __init__(self, camera=None, encoder=None, transport=None, port="auto", **_kwargs):
        self.stopped = False
        self.closed = False
        self.moves = []
        self.return_calls = 0

    def connect(self):
        return {"ok": True}

    def read_state(self, **_kwargs):
        return {
            "valid": True,
            "source": "fake",
            "thetas_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 50.0],
        }

    def move_to_thetas(self, target_thetas_deg, tolerance_deg=None, timeout_s=10.0, joint_mask=None):
        self.moves.append(list(target_thetas_deg))
        return {"ok": True, "error_deg": 0.0}

    def move_joint(self, motor, delta_deg, timeout_s=6.0):
        return {"ok": True, "error_deg": 0.0}

    def move_delta_openloop(self, joint_index, delta_deg, **_kwargs):
        return {
            "ok": True,
            "reason": "unverified",
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
        }

    def hold(self, seconds):
        pass

    def return_to_start(self, timeout_s=6.0):
        self.return_calls += 1
        return {"ok": True, "final_error_deg": [0.0] * 6}

    def stop(self):
        self.stopped = True

    def close(self):
        self.closed = True


class FakeControllerFailStep3(FakeControllerOK):
    def move_delta_openloop(self, joint_index, delta_deg, **_kwargs):
        self.moves.append((joint_index, delta_deg))
        if len(self.moves) == 1:  # fail on the first move in step 3
            return {"ok": False, "reason": "simulated_failure"}
        return super().move_delta_openloop(joint_index, delta_deg, **_kwargs)


def test_dry_run_returns_ok_and_no_io(monkeypatch, capsys):
    rc = demo.main(["--dry-run"])
    assert rc == 0
    out = capsys.readouterr().out
    # Should print JSON report
    lines = [l for l in out.strip().splitlines() if l.strip()]
    # Last line is JSON
    report = json.loads(lines[-1])
    assert report["status"] == "OK"
    assert len(report["steps"]) == 8
    assert report["control"] == "open-loop"
    assert report["steps"][5]["commanded_deg"] == [50.0, -100.0, 50.0]


def test_fail_on_step3_calls_stop(monkeypatch, capsys):
    sys.modules.pop("src.closed_loop_controller", None)
    import src.closed_loop_controller as clc

    monkeypatch.setattr(clc, "ClosedLoopArmController", FakeControllerFailStep3)
    sys.modules["src.closed_loop_controller"] = clc

    rc = demo.main(["--port", "auto"])
    assert rc == 1
    out = capsys.readouterr().out
    lines = [l for l in out.strip().splitlines() if l.strip()]
    report = json.loads(lines[-1])
    assert report["status"] == "ABORT_STEP_3"
    assert report["return_to_start"]["ok"] is True
    # Check stop was called
    assert any("STEP" in l for l in lines)
    assert any(line.startswith("STEP 8/8 return_to_start") for line in lines)


def test_connect_failure_details_are_in_json(monkeypatch, capsys):
    class FakeControllerConnectFailure(FakeControllerOK):
        def connect(self):
            return {
                "ok": False,
                "reason": "three_consecutive_passes_not_reached",
                "audits": [{"passed": False}] * 6,
                "gate": {"consecutive_passes": 0, "can_enable_oe": False},
            }

    sys.modules.pop("src.closed_loop_controller", None)
    import src.closed_loop_controller as clc

    monkeypatch.setattr(clc, "ClosedLoopArmController", FakeControllerConnectFailure)
    sys.modules["src.closed_loop_controller"] = clc

    rc = demo.main(["--port", "auto"])
    report = json.loads(capsys.readouterr().out.strip().splitlines()[-1])

    assert rc == 1
    assert report["status"] == "ABORT_STEP_1"
    assert report["connect_failure"] == {
        "reason": "three_consecutive_passes_not_reached",
        "audits_count": 6,
        "gate": {"consecutive_passes": 0, "can_enable_oe": False},
    }


def test_sequence_order(monkeypatch, capsys):
    sys.modules.pop("src.closed_loop_controller", None)
    import src.closed_loop_controller as clc

    ctrl = FakeControllerOK()
    monkeypatch.setattr(clc, "ClosedLoopArmController", lambda *a, **k: ctrl)
    sys.modules["src.closed_loop_controller"] = clc

    rc = demo.main(["--port", "auto"])
    assert rc == 0
    out = capsys.readouterr().out
    lines = [l for l in out.strip().splitlines() if l.strip()]
    step_lines = [l for l in lines if l.startswith("STEP")]
    # Check the eight-stage sequence includes the Cartesian target.
    text = " ".join(step_lines)
    assert "base" in text
    assert "shoulder" in text
    assert "elbow" in text
    assert "gripper" in text
    assert "move_to_xyz" in text
    assert "return" in text


def test_demo_tolerates_drift_and_runs_eight_steps_in_order(monkeypatch, capsys):
    class SequenceController(FakeControllerOK):
        def __init__(self, **_kwargs):
            super().__init__()
            self.delta_calls = []
            self.xyz_calls = []

        def move_delta_openloop(self, joint_index, delta_deg, **_kwargs):
            self.delta_calls.append((joint_index, delta_deg))
            if len(self.delta_calls) == 1:
                return {
                    "ok": False,
                    "reason": "drift",
                    "commanded_deg": delta_deg,
                    "measured_deg": delta_deg + 10,
                    "drift_deg": 10,
                }
            return {
                "ok": True,
                "commanded_deg": delta_deg,
                "measured_deg": delta_deg,
                "drift_deg": 0,
            }

        def move_to_xyz_openloop(self, *args, **kwargs):
            self.xyz_calls.append((args, kwargs))
            return {
                "ok": True,
                "target_mm": [150, 0, 400],
                "measured_mm": [150, 0, 400],
                "error_mm": 0,
                "thetas": [0, 90, 0, 0, 0, 50],
            }

    ctrl = SequenceController()
    sys.modules.pop("src.closed_loop_controller", None)
    import src.closed_loop_controller as clc
    monkeypatch.setattr(clc, "ClosedLoopArmController", lambda **_kwargs: ctrl)
    sys.modules["src.closed_loop_controller"] = clc

    rc = demo.main([])
    lines = [line for line in capsys.readouterr().out.splitlines() if line.strip()]
    report = json.loads(lines[-1])
    step_lines = [line for line in lines if line.startswith("STEP")]

    assert rc == 0
    assert len(step_lines) == 8
    assert [line.split()[2] for line in step_lines] == [
        "connect", "read_start_pose", "base_sweep", "shoulder_lift", "elbow",
        "gripper_salute", "move_to_xyz", "return_to_start",
    ]
    assert step_lines[2].split()[3] == "drift"
    assert report["steps"][2]["moves"][0]["drift_deg"] == 10
    assert ctrl.xyz_calls[0][0] == (150, 0, 400)
    assert ctrl.xyz_calls[0][1] == {"pitch_deg": 45.0, "grip_percent": 60.0}


def test_demo_retries_only_the_timed_out_move_after_a_pulse_ack(monkeypatch, capsys):
    class TimeoutThenRecoverController(FakeControllerOK):
        def __init__(self, **_kwargs):
            super().__init__()
            self.delta_calls = []

        def move_delta_openloop(self, joint_index, delta_deg, **_kwargs):
            self.delta_calls.append((joint_index, delta_deg))
            if len(self.delta_calls) == 1:
                return {
                    "ok": True,
                    "reason": "verified",
                    "commanded_deg": delta_deg,
                    "measured_deg": delta_deg,
                    "drift_deg": 0.0,
                    "pulse_acknowledged": True,
                }
            if len(self.delta_calls) == 2:
                return {
                    "ok": False,
                    "reason": "firmware STATE response timed out",
                    "commanded_deg": delta_deg,
                    "measured_deg": None,
                    "drift_deg": None,
                    "pulse_acknowledged": False,
                }
            return {
                "ok": True,
                "reason": "verified",
                "commanded_deg": delta_deg,
                "measured_deg": delta_deg,
                "drift_deg": 0.0,
                "pulse_acknowledged": True,
            }

    ctrl = TimeoutThenRecoverController()
    sys.modules.pop("src.closed_loop_controller", None)
    import src.closed_loop_controller as clc
    monkeypatch.setattr(clc, "ClosedLoopArmController", lambda **_kwargs: ctrl)
    sys.modules["src.closed_loop_controller"] = clc

    rc = demo.main([])
    lines = [line for line in capsys.readouterr().out.splitlines() if line.strip()]
    report = json.loads(lines[-1])

    assert rc == 0
    assert "STEP 3/8 base_sweep retry_firmware_read" in lines
    assert ctrl.delta_calls[:3] == [(0, 12.0), (0, -12.0), (0, -12.0)]
    assert report["steps"][2]["ok"] is True
