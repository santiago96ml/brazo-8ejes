"""Tests for autonomous demo."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import tools.autonomous_demo as demo


class FakeControllerOK:
    def __init__(self, camera=None, encoder=None, transport=None, port="auto"):
        self.stopped = False
        self.closed = False
        self.moves = []

    def connect(self):
        return {"ok": True}

    def read_state(self):
        return {"thetas_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 50.0]}

    def move_to_thetas(self, target_thetas_deg, tolerance_deg=None, timeout_s=10.0, joint_mask=None):
        self.moves.append(list(target_thetas_deg))
        return {"ok": True, "error_deg": 0.0}

    def move_joint(self, motor, delta_deg, timeout_s=6.0):
        return {"ok": True, "error_deg": 0.0}

    def hold(self, seconds):
        pass

    def stop(self):
        self.stopped = True

    def close(self):
        self.closed = True


class FakeControllerFailStep3(FakeControllerOK):
    def move_to_thetas(self, target_thetas_deg, tolerance_deg=None, timeout_s=10.0, joint_mask=None):
        self.moves.append(list(target_thetas_deg))
        if len(self.moves) == 4:  # fail on 4th move
            return {"ok": False, "error_deg": 2.5}
        return {"ok": True, "error_deg": 0.0}


def test_dry_run_returns_ok_and_no_io(monkeypatch, capsys):
    rc = demo.main(["--dry-run"])
    assert rc == 0
    out = capsys.readouterr().out
    # Should print JSON report
    lines = [l for l in out.strip().splitlines() if l.strip()]
    # Last line is JSON
    report = json.loads(lines[-1])
    assert report["status"] == "OK"
    assert len(report["steps"]) == 7


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
    # Check stop was called
    assert any("STEP" in l for l in lines)


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
    # Check order contains base, shoulder, elbow, gripper, return
    text = " ".join(step_lines)
    assert "base" in text
    assert "shoulder" in text
    assert "elbow" in text
    assert "gripper" in text
    assert "return" in text
