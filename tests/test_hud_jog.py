"""Keyboard jog command selection for the live HUD."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.live_vision_hud import jog_command, _load_joint_modes  # noqa: E402


def test_positional_jog_steps_and_clamps():
    assert jog_command(2, +1, "P", 90.0, step_deg=3.0) == "M2:93:400"
    assert jog_command(2, -1, "P", 90.0, step_deg=3.0) == "M2:87:400"
    assert jog_command(5, +1, "P", 179.0, step_deg=5.0) == "M5:180:400"
    assert jog_command(5, -1, "P", 1.0, step_deg=5.0) == "M5:0:400"


def test_continuous_jog_uses_timed_burst():
    assert jog_command(7, +1, "C", 0.0, offset=40, burst_ms=150) == "T7:40:150"
    assert jog_command(7, -1, "C", 0.0, offset=40, burst_ms=150) == "T7:-40:150"


def test_continuous_offset_is_clamped():
    assert jog_command(1, +1, "C", 0.0, offset=400, burst_ms=100) == "T1:150:100"


def test_invalid_inputs_raise():
    for motor, direction in ((0, 1), (9, 1), (2, 0)):
        try:
            jog_command(motor, direction, "P", 90.0)
        except ValueError:
            continue
        raise AssertionError("expected ValueError for %r %r" % (motor, direction))


def test_joint_modes_load_from_repo_config():
    modes = _load_joint_modes()
    assert set(modes) == set(range(1, 9))
    assert all(m in ("P", "C") for m in modes.values())
