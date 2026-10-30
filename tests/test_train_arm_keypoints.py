"""Tests for tools/train_arm_keypoints.py."""
import os
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.train_arm_keypoints import (  # noqa: E402
    build_dataset_yaml,
    check_environment,
    train,
)


def _make_dataset(tmp, valid=True):
    os.makedirs(os.path.join(tmp, "images"), exist_ok=True)
    os.makedirs(os.path.join(tmp, "labels"), exist_ok=True)
    # dummy image file
    with open(os.path.join(tmp, "images", "000001.jpg"), "wb") as f:
        f.write(b"\xff\xd8\xff\xd9")
    if valid:
        toks = ["0"] + ["0.5"] * 22
        with open(os.path.join(tmp, "labels", "000001.txt"), "w") as f:
            f.write(" ".join(toks) + "\n")
    else:
        with open(os.path.join(tmp, "labels", "000001.txt"), "w") as f:
            f.write("0 0.5 0.5\n")


def test_build_dataset_yaml_valid():
    with tempfile.TemporaryDirectory() as tmp:
        _make_dataset(tmp, valid=True)
        yaml_path = build_dataset_yaml(tmp)
        assert os.path.exists(yaml_path)
        with open(yaml_path) as f:
            content = f.read()
        assert "kpt_shape: [6, 3]" in content
        assert "names: {0: arm}" in content
        # header comment lists the 6 keypoints
        assert "base, shoulder, elbow, wrist_pitch, wrist_roll, gripper_tip" in content


def test_build_dataset_yaml_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        _make_dataset(tmp, valid=False)
        try:
            build_dataset_yaml(tmp)
            raised = False
        except ValueError:
            raised = True
        assert raised, "expected ValueError for invalid label"


def test_build_dataset_yaml_missing_dirs():
    with tempfile.TemporaryDirectory() as tmp:
        try:
            build_dataset_yaml(tmp)
            raised = False
        except ValueError:
            raised = True
        assert raised


def test_check_environment():
    env = check_environment()
    for key in ("torch", "cuda", "ultralytics", "device", "ready", "hint"):
        assert key in env
    assert isinstance(env["ready"], bool)
    assert isinstance(env["cuda"], bool)
    # ready is a property of the host, not of the test: assert the contract only.
    if env["cuda"] and env["ultralytics"]:
        assert env["ready"] is True
    else:
        assert env["ready"] is False


def test_train_missing_ultralytics(monkeypatch):
    monkeypatch.setitem(__import__("sys").modules, "ultralytics", None)
    res = train("dummy.yaml")
    assert res["ok"] is False
    assert "pip install" in res.get("hint", "")
