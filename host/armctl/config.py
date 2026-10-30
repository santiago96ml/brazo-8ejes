"""Configuration loading and joint/pose helpers."""

import json
import os


def _default_config_path():
    base = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(base, "config", "arm.json")


def load_config(path=None):
    if path is None:
        path = _default_config_path()
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    validate_config(cfg)
    return cfg


def validate_config(cfg):
    if not isinstance(cfg, dict):
        raise ValueError("config must be an object")
    if "joints" not in cfg:
        raise ValueError("missing joints")
    joints = cfg["joints"]
    if not isinstance(joints, list) or len(joints) != 8:
        raise ValueError("joints must be a list of 8")
    names = set()
    motors = set()
    for i, j in enumerate(joints):
        if not isinstance(j, dict):
            raise ValueError("joint %d invalid" % i)
        for req in ("name", "motor", "mode"):
            if req not in j:
                raise ValueError("joint %d missing %s" % (i, req))
        if j["name"] in names:
            raise ValueError("duplicate joint name %s" % j["name"])
        names.add(j["name"])
        motor = j["motor"]
        if not isinstance(motor, int) or isinstance(motor, bool):
            try:
                motor = int(motor)
            except (ValueError, TypeError):
                raise ValueError("joint %d invalid motor" % i)
        if motor < 1 or motor > 8:
            raise ValueError("joint %d motor out of range" % i)
        if motor in motors:
            raise ValueError("duplicate motor %d" % motor)
        motors.add(motor)
    if motors != set(range(1, 9)):
        raise ValueError("motors must be 1..8")
    return True


def joint_index(cfg, name_or_index):
    joints = cfg["joints"]
    # motor number (int) resolution
    if isinstance(name_or_index, bool):
        raise ValueError("unknown joint %r" % (name_or_index,))
    if isinstance(name_or_index, int):
        # motor number first
        for i, j in enumerate(joints):
            if j.get("motor") == name_or_index:
                return i
        if 0 <= name_or_index < len(joints):
            return name_or_index
        raise ValueError("unknown joint %r" % (name_or_index,))
    # string
    s = str(name_or_index).strip()
    # name
    for i, j in enumerate(joints):
        if j["name"] == s:
            return i
    # motor number as string
    try:
        mi = int(s)
    except (ValueError, TypeError):
        mi = None
    if mi is not None:
        for i, j in enumerate(joints):
            if j.get("motor") == mi:
                return i
        if 0 <= mi < len(joints):
            return mi
    raise ValueError("unknown joint %r" % (name_or_index,))


def get_joint(cfg, name_or_index):
    return cfg["joints"][joint_index(cfg, name_or_index)]


def joint_motor(cfg, name_or_index):
    return get_joint(cfg, name_or_index).get("motor", joint_index(cfg, name_or_index) + 1)


def joint_name(cfg, index):
    return cfg["joints"][index]["name"]


def deg_to_pulse(cfg_joint, deg):
    pmin = cfg_joint.get("pulse_min", 110)
    pmax = cfg_joint.get("pulse_max", 500)
    return int(pmin + (deg / 180.0) * (pmax - pmin))


def _default_poses_path():
    base = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(base, "config", "poses.json")


def load_poses(path=None):
    if path is None:
        path = _default_poses_path()
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_poses(poses, path=None):
    if path is None:
        path = _default_poses_path()
    with open(path, "w", encoding="utf-8") as f:
        json.dump(poses, f, indent=2)
        f.write("\n")


def pose_to_moves(cfg, pose):
    moves = []
    for name, deg in pose.items():
        idx = joint_index(cfg, name)
        moves.append((idx, deg))
    return moves


def motor_for_move(cfg, idx):
    return cfg["joints"][idx].get("motor", idx + 1)