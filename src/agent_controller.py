"""Hardware integrator, CLI and AI Agent API for the 8-axis arm (S8).

Exposes a clean ``AgentController`` class plus module-level convenience
functions backed by a configurable (simulated by default) singleton.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

# --- optional imports (robust to different invocation styles) ---
try:
    from .kinematics_6dof import RobotArm6DOF, WorkspaceCollisionError
except Exception:  # pragma: no cover
    try:
        from kinematics_6dof import RobotArm6DOF, WorkspaceCollisionError  # type: ignore
    except Exception:
        RobotArm6DOF = None  # type: ignore

        class WorkspaceCollisionError(Exception):  # type: ignore
            pass

try:
    from .state_manager import RobotStateManager
except Exception:  # pragma: no cover
    try:
        from state_manager import RobotStateManager  # type: ignore
    except Exception:
        RobotStateManager = None  # type: ignore

try:
    from host.armctl.transport import SimTransport as _HostSimTransport
except Exception:
    _HostSimTransport = None

try:
    from host.armctl.sim import SimDevice as _SimDevice
except Exception:
    _SimDevice = None


class _DualKeyDict(dict):
    """Dict whose keys are ints but also reachable via their str form."""

    def __getitem__(self, key):
        if key in self:
            return super().__getitem__(key)
        alt = None
        if isinstance(key, str):
            try:
                alt = int(key)
            except ValueError:
                alt = None
        elif isinstance(key, int):
            alt = str(key)
        if alt is not None and alt in self:
            return super().__getitem__(alt)
        raise KeyError(key)

    def get(self, key, default=None):
        try:
            return self[key]
        except KeyError:
            return default


# ---------------------------------------------------------------------------
# Simulated transport (offline) - reuse host implementation when available
# ---------------------------------------------------------------------------
class SimulatedTransport:
    def __init__(self, config=None):
        if _HostSimTransport is not None:
            self._impl = _HostSimTransport(config)
        else:
            # minimal fallback
            self._impl = None
            self.rx_queue = __import__("queue").Queue()
            self.ready_seen = True
            self.boot_lines = []
            self._state = {"status": "OK", "pca9685": "CONECTADO_0x40"}
        self.rx_queue = self._impl.rx_queue if self._impl is not None else self.rx_queue
        self.ready_seen = True
        self.boot_lines = []

    def send_line(self, line):
        if self._impl is not None:
            self._impl.send_line(line)
        else:
            import json
            if line.strip() == "I":
                self.rx_queue.put(json.dumps({"status": "OK", "pca9685": "CONECTADO_0x40"}))
            elif line.strip() == "S":
                self.rx_queue.put(json.dumps({"event": "EMERGENCY_STOP", "oe": "HIGH"}))
            else:
                self.rx_queue.put(json.dumps({"ack": "OK"}))

    def close(self):
        if self._impl is not None:
            try:
                self._impl.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# AgentController
# ---------------------------------------------------------------------------
class AgentController:
    def __init__(self, sim: bool = True, port: Optional[str] = None,
                 state_path: str = "robot_state.json",
                 config_path: Optional[str] = None):
        self.sim = bool(sim)
        self.port = port
        self.arm = RobotArm6DOF() if RobotArm6DOF is not None else None
        if config_path is None:
            config_path = os.path.join(REPO_ROOT, "config", "config_geometria.json")
        if state_path == "robot_state.json":
            state_path = os.path.join(REPO_ROOT, "robot_state.json")
        self.config_path = config_path
        self.state_path = state_path
        self.state = RobotStateManager(state_path=state_path, config_path=config_path) if RobotStateManager is not None else None

        # transport selection
        if self.sim:
            self.transport = SimulatedTransport()
        else:
            detected = port or detect_mega_port()
            if detected is None:
                self.sim = True
                self.transport = SimulatedTransport()
            else:
                try:
                    import serial  # noqa
                    from host.armctl.transport import SerialTransport  # type: ignore
                    self.transport = SerialTransport(detected, 115200)
                except Exception:
                    self.sim = True
                    self.transport = SimulatedTransport()

    # ---------- helpers ----------
    def _state_dict(self) -> Dict[str, Any]:
        return self.state.state if self.state is not None else {}

    def _current_thetas(self) -> List[float]:
        st = self._state_dict()
        t = st.get("thetas_deg")
        if t is None:
            t = [0.0, 90.0, 0.0, 0.0, 0.0, 50.0]
        return [float(v) for v in t]

    def get_current_pose(self) -> Dict[str, Any]:
        st = self._state_dict()
        thetas = self._current_thetas()
        res = None
        if self.arm is not None:
            try:
                res = self.arm.forward_kinematics(thetas)
            except Exception:
                res = None
        if res is not None:
            T = res["T_0_6"]
            x = float(res["x_mm"])
            y = float(res["y_mm"])
            z = float(res["z_mm"])
            pitch = float(res["pitch_deg"])
            roll = float(res["roll_deg"])
            yaw = float(res["yaw_deg"])
            grip = float(res["grip_percent"])
        else:
            cart = st.get("cartesian_mm", {"x": 0.0, "y": 0.0, "z": 570.0})
            x = float(cart.get("x", 0.0))
            y = float(cart.get("y", 0.0))
            z = float(cart.get("z", 570.0))
            ori = st.get("orientation_deg", {"pitch": 90.0, "roll": 0.0, "yaw": 0.0})
            pitch = float(ori.get("pitch", 90.0))
            roll = float(ori.get("roll", 0.0))
            yaw = float(ori.get("yaw", 0.0))
            grip = float(st.get("grip_percent", 50.0))
            T = st.get("T_0_6", [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)])
            if T is None:
                T = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
                T[2][3] = 570.0

        T_list = [[float(v) for v in row] for row in T]

        # motors_deg
        motors = _DualKeyDict()
        servo = None
        if self.arm is not None:
            try:
                servo = self.arm.kinematic_to_servo_angles(thetas)
            except Exception:
                servo = None
        state_motors = st.get("motors_deg", {})
        for m in range(1, 8):
            if servo is not None and m in servo:
                motors[m] = float(servo[m])
            else:
                v = state_motors.get(str(m), state_motors.get(m, 0.0))
                motors[m] = float(v) if v is not None else 0.0

        return {
            "x_mm": x,
            "y_mm": y,
            "z_mm": z,
            "pitch_deg": pitch,
            "roll_deg": roll,
            "yaw_deg": yaw,
            "grip_percent": grip,
            "T_0_6": T_list,
            "thetas_deg": [float(v) for v in thetas],
            "motors_deg": motors,
        }

    # ---------- high-level API ----------
    def move_to_xyz(self, x_mm: float, y_mm: float, z_mm: float,
                    pitch_deg: float = 0.0, roll_deg: float = 0.0,
                    speed_mm_s: float = 60.0) -> Dict[str, Any]:
        if float(z_mm) < 10.0:
            raise WorkspaceCollisionError(
                f"Target Z={z_mm:.2f} mm collides with table (z_min=10.0 mm)"
            )
        pose = self.get_current_pose()
        start_xyz = (float(pose["x_mm"]), float(pose["y_mm"]), float(pose["z_mm"]))
        grip = float(pose["grip_percent"])
        # validate whole path first
        waypoints = self.arm.move_linear_xyz(
            start_xyz, (float(x_mm), float(y_mm), float(z_mm)),
            pitch_deg=float(pitch_deg), roll_deg=float(roll_deg),
            grip_percent=grip, speed_mm_s=float(speed_mm_s), check_collision=True,
        )
        commands: List[str] = []
        for wp in waypoints:
            servo = wp["servo_angles"]
            motors = {
                1: float(servo[1]), 2: float(servo[2]), 3: float(servo[3]),
                4: float(servo[4]), 5: float(servo[5]), 7: float(servo[7]),
            }
            duration_ms = int(round(float(wp["duration_ms"]))) if float(wp["duration_ms"]) > 0 else 1
            if duration_ms < 1:
                duration_ms = 1
            cmd = self.state.build_coordinated_command(motors, duration_ms=duration_ms)
            self.transport.send_line(cmd)
            commands.append(cmd)
        final_thetas = waypoints[-1]["thetas_deg"]
        self.state.update_from_thetas(final_thetas)
        new_pose = self.get_current_pose()
        return {"pose": new_pose, "commands": commands}

    def rotate_wrist(self, roll_deg_360: float) -> Dict[str, Any]:
        theta5 = float(roll_deg_360) % 360.0
        thetas = self._current_thetas()
        thetas[4] = theta5
        self.transport.send_line("M2:%s" % _fmt(theta5))
        st = self._state_dict()
        st.setdefault("motors_deg", {})[str(2)] = theta5
        st.setdefault("pca9685_channels_deg", {})[str(1)] = theta5
        self.state.update_from_thetas(thetas)
        return self.get_current_pose()

    def set_gripper(self, open_percent: float) -> Dict[str, Any]:
        op = float(open_percent)
        op = max(0.0, min(100.0, op))
        thetas = self._current_thetas()
        thetas[5] = op
        self.transport.send_line("M1:%s" % _fmt(op))
        st = self._state_dict()
        st.setdefault("motors_deg", {})[str(1)] = op
        st.setdefault("pca9685_channels_deg", {})[str(0)] = op
        self.state.update_from_thetas(thetas)
        return self.get_current_pose()

    def pick_and_place(self, origin_xyz: Tuple[float, float, float],
                       target_xyz: Tuple[float, float, float],
                       approach_height_mm: float = 60.0,
                       pitch_deg: float = 0.0, roll_deg: float = 0.0,
                       speed_mm_s: float = 60.0) -> Dict[str, Any]:
        ox, oy, oz = [float(v) for v in origin_xyz]
        tx, ty, tz = [float(v) for v in target_xyz]
        approach_o = (ox, oy, oz + float(approach_height_mm))
        approach_t = (tx, ty, tz + float(approach_height_mm))
        origin = (ox, oy, oz)
        target = (tx, ty, tz)

        # validate all transitions first
        pose = self.get_current_pose()
        start = (float(pose["x_mm"]), float(pose["y_mm"]), float(pose["z_mm"]))
        grip = float(pose["grip_percent"])
        transitions = [
            (start, approach_o),
            (approach_o, origin),
            (origin, approach_o),
            (approach_o, approach_t),
            (approach_t, target),
            (target, approach_t),
        ]
        for s, t in transitions:
            self.arm.move_linear_xyz(
                s, t, pitch_deg=float(pitch_deg), roll_deg=float(roll_deg),
                grip_percent=grip, speed_mm_s=float(speed_mm_s), check_collision=True,
            )

        # execute
        self.set_gripper(85.0)
        self.move_to_xyz(*approach_o, pitch_deg=pitch_deg, roll_deg=roll_deg, speed_mm_s=speed_mm_s)
        self.move_to_xyz(*origin, pitch_deg=pitch_deg, roll_deg=roll_deg, speed_mm_s=speed_mm_s)
        self.set_gripper(25.0)
        self.move_to_xyz(*approach_o, pitch_deg=pitch_deg, roll_deg=roll_deg, speed_mm_s=speed_mm_s)
        self.move_to_xyz(*approach_t, pitch_deg=pitch_deg, roll_deg=roll_deg, speed_mm_s=speed_mm_s)
        self.move_to_xyz(*target, pitch_deg=pitch_deg, roll_deg=roll_deg, speed_mm_s=speed_mm_s)
        self.set_gripper(85.0)
        self.move_to_xyz(*approach_t, pitch_deg=pitch_deg, roll_deg=roll_deg, speed_mm_s=speed_mm_s)
        return self.get_current_pose()

    def emergency_stop(self) -> Dict[str, Any]:
        self.transport.send_line("S")
        st = self._state_dict()
        st["emergency_stop"] = True
        try:
            self.state._save()
        except Exception:
            pass
        return {"event": "EMERGENCY_STOP", "oe": "HIGH"}

    # ---------- hardware helpers ----------
    def detect_mega_port(self) -> Optional[str]:
        return detect_mega_port()

    def compile_firmware(self, sketch_dir: str = "firmware_mega",
                         fqbn: str = "arduino:avr:mega") -> Dict[str, Any]:
        return compile_firmware(sketch_dir=sketch_dir, fqbn=fqbn)

    def verify_i2c_link(self) -> Dict[str, Any]:
        self.transport.send_line("I")
        import queue as _q
        deadline = time.time() + 2.0
        while time.time() < deadline:
            try:
                raw = self.transport.rx_queue.get(timeout=0.1)
            except Exception:
                continue
            try:
                obj = json.loads(raw)
            except Exception:
                obj = {"raw": raw}
            if (isinstance(obj, dict) and obj.get("status") == "OK"
                    and obj.get("pca9685") == "CONECTADO_0x40"):
                return {"ok": True, "response": obj}
        return {"ok": False, "error": "no valid I response"}

    def demo_wave_and_twist(self) -> Dict[str, Any]:
        steps: List[str] = []
        # lift to home
        try:
            r = self.move_to_xyz(0.0, 0.0, 570.0, pitch_deg=90.0, speed_mm_s=80.0)
            steps.append("home")
        except Exception as e:
            steps.append("home-failed: %s" % e)
        # wave gripper
        for op in (100.0, 0.0, 50.0, 100.0, 50.0):
            try:
                self.set_gripper(op)
                steps.append("gripper %s" % op)
            except Exception as e:
                steps.append("gripper-failed: %s" % e)
        # rotate wrist
        for ang in (90.0, 180.0, 270.0, 0.0):
            try:
                self.rotate_wrist(ang)
                steps.append("wrist %s" % ang)
            except Exception as e:
                steps.append("wrist-failed: %s" % e)
        # return home
        try:
            self.move_to_xyz(0.0, 0.0, 570.0, pitch_deg=90.0, speed_mm_s=80.0)
            steps.append("home")
        except Exception as e:
            steps.append("home-failed: %s" % e)
        return {"steps": steps, "pose": self.get_current_pose()}


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------
def _fmt(v: float) -> str:
    if float(v).is_integer():
        return str(int(v))
    return repr(float(v))


def detect_mega_port() -> Optional[str]:
    try:
        from serial.tools import list_ports
    except Exception:
        return None
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        vid = getattr(p, "vid", None)
        pid = getattr(p, "pid", None)
        if vid == 0x1A86 and pid == 0x7523:
            return p.device
        if vid == 0x2341:
            return p.device
        if "arduino" in desc or "ch340" in desc or "mega" in desc:
            return p.device
    return None


def compile_firmware(sketch_dir: str = "firmware_mega",
                     fqbn: str = "arduino:avr:mega") -> Dict[str, Any]:
    cli = None
    found = shutil.which("arduino-cli")
    if found:
        cli = found
    else:
        cand = os.path.join(
            r"C:\Users\merca\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources",
            "arduino-cli.exe",
        )
        if os.path.exists(cand):
            cli = cand
    if cli is None:
        return {"ok": False, "error": "arduino-cli not found"}
    sketch = sketch_dir
    if not os.path.isabs(sketch):
        sketch = os.path.join(REPO_ROOT, sketch)
    cmd = [cli, "compile", "--fqbn", fqbn, sketch]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        return {
            "ok": r.returncode == 0,
            "stdout": r.stdout,
            "stderr": r.stderr,
            "returncode": r.returncode,
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}


# ---------------------------------------------------------------------------
# Singleton + module-level convenience functions
# ---------------------------------------------------------------------------
_default_controller: Optional[AgentController] = None


def get_default_controller() -> AgentController:
    global _default_controller
    if _default_controller is None:
        _default_controller = AgentController(sim=True)
    return _default_controller


def set_default_controller(ctrl: AgentController) -> None:
    global _default_controller
    _default_controller = ctrl


def get_current_pose() -> Dict[str, Any]:
    return get_default_controller().get_current_pose()


def move_to_xyz(x_mm: float, y_mm: float, z_mm: float,
                pitch_deg: float = 0.0, roll_deg: float = 0.0,
                speed_mm_s: float = 60.0) -> Dict[str, Any]:
    return get_default_controller().move_to_xyz(x_mm, y_mm, z_mm, pitch_deg, roll_deg, speed_mm_s)


def rotate_wrist(roll_deg_360: float) -> Dict[str, Any]:
    return get_default_controller().rotate_wrist(roll_deg_360)


def set_gripper(open_percent: float) -> Dict[str, Any]:
    return get_default_controller().set_gripper(open_percent)


def pick_and_place(origin_xyz: Tuple[float, float, float],
                   target_xyz: Tuple[float, float, float],
                   approach_height_mm: float = 60.0,
                   pitch_deg: float = 0.0, roll_deg: float = 0.0,
                   speed_mm_s: float = 60.0) -> Dict[str, Any]:
    return get_default_controller().pick_and_place(
        origin_xyz, target_xyz, approach_height_mm, pitch_deg, roll_deg, speed_mm_s)


def emergency_stop() -> Dict[str, Any]:
    return get_default_controller().emergency_stop()


# expose helpers as module-level too
def verify_i2c_link() -> Dict[str, Any]:
    return get_default_controller().verify_i2c_link()


def demo_wave_and_twist() -> Dict[str, Any]:
    return get_default_controller().demo_wave_and_twist()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    raise TypeError("not serializable: %r" % (o,))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="python -m src.agent_controller")
    parser.add_argument("--sim", action="store_true", help="force simulation mode")
    parser.add_argument("--check-port", action="store_true")
    parser.add_argument("--pose", action="store_true")
    parser.add_argument("--demo", action="store_true")
    args = parser.parse_args(argv)

    sim = bool(args.sim)
    if not sim:
        if detect_mega_port() is None:
            sim = True
    ctrl = AgentController(sim=sim)
    set_default_controller(ctrl)

    if args.check_port:
        print("detected ports:")
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                print("  %s - %s" % (p.device, p.description))
        except Exception as e:
            print("  (serial unavailable: %s)" % e)
        print("i2c:", json.dumps(ctrl.verify_i2c_link()))
        return 0

    if args.pose:
        print(json.dumps(ctrl.get_current_pose(), indent=2, default=_json_default))
        return 0

    if args.demo:
        print(json.dumps(ctrl.demo_wave_and_twist(), indent=2, default=_json_default))
        return 0

    # default: print help-ish
    print("agent_controller ready (sim=%s). Use --pose / --demo / --check-port." % sim)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
