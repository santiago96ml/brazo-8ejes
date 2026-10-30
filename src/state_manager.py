"""Persistent real-time state manager for the 8-axis robotic arm."""

import json
import os
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

try:  # works as `from src.state_manager import ...`
    from src.kinematics_6dof import RobotArm6DOF
except Exception:  # pragma: no cover - fallback for running from src/
    try:
        from kinematics_6dof import RobotArm6DOF  # type: ignore
    except Exception:
        RobotArm6DOF = None  # type: ignore


class ShoulderCouplingError(ValueError):
    """Raised when shoulder motors 5/6 receive contradictory commands."""

    pass


_MOTOR_NAMES = {
    1: "PINZA",
    2: "GIRO_PINZA",
    3: "INCLINA_PINZA",
    4: "CODO",
    5: "HOMBRO_IZQ",
    6: "HOMBRO_DER",
    7: "BASE",
    8: "AUX",
}


class RobotStateManager:
    def __init__(self, state_path: str = "robot_state.json",
                 config_path: Optional[str] = None, max_history: int = 100):
        self.state_path = state_path
        self.config_path = config_path
        self.max_history = max_history
        self._config = {}
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    self._config = json.load(f)
            except Exception:
                self._config = {}
        if os.path.exists(state_path):
            try:
                with open(state_path, "r", encoding="utf-8") as f:
                    self.state = json.load(f)
            except Exception:
                self.state = self._default_state()
                self._save()
        else:
            self.state = self._default_state()
            self._save()

    def _default_state(self) -> Dict[str, Any]:
        coupling = int(self._config.get("shoulder_coupling", 1))
        if coupling not in (1, -1):
            coupling = 1
        calib = {}
        for m in range(1, 9):
            calib[str(m)] = {
                "channel": m - 1,
                "name": _MOTOR_NAMES.get(m, f"MOTOR_{m}"),
                "stop_pulse": 307,
                "min_pulse": 110,
                "max_pulse": 500,
                "deg_per_sec": 60.0,
                "mode": "P",
            }
        return {
            "timestamp": self._now(),
            "shoulder_coupling": coupling,
            "thetas_deg": [0.0, 90.0, 0.0, 0.0, 0.0, 50.0],
            "motors_deg": {"1": 50.0, "2": 0.0, "3": 90.0, "4": 90.0,
                           "5": 90.0, "6": 90.0, "7": 0.0, "8": 90.0},
            "pca9685_channels_deg": {"0": 50.0, "1": 0.0, "2": 90.0, "3": 90.0,
                                     "4": 90.0, "5": 90.0, "6": 0.0, "7": 90.0},
            "cartesian_mm": {"x": 0.0, "y": 0.0, "z": 570.0},
            "orientation_deg": {"pitch": 90.0, "roll": 0.0, "yaw": 0.0},
            "grip_percent": 50.0,
            "T_0_6": self._home_T(),
            "calibration": calib,
            "command_history": [],
        }

    @staticmethod
    def _now() -> str:
        return datetime.now(timezone.utc).isoformat()

    @staticmethod
    def _home_T() -> List[List[float]]:
        T = [[0.0] * 4 for _ in range(4)]
        for i in range(3):
            T[i][i] = 1.0
        T[3][3] = 1.0
        T[0][3] = 0.0
        T[1][3] = 0.0
        T[2][3] = 570.0
        return T

    def _save(self) -> None:
        self.state["timestamp"] = self._now()
        path = self.state_path
        directory = os.path.dirname(os.path.abspath(path))
        os.makedirs(directory, exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.state, f, indent=2)
            f.flush()
            try:
                os.fsync(f.fileno())
            except OSError:
                pass
        os.replace(tmp, path)

    # ---------- shoulder coupling ----------
    def _coupling(self, coupling: Optional[int]) -> int:
        if coupling is None:
            coupling = self.state.get("shoulder_coupling", 1)
        return int(coupling)

    def compute_coupled_shoulder_angle(self, m5_deg: float,
                                      coupling: Optional[int] = None) -> float:
        c = self._coupling(coupling)
        if c == 1:
            return float(m5_deg)
        return float(180.0 - m5_deg)

    def compute_coupled_shoulder_offset(self, m5_offset: int,
                                       coupling: Optional[int] = None) -> int:
        c = self._coupling(coupling)
        if c == 1:
            return int(m5_offset)
        return int(-m5_offset)

    def validate_shoulder_pair(self, m5_deg: float, m6_deg: float,
                               tol: float = 1e-3) -> bool:
        expected = self.compute_coupled_shoulder_angle(m5_deg)
        if abs(float(m6_deg) - expected) > tol:
            raise ShoulderCouplingError(
                "Motor 6 must equal coupled shoulder angle of Motor 5")
        return True

    def validate_channel_pulses(self, channel_pulses: Dict[int, int]) -> bool:
        if 4 not in channel_pulses or 5 not in channel_pulses:
            return True
        coupling = self.state.get("shoulder_coupling", 1)
        stop4 = self.state["calibration"]["5"]["stop_pulse"]
        stop5 = self.state["calibration"]["6"]["stop_pulse"]
        off4 = int(channel_pulses[4]) - int(stop4)
        off5 = int(channel_pulses[5]) - int(stop5)

        def _sign(v: int) -> int:
            if v > 0:
                return 1
            if v < 0:
                return -1
            return 0

        s4 = _sign(off4)
        s5 = _sign(off5)
        if s4 == 0 or s5 == 0:
            return True
        if coupling == 1 and s4 != s5:
            raise ShoulderCouplingError(
                "Contradictory commands to PCA9685 shoulder channels 4 and 5")
        if coupling == -1 and s4 == s5:
            raise ShoulderCouplingError(
                "Contradictory commands to PCA9685 shoulder channels 4 and 5")
        return True

    def build_coordinated_command(self, motors_deg: Dict[int, float],
                                  duration_ms: int = 1000) -> str:
        coupling = self.state.get("shoulder_coupling", 1)
        m5 = float(motors_deg[5])
        if 6 in motors_deg:
            self.validate_shoulder_pair(m5, float(motors_deg[6]))
        coupled = self.compute_coupled_shoulder_angle(m5, coupling)
        motors_deg = dict(motors_deg)
        motors_deg[6] = coupled

        # update state motors_deg / pca channels
        for m, val in motors_deg.items():
            mi = int(m)
            if 1 <= mi <= 8:
                self.state["motors_deg"][str(mi)] = float(val)
                self.state["pca9685_channels_deg"][str(mi - 1)] = float(val)
        self.state["timestamp"] = self._now()

        # command history
        hist = self.state.setdefault("command_history", [])
        hist.append({
            "timestamp": self._now(),
            "cmd": "G",
            "summary": "motors=%s dur=%d" % (motors_deg, duration_ms),
        })
        if len(hist) > self.max_history:
            del hist[: len(hist) - self.max_history]

        self._save()

        m1 = round(motors_deg.get(1, self.state["motors_deg"].get("1", 0)))
        m2 = round(motors_deg.get(2, self.state["motors_deg"].get("2", 0)))
        m3 = round(motors_deg.get(3, self.state["motors_deg"].get("3", 0)))
        m4 = round(motors_deg.get(4, self.state["motors_deg"].get("4", 0)))
        m5r = round(motors_deg.get(5, self.state["motors_deg"].get("5", 0)))
        m7 = round(motors_deg.get(7, self.state["motors_deg"].get("7", 0)))
        return "G:%d,%d,%d,%d,%d,%d,%d" % (m1, m2, m3, m4, m5r, m7,
                                            int(duration_ms))

    def build_preload_command(self, motors_deg: Dict[int, float]) -> str:
        """Build silent PWM preload command ``Z:m1,..,m5,m7``.

        Validates the shoulder coupling on motors 5/6, computes the coupled
        motor 6 angle, updates persisted state, and returns the firmware
        command string used while ``PIN_OE`` is still ``HIGH``.
        """
        md: Dict[int, float] = {}
        for key, value in (motors_deg or {}).items():
            md[int(key)] = float(value)

        coupling = self.state.get("shoulder_coupling", 1)
        m5 = float(md.get(5, self.state.get("motors_deg", {}).get("5", 0.0) or 0.0))
        if 6 in md:
            self.validate_shoulder_pair(m5, float(md[6]))
        m6 = self.compute_coupled_shoulder_angle(m5, coupling)
        md[6] = float(m6)

        # Update persisted motor state (motors 1..8 and PCA channels).
        for motor, value in md.items():
            mi = int(motor)
            if 1 <= mi <= 8:
                self.state.setdefault("motors_deg", {})[str(mi)] = float(value)
                self.state.setdefault("pca9685_channels_deg", {})[str(mi - 1)] = float(value)

        self.state["timestamp"] = self._now()

        hist = self.state.setdefault("command_history", [])
        hist.append({"timestamp": self._now(), "cmd": "Z",
                     "summary": "preload motors=%s" % md})
        if len(hist) > self.max_history:
            del hist[: len(hist) - self.max_history]

        self._save()

        def _motor_value(motor: int) -> float:
            if motor in md:
                return float(md[motor])
            stored = self.state.get("motors_deg", {}).get(str(motor), 0.0)
            return float(stored) if stored is not None else 0.0

        m1 = round(_motor_value(1))
        m2 = round(_motor_value(2))
        m3 = round(_motor_value(3))
        m4 = round(_motor_value(4))
        m5r = round(_motor_value(5))
        m7 = round(_motor_value(7))
        return "Z:%d,%d,%d,%d,%d,%d" % (m1, m2, m3, m4, m5r, m7)

    def update_from_thetas(self, thetas_deg, command_str: Optional[str] = None) -> Dict[str, Any]:
        thetas = [float(x) for x in thetas_deg]
        self.state["thetas_deg"] = thetas
        # compute T_0_6, cartesian, orientation if kinematics available
        T = None
        cart = None
        orient = None
        arm = None
        if RobotArm6DOF is not None:
            try:
                arm = RobotArm6DOF(self.config_path) if self.config_path else RobotArm6DOF()
            except Exception:
                try:
                    arm = RobotArm6DOF()
                except Exception:
                    arm = None
        if arm is not None:
            try:
                res = arm.forward_kinematics(thetas)
                if isinstance(res, dict):
                    T = (res.get("T_0_6") or res.get("t_0_6")
                         or res.get("transform"))
                    tcp = (res.get("TCP") or res.get("tcp") or res.get("X_Y_Z")
                           or res.get("cartesian"))
                    if tcp is not None:
                        if isinstance(tcp, (list, tuple)):
                            cart = {"x": float(tcp[0]), "y": float(tcp[1]),
                                    "z": float(tcp[2])}
                        elif isinstance(tcp, dict):
                            cart = {"x": float(tcp.get("x", tcp.get("X", 0))),
                                    "y": float(tcp.get("y", tcp.get("Y", 0))),
                                    "z": float(tcp.get("z", tcp.get("Z", 0)))}
                    ori = res.get("orientation") or res.get("orientation_deg")
                    if ori is not None:
                        if isinstance(ori, (list, tuple)):
                            orient = {"pitch": float(ori[0]),
                                      "roll": float(ori[1]),
                                      "yaw": float(ori[2])}
                        elif isinstance(ori, dict):
                            orient = {
                                "pitch": float(ori.get("pitch", ori.get("Pitch", 90))),
                                "roll": float(ori.get("roll", ori.get("Roll", 0))),
                                "yaw": float(ori.get("yaw", ori.get("Yaw", 0)))}
            except Exception:
                T = None
        if T is None:
            T = self._home_T() if all(abs(thetas[i] - [0, 90, 0, 0, 0, 50][i]) < 1e-9
                                     for i in range(6)) else self._home_T()
        if cart is None:
            cart = self.state.get("cartesian_mm", {"x": 0.0, "y": 0.0, "z": 570.0})
        if orient is None:
            orient = self.state.get("orientation_deg",
                                    {"pitch": 90.0, "roll": 0.0, "yaw": 0.0})

        # normalize T to 4x4 floats
        try:
            T = [[float(v) for v in row] for row in T]
        except Exception:
            T = self._home_T()

        self.state["T_0_6"] = T
        self.state["cartesian_mm"] = {
            "x": float(cart["x"]), "y": float(cart["y"]), "z": float(cart["z"])}
        self.state["orientation_deg"] = {
            "pitch": float(orient["pitch"]),
            "roll": float(orient["roll"]),
            "yaw": float(orient["yaw"])}
        self.state["grip_percent"] = float(thetas[5]) if len(thetas) > 5 else self.state.get("grip_percent", 50.0)
        # note: grip_percent is percent 0..100; theta6 stored as given

        if command_str:
            hist = self.state.setdefault("command_history", [])
            hist.append({"timestamp": self._now(), "cmd": command_str,
                         "summary": "thetas=%s" % thetas})
            if len(hist) > self.max_history:
                del hist[: len(hist) - self.max_history]

        self._save()
        return self.state

    def set_shoulder_coupling(self, coupling: int) -> Dict[str, Any]:
        c = int(coupling)
        if c not in (1, -1):
            raise ValueError("shoulder_coupling must be +1 or -1")
        self.state["shoulder_coupling"] = c
        # mirror config if present
        if "shoulder_coupling" in self.state:
            pass
        self._save()
        return self.state

    def update_motor_calibration(self, motor: int, stop_pulse: Optional[int] = None,
                                 deg_per_sec: Optional[float] = None) -> Dict[str, Any]:
        key = str(int(motor))
        cal = self.state.setdefault("calibration", {}).setdefault(key, {})
        if stop_pulse is not None:
            cal["stop_pulse"] = int(stop_pulse)
        if deg_per_sec is not None:
            cal["deg_per_sec"] = float(deg_per_sec)
        # keep top-level? calibration only
        self._save()
        return self.state


# convenience alias
ShoulderCouplingError = ShoulderCouplingError
