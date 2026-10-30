"""Pulse odometry bridge for PCA9685 pulse <-> motor degrees <-> kinematic thetas.

Mirrors firmware_mega/firmware_mega.ino degToPulse/pulseToDeg behavior exactly.
"""

from __future__ import annotations

import json
import math
import os
from typing import Any, Dict, List, Optional


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

# Firmware defaults
_DEFAULT_HOME = {
    1: 50.0,   # PINZA
    2: 0.0,    # GIRO_PINZA
    3: 90.0,   # INCLINA_PINZA
    4: 90.0,   # CODO
    5: 90.0,   # HOMBRO_IZQ
    6: 90.0,   # HOMBRO_DER
    7: 0.0,    # BASE
    8: 90.0,   # AUX
}


class PulseOdometry:
    def __init__(self, config_path: str | None = None, shoulder_coupling: int = 1):
        self.shoulder_coupling = 1 if shoulder_coupling != -1 else -1
        self._calib: Dict[int, Dict[str, Any]] = {}
        self._load_config(config_path)
        self._validate()

    def _find_config(self, config_path: str | None) -> str:
        if config_path and os.path.exists(config_path):
            return config_path
        # Try common locations
        candidates = [
            os.path.join("config", "arm.json"),
            os.path.join(os.path.dirname(__file__), "..", "config", "arm.json"),
            "arm.json",
        ]
        for c in candidates:
            if os.path.exists(c):
                return c
        return config_path or os.path.join("config", "arm.json")

    def _load_config(self, config_path: str | None) -> None:
        path = self._find_config(config_path)
        cfg = {}
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
            except Exception:
                cfg = {}
        joints = cfg.get("joints", [])
        # Initialize defaults from firmware
        for m in range(1, 9):
            self._calib[m] = {
                "pulse_min": 110,
                "pulse_stop": 307,
                "pulse_max": 500,
                "home_deg": _DEFAULT_HOME.get(m, 90.0),
                "min_deg": 0.0,
                "max_deg": 180.0,
            }
        # Firmware specific defaults
        self._calib[1] = {"pulse_min": 110, "pulse_stop": 307, "pulse_max": 500, "home_deg": 50.0, "min_deg": 0.0, "max_deg": 100.0}
        self._calib[2] = {"pulse_min": 110, "pulse_stop": 307, "pulse_max": 500, "home_deg": 0.0, "min_deg": -360.0, "max_deg": 360.0}
        self._calib[3] = {"pulse_min": 110, "pulse_stop": 307, "pulse_max": 500, "home_deg": 90.0, "min_deg": 0.0, "max_deg": 180.0}
        self._calib[4] = {"pulse_min": 110, "pulse_stop": 307, "pulse_max": 500, "home_deg": 90.0, "min_deg": 0.0, "max_deg": 180.0}
        self._calib[5] = {"pulse_min": 110, "pulse_stop": 307, "pulse_max": 500, "home_deg": 90.0, "min_deg": 15.0, "max_deg": 165.0}
        self._calib[6] = {"pulse_min": 110, "pulse_stop": 307, "pulse_max": 500, "home_deg": 90.0, "min_deg": 15.0, "max_deg": 165.0}
        self._calib[7] = {"pulse_min": 110, "pulse_stop": 307, "pulse_max": 500, "home_deg": 0.0, "min_deg": -360.0, "max_deg": 360.0}
        self._calib[8] = {"pulse_min": 110, "pulse_stop": 307, "pulse_max": 500, "home_deg": 90.0, "min_deg": 0.0, "max_deg": 180.0}
        # Override from config
        for j in joints:
            m = j.get("motor")
            if not isinstance(m, int) or m < 1 or m > 8:
                continue
            c = self._calib[m]
            for k in ("pulse_min", "pulse_stop", "pulse_max", "home_deg"):
                if k in j and j[k] is not None:
                    c[k] = float(j[k])
            # limits_deg
            limits = j.get("limits_deg")
            if limits and isinstance(limits, (list, tuple)) and len(limits) == 2:
                c["min_deg"] = float(limits[0])
                c["max_deg"] = float(limits[1])
            # Also check alternate keys
            if "min_deg" in j:
                c["min_deg"] = float(j["min_deg"])
            if "max_deg" in j:
                c["max_deg"] = float(j["max_deg"])
        # Enforce firmware defaults as per spec
        self._calib[1]["min_deg"] = 0.0
        self._calib[1]["max_deg"] = 100.0
        self._calib[2]["min_deg"] = -360.0
        self._calib[2]["max_deg"] = 360.0
        self._calib[3]["min_deg"] = 0.0
        self._calib[3]["max_deg"] = 180.0
        self._calib[4]["min_deg"] = 0.0
        self._calib[4]["max_deg"] = 180.0
        self._calib[5]["min_deg"] = 15.0
        self._calib[5]["max_deg"] = 165.0
        self._calib[6]["min_deg"] = 15.0
        self._calib[6]["max_deg"] = 165.0
        self._calib[7]["min_deg"] = -360.0
        self._calib[7]["max_deg"] = 360.0
        self._calib[8]["min_deg"] = 0.0
        self._calib[8]["max_deg"] = 180.0

    def _validate(self) -> None:
        for m in range(1, 9):
            c = self._calib[m]
            pmin = c["pulse_min"]
            pstop = c["pulse_stop"]
            pmax = c["pulse_max"]
            if not (pmin < pstop < pmax):
                raise ValueError(
                    f"Invalid pulse calibration for motor {m}: "
                    f"pulse_min({pmin}) < pulse_stop({pstop}) < pulse_max({pmax}) required"
                )

    def pulse_to_deg(self, motor: int, pulse: int) -> float:
        if motor < 1 or motor > 8:
            raise ValueError("Motor must be 1..8")
        c = self._calib[motor]
        mn = float(c["min_deg"])
        mx = float(c["max_deg"])
        hm = float(c["home_deg"])
        st = float(c["pulse_stop"])
        pf = float(pulse)
        pmin = float(c["pulse_min"])
        pmax = float(c["pulse_max"])
        if abs(pf - st) < 0.5:
            return hm
        if pf > st and pmax > st:
            t = (pf - st) / (pmax - st)
            if t < 0.0:
                t = 0.0
            if t > 1.0:
                t = 1.0
            return hm + t * (mx - hm)
        if pf < st and st > pmin:
            t = (st - pf) / (st - pmin)
            if t < 0.0:
                t = 0.0
            if t > 1.0:
                t = 1.0
            return hm - t * (hm - mn)
        return hm

    def deg_to_pulse(self, motor: int, deg: float) -> int:
        if motor < 1 or motor > 8:
            raise ValueError("Motor must be 1..8")
        c = self._calib[motor]
        mn = float(c["min_deg"])
        mx = float(c["max_deg"])
        hm = float(c["home_deg"])
        st = float(c["pulse_stop"])
        pmin = float(c["pulse_min"])
        pmax = float(c["pulse_max"])
        d = float(deg)
        # clamp to joint limits
        if d < mn:
            d = mn
        if d > mx:
            d = mx
        p = st
        if abs(d - hm) < 0.05:
            p = st
        elif d > hm and mx > hm:
            t = (d - hm) / (mx - hm)
            if t < 0.0:
                t = 0.0
            if t > 1.0:
                t = 1.0
            p = st + t * (pmax - st)
        elif d < hm and hm > mn:
            t = (hm - d) / (hm - mn)
            if t < 0.0:
                t = 0.0
            if t > 1.0:
                t = 1.0
            p = st - t * (st - pmin)
        if p < 0:
            p = 0
        if p > 4095:
            p = 4095
        return int(round(p))

    def pulses_to_motors(self, pulses: Dict[int, int]) -> Dict[int, float]:
        result = {}
        for m in range(1, 9):
            if m in pulses:
                result[m] = self.pulse_to_deg(m, pulses[m])
            else:
                result[m] = float(self._calib[m]["home_deg"])
        return result

    def motors_to_pulses(self, motors: Dict[int, float]) -> Dict[int, int]:
        result = {}
        for m in range(1, 9):
            if m in motors:
                result[m] = self.deg_to_pulse(m, motors[m])
            else:
                result[m] = int(round(self._calib[m]["pulse_stop"]))
        return result

    def motors_to_thetas(self, motors: Dict[int, float]) -> List[float]:
        m1 = float(motors.get(1, self._calib[1]["home_deg"]))
        m2 = float(motors.get(2, self._calib[2]["home_deg"]))
        m3 = float(motors.get(3, self._calib[3]["home_deg"]))
        m4 = float(motors.get(4, self._calib[4]["home_deg"]))
        m5 = float(motors.get(5, self._calib[5]["home_deg"]))
        m7 = float(motors.get(7, self._calib[7]["home_deg"]))
        # Apply coupling to get m6 if present? motors_to_thetas takes motors dict
        theta6 = m1  # PINZA
        theta5 = m2 % 360.0  # GIRO_PINZA
        theta4 = m3 - 90.0  # INCLINA_PINZA
        theta3 = m4 - 90.0  # CODO
        theta2 = m5  # HOMBRO_IZQ
        theta1 = m7 % 360.0  # BASE
        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def thetas_to_motors(self, thetas: List[float]) -> Dict[int, float]:
        if len(thetas) < 6:
            raise ValueError("thetas must have at least 6 elements")
        theta1 = float(thetas[0])
        theta2 = float(thetas[1])
        theta3 = float(thetas[2])
        theta4 = float(thetas[3])
        theta5 = float(thetas[4])
        theta6 = float(thetas[5])
        m1 = theta6  # PINZA 0..100
        m2 = theta5 % 360.0  # GIRO_PINZA
        m3 = 90.0 + theta4  # INCLINA_PINZA
        m4 = 90.0 + theta3  # CODO
        m5 = theta2  # HOMBRO_IZQ
        if self.shoulder_coupling == 1:
            m6 = m5
        else:
            m6 = 180.0 - m5  # mirror
        m7 = theta1 % 360.0  # BASE
        m8 = 90.0  # AUX passthrough
        # Apply clipping as per rules
        # m1 = theta6 (PINZA 0..100) - clip to 0..100? rule says "m1 = theta6 (PINZA 0..100)"
        if m1 < 0.0:
            m1 = 0.0
        if m1 > 100.0:
            m1 = 100.0
        m2 = ((m2 % 360.0) + 360.0) % 360.0
        if m3 < 0.0:
            m3 = 0.0
        if m3 > 180.0:
            m3 = 180.0
        if m4 < 0.0:
            m4 = 0.0
        if m4 > 180.0:
            m4 = 180.0
        if m5 < 15.0:
            m5 = 15.0
        if m5 > 165.0:
            m5 = 165.0
        if self.shoulder_coupling == 1:
            m6 = m5
        else:
            m6 = 180.0 - m5
        m7 = ((m7 % 360.0) + 360.0) % 360.0
        m8 = 90.0
        return {1: m1, 2: m2, 3: m3, 4: m4, 5: m5, 6: m6, 7: m7, 8: m8}

    def clamp(self, motor: int, deg: float) -> float:
        if motor < 1 or motor > 8:
            raise ValueError("Motor must be 1..8")
        c = self._calib[motor]
        mn = float(c["min_deg"])
        mx = float(c["max_deg"])
        d = float(deg)
        if d < mn:
            return mn
        if d > mx:
            return mx
        return d

    def calibration(self) -> Dict[int, Dict]:
        return {k: dict(v) for k, v in self._calib.items()}

    def set_shoulder_coupling(self, coupling: int) -> None:
        if coupling == 1 or coupling == -1:
            self.shoulder_coupling = coupling
        else:
            raise ValueError("coupling must be +1 or -1")
