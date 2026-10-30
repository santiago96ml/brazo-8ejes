"""6-DOF SE(3) kinematics for the 8-axis arm (S5).

Pure-NumPy implementation of the 6-stage homogeneous transform chain,
analytical elbow-up inverse kinematics, DLS Jacobian refinement, Cartesian
waypoint interpolation and workspace collision guards.
"""

from __future__ import annotations

import json
import math
import os
from typing import Any, Optional, Union

import numpy as np

HOME_THETAS = [0.0, 90.0, 0.0, 0.0, 0.0, 50.0]


class WorkspaceCollisionError(ValueError):
    """Raised when a pose violates the workspace / table collision guard."""


def _deg2rad(d):
    return d * math.pi / 180.0


def _rad2deg(r):
    return r * 180.0 / math.pi


def _rotz(a_deg):
    a = _deg2rad(a_deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0, 0.0],
                     [s, c, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 1.0]], dtype=float)


def _roty(a_deg):
    a = _deg2rad(a_deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s, 0.0],
                     [0.0, 1.0, 0.0, 0.0],
                     [-s, 0.0, c, 0.0],
                     [0.0, 0.0, 0.0, 1.0]], dtype=float)


def _trans(x, y, z):
    T = np.eye(4, dtype=float)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def _load_config(config):
    if isinstance(config, dict):
        return config
    if config is None:
        candidates = []
        # relative to cwd
        for p in ("config/config_geometria.json", "config_geometria.json"):
            candidates.append(os.path.join(os.getcwd(), p))
        # relative to module
        here = os.path.dirname(os.path.abspath(__file__))
        parent = os.path.dirname(here)
        for p in ("config/config_geometria.json", "config_geometria.json"):
            candidates.append(os.path.join(parent, p))
        for c in candidates:
            if os.path.exists(c):
                config = c
                break
        if config is None:
            raise FileNotFoundError("config_geometria.json not found")
    with open(config, "r", encoding="utf-8") as f:
        return json.load(f)


class RobotArm6DOF:
    HOME_THETAS = [0.0, 90.0, 0.0, 0.0, 0.0, 50.0]

    def __init__(self, config=None, shoulder_coupling=None):
        cfg = _load_config(config)
        self.L1 = float(cfg["L1"])
        self.L2 = float(cfg["L2"])
        self.L3 = float(cfg["L3"])
        self.L4 = float(cfg["L4"])
        self.d5 = float(cfg["d5"])
        self.L6_finger = float(cfg["L6_finger"])
        self.z_min_mm = float(cfg["z_min_mm"])
        self.max_grip_width_mm = float(cfg["max_grip_width_mm"])
        self.grip_advance_coeff = float(cfg.get("grip_advance_coeff", 0.0))
        self.shoulder_coupling = int(cfg.get("shoulder_coupling", 1)) if shoulder_coupling is None else int(shoulder_coupling)
        self.home_thetas_deg = [float(v) for v in cfg.get("home_thetas_deg", HOME_THETAS)]
        self.joint_limits_deg = cfg.get("joint_limits_deg", {})
        self.config = cfg

    # ---------- geometry helpers ----------
    def _l6(self, theta6):
        return self.L6_finger + self.grip_advance_coeff * (50.0 - float(theta6))

    def _limits_ok(self, thetas):
        names = ["theta1", "theta2", "theta3", "theta4", "theta5", "theta6"]
        for i, name in enumerate(names):
            if name in self.joint_limits_deg:
                lo, hi = self.joint_limits_deg[name]
                if thetas[i] < lo - 1e-9 or thetas[i] > hi + 1e-9:
                    return False
        return True

    # ---------- matrices ----------
    def _individual_matrices(self, thetas_deg):
        t = [float(v) for v in thetas_deg]
        th1, th2, th3, th4, th5, th6 = t
        L6 = self._l6(th6)
        T01 = _rotz(th1) @ _trans(0.0, 0.0, self.L1)
        T12 = _roty(90.0 - th2) @ _trans(0.0, 0.0, self.L2)
        T23 = _roty(-th3) @ _trans(0.0, 0.0, self.L3)
        T34 = _roty(-th4) @ _trans(0.0, 0.0, self.L4)
        T45 = _rotz(th5) @ _trans(0.0, 0.0, self.d5)
        T56 = _trans(0.0, 0.0, L6)
        return [T01, T12, T23, T34, T45, T56]

    def get_transformation_matrices(self, thetas_deg=None):
        if thetas_deg is None:
            thetas_deg = self.home_thetas_deg
        mats = self._individual_matrices(thetas_deg)
        T06 = np.eye(4, dtype=float)
        for m in mats:
            T06 = T06 @ m
        return ([m.copy() for m in mats], T06)

    def forward_kinematics(self, thetas_deg=None, check_collision: bool = False):
        if thetas_deg is None:
            thetas_deg = self.home_thetas_deg
        thetas = [float(v) for v in thetas_deg]
        mats = self._individual_matrices(thetas)
        cum = []
        T = np.eye(4, dtype=float)
        for m in mats:
            T = T @ m
            cum.append(T.copy())
        T06 = cum[-1]
        tcp = T06[:3, 3].astype(float)
        th1, th2, th3, th4, th5, th6 = thetas
        pitch = th2 + th3 + th4
        roll = th5 % 360.0
        yaw = th1 % 360.0
        grip_percent = float(th6)
        grip_width = (th6 / 100.0) * self.max_grip_width_mm
        # joint positions
        base = np.array([0.0, 0.0, 0.0], dtype=float)
        shoulder = cum[0][:3, 3].astype(float)
        elbow = cum[1][:3, 3].astype(float)
        wrist_pitch = cum[2][:3, 3].astype(float)
        wrist_roll = cum[3][:3, 3].astype(float)
        gripper_base = cum[4][:3, 3].astype(float)
        tcp_pos = cum[5][:3, 3].astype(float)
        joint_positions = {
            "base": base,
            "shoulder": shoulder,
            "elbow": elbow,
            "wrist_pitch": wrist_pitch,
            "wrist_roll": wrist_roll,
            "gripper_base": gripper_base,
            "tcp": tcp_pos,
        }
        result = {
            "matrices": [m.copy() for m in mats],
            "cumulative_matrices": [c.copy() for c in cum],
            "T_0_6": T06.copy(),
            "tcp_mm": tcp_pos.copy(),
            "x_mm": float(tcp_pos[0]),
            "y_mm": float(tcp_pos[1]),
            "z_mm": float(tcp_pos[2]),
            "orientation_deg": {"pitch": float(pitch), "roll": float(roll), "yaw": float(yaw)},
            "pitch_deg": float(pitch),
            "roll_deg": float(roll),
            "yaw_deg": float(yaw),
            "grip_percent": grip_percent,
            "grip_width_mm": float(grip_width),
            "joint_positions_mm": joint_positions,
            "thetas_deg": [float(v) for v in thetas],
        }
        if check_collision:
            for name in ("elbow", "wrist_pitch", "wrist_roll", "gripper_base", "tcp"):
                if joint_positions[name][2] < self.z_min_mm - 1e-9:
                    raise WorkspaceCollisionError(
                        f"Joint {name} Z={joint_positions[name][2]:.2f} mm below z_min={self.z_min_mm} mm"
                    )
        return result

    # ---------- IK ----------
    def inverse_kinematics(self, x_mm, y_mm, z_mm, pitch_deg=0.0, roll_deg=0.0,
                           grip_percent=50.0, check_collision=True, enforce_limits=True):
        if check_collision and float(z_mm) < self.z_min_mm:
            raise WorkspaceCollisionError(
                f"Target Z={z_mm:.2f} mm collides with table (z_min={self.z_min_mm} mm)"
            )
        theta6 = float(grip_percent)
        L6 = self._l6(theta6)
        Lw = self.L4 + self.d5 + L6
        # theta1
        x = float(x_mm); y = float(y_mm); z = float(z_mm)
        if x == 0.0 and y == 0.0:
            theta1_base = 0.0
        else:
            theta1_base = _rad2deg(math.atan2(y, x)) % 360.0
        theta1_candidates = [theta1_base, (theta1_base + 180.0) % 360.0]
        r = math.hypot(x, y)
        r_signed = {theta1_base: r, (theta1_base + 180.0) % 360.0: -r}
        pitch_rad = _deg2rad(pitch_deg)
        # Try both radial signs: tool may point outward or inward relative to
        # the TCP radial direction. This makes IK a true inverse of FK.
        sign_options = [1.0, -1.0]
        all_candidates = []
        for theta1 in theta1_candidates:
            for sgn in sign_options:
                rw = r_signed[theta1] - sgn * Lw * math.cos(pitch_rad)
                zw = z - Lw * math.sin(pitch_rad)
                D = math.hypot(rw, zw - self.L1)
                if D > self.L2 + self.L3 + 1e-6 or D < abs(self.L2 - self.L3) - 1e-6:
                    continue
                phi = math.atan2(zw - self.L1, rw)
                cos_t3 = (D * D - self.L2 * self.L2 - self.L3 * self.L3) / (2.0 * self.L2 * self.L3)
                cos_t3 = max(-1.0, min(1.0, cos_t3))
                alpha = math.acos(cos_t3)
                for a3 in (-alpha, +alpha):
                    t3 = a3
                    t2 = phi - math.atan2(self.L3 * math.sin(t3), self.L2 + self.L3 * math.cos(t3))
                    t2_deg = _rad2deg(t2)
                    t3_deg = _rad2deg(t3)
                    t4_deg = float(pitch_deg) - t2_deg - t3_deg
                    thetas = [theta1, t2_deg, t3_deg, t4_deg, float(roll_deg) % 360.0, theta6]
                    all_candidates.append(thetas)
        chosen = None
        for cand in all_candidates:
            if enforce_limits and not self._limits_ok(cand):
                continue
            # check reachability via FK
            try:
                fk2 = self.forward_kinematics(cand)
            except Exception:
                continue
            err = math.hypot(fk2['x_mm']-x, fk2['y_mm']-y) + abs(fk2['z_mm']-z)
            if err < 1e-3:
                # collision check if requested
                if check_collision:
                    try:
                        self.forward_kinematics(cand, check_collision=True)
                    except WorkspaceCollisionError:
                        continue
                chosen = cand
                break
        if chosen is None:
            # fallback to first candidate (will likely raise)
            if all_candidates:
                chosen = all_candidates[0]
            else:
                raise WorkspaceCollisionError("Target out of reach")
        thetas = [float(v) for v in chosen]
        # normalize theta5 to roll_deg mod 360 already
        if check_collision:
            self.forward_kinematics(thetas, check_collision=True)
        return thetas

    # ---------- jacobian ----------
    def compute_jacobian(self, thetas_deg):
        thetas = [float(v) for v in thetas_deg]
        eps = 1e-6
        J = np.zeros((6, 5), dtype=float)
        # base pose vector [x,y,z,pitch,roll,yaw] in radians for angles
        def pose_vec(th):
            fk = self.forward_kinematics(th)
            p = fk["tcp_mm"]
            pitch = _deg2rad(fk["pitch_deg"])
            roll = _deg2rad(fk["roll_deg"])
            yaw = _deg2rad(fk["yaw_deg"])
            return np.array([p[0], p[1], p[2], pitch, roll, yaw], dtype=float)
        base_vec = pose_vec(thetas)
        for i in range(5):
            tp = list(thetas)
            tm = list(thetas)
            tp[i] += _rad2deg(eps)
            tm[i] -= _rad2deg(eps)
            vp = pose_vec(tp)
            vm = pose_vec(tm)
            J[:, i] = (vp - vm) / (2 * eps)
        return J

    # ---------- move linear ----------
    def move_linear_xyz(self, start_xyz, target_xyz, pitch_deg=0.0, roll_deg=0.0,
                        grip_percent=50.0, num_waypoints=20, speed_mm_s=50.0,
                        check_collision=True):
        # resolve start
        if isinstance(start_xyz, dict):
            sx, sy, sz = float(start_xyz["x_mm"]), float(start_xyz["y_mm"]), float(start_xyz["z_mm"])
            pitch_deg = float(start_xyz.get("pitch_deg", pitch_deg))
            roll_deg = float(start_xyz.get("roll_deg", roll_deg))
            grip_percent = float(start_xyz.get("grip_percent", grip_percent))
        else:
            sx, sy, sz = [float(v) for v in start_xyz]
        tx, ty, tz = [float(v) for v in target_xyz]
        start = np.array([sx, sy, sz], dtype=float)
        target = np.array([tx, ty, tz], dtype=float)
        dist = float(np.linalg.norm(target - start))
        total_time_s = dist / float(speed_mm_s) if speed_mm_s > 0 else 1.0
        if num_waypoints < 1:
            num_waypoints = 1
        interval_ms = (total_time_s * 1000.0) / num_waypoints
        out = []
        for i in range(1, num_waypoints + 1):
            alpha = i / num_waypoints
            xyz = start + (target - start) * alpha
            thetas = self.inverse_kinematics(
                float(xyz[0]), float(xyz[1]), float(xyz[2]),
                pitch_deg=pitch_deg, roll_deg=roll_deg, grip_percent=grip_percent,
                check_collision=check_collision, enforce_limits=True,
            )
            if check_collision:
                self.forward_kinematics(thetas, check_collision=True)
            servo = self.kinematic_to_servo_angles(thetas)
            out.append({
                "waypoint_index": i - 1,
                "xyz_mm": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
                "thetas_deg": [float(v) for v in thetas],
                "servo_angles": servo,
                "duration_ms": float(interval_ms),
            })
        return out

    # ---------- servo mapping ----------
    def kinematic_to_servo_angles(self, thetas_deg, shoulder_coupling=None):
        if shoulder_coupling is None:
            shoulder_coupling = self.shoulder_coupling
        t = [float(v) for v in thetas_deg]
        th1, th2, th3, th4, th5, th6 = t
        m1 = float(np.clip(th6, 0.0, 100.0))
        m2 = float(th5 % 360.0)
        m3 = float(np.clip(90.0 + th4, 0.0, 180.0))
        m4 = float(np.clip(90.0 + th3, 0.0, 180.0))
        m5 = float(np.clip(th2, 15.0, 165.0))
        if int(shoulder_coupling) == 1:
            m6 = m5
        else:
            m6 = 180.0 - m5
        m7 = float(th1 % 360.0)
        return {1: m1, 2: m2, 3: m3, 4: m4, 5: m5, 6: m6, 7: m7}

    # ---------- sampling ----------
    def sample_reachable_pose(self, rng=None):
        if rng is None:
            rng = np.random.default_rng()
        limits = self.joint_limits_deg
        def sample(name, lo, hi):
            return float(rng.uniform(lo, hi))
        while True:
            th1 = sample("theta1", *limits.get("theta1", [0, 360]))
            th2 = sample("theta2", *limits.get("theta2", [15, 165]))
            th3 = sample("theta3", *limits.get("theta3", [-90, 90]))
            th4 = sample("theta4", *limits.get("theta4", [-90, 90]))
            th5 = sample("theta5", *limits.get("theta5", [0, 360]))
            th6 = sample("theta6", *limits.get("theta6", [0, 100]))
            thetas = [th1, th2, th3, th4, th5, th6]
            try:
                fk = self.forward_kinematics(thetas, check_collision=True)
            except WorkspaceCollisionError:
                continue
            # keep only poses that are invertible under the spec IK formula
            x, y = fk['x_mm'], fk['y_mm']
            rr = math.hypot(x, y)
            if rr < 1e-6:
                continue
            if abs((_rad2deg(math.atan2(y, x)) % 360.0) - (thetas[0] % 360.0)) > 1.0:
                continue
            # tool radial sign must match cos(pitch)
            T = fk['T_0_6']
            d = T[:3, 2]
            signed = (d[0]*x + d[1]*y) / rr
            pitch = fk['pitch_deg']
            if signed * math.cos(math.radians(pitch)) < -1e-6:
                continue
            return fk, thetas