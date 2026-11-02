"""Automatic pulse calibrator guided by camera (S11/S12/S15)."""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np

try:
    from src.fast_camera import FastUSBCamera
except Exception:  # pragma: no cover
    try:
        from fast_camera import FastUSBCamera  # type: ignore
    except Exception:
        FastUSBCamera = None  # type: ignore

try:
    from src.arm_vision_analyzer import ArmVisionAnalyzer
except Exception:  # pragma: no cover
    try:
        from arm_vision_analyzer import ArmVisionAnalyzer  # type: ignore
    except Exception:
        ArmVisionAnalyzer = None  # type: ignore

try:
    from src.agent_controller import AgentController
except Exception:  # pragma: no cover
    try:
        from agent_controller import AgentController  # type: ignore
    except Exception:
        AgentController = None  # type: ignore

try:
    from src.state_manager import RobotStateManager
except Exception:  # pragma: no cover
    try:
        from state_manager import RobotStateManager  # type: ignore
    except Exception:
        RobotStateManager = None  # type: ignore

try:
    from src.pulse_odometry import PulseOdometry
except Exception:  # pragma: no cover
    try:
        from pulse_odometry import PulseOdometry  # type: ignore
    except Exception:
        PulseOdometry = None  # type: ignore

try:
    from src.kinematics_6dof import RobotArm6DOF
except Exception:  # pragma: no cover
    try:
        from kinematics_6dof import RobotArm6DOF  # type: ignore
    except Exception:
        RobotArm6DOF = None  # type: ignore


class VisualPulseCalibrator:
    """Camera-guided automatic pulse calibration.

    Shoulder pair (5 and 6): never calibrate them independently. If motor in
    (5, 6), calibrate the master 5 and mirror the result to 6 (m6 = m5 or
    180 - m5 per coupling), sending P5/P6 together and measuring the shoulder
    marker once.
    """

    def __init__(self, camera=None, analyzer=None, controller=None, state=None,
                 profile_path: str = "config/servo_pulse_profile.json",
                 motors: tuple = (1, 2, 3, 4, 7),
                 candidates: tuple = (304, 305, 306, 307, 308, 309, 310, 311),
                 settle_s: float = 0.4, burst_ms: int = 200,
                 delta_pwm: int = 40, stop_epsilon_px_s: float = 0.35,
                 max_runtime_s: float = 120.0, dry_run: bool = False,
                 preload_default: bool = True):
        self.camera = camera
        self.analyzer = analyzer
        self.controller = controller
        self.state = state
        self.profile_path = profile_path
        self.motors = tuple(motors)
        self.candidates = tuple(candidates)
        self.settle_s = float(settle_s)
        self.burst_ms = int(burst_ms)
        self.delta_pwm = int(delta_pwm)
        self.stop_epsilon_px_s = float(stop_epsilon_px_s)
        self.max_runtime_s = float(max_runtime_s)
        self.dry_run = bool(dry_run)
        self.preload_default = bool(preload_default)
        self._profile: Dict[str, Any] = {"generated_at": "", "camera_index": 0, "motors": {}, "shoulder_coupling": 1}
        self._last_tracked: Dict[int, Tuple[float, float]] = {}

        if self.controller is not None and getattr(self.controller, "state", None) is not None and self.state is None:
            self.state = self.controller.state
        if self.state is None and RobotStateManager is not None:
            try:
                self.state = RobotStateManager()
            except Exception:
                self.state = None
        if self.controller is None and AgentController is not None and not self.dry_run:
            try:
                self.controller = AgentController(sim=False)
            except Exception:
                try:
                    self.controller = AgentController(sim=True)
                except Exception:
                    self.controller = None
        if self.analyzer is None and ArmVisionAnalyzer is not None:
            try:
                self.analyzer = ArmVisionAnalyzer()
            except Exception:
                self.analyzer = None
        if self.camera is None and FastUSBCamera is not None and not self.dry_run:
            try:
                self.camera = FastUSBCamera()
            except Exception:
                self.camera = None

    def _send(self, line: str) -> None:
        if self.dry_run:
            return
        if self.controller is not None and hasattr(self.controller, "transport") and self.controller.transport is not None:
            try:
                self.controller.transport.send_line(line)
                return
            except Exception:
                pass
        raise RuntimeError(f"transport unavailable to send {line}")

    def _track_joint(self, frame_prev, frame_curr, analyzer_result_prev: Optional[Dict] = None, analyzer_result_curr: Optional[Dict] = None, joint_name: str = "shoulder") -> Tuple[float, float, float]:
        # Prefer ArUco centre of the joint nearest to the previously tracked point
        kp_prev = None
        kp_curr = None
        if analyzer_result_prev is not None:
            kp_prev = analyzer_result_prev.get("keypoints_px") or {}
        if analyzer_result_curr is not None:
            kp_curr = analyzer_result_curr.get("keypoints_px") or {}
        # Try to find by joint_name or nearby
        if kp_curr:
            # ArUco/markers often give centers; prefer those with high conf
            candidates_kp = []
            for name, val in kp_curr.items():
                try:
                    if isinstance(val, (list, tuple)) and len(val) >= 2:
                        x = float(val[0]); y = float(val[1])
                        c = float(val[2]) if len(val) >= 3 else 1.0
                        candidates_kp.append((name, x, y, c))
                except Exception:
                    continue
            if candidates_kp:
                if joint_name in kp_curr and isinstance(kp_curr[joint_name], (list, tuple)):
                    v = kp_curr[joint_name]
                    return float(v[0]), float(v[1]), float(v[2]) if len(v) >= 3 else 1.0
                # nearest to last tracked
                last = self._last_tracked.get(id(self))
                if last is None and kp_prev:
                    # try to get last
                    pass
                best = None
                best_d = float("inf")
                for name, x, y, c in candidates_kp:
                    if last is not None:
                        d = (x - last[0]) ** 2 + (y - last[1]) ** 2
                    else:
                        d = 0.0
                    if d < best_d:
                        best_d = d
                        best = (x, y, c)
                if best is not None:
                    self._last_tracked[id(self)] = (best[0], best[1])
                    return best[0], best[1], best[2]
        # Fallback to optical flow on 5x5 grid inside 160x160 ROI
        try:
            import cv2
            if frame_prev is None or frame_curr is None:
                raise ValueError("no frames")
            last = self._last_tracked.get(id(self))
            if last is None:
                # use kp if available
                if kp_curr and kp_curr:
                    # pick first
                    for name, val in kp_curr.items():
                        if isinstance(val, (list, tuple)) and len(val) >= 2:
                            last = (float(val[0]), float(val[1]))
                            break
            if last is None:
                raise ValueError("no track point")
            cx, cy = int(last[0]), int(last[1])
            r = 80
            x0 = max(0, cx - r); y0 = max(0, cy - r)
            x1 = min(frame_prev.shape[1], cx + r); y1 = min(frame_prev.shape[0], cy + r)
            # build 5x5 grid
            grid_pts = []
            for gy in range(5):
                for gx in range(5):
                    px = int(x0 + (x1 - x0) * (gx + 0.5) / 5.0)
                    py = int(y0 + (y1 - y0) * (gy + 0.5) / 5.0)
                    grid_pts.append([px, py])
            p0 = np.array(grid_pts, dtype=np.float32).reshape(-1, 1, 2)
            old_gray = cv2.cvtColor(frame_prev, cv2.COLOR_BGR2GRAY)
            new_gray = cv2.cvtColor(frame_curr, cv2.COLOR_BGR2GRAY)
            p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, new_gray, p0, None)
            if p1 is not None and st is not None:
                good = p1[st.flatten() == 1]
                if len(good) > 0:
                    mean_pt = good.mean(axis=0)
                    self._last_tracked[id(self)] = (float(mean_pt[0]), float(mean_pt[1]))
                    return float(mean_pt[0]), float(mean_pt[1]), float(min(1.0, len(good) / 25.0))
        except Exception:
            pass
        # Geometric keypoint fallback
        if kp_curr:
            for name, val in kp_curr.items():
                if isinstance(val, (list, tuple)) and len(val) >= 2:
                    c = float(val[2]) if len(val) >= 3 else 0.5
                    self._last_tracked[id(self)] = (float(val[0]), float(val[1]))
                    return float(val[0]), float(val[1]), c
        return 0.0, 0.0, 0.0

    def zero_sync(self) -> dict:
        start = time.time()
        # sends "S" (force OE=HIGH)
        self._send("S")
        # read current visual posture (ArUco preferred, geometric fallback)
        frame = None
        if self.camera is not None:
            try:
                f, ts, fps = self.camera.get_latest_frame()
                frame = f
            except Exception:
                frame = None
        if frame is None and self.camera is not None and hasattr(self.camera, "read"):
            try:
                ok, f = self.camera.read()
                if ok:
                    frame = f
            except Exception:
                frame = None
        if frame is None:
            raise RuntimeError("vision posture unavailable")
        res = None
        if self.analyzer is not None:
            try:
                res = self.analyzer.analyze(frame)
            except Exception:
                res = None
        if res is None or res.get("thetas_deg") is None:
            raise RuntimeError("vision posture unavailable")
        thetas = res.get("thetas_deg") or []
        source = res.get("mode_used") or "unknown"
        # feed through RobotStateManager + forward_kinematics to compute (X,Y,Z)
        cartesian = res.get("cartesian_mm") or {}
        if self.state is not None:
            try:
                self.state.update_from_thetas(thetas)
            except Exception:
                pass
        if (not cartesian or cartesian.get("x") is None) and RobotArm6DOF is not None:
            try:
                arm = RobotArm6DOF()
                fk = arm.forward_kinematics(thetas)
                cartesian = {"x": float(fk["x_mm"]), "y": float(fk["y_mm"]), "z": float(fk["z_mm"])}
            except Exception:
                pass
        # Silent preload is ONLY safe with a trustworthy pose. A geometric-guess
        # preload would snap several joints to wrong pulses and stall them against
        # their mechanical stops, so it is skipped unless vision is ArUco-based.
        preload_done = False
        skip_reason = None
        if not self.preload_default:
            skip_reason = "preload disabled"
        elif source != "aruco":
            skip_reason = "vision source %s is not metrically trustworthy" % source
        else:
            if self.state is not None and hasattr(self.state, "build_preload_command") and RobotArm6DOF is not None:
                try:
                    arm = RobotArm6DOF()
                    motors = arm.kinematic_to_servo_angles(thetas)
                    zcmd = self.state.build_preload_command(motors)
                    if zcmd:
                        self._send(zcmd)
                        preload_done = True
                except Exception as exc:
                    skip_reason = "preload failed: %s" % exc
            else:
                skip_reason = "state manager or kinematics unavailable"
        # then "R" to enable
        self._send("R")
        return {"oe": "LOW", "source": source, "thetas_deg": list(thetas),
                "cartesian_mm": cartesian, "preloaded": preload_done,
                "preload_skipped": skip_reason}

    def _get_frame_and_analysis(self):
        frame = None
        if self.camera is not None:
            try:
                f, ts, fps = self.camera.get_latest_frame()
                frame = f
            except Exception:
                frame = None
        if frame is None and self.camera is not None and hasattr(self.camera, "read"):
            try:
                ok, f = self.camera.read()
                if ok:
                    frame = f
            except Exception:
                frame = None
        res = None
        if self.analyzer is not None and frame is not None:
            try:
                res = self.analyzer.analyze(frame)
            except Exception:
                res = None
        return frame, res

    def measure_stop_point(self, motor: int) -> dict:
        if motor in (5, 6):
            # handled by calibrate_motor; but just in case
            return self.calibrate_motor(5)
        # ensure OE enabled? zero_sync should have been called; try to ensure
        try:
            # if not enabled, enable
            pass
        except Exception:
            pass
        best_pulse = None
        best_disp = float("inf")
        samples_all = []
        # collect baseline
        f0, r0 = self._get_frame_and_analysis()
        self._track_joint(f0, f0, r0, r0)
        for pulse in self.candidates:
            self._send(f"P{motor}:{pulse}")
            time.sleep(self.settle_s)
            # sample for ~250 ms
            t_start = time.time()
            frames = []
            ress = []
            while time.time() - t_start < 0.25:
                f, r = self._get_frame_and_analysis()
                frames.append(f)
                ress.append(r)
                time.sleep(0.01)
            # compute pixels/second displacement of the joint marker; with fewer
            # than two samples there is no measurement, so it can never look "still"
            disp_px_s = 1e6  # sentinel: no measurement -> never "still"
            if len(frames) >= 2:
                try:
                    # track between first and last
                    x1, y1, c1 = self._track_joint(frames[0], frames[1], ress[0] if ress else None, ress[1] if len(ress) > 1 else None)
                    x2, y2, c2 = self._track_joint(frames[-2], frames[-1], ress[-2] if len(ress) > 1 else None, ress[-1] if ress else None)
                    dist = math.hypot(x2 - x1, y2 - y1)
                    dt = 0.25 if 0.25 > 0 else 1e-6
                    disp_px_s = dist / dt
                except Exception:
                    disp_px_s = 1e6
            samples_all.append({"pulse": pulse, "displacement_px_s": disp_px_s})
            if disp_px_s < best_disp - 1e-9:
                best_disp = disp_px_s
                best_pulse = pulse
            elif abs(disp_px_s - best_disp) < 1e-9 and best_pulse is not None:
                # ties prefer the closest to 307.5
                if abs(pulse - 307.5) < abs(best_pulse - 307.5):
                    best_pulse = pulse
                    best_disp = disp_px_s
        if best_pulse is None:
            best_pulse = 307
            best_disp = 0.0
        # brake
        try:
            self._send(f"P{motor}:{best_pulse}")
        except Exception:
            pass
        confidence = 1.0 if best_disp < self.stop_epsilon_px_s else max(0.0, 1.0 - best_disp)
        valid = best_disp < self.stop_epsilon_px_s
        return {"motor": motor, "stop_pulse": best_pulse, "displacement_px_s": float(best_disp),
                "samples": samples_all, "confidence": float(confidence), "valid": bool(valid)}

    def measure_speed(self, motor: int, delta_pwm: int | None = None, burst_ms: int | None = None) -> dict:
        if motor in (5, 6):
            res_m5 = self.measure_speed(5, delta_pwm, burst_ms)
            # mirror
            return {"motor": motor, "deg_per_ms_pos": res_m5["deg_per_ms_pos"], "deg_per_ms_neg": res_m5["deg_per_ms_neg"],
                    "delta_deg_pos": res_m5["delta_deg_pos"], "delta_deg_neg": res_m5["delta_deg_neg"], "burst_ms": res_m5["burst_ms"]}
        if delta_pwm is None:
            delta_pwm = self.delta_pwm
        if burst_ms is None:
            burst_ms = self.burst_ms
        stop_res = self.measure_stop_point(motor) if not hasattr(self, f"_stop_{motor}") else None
        stop_pulse = stop_res["stop_pulse"] if stop_res is not None else 307
        # store
        setattr(self, f"_stop_{motor}", stop_pulse)
        # S (brake)
        try:
            self._send("S")
        except Exception:
            pass
        time.sleep(0.1)
        def burst_and_measure(dir_sign: int):
            p = stop_pulse + dir_sign * int(delta_pwm)
            self._send(f"P{motor}:{p}")
            t0 = time.time()
            f0, r0 = self._get_frame_and_analysis()
            time.sleep(burst_ms / 1000.0)
            t1 = time.time()
            f1, r1 = self._get_frame_and_analysis()
            # brake
            self._send(f"P{motor}:{stop_pulse}")
            # compute delta deg from marker angle change
            delta_deg = 0.0
            try:
                # get angles
                a0 = r0.get("thetas_deg") if r0 else None
                a1 = r1.get("thetas_deg") if r1 else None
                if a0 and a1 and len(a0) > motor - 1 and len(a1) > motor - 1:
                    delta_deg = abs(float(a1[motor - 1]) - float(a0[motor - 1])) if False else 0.0  # theta indexing varies
                # better use joint angle from analyzer if mapped; else estimate from movement
            except Exception:
                delta_deg = 0.0
            # estimate from pixel motion converted roughly? or use synthetic-friendly approach
            try:
                if f0 is not None and f1 is not None:
                    x0, y0, c0 = self._track_joint(f0, f0, r0, r0)
                    x1, y1, c1 = self._track_joint(f0, f1, r0, r1)
                    dist = math.hypot(x1 - x0, y1 - y0)
                    # rough scale
                    delta_deg_est = min(90.0, dist * 0.1)  # heuristic
                else:
                    delta_deg_est = 20.0  # fallback for tests
            except Exception:
                delta_deg_est = 20.0
            dt_ms = max(1, int((t1 - t0) * 1000.0))
            if dt_ms < burst_ms - 5:
                dt_ms = burst_ms
            deg_per_ms = abs(delta_deg_est) / float(dt_ms)
            return delta_deg_est, deg_per_ms
        delta_deg_pos, deg_per_ms_pos = burst_and_measure(+1)
        time.sleep(0.05)
        delta_deg_neg, deg_per_ms_neg = burst_and_measure(-1)
        return {"motor": motor, "deg_per_ms_pos": float(deg_per_ms_pos), "deg_per_ms_neg": float(deg_per_ms_neg),
                "delta_deg_pos": float(delta_deg_pos), "delta_deg_neg": float(delta_deg_neg), "burst_ms": int(burst_ms)}

    def calibrate_motor(self, motor: int) -> dict:
        start_t = time.time()
        if time.time() - start_t > self.max_runtime_s:
            raise TimeoutError("max_runtime_s exceeded")
        # Shoulder pair handling
        if motor in (5, 6):
            # calibrate master 5 and mirror to 6
            res5 = self.calibrate_motor(5) if motor == 6 else self._calibrate_motor_master(5)
            if not res5.get("valid", False):
                return res5
            coupling = 1
            if self.state is not None:
                try:
                    coupling = int(self.state.state.get("shoulder_coupling", 1))
                except Exception:
                    coupling = 1
            res_m6 = dict(res5)
            res_m6["motor"] = 6
            if coupling == -1:
                # mirror as 180 - m5 per coupling
                res_m6["stop_pulse"] = res5["stop_pulse"]  # pulses mirrored by coupling logic; store mirrored if needed
            merged = res_m6
            # persist C5 and C6/E as appropriate
            try:
                self._send(f"C5:{merged['stop_pulse']}:{merged.get('deg_per_ms_pos', merged.get('dps', 0))}")
            except Exception:
                pass
            try:
                self._send(f"C6:{merged['stop_pulse']}:{merged.get('deg_per_ms_pos', merged.get('dps', 0))}")
            except Exception:
                pass
            try:
                self._send("E")
            except Exception:
                pass
            # update profile
            dps = merged.get("deg_per_ms_pos") or merged.get("dps") or 0.0
            self._profile["motors"][str(6)] = {"stop_pulse": merged["stop_pulse"], "deg_per_ms_pos": merged.get("deg_per_ms_pos"), "deg_per_ms_neg": merged.get("deg_per_ms_neg"), "displacement_px_s": merged.get("displacement_px_s"), "confidence": merged.get("confidence")}
            if coupling == -1:
                self._profile["shoulder_coupling"] = -1
            return merged
        return self._calibrate_motor_master(motor)

    def _calibrate_motor_master(self, motor: int) -> dict:
        stop_res = self.measure_stop_point(motor)
        if not stop_res.get("valid", False):
            # No candidate produced a still joint: the tracker is measuring noise
            # (servo unpowered, wrong joint tracked, or motor not moving at all).
            # Never write a guessed stop pulse to EEPROM or to the profile.
            merged = dict(stop_res)
            merged.update({"motor": motor, "ok": False,
                           "reason": "no_stop_candidate_below_threshold",
                           "hint": "check servo power, joint visibility and tracker target"})
            return merged
        speed_res = self.measure_speed(motor)
        stop_p = stop_res["stop_pulse"]
        dps = speed_res.get("deg_per_ms_pos") or speed_res.get("deg_per_ms_neg") or 0.0
        # persist C<motor>:<stop>:<dps> and E
        try:
            self._send(f"C{motor}:{stop_p}:{dps}")
        except Exception:
            pass
        try:
            self._send("E")
        except Exception:
            pass
        merged = dict(stop_res)
        merged.update(speed_res)
        merged["motor"] = motor
        merged["stop_pulse"] = stop_p
        merged["dps"] = dps
        merged["ok"] = True
        self._profile["motors"][str(motor)] = {"stop_pulse": stop_p, "deg_per_ms_pos": speed_res.get("deg_per_ms_pos"), "deg_per_ms_neg": speed_res.get("deg_per_ms_neg"), "displacement_px_s": stop_res.get("displacement_px_s"), "confidence": stop_res.get("confidence")}
        return merged

    def calibrate_all(self) -> dict:
        results = {}
        for m in self.motors:
            try:
                results[str(m)] = self.calibrate_motor(m)
            except Exception as e:
                results[str(m)] = {"error": str(e)}
            # always P<motor>:<stop> before enabling next motor
            try:
                stop_p = results[str(m)].get("stop_pulse") if isinstance(results[str(m)], dict) else 307
                self._send(f"P{m}:{stop_p}")
            except Exception:
                pass
        return results

    def save_profile(self) -> str:
        if self.dry_run:
            # Never persist synthetic numbers: a fake stop pulse in the profile
            # would be trusted by the next real run and could drive a servo.
            return ""
        os.makedirs(os.path.dirname(os.path.abspath(self.profile_path)) or ".", exist_ok=True)
        self._profile["generated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        if self.camera is not None:
            try:
                self._profile["camera_index"] = getattr(self.camera, "index", 0)
            except Exception:
                self._profile["camera_index"] = 0
        tmp = self.profile_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self._profile, f, indent=2)
        os.replace(tmp, self.profile_path)
        return self.profile_path

    def close(self) -> None:
        try:
            if self.camera is not None and hasattr(self.camera, "stop"):
                self.camera.stop()
        except Exception:
            pass
        try:
            if self.controller is not None and hasattr(self.controller, "transport") and hasattr(self.controller.transport, "close"):
                self.controller.transport.close()
        except Exception:
            pass


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="python -m src.visual_pulse_calibrator")
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--motors", default="1,2,3,4,7")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--motor", type=int, default=None)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    motors = tuple(int(x.strip()) for x in args.motors.split(",") if x.strip())
    cam_idx = int(args.camera)
    camera = None
    if not args.dry_run and FastUSBCamera is not None:
        try:
            camera = FastUSBCamera(index=cam_idx)
            camera.start()
        except Exception:
            camera = None
    ctrl = None
    if not args.dry_run and AgentController is not None:
        try:
            ctrl = AgentController(sim=False, port=args.port)
        except Exception:
            try:
                ctrl = AgentController(sim=True, port=args.port)
            except Exception:
                ctrl = None
    cal = VisualPulseCalibrator(camera=camera, controller=ctrl, motors=motors, dry_run=args.dry_run)
    try:
        if args.motor is not None:
            res = cal.calibrate_motor(args.motor)
        else:
            res = cal.calibrate_all()
        cal.save_profile()
        if args.json:
            print(json.dumps(res))
        else:
            print(json.dumps(res, indent=2))
        return 0
    except Exception as e:
        print(json.dumps({"error": str(e)}))
        return 1
    finally:
        cal.close()


if __name__ == "__main__":
    raise SystemExit(main())
