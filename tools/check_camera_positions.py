#!/usr/bin/env python
"""Phase 1 Camera & Motor Position Reading checker for local HP Victus.

Runs the pre-power optical audit WITHOUT moving or energizing any servo
(OE = HIGH). Uses FastUSBCamera and VirtualOpticalEncoder.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

# Add repo root to path
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

try:
    import cv2
except Exception:
    cv2 = None

try:
    import numpy as np
except Exception:
    np = None

from src.fast_camera import FastUSBCamera
from src.virtual_optical_encoder import VirtualOpticalEncoder
from src.kinematics_6dof import RobotArm6DOF
from src.pid_pwm_controller import SafeVisualHomingMachine


def _json_safe(value):
    """Replace non-finite numbers before emitting RFC-8259 JSON."""
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return 999.0
    return value


def _write_annotated_snapshot(frame, path, encoder, optical_audit, camera_status, joint_report):
    """Write a diagnostic image, preserving the raw frame if annotations fail."""
    if frame is None or cv2 is None:
        return
    annotated = frame.copy()
    try:
        markers = encoder._detect_aruco(annotated)
        for marker_id, marker in markers.items():
            corners = marker.get("corners")
            if corners is None:
                continue
            corners = corners.astype(int)
            cv2.polylines(annotated, [corners.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
            cv2.putText(
                annotated, f"ID {marker_id}", (int(corners[0][0]), int(corners[0][1]) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )
            top_left = marker.get("tl")
            top_vec = marker.get("top_vec")
            if top_left is not None and top_vec is not None:
                start = (int(top_left[0]), int(top_left[1]))
                end = (int(top_left[0] + top_vec[0]), int(top_left[1] + top_vec[1]))
                cv2.arrowedLine(annotated, start, end, (0, 0, 255), 2)

        tilt_deg = 0.0
        ellipse_ratio = 0.0
        tilt_optimal = False
        analyzer = getattr(encoder, "fallback_analyzer", None)
        if analyzer is not None:
            try:
                geometry = analyzer.analyze(frame)
                keypoints = geometry.get("keypoints_px") or {}
                for start_name, end_name in geometry.get("skeleton", []):
                    if start_name not in keypoints or end_name not in keypoints:
                        continue
                    start = tuple(int(v) for v in keypoints[start_name][:2])
                    end = tuple(int(v) for v in keypoints[end_name][:2])
                    cv2.line(annotated, start, end, (0, 255, 255), 6)
                labels = {
                    "base": "BASE",
                    "shoulder": "SHOULDER",
                    "elbow": "ELBOW",
                    "wrist_pitch": "WRIST_P",
                    "wrist_roll": "WRIST_R",
                    "gripper_tip": "GRIPPER",
                }
                for index, (name, label) in enumerate(labels.items()):
                    if name not in keypoints:
                        continue
                    point = tuple(int(v) for v in keypoints[name][:2])
                    color = (0, 0, 255) if index % 2 == 0 else (255, 255, 0)
                    cv2.circle(annotated, point, 8, color, -1)
                    cv2.putText(
                        annotated, label, (point[0] + 9, point[1] - 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
                    )
                debug = geometry.get("debug", {}) if isinstance(geometry, dict) else {}
                tilt_deg = float(debug.get("camera_tilt_deg", tilt_deg))
                ellipse_ratio = float(debug.get("turntable_ellipse_ratio", ellipse_ratio))
                tilt_optimal = bool(debug.get("camera_tilt_optimal", tilt_optimal))
            except Exception:
                pass

        status_text = (
            f"PHASE 1 OPTICAL AUDIT: {'PASSED' if optical_audit.get('passed') else 'HOLD_HIGH'} "
            f"| var={optical_audit.get('max_variance_deg2', 999.0):.4f} "
            f"| FPS={camera_status.get('fps', 0.0):.1f} | OE = HIGH [SERVOS OFF]"
        )
        cv2.putText(
            annotated, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (0, 255, 0) if optical_audit.get("passed") else (0, 0, 255), 2,
        )
        if tilt_optimal:
            tilt_text = "CAM TILT: {:.1f} deg (b/a={:.2f}) [OPTIMAL 35-55 DEG]".format(tilt_deg, ellipse_ratio)
            tilt_color = (0, 255, 0)
        else:
            tilt_text = "CAM TILT: {:.1f} deg (b/a={:.2f}) [ADJUST CAMERA TILT TO 35-55 DEG]".format(tilt_deg, ellipse_ratio)
            tilt_color = (0, 255, 255)
        cv2.putText(
            annotated, tilt_text, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            tilt_color, 2,
        )
        motor_values = [
            joint_report.get("motor1_pinza", {}).get("motor_deg", 0.0),
            joint_report.get("motor2_giro_pinza", {}).get("motor_deg", 0.0),
            joint_report.get("motor3_inclina_pinza", {}).get("motor_deg", 0.0),
            joint_report.get("motor4_codo", {}).get("motor_deg", 0.0),
            joint_report.get("motors5_6_hombro_doble", {}).get("motor5_deg", 0.0),
            joint_report.get("motors5_6_hombro_doble", {}).get("motor6_deg", 0.0),
            joint_report.get("motor7_base", {}).get("motor_deg", 0.0),
        ]
        for index, angle in enumerate(motor_values, start=1):
            cv2.putText(
                annotated, f"Motor {index}: {angle:.1f} deg", (10, 60 + (index - 1) * 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1,
            )
    except Exception:
        # A diagnostic drawing failure must not prevent saving the captured image.
        pass
    try:
        cv2.imwrite(path, annotated)
    except Exception:
        pass


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Phase 1 optical audit (servos OFF, OE=HIGH)"
    )
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    parser.add_argument(
        "--frames",
        type=int,
        default=30,
        help="Number of frames for pre-power optical audit (1.0 s @ 30 FPS)",
    )
    parser.add_argument(
        "--calibrate-zero",
        action="store_true",
        help="Call encoder.calibrate_home_offsets on a frame to set current posture as 0.0° Punto 0",
    )
    parser.add_argument(
        "--snapshot",
        default="tools/camera_previews/optical_encoder_audit.jpg",
        help="Path to save annotated snapshot",
    )
    args = parser.parse_args(argv)

    encoder = VirtualOpticalEncoder()
    kinematics = RobotArm6DOF()
    homing = SafeVisualHomingMachine()

    os.makedirs(os.path.dirname(os.path.abspath(args.snapshot)) or ".", exist_ok=True)

    camera_status = {"fps": 0.0, "resolution": None, "backend": None}
    optical_audit = {}
    joint_positions_report = {}
    tcp_cartesian_mm = {"x": 0.0, "y": 0.0, "z": 0.0}
    frame = None

    try:
        with FastUSBCamera(index=args.camera, width=1280, height=720, fps=30) as cam:
            # Discard the camera's auto-exposure and white-balance settling period.
            warmup_deadline = time.monotonic() + 1.2
            while time.monotonic() < warmup_deadline:
                cam.get_latest_frame()
                time.sleep(0.005)

            # MSMF may suppress its first frames; wait briefly for a published frame.
            first_frame_deadline = time.monotonic() + 5.0
            frame = None
            while time.monotonic() < first_frame_deadline:
                frame, _ts_ns, _fps_meas = cam.get_latest_frame()
                if frame is not None:
                    break
                time.sleep(0.005)
            stats = cam.stats()
            backend_id = getattr(cam, "_resolved_backend", None)
            if cv2 is not None and backend_id == cv2.CAP_MSMF:
                backend_name = "CAP_MSMF"
            elif cv2 is not None and backend_id == cv2.CAP_DSHOW:
                backend_name = "CAP_DSHOW"
            else:
                backend_name = "unknown"
            camera_status = {
                "fps": float(stats.get("fps", 0.0)),
                "resolution": [int(stats.get("width", 1280)), int(stats.get("height", 720))],
                "backend": backend_name,
            }

            if args.calibrate_zero and frame is not None:
                try:
                    encoder.calibrate_home_offsets(frame, save=True)
                except Exception:
                    pass

            # Run pre-power audit
            try:
                audit_res = encoder.audit_pre_power(cam, num_frames=args.frames, max_variance_deg=0.15)
            except Exception as e:
                audit_res = {
                    "passed": False,
                    "oe_recommendation": "HOLD_HIGH",
                    "frames_evaluated": 0,
                    "mode_used": "none",
                    "visible_marker_ids": [],
                    "resting_thetas_deg": [0.0] * 6,
                    "resting_motors_deg": {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0, 7: 0.0},
                    "variances_deg2": [999.0] * 6,
                    "max_variance_deg2": 999.0,
                    "reason": f"error:{e}",
                }
            # The audit consumes another 10 settling and 30 measured frames;
            # report the latest measured stream rate rather than startup state.
            stats = cam.stats()
            camera_status["fps"] = float(stats.get("fps", 0.0))

            optical_audit = {
                "passed": bool(audit_res.get("passed", False)),
                "frames_evaluated": int(audit_res.get("frames_evaluated", 0)),
                "mode_used": audit_res.get("mode_used", "none"),
                "visible_marker_ids": audit_res.get("visible_marker_ids", []),
                "max_variance_deg2": float(audit_res.get("max_variance_deg2", 999.0)),
                "oe_recommendation": audit_res.get("oe_recommendation", "HOLD_HIGH"),
            }
            if "primary_max_variance_deg2" in audit_res:
                optical_audit["primary_max_variance_deg2"] = audit_res.get("primary_max_variance_deg2")
            if "secondary_max_variance_deg2" in audit_res:
                optical_audit["secondary_max_variance_deg2"] = audit_res.get("secondary_max_variance_deg2")

            # Read positions from latest frame
            pos_res = encoder.read_positions(frame) if frame is not None else encoder._empty_result(0.0)

            # Build joint positions report
            thetas_deg = pos_res.get("thetas_deg", [0.0] * 6)
            motors_deg = pos_res.get("motors_deg", {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0, 7: 0.0})
            omega_vis = pos_res.get("omega_vis_dps", [0.0] * 6)
            variances = audit_res.get("variances_deg2", [0.0] * 6)

            joint_positions_report = {
                "joint1": {
                    "theta_deg": float(thetas_deg[0]) if len(thetas_deg) > 0 else 0.0,
                    "motor_deg": float(motors_deg.get(7, 0.0)),
                    "omega_dps": float(omega_vis[0]) if len(omega_vis) > 0 else 0.0,
                    "variance_deg2": float(variances[0]) if len(variances) > 0 else 0.0,
                    "u_stop_preload": 307,
                },
                "joint2": {
                    "theta_deg": float(thetas_deg[1]) if len(thetas_deg) > 1 else 0.0,
                    "motor_deg": float(motors_deg.get(5, 0.0)),
                    "omega_dps": float(omega_vis[1]) if len(omega_vis) > 1 else 0.0,
                    "variance_deg2": float(variances[1]) if len(variances) > 1 else 0.0,
                    "u_stop_preload": 307,
                },
                "joint3": {
                    "theta_deg": float(thetas_deg[2]) if len(thetas_deg) > 2 else 0.0,
                    "motor_deg": float(motors_deg.get(6, 0.0)),
                    "omega_dps": float(omega_vis[2]) if len(omega_vis) > 2 else 0.0,
                    "variance_deg2": float(variances[2]) if len(variances) > 2 else 0.0,
                    "u_stop_preload": 307,
                },
                "joint4": {
                    "theta_deg": float(thetas_deg[3]) if len(thetas_deg) > 3 else 0.0,
                    "motor_deg": float(motors_deg.get(4, 0.0)),
                    "omega_dps": float(omega_vis[3]) if len(omega_vis) > 3 else 0.0,
                    "variance_deg2": float(variances[3]) if len(variances) > 3 else 0.0,
                    "u_stop_preload": 307,
                },
                "joint5": {
                    "theta_deg": float(thetas_deg[4]) if len(thetas_deg) > 4 else 0.0,
                    "motor_deg": float(motors_deg.get(3, 0.0)),
                    "omega_dps": float(omega_vis[4]) if len(omega_vis) > 4 else 0.0,
                    "variance_deg2": float(variances[4]) if len(variances) > 4 else 0.0,
                    "u_stop_preload": 307,
                },
                "joint6": {
                    "theta_deg": float(thetas_deg[5]) if len(thetas_deg) > 5 else 0.0,
                    "motor_deg": float(motors_deg.get(2, 0.0)),
                    "omega_dps": float(omega_vis[5]) if len(omega_vis) > 5 else 0.0,
                    "variance_deg2": float(variances[5]) if len(variances) > 5 else 0.0,
                    "u_stop_preload": 307,
                },
                "motor1_pinza": {
                    "theta_deg": float(thetas_deg[5]) if len(thetas_deg) > 5 else 0.0,
                    "motor_deg": float(motors_deg.get(1, 0.0)),
                    "omega_dps": float(omega_vis[5]) if len(omega_vis) > 5 else 0.0,
                    "variance_deg2": float(variances[5]) if len(variances) > 5 else 0.0,
                    "u_stop_preload": 307,
                },
                "motor2_giro_pinza": {
                    "theta_deg": float(thetas_deg[4]) if len(thetas_deg) > 4 else 0.0,
                    "motor_deg": float(motors_deg.get(2, 0.0)),
                    "omega_dps": float(omega_vis[4]) if len(omega_vis) > 4 else 0.0,
                    "variance_deg2": float(variances[4]) if len(variances) > 4 else 0.0,
                    "u_stop_preload": 307,
                },
                "motor3_inclina_pinza": {
                    "theta_deg": float(thetas_deg[3]) if len(thetas_deg) > 3 else 0.0,
                    "motor_deg": float(motors_deg.get(3, 0.0)),
                    "omega_dps": float(omega_vis[3]) if len(omega_vis) > 3 else 0.0,
                    "variance_deg2": float(variances[3]) if len(variances) > 3 else 0.0,
                    "u_stop_preload": 307,
                },
                "motor4_codo": {
                    "theta_deg": float(thetas_deg[2]) if len(thetas_deg) > 2 else 0.0,
                    "motor_deg": float(motors_deg.get(4, 0.0)),
                    "omega_dps": float(omega_vis[2]) if len(omega_vis) > 2 else 0.0,
                    "variance_deg2": float(variances[2]) if len(variances) > 2 else 0.0,
                    "u_stop_preload": 307,
                },
                "motors5_6_hombro_doble": {
                    "theta_deg": float(thetas_deg[1]) if len(thetas_deg) > 1 else 0.0,
                    "motor5_deg": float(motors_deg.get(5, 0.0)),
                    "motor6_deg": float(motors_deg.get(6, 0.0)),
                    "omega_dps": float(omega_vis[1]) if len(omega_vis) > 1 else 0.0,
                    "variance_deg2": float(variances[1]) if len(variances) > 1 else 0.0,
                    "u_stop_preload": 307,
                },
                "motor7_base": {
                    "theta_deg": float(thetas_deg[0]) if len(thetas_deg) > 0 else 0.0,
                    "motor_deg": float(motors_deg.get(7, 0.0)),
                    "omega_dps": float(omega_vis[0]) if len(omega_vis) > 0 else 0.0,
                    "variance_deg2": float(variances[0]) if len(variances) > 0 else 0.0,
                    "u_stop_preload": 307,
                },
            }

            # Compute FK for TCP
            try:
                fk_res = kinematics.forward_kinematics(thetas_deg)
                tcp_cartesian_mm = {
                    "x": float(fk_res.get("x_mm", 0.0)),
                    "y": float(fk_res.get("y_mm", 0.0)),
                    "z": float(fk_res.get("z_mm", 0.0)),
                }
            except Exception:
                tcp_cartesian_mm = {"x": 0.0, "y": 0.0, "z": 0.0}

            camera_framing_45deg = {"tilt_deg": 0.0, "ellipse_ratio_b_a": 0.0, "optimal_35_to_55": False, "status": "ADJUST_TO_35_55_DEG"}
            try:
                _an = getattr(encoder, "fallback_analyzer", None)
                if _an is None:
                    from src.arm_vision_analyzer import ArmVisionAnalyzer
                    _an = ArmVisionAnalyzer(mode="geometric")
                if frame is not None:
                    _geom = _an.analyze(frame)
                    _dbg = _geom.get("debug", {}) if isinstance(_geom, dict) else {}
                    _tilt = float(_dbg.get("camera_tilt_deg", 0.0))
                    _ratio = float(_dbg.get("turntable_ellipse_ratio", 0.0))
                    _opt = bool(_dbg.get("camera_tilt_optimal", False))
                    camera_framing_45deg = {
                        "tilt_deg": _tilt,
                        "ellipse_ratio_b_a": _ratio,
                        "optimal_35_to_55": _opt,
                        "status": "OPTIMAL_45_DEG" if _opt else "ADJUST_TO_35_55_DEG",
                    }
            except Exception:
                pass

    except Exception as e:
        # Still produce a report even if camera fails
        optical_audit = {
            "passed": False,
            "frames_evaluated": 0,
            "mode_used": "none",
            "visible_marker_ids": [],
            "max_variance_deg2": 999.0,
            "oe_recommendation": "HOLD_HIGH",
        }
        joint_positions_report = {
            "motor1_pinza": {"motor_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": 999.0, "u_stop_preload": 307},
            "motor2_giro_pinza": {"motor_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": 999.0, "u_stop_preload": 307},
            "motor3_inclina_pinza": {"motor_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": 999.0, "u_stop_preload": 307},
            "motor4_codo": {"motor_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": 999.0, "u_stop_preload": 307},
            "motors5_6_hombro_doble": {"motor5_deg": 0.0, "motor6_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": 999.0, "u_stop_preload": 307},
            "motor7_base": {"motor_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": 999.0, "u_stop_preload": 307},
        }
        tcp_cartesian_mm = {"x": 0.0, "y": 0.0, "z": 0.0}
        camera_status = {"fps": 0.0, "resolution": None, "backend": f"error:{e}"}
        camera_framing_45deg = {"tilt_deg": 0.0, "ellipse_ratio_b_a": 0.0, "optimal_35_to_55": False, "status": "ADJUST_TO_35_55_DEG"}

    if frame is not None:
        _write_annotated_snapshot(
            frame, args.snapshot, encoder, optical_audit, camera_status, joint_positions_report
        )

    report = {
        "camera_status": camera_status,
        "optical_audit": optical_audit,
        "joint_positions": joint_positions_report,
        "tcp_cartesian_mm": tcp_cartesian_mm,
        "camera_framing_45deg": camera_framing_45deg,
        "next_bringup_sequence": homing.bringup_order,
    }

    # Also add human readable mapping
    report["next_bringup_sequence_named"] = [
        "Motor 1 Pinza",
        "Motor 2 Giro Pinza",
        "Motor 3 Inclina Pinza",
        "Motor 4 Codo",
        "Motors 5 & 6 Hombro",
        "Motor 7 Base",
    ]

    print(json.dumps(_json_safe(report), indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
