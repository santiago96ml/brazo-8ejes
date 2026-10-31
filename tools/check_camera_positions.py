#!/usr/bin/env python
"""Phase 1 Camera & Motor Position Reading checker for local HP Victus.

Runs the pre-power optical audit WITHOUT moving or energizing any servo
(OE = HIGH). Uses FastUSBCamera and VirtualOpticalEncoder.
"""

from __future__ import annotations

import argparse
import json
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
    annotated_frame = None

    try:
        with FastUSBCamera(index=args.camera, width=1280, height=720, fps=30) as cam:
            # warm up
            time.sleep(0.5)
            stats = cam.stats()
            camera_status = {
                "fps": float(stats.get("fps", 0.0)),
                "resolution": [int(stats.get("width", 1280)), int(stats.get("height", 720))],
                "backend": stats.get("fourcc", "unknown"),
            }

            # Get latest frame for snapshot/calibration
            frame, ts_ns, fps_meas = cam.get_latest_frame()
            if frame is None:
                # try read
                ret, frame = cam.read()
                if not ret:
                    frame = None

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
                    "variances_deg2": [float("inf")] * 6,
                    "max_variance_deg2": float("inf"),
                    "reason": f"error:{e}",
                }

            optical_audit = {
                "passed": bool(audit_res.get("passed", False)),
                "frames_evaluated": int(audit_res.get("frames_evaluated", 0)),
                "mode_used": audit_res.get("mode_used", "none"),
                "visible_marker_ids": audit_res.get("visible_marker_ids", []),
                "max_variance_deg2": float(audit_res.get("max_variance_deg2", float("inf"))),
                "oe_recommendation": audit_res.get("oe_recommendation", "HOLD_HIGH"),
            }

            # Read positions from latest frame
            if frame is None:
                ret, frame = cam.read()
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

            # Save annotated snapshot
            annotated_frame = frame.copy() if frame is not None else None
            if annotated_frame is not None and cv2 is not None:
                try:
                    markers = encoder._detect_aruco(annotated_frame)
                    for mid, minfo in markers.items():
                        corners = minfo.get("corners")
                        if corners is not None:
                            c = corners.astype(int)
                            cv2.polylines(annotated_frame, [c.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
                            cv2.putText(
                                annotated_frame,
                                f"ID {mid}",
                                (int(c[0][0]), int(c[0][1]) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                1,
                            )
                            # draw top edge direction
                            tl = minfo.get("tl")
                            vtop = minfo.get("top_vec")
                            if tl is not None and vtop is not None:
                                p1 = (int(tl[0]), int(tl[1]))
                                p2 = (int(tl[0] + vtop[0]), int(tl[1] + vtop[1]))
                                cv2.arrowedLine(annotated_frame, p1, p2, (0, 0, 255), 2)
                    # status banner
                    status_text = f"PHASE 1 OPTICAL AUDIT: {'PASSED' if optical_audit['passed'] else 'HOLD_HIGH'} | var={optical_audit['max_variance_deg2']:.4f} | FPS={camera_status['fps']:.1f} | OE = HIGH [SERVOS OFF]"
                    cv2.putText(
                        annotated_frame,
                        status_text,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255) if not optical_audit["passed"] else (0, 255, 0),
                        2,
                    )
                    # motor readouts
                    y = 60
                    lines = [
                        f"Motor 1 (PINZA): {joint_positions_report['motor1_pinza']['motor_deg']:.1f}°",
                        f"Motor 2 (GIRO_PINZA): {joint_positions_report['motor2_giro_pinza']['motor_deg']:.1f}°",
                        f"Motor 3 (INCLINA_PINZA): {joint_positions_report['motor3_inclina_pinza']['motor_deg']:.1f}°",
                        f"Motor 4 (CODO): {joint_positions_report['motor4_codo']['motor_deg']:.1f}°",
                        f"Motors 5&6 (HOMBRO_DOBLE): {joint_positions_report['motors5_6_hombro_doble']['motor5_deg']:.1f}°/{joint_positions_report['motors5_6_hombro_doble']['motor6_deg']:.1f}°",
                        f"Motor 7 (BASE): {joint_positions_report['motor7_base']['motor_deg']:.1f}°",
                    ]
                    for l in lines:
                        cv2.putText(annotated_frame, l, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                        y += 20
                    cv2.imwrite(args.snapshot, annotated_frame)
                except Exception:
                    try:
                        if annotated_frame is not None:
                            cv2.imwrite(args.snapshot, annotated_frame)
                    except Exception:
                        pass
            elif frame is not None and cv2 is not None:
                try:
                    cv2.imwrite(args.snapshot, frame)
                except Exception:
                    pass

    except Exception as e:
        # Still produce a report even if camera fails
        optical_audit = {
            "passed": False,
            "frames_evaluated": 0,
            "mode_used": "none",
            "visible_marker_ids": [],
            "max_variance_deg2": float("inf"),
            "oe_recommendation": "HOLD_HIGH",
        }
        joint_positions_report = {
            "motor1_pinza": {"motor_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": float("inf"), "u_stop_preload": 307},
            "motor2_giro_pinza": {"motor_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": float("inf"), "u_stop_preload": 307},
            "motor3_inclina_pinza": {"motor_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": float("inf"), "u_stop_preload": 307},
            "motor4_codo": {"motor_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": float("inf"), "u_stop_preload": 307},
            "motors5_6_hombro_doble": {"motor5_deg": 0.0, "motor6_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": float("inf"), "u_stop_preload": 307},
            "motor7_base": {"motor_deg": 0.0, "theta_deg": 0.0, "omega_dps": 0.0, "variance_deg2": float("inf"), "u_stop_preload": 307},
        }
        tcp_cartesian_mm = {"x": 0.0, "y": 0.0, "z": 0.0}
        camera_status = {"fps": 0.0, "resolution": None, "backend": f"error:{e}"}

    report = {
        "camera_status": camera_status,
        "optical_audit": optical_audit,
        "joint_positions": joint_positions_report,
        "tcp_cartesian_mm": tcp_cartesian_mm,
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

    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
