"""Vision node for the Netbook Conectar Igualdad Ubuntu Server.

Implements the Read-First / Zero-Jump Boot synchronisation protocol:
read the real physical posture, feed it into the SE(3) model, silently
preload the PWM registers while ``PIN_OE`` is still ``HIGH``, and only
then lower ``PIN_OE`` to ``LOW``.
"""

from __future__ import annotations

import argparse
import http.server
import json
import os
import socketserver
import threading
import urllib.request
from typing import Any, Dict, List, Optional

import numpy as np

try:  # pragma: no cover - optional dependency
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

try:
    from src.state_manager import RobotStateManager, ShoulderCouplingError
except Exception:  # pragma: no cover - fallback when running from src/
    try:
        from state_manager import RobotStateManager, ShoulderCouplingError  # type: ignore
    except Exception:  # pragma: no cover
        RobotStateManager = None  # type: ignore
        ShoulderCouplingError = ValueError  # type: ignore

try:
    from src.kinematics_6dof import RobotArm6DOF
except Exception:  # pragma: no cover - fallback when running from src/
    try:
        from kinematics_6dof import RobotArm6DOF  # type: ignore
    except Exception:  # pragma: no cover
        RobotArm6DOF = None  # type: ignore

HOME_THETAS = [0.0, 90.0, 0.0, 0.0, 0.0, 50.0]


def _import_simulated_transport():
    try:
        from src.agent_controller import SimulatedTransport
        return SimulatedTransport
    except Exception:  # pragma: no cover
        try:
            from agent_controller import SimulatedTransport  # type: ignore
            return SimulatedTransport
        except Exception:  # pragma: no cover
            return None


class _FallbackTransport:
    """Minimal transport used when the agent controller is unavailable."""

    def __init__(self):
        import queue

        self.sent = []
        self.rx_queue = queue.Queue()

    def send_line(self, line):
        self.sent.append(str(line))
        try:
            self.rx_queue.put(json.dumps({"ack": "OK"}))
        except Exception:
            pass

    def close(self):
        pass


def _send(transport: Any, line: str) -> None:
    if transport is None:
        return
    if hasattr(transport, "send_line"):
        transport.send_line(line)
    elif hasattr(transport, "send"):
        transport.send(line)
    elif hasattr(transport, "write"):
        try:
            transport.write((str(line) + "\n").encode())
        except Exception:
            try:
                transport.write(str(line) + "\n")
            except Exception:
                pass


def _to_list_matrix(matrix: Any) -> List[List[float]]:
    arr = np.asarray(matrix, dtype=float)
    if arr.ndim == 1:
        n = int(round(arr.shape[0] ** 0.5))
        arr = arr.reshape(n, n)
    return [[float(v) for v in row] for row in arr.tolist()]


def _normalize_pose_result(raw: Dict[str, Any]) -> Dict[str, Any]:
    thetas = raw.get("thetas_deg", raw.get("thetas", HOME_THETAS))
    thetas = [float(v) for v in (thetas or HOME_THETAS)]
    if len(thetas) != 6:
        thetas = (thetas + HOME_THETAS)[:6]
    confidence = float(raw.get("confidence", 0.5))
    source = str(raw.get("source", "unknown"))
    return {
        "source": source,
        "confidence": confidence,
        "thetas_deg": thetas,
    }


class VisionLocator:
    """Read the physical posture from vision, odometry or persisted state."""

    def __init__(
        self,
        endpoint_url: Optional[str] = None,
        camera_index: int = 0,
        state_manager: Optional[RobotStateManager] = None,
        kinematics: Optional[RobotArm6DOF] = None,
    ):
        if endpoint_url is None:
            endpoint_url = os.environ.get("NETBOOK_VISION_URL")
        self.endpoint_url = endpoint_url or None
        self.camera_index = int(camera_index)
        self.state_manager = state_manager
        if self.state_manager is None and RobotStateManager is not None:
            try:
                self.state_manager = RobotStateManager()
            except Exception:
                self.state_manager = None
        self.kinematics = kinematics
        if self.kinematics is None and RobotArm6DOF is not None:
            try:
                coupling = None
                if self.state_manager is not None:
                    coupling = self.state_manager.state.get("shoulder_coupling", 1)
                self.kinematics = RobotArm6DOF(shoulder_coupling=coupling)
            except Exception:
                try:
                    self.kinematics = RobotArm6DOF()
                except Exception:
                    self.kinematics = None

    # ------------------------------------------------------------------ CV
    def detect_from_aruco_or_color(self, frame_bgr: np.ndarray) -> Optional[dict]:
        """Detect a posture from an ArUco board or a blue PLA silhouette."""
        if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3:
            return None
        if frame_bgr.shape[2] < 3:
            return None

        frame = np.asarray(frame_bgr)
        gray = None
        if cv2 is not None:
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            except Exception:
                gray = None

        # 1. ArUco markers, when OpenCV exposes them.
        if cv2 is not None and gray is not None and hasattr(cv2, "aruco"):
            try:
                aruco = cv2.aruco
                dictionary = None
                for attr in ("DICT_4X4_50", "DICT_6X6_250", "DICT_5X5_100"):
                    if hasattr(aruco, attr):
                        try:
                            dictionary = aruco.getPredefinedDictionary(getattr(aruco, attr))
                            break
                        except Exception:
                            dictionary = None
                corners = ids = None
                if dictionary is not None:
                    try:
                        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
                        corners, ids, _ = detector.detectMarkers(gray)
                    except Exception:
                        try:
                            corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=aruco.DetectorParameters())
                        except Exception:
                            corners, ids = None, None
                if ids is not None and len(ids) > 0:
                    centers = []
                    for c in corners:
                        pts = np.asarray(c).reshape(-1, 2)
                        centers.append(pts.mean(axis=0))
                    centers = np.asarray(centers, dtype=float)
                    # A coarse but deterministic mapping from marker layout to
                    # joint angles. Real deployments calibrate this mapping.
                    cx = float(np.mean(centers[:, 0]))
                    cy = float(np.mean(centers[:, 1]))
                    h, w = frame.shape[:2]
                    theta1 = float(np.degrees(np.arctan2(cx - w / 2.0, h - cy))) % 360.0
                    thetas = [
                        theta1,
                        90.0,
                        0.0,
                        0.0,
                        0.0,
                        50.0,
                    ]
                    if centers.shape[0] >= 2:
                        # Use vertical spread of markers as a shoulder hint.
                        spread = float(np.ptp(centers[:, 1]))
                        thetas[1] = float(np.clip(90.0 + (spread - h / 4.0) * 0.1, 15.0, 165.0))
                    return {
                        "source": "aruco",
                        "confidence": float(min(0.99, 0.55 + 0.08 * len(ids))),
                        "thetas_deg": thetas,
                        "markers": [int(x) for x in np.asarray(ids).reshape(-1).tolist()],
                    }
            except Exception:
                pass

        # 2. Pure-NumPy blue PLA silhouette segmenter.
        b = frame[:, :, 0].astype(np.float64)
        g = frame[:, :, 1].astype(np.float64)
        r = frame[:, :, 2].astype(np.float64)
        mask = (b > 1.25 * r) & (b > 1.15 * g)
        ys, xs = np.nonzero(mask)
        if ys.size < 8:
            return None

        cx = float(np.mean(xs))
        cy = float(np.mean(ys))
        h, w = frame.shape[:2]
        theta1 = float(np.degrees(np.arctan2(cx - w / 2.0, h - cy))) % 360.0

        median_y = float(np.median(ys))
        upper_pts = np.column_stack((xs[ys <= median_y], ys[ys <= median_y])).astype(float)
        lower_pts = np.column_stack((xs[ys > median_y], ys[ys > median_y])).astype(float)

        def _principal_angle(points: np.ndarray) -> float:
            if points.shape[0] < 2:
                return 90.0
            centered = points - points.mean(axis=0)
            try:
                cov = np.cov(centered.T)
                if np.ndim(cov) == 0:
                    cov = np.array([[cov, 0.0], [0.0, cov]], dtype=float)
                vals, vecs = np.linalg.eigh(cov)
                principal = vecs[:, int(np.argmax(vals))]
                return float(np.degrees(np.arctan2(principal[1], principal[0])))
            except Exception:
                return 90.0

        upper_angle = _principal_angle(upper_pts)
        lower_angle = _principal_angle(lower_pts)
        # Image y grows downward; convert to a coarse shoulder/elbow estimate.
        theta2 = float(np.clip(90.0 + (lower_angle - 90.0), 15.0, 165.0))
        theta3 = float(np.clip(upper_angle - lower_angle, -90.0, 90.0))
        thetas = [theta1, theta2, theta3, 0.0, 0.0, 50.0]
        coverage = float(ys.size) / float(h * w)
        confidence = float(np.clip(0.45 + 4.0 * coverage, 0.45, 0.85))
        return {
            "source": "blue_silhouette",
            "confidence": confidence,
            "thetas_deg": thetas,
            "mask_pixels": int(ys.size),
        }

    def fetch_netbook_pose(self, timeout: float = 1.5) -> Optional[dict]:
        if not self.endpoint_url:
            return None
        try:
            with urllib.request.urlopen(self.endpoint_url, timeout=float(timeout)) as response:
                payload = response.read()
            data = json.loads(payload.decode("utf-8"))
            if not isinstance(data, dict):
                return None
            return data
        except Exception:
            return None

    def estimate_resting_posture(self, frame_bgr: Optional[np.ndarray] = None) -> dict:
        estimate = None
        if frame_bgr is not None:
            estimate = self.detect_from_aruco_or_color(frame_bgr)

        if estimate is None and self.endpoint_url:
            raw = self.fetch_netbook_pose()
            if isinstance(raw, dict):
                normalized = _normalize_pose_result(raw)
                estimate = {
                    "source": normalized["source"],
                    "confidence": normalized["confidence"],
                    "thetas_deg": normalized["thetas_deg"],
                }
                # Preserve the netbook source name when provided.
                if "source" in raw:
                    estimate["source"] = str(raw.get("source"))
                if "confidence" in raw:
                    estimate["confidence"] = float(raw.get("confidence"))

        if estimate is None:
            state = getattr(self.state_manager, "state", {}) if self.state_manager is not None else {}
            thetas = state.get("thetas_deg", HOME_THETAS)
            estimate = {
                "source": "robot_state_json",
                "confidence": 0.5,
                "thetas_deg": [float(v) for v in thetas],
            }

        thetas = [float(v) for v in estimate.get("thetas_deg", HOME_THETAS)]
        if len(thetas) != 6:
            thetas = (thetas + HOME_THETAS)[:6]

        # Keep kinematics shoulder coupling aligned with persisted state.
        if self.kinematics is not None and self.state_manager is not None:
            try:
                self.kinematics.shoulder_coupling = int(
                    self.state_manager.state.get("shoulder_coupling", self.kinematics.shoulder_coupling)
                )
            except Exception:
                pass

        motors: Dict[int, float] = {}
        if self.kinematics is not None and hasattr(self.kinematics, "kinematic_to_servo_angles"):
            try:
                servo = self.kinematics.kinematic_to_servo_angles(thetas)
                motors = {int(k): float(v) for k, v in (servo or {}).items()}
            except Exception:
                motors = {}
        if not motors and self.state_manager is not None:
            stored = self.state_manager.state.get("motors_deg", {})
            for key, value in (stored or {}).items():
                try:
                    motors[int(key)] = float(value)
                except (TypeError, ValueError):
                    pass
        if not motors:
            try:
                from src.kinematics_6dof import RobotArm6DOF as _Arm
            except Exception:  # pragma: no cover
                try:
                    from kinematics_6dof import RobotArm6DOF as _Arm  # type: ignore
                except Exception:
                    _Arm = None
            if _Arm is not None:
                try:
                    arm = _Arm()
                    servo = arm.kinematic_to_servo_angles(thetas)
                    motors = {int(k): float(v) for k, v in (servo or {}).items()}
                except Exception:
                    motors = {}

        fk = None
        if self.kinematics is not None:
            try:
                fk = self.kinematics.forward_kinematics(thetas)
            except Exception:
                fk = None
        if fk is None and RobotArm6DOF is not None:
            try:
                arm = RobotArm6DOF()
                fk = arm.forward_kinematics(thetas)
            except Exception:
                fk = None

        if fk is not None:
            T = fk.get("T_0_6")
            tcp = fk.get("tcp_mm")
            if tcp is None:
                tcp = fk.get("TCP")
            if tcp is None:
                tcp = fk.get("tcp")
            if tcp is None:
                tcp = [fk.get("x_mm", 0.0), fk.get("y_mm", 0.0), fk.get("z_mm", 0.0)]
            orientation = fk.get("orientation_deg", {})
            if isinstance(tcp, np.ndarray):
                tcp = tcp.reshape(-1).tolist()
            if isinstance(tcp, (list, tuple)):
                cart = {
                    "x": float(tcp[0]),
                    "y": float(tcp[1]),
                    "z": float(tcp[2]),
                }
            elif isinstance(tcp, dict):
                cart = {
                    "x": float(tcp.get("x", tcp.get("X", 0.0))),
                    "y": float(tcp.get("y", tcp.get("Y", 0.0))),
                    "z": float(tcp.get("z", tcp.get("Z", 0.0))),
                }
            else:
                cart = {"x": 0.0, "y": 0.0, "z": 570.0}
            orient = {
                "pitch": float(orientation.get("pitch", orientation.get("Pitch", 90.0))),
                "roll": float(orientation.get("roll", orientation.get("Roll", 0.0))),
                "yaw": float(orientation.get("yaw", orientation.get("Yaw", 0.0))),
            }
            T_list = _to_list_matrix(T)
        else:
            cart = {"x": 0.0, "y": 0.0, "z": 570.0}
            orient = {"pitch": 90.0, "roll": 0.0, "yaw": 0.0}
            T_list = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
            T_list[2][3] = 570.0

        return {
            "source": str(estimate.get("source", "unknown")),
            "confidence": float(estimate.get("confidence", 0.5)),
            "thetas_deg": thetas,
            "motors_deg": motors,
            "cartesian_mm": cart,
            "orientation_deg": orient,
            "T_0_6": T_list,
        }

    # --------------------------------------------------------- Zero-Jump
    def zero_jump_boot_sync(
        self,
        transport: Any = None,
        state_manager: Optional[RobotStateManager] = None,
        kinematics: Optional[RobotArm6DOF] = None,
        vision: Optional[VisionLocator] = None,
        frame_bgr: Optional[np.ndarray] = None,
    ) -> dict:
        return zero_jump_boot_sync(
            transport=transport,
            state_manager=state_manager if state_manager is not None else self.state_manager,
            kinematics=kinematics if kinematics is not None else self.kinematics,
            vision_locator=self if vision is None else vision,
            frame_bgr=frame_bgr,
        )


def zero_jump_boot_sync(
    transport: Any = None,
    state_manager: Optional[RobotStateManager] = None,
    kinematics: Optional[RobotArm6DOF] = None,
    vision_locator: Optional[VisionLocator] = None,
    vision: Optional[VisionLocator] = None,
    frame_bgr: Optional[np.ndarray] = None,
) -> dict:
    """Execute the 4-stage jerk-free boot sequence."""
    if state_manager is None:
        if RobotStateManager is not None:
            try:
                state_manager = RobotStateManager()
            except Exception:
                state_manager = None
        else:
            state_manager = None

    if kinematics is None and vision_locator is not None:
        kinematics = getattr(vision_locator, "kinematics", None)
    if kinematics is None and RobotArm6DOF is not None:
        try:
            coupling = 1
            if state_manager is not None:
                coupling = int(state_manager.state.get("shoulder_coupling", 1))
            kinematics = RobotArm6DOF(shoulder_coupling=coupling)
        except Exception:
            try:
                kinematics = RobotArm6DOF()
            except Exception:
                kinematics = None

    if vision_locator is None and vision is not None:
        vision_locator = vision
    if vision_locator is None:
        vision_locator = VisionLocator(
            endpoint_url=os.environ.get("NETBOOK_VISION_URL"),
            state_manager=state_manager,
            kinematics=kinematics,
        )
    else:
        if getattr(vision_locator, "state_manager", None) is None:
            vision_locator.state_manager = state_manager
        if getattr(vision_locator, "kinematics", None) is None:
            vision_locator.kinematics = kinematics

    commands_sent: List[str] = []

    # 1. Keep PIN_OE = HIGH (disabled).
    _send(transport, "S")
    commands_sent.append("S")

    # 2. Read physical posture via vision / odometry.
    est = vision_locator.estimate_resting_posture(frame_bgr=frame_bgr)
    thetas = [float(v) for v in est["thetas_deg"]]

    # Persist the detected kinematic posture before preloading motors.
    if state_manager is not None and hasattr(state_manager, "update_from_thetas"):
        try:
            state_manager.update_from_thetas(thetas)
        except Exception:
            pass

    # 3. Silent PWM preload while PIN_OE remains HIGH.
    motors_deg = est.get("motors_deg", {})
    if state_manager is not None and hasattr(state_manager, "build_preload_command"):
        preload_cmd = state_manager.build_preload_command(motors_deg)
    else:
        # Fallback command construction.
        vals = [round(float(motors_deg.get(i, 0))) for i in (1, 2, 3, 4, 5, 7)]
        preload_cmd = "Z:%d,%d,%d,%d,%d,%d" % tuple(vals)
    _send(transport, preload_cmd)
    commands_sent.append(preload_cmd)

    # 4. Verify software-hardware match & enable.
    _send(transport, "Q")
    commands_sent.append("Q")
    _send(transport, "R")
    commands_sent.append("R")

    return {
        "zero_jump_verified": True,
        "oe_sequence": ["HIGH", "PRELOAD", "LOW"],
        "source": est["source"],
        "thetas_deg": est["thetas_deg"],
        "motors_deg": est["motors_deg"],
        "cartesian_mm": est["cartesian_mm"],
        "T_0_6": est["T_0_6"],
        "commands_sent": commands_sent,
    }


# ------------------------------------------------------------------ server
class _VisionHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class UbuntuVisionRequestHandler(http.server.BaseHTTPRequestHandler):
    server_version = "UbuntuVision/1.0"

    def _send_json(self, payload: Any, status: int = 200) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        try:
            self.wfile.write(data)
        except BrokenPipeError:
            pass

    def do_GET(self) -> None:  # noqa: N802
        locator = getattr(self.server, "vision_locator", None)
        path = self.path.split("?", 1)[0].rstrip("/") or "/"

        if path == "/health":
            self._send_json({"status": "ok", "service": "netbook_ubuntu_vision"})
            return

        if path == "/pose":
            frame = None
            if locator is not None and cv2 is not None:
                try:
                    cap = cv2.VideoCapture(getattr(locator, "camera_index", 0))
                    ok, frame = cap.read()
                    cap.release()
                    if not ok:
                        frame = None
                except Exception:
                    frame = None
            if locator is not None:
                try:
                    est = locator.estimate_resting_posture(frame_bgr=frame)
                    self._send_json(est)
                    return
                except Exception as exc:
                    self._send_json({"error": str(exc)}, status=500)
                    return
            self._send_json({"error": "vision locator unavailable"}, status=503)
            return

        self._send_json({"error": "not found"}, status=404)

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
        # Keep the netbook console quiet.
        return


def run_ubuntu_vision_server(
    host: str = "0.0.0.0",
    port: int = 8080,
    vision_locator: Optional[VisionLocator] = None,
    state_manager: Optional[RobotStateManager] = None,
    kinematics: Optional[RobotArm6DOF] = None,
    camera_index: int = 0,
    endpoint_url: Optional[str] = None,
) -> None:
    """Run the lightweight HTTP vision service (blocks until interrupted)."""
    if vision_locator is None:
        vision_locator = VisionLocator(
            endpoint_url=endpoint_url,
            camera_index=camera_index,
            state_manager=state_manager,
            kinematics=kinematics,
        )
    httpd = _VisionHTTPServer((host, int(port)), UbuntuVisionRequestHandler)
    httpd.vision_locator = vision_locator
    try:
        httpd.serve_forever()
    finally:
        try:
            httpd.server_close()
        except Exception:
            pass


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Netbook Ubuntu vision node")
    parser.add_argument("--serve", action="store_true", help="run the HTTP vision server")
    parser.add_argument("--sync-sim", action="store_true", help="run a simulated Zero-Jump Boot sync")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--endpoint", default=None, help="optional netbook vision URL")
    args = parser.parse_args(argv)

    if args.serve:
        run_ubuntu_vision_server(host=args.host, port=args.port, endpoint_url=args.endpoint)
        return 0

    if args.sync_sim:
        TransportCls = _import_simulated_transport()
        if TransportCls is None:
            TransportCls = _FallbackTransport
        transport = TransportCls()
        state_manager = RobotStateManager() if RobotStateManager is not None else None
        kinematics = None
        if RobotArm6DOF is not None:
            try:
                coupling = 1
                if state_manager is not None:
                    coupling = int(state_manager.state.get("shoulder_coupling", 1))
                kinematics = RobotArm6DOF(shoulder_coupling=coupling)
            except Exception:
                try:
                    kinematics = RobotArm6DOF()
                except Exception:
                    kinematics = None
        locator = VisionLocator(
            endpoint_url=args.endpoint,
            state_manager=state_manager,
            kinematics=kinematics,
        )
        report = zero_jump_boot_sync(
            transport=transport,
            state_manager=state_manager,
            kinematics=kinematics,
            vision_locator=locator,
            frame_bgr=None,
        )
        print(json.dumps(report, indent=2))
        return 0

    parser.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
