# Vision pipeline

Real-time vision for the 6-DOF arm on this Windows PC.

## Cameras on this machine

| Index | Device | Use |
|---|---|---|
| 0 | Trust USB Camera (VID_145F:PID_0203) | **arm camera** — points at the shelf where the arm lives |
| 1 | HP Wide Vision HD Camera (integrated) | laptop webcam, not used |
| 2 | NVIDIA virtual camera | black frames, ignore |

`python tools\camera_probe.py` re-scans and writes one JPEG per index to
`tools/camera_previews/`.

## Backend choice (measured, not theoretical)

The same camera behaves very differently per backend at 1280x720:

| Backend | Negotiated format | Effective FPS |
|---|---|---|
| `CAP_DSHOW` | YUY2 | **8.1** (USB 2.0 bandwidth starved) |
| `CAP_MSMF` | MJPG | **30.0** |
| `CAP_DSHOW` at 640x480 | YUY2 | 30.2 |

So `FastUSBCamera(backend="auto")` (the default) uses **MSMF for 720p and above**,
DSHOW for smaller frames, and falls back to the other backend if opening fails.
Forcing DSHOW at 720p is a known-bad configuration on this hardware.

Measured with `python tools\bench_camera.py --index 0 --frames 100`:

```text
unique_frames        100
capture_fps_median   32.07
latency p50          1.19 ms
latency p95          1.28 ms
latency max          1.52 ms
```

`latency_ms` is **delivery latency**: how old the frame is when the consumer gets
it. It is under 2 ms because the reader thread keeps only the newest frame.
The physical floor of a 30 FPS sensor is one frame period (33 ms); this camera does
not expose a 60 FPS mode (requesting 60 still yields ~29.3), so sub-15 ms
sensor-to-screen latency is not achievable with this unit. Add a 60 FPS camera if
that is a hard requirement.

`dropped` in `stats()` counts frames overwritten before a consumer read them. A high
value is normal and desirable: it proves there is no backlog.

## Blue-arm segmentation is fragile in this room

The scene is lit with a strong blue cast (mean BGR is roughly `165/163/157`, median
saturation 16). Colour alone cannot separate "blue printed arm" from "blue-lit
walls": with permissive HSV thresholds the mask covers ~35% of the frame.

`ArmVisionAnalyzer(mode="geometric")` still produces a skeleton (contour filtering
rejects blobs clipped by the frame edge and filled rectangles such as the laptop),
but on this lighting it locks onto the upper links only and the angles are
**indicative, not metrology**.

**Use ArUco for real angle measurements.** Print the sheet and mount the markers:

```powershell
python tools\make_aruco_sheet.py      # -> tools/aruco_sheet.png
```

| Marker id | Joint |
|---|---|
| 0 | base |
| 1 | shoulder |
| 2 | elbow |
| 3 | wrist pitch |
| 4 | wrist roll |
| 5 | gripper tip |

Print at 100%, cut with a 10 mm white border, and glue each marker flat on its link.
With 4+ markers visible the analyzer switches to `mode_used="aruco"` and derives the
angles trigonometrically instead of by colour.

## Modules

| File | Purpose |
|---|---|
| `config/env_performance.py` | Windows env vars + OpenCV thread/log setup, imported before cv2 |
| `src/fast_camera.py` | `FastUSBCamera`: daemon thread, newest-frame-only, FPS/latency stats |
| `src/arm_vision_analyzer.py` | geometric (HSV + black MG946R boxes), ArUco, optional ONNX/YOLO |
| `src/pulse_odometry.py` | pulse <-> motor degrees <-> kinematic thetas (firmware-exact) |
| `src/dataset_recorder.py` | `dataset/images` + YOLO-keypoint labels + JSON sidecar |
| `src/live_vision_hud.py` | live overlay + keys, headless benchmark mode |

## Running

```powershell
# Live window with skeleton, angles, XYZ, OE state, FPS
python -m src.live_vision_hud --camera 0

# Headless benchmark (no window)
python -m src.live_vision_hud --camera 0 --max-frames 100 --headless

# Dataset capture while jogging the arm with the CLI in another terminal
python -m src.live_vision_hud --camera 0
# press G at each pose
```

HUD keys: `SPACE` emergency stop (`S`), `Z` zero/real-state sync, `G` save dataset
sample, `Q`/`ESC` quit.

The HUD analyzes every second frame (`analyze_every=2`) because one analysis costs
~16-21 ms; the display still runs at camera rate with the last valid overlay.

## Dataset format

`dataset/labels/000001.txt`:

```text
0 cx cy w h  x0 y0 v0  x1 y1 v1  x2 y2 v2  x3 y3 v3  x4 y4 v4  x5 y5 v5
```

All coordinates normalized to [0,1]; `v = 2` visible, `1` low confidence, `0` missing.
`dataset/images/000001.jpg` is the raw frame and `000001.json` carries
`thetas_deg`, `motors_deg`, `T_0_6`, `cartesian_mm`, timestamp and source mode.
