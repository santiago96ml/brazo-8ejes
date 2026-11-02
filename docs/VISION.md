# Vision pipeline

Real-time vision for the 6-DOF arm on this Windows PC.

## Cameras on this machine

| Index | Device | Use |
|---|---|---|
| 0 | Trust USB Camera (VID_145F:PID_0203) | **arm camera** â€” points at the shelf where the arm lives |
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


## UVC colour control and CLAHE

`FastUSBCamera` accepts `auto_wb`, `wb_temperature`, `saturation`, `brightness`,
`contrast`, `gain` and `sharpness`; they are applied after format negotiation and are
best-effort (MSMF silently ignores unsupported controls). Recommended start for the
blue cast:

```python
FastUSBCamera(index=0, auto_wb=0, wb_temperature=4600, saturation=160)
```

`ArmVisionAnalyzer(use_clahe=True)` (default) applies CLAHE on the LAB `L` channel at
segmentation resolution. Measured on the saved real frame: the blue mask drops from
166 212 px to 88 464 px (less wall bleed) with an identical skeleton, and steady-state
analysis costs ~21 ms instead of ~18 ms.

## Camera-guided pulse calibration

`src/visual_pulse_calibrator.py` closes the loop camera -> Mega -> servos:

```powershell
# Dry run: no serial, no camera, prints the plan
python -m src.visual_pulse_calibrator --dry-run --motors 2 --json

# Real calibration of one motor (camera index 0, Mega on COM4)
python -m src.visual_pulse_calibrator --port COM4 --camera 0 --motor 2
```

Stages per motor: `S` (OE HIGH) -> vision read -> `Z:...` preload -> `R`; then a
304..311 sweep picking the pulse with the smallest pixel displacement (< 0.35 px/s),
then two 200 ms bursts to measure `deg_per_ms_pos` / `deg_per_ms_neg`. Results go to
`config/servo_pulse_profile.json` and to the Mega (`C<motor>:<stop>:<dps>` + `E`).
Motors 5 and 6 are calibrated as one pair (master 5, mirrored 6). A dry run never
writes the profile.

**The camera is exclusive**: close any open HUD window before calibrating, otherwise
MSMF opens the device but cannot grab frames.

## Dataset collection and YOLO11-pose training

```powershell
python tools\auto_collect_dataset.py --camera 0 --count 100 --out dataset --inpaint
python tools\train_arm_keypoints.py --dataset dataset --epochs 100 --imgsz 640 --model yolo11n-pose.pt --export-onnx
```

`auto_collect_dataset.py` walks 100 deterministic safe poses (radius 120..320 mm,
Z 200..520 mm, pitch 30..80 deg), requires at least 4 ArUco markers, optionally
inpaints the marker squares so the network learns the blue link and the black servo,
and writes image + YOLO-keypoint label + JSON sidecar through `DatasetRecorder`.

`train_arm_keypoints.py` builds `dataset/arm_pose.yaml` (`kpt_shape: [6, 3]`,
keypoints `base, shoulder, elbow, wrist_pitch, wrist_roll, gripper_tip`), trains
`yolo11n-pose.pt`, and exports `models/arm_keypoints.onnx`.

### Host status (verified 2026-10-30)

```text
torch          2.14.1+cu126   CUDA 12.6, RTX 4050 Laptop visible
torchvision    0.29.1+cu126
ultralytics    8.4.194
onnx / onnxruntime installed
```

Smoke test on 12 synthetic images, `imgsz=320`, 1 epoch:

```text
CUDA:0 (NVIDIA GeForce RTX 4050 Laptop GPU, 6140MiB)
1 epoch completed in 8.2 s of training batch
weights/best.pt 5.6 MB
ONNX export OK (22.6 s)
```

That proves dataset -> train -> export works on the GPU. The model itself is
meaningless because the images are synthetic; real training needs the ArUco-labelled
dataset.

Python 3.14 CUDA wheels exist only under the **cu126** index; cu124 has no cp314
build and `pip install torch` from it silently keeps the CPU wheel. Pin explicitly:

```powershell
python -m pip install --index-url https://download.pytorch.org/whl/cu126 --no-deps --force-reinstall torch==2.14.1+cu126 torchvision==0.29.1+cu126
```

`KeypointNeuralPredictor` loads only `models/arm_keypoints.onnx` or
`models/arm_keypoints.pt`. A generic COCO `yolo11n-pose.pt` is never auto-loaded:
its keypoints are human joints and would silently poison `mode="hybrid"`.

## Operational vs strict gate

- **Operational gate (geometric/bring-up):** `max_variance_deg2 < 1.5` is the operational gate used by `DecisionGate` during Phase 1 pre-power audit. This allows progression to `R` after 3 consecutive passes.
- **Strict/ArUco-grade target:** `0.15 deg^2` (`--strict`) is the metrology-grade target when using ArUco markers. It is **not required in geometric mode**; geometric mode is for bring-up supervision only because the distal wrist segment amplifies noise. For accurate angle measurements, use ArUco (IDs 0..6).

## Keyboard jog (teleoperation)

| Key | Action |
|---|---|
| `1`..`8` | select the active motor |
| `w` / up arrow / `+` | jog +one step |
| `s` / down arrow / `-` | jog -one step |
| `SPACE` | emergency stop (`S`) |
| `Z` | zero / real-state sync |
| `G` | save dataset sample |
| `Q` / `ESC` | quit |

Positional joints send `M<motor>:<deg>:400` (3 deg step, clamped 0..180 by the
firmware). Continuous joints send a timed burst `T<motor>:<offset>:150` (+/-40).
The mode comes from `config/arm.json` (`mode`, default `P`).

The HUD prints `controls are SIMULATED` at startup unless `--port COM4` is passed;
the overlay prefixes `SIM (no hardware)` so nobody mistakes a simulated E-STOP for a
real one.
