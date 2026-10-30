# Feature: 6-DOF Kinematic Control System (7x MG946R + AUX + PCA9685 + Arduino Mega 2560)

Branch `feat/arm-8axis` · Delivery: local feature branch, work-unit commits (`exception-ok`: no git remote configured) · Test runner: `python -m pytest tests/ host/tests/ -v` + `arduino-cli compile --fqbn arduino:avr:mega firmware_mega`.

## Specs

S1 — Native PCA9685 Firmware (`firmware_mega/firmware_mega.ino`). Builds for `arduino:avr:mega` using only `<Wire.h>` and AVR core (no external libraries). PCA9685 at `0x40`, 50 Hz (`PRESCALE = 121`), `SDA` -> Pin 20, `SCL` -> Pin 21, `PIN_OE` -> Pin 2 (`LOW` = active, `HIGH` = physical emergency stop). Dual UART: `Serial` (USB, `115200`) and `Serial1` (pins 18/19, `115200`).

S2 — Physical Motor Map (1 to 7 from Gripper to Base, plus 8 AUX):
- Motor 1 (Channel 0) `PINZA`: Gripper jaws ($\theta_6 \in [0^\circ, 100^\circ]$).
- Motor 2 (Channel 1) `GIRO_PINZA`: Wrist roll ($\theta_5 \in [0^\circ, 360^\circ]$).
- Motor 3 (Channel 2) `INCLINA_PINZA`: Wrist pitch ($\theta_4 \in [-90^\circ, +90^\circ]$, $0^\circ$ kinematic = $90^\circ$ vertical servo).
- Motor 4 (Channel 3) `CODO`: Elbow pitch ($\theta_3 \in [-90^\circ, +90^\circ]$).
- Motor 5 (Channel 4) `HOMBRO_IZQ`: Shoulder master ($\theta_2 \in [15^\circ, 165^\circ]$).
- Motor 6 (Channel 5) `HOMBRO_DER`: Shoulder slave coupled to Motor 5. Default `SHOULDER_COUPLING = +1` (same spatial orientation), configurable to `-1` (`180 - deg` mirror) via serial command or JSON without recompiling. Synchronized in the same I2C tick (< 1 ms delta).
- Motor 7 (Channel 6) `BASE`: Base turntable yaw ($\theta_1 \in [0^\circ, 360^\circ]$).
- Motor 8 (Channel 7) `AUX`: Spare channel.

S3 — `HOME` Zero Posture:
- Physically vertical pointing straight up: $\theta_1 = 0^\circ$, $\theta_2 = 90^\circ$, $\theta_3 = 0^\circ$, $\theta_4 = 0^\circ$, $\theta_5 = 0^\circ$, $\theta_6 = 50\%$.

S4 — Non-blocking 50 Hz Motion Engine & Serial Protocol (`115200` baud):
- Tick every 20 ms (`millis()`), no blocking `delay()` during motion. Supports positional servos (trapezoidal/linear interpolation) and 360° continuous servos (timed pulse activation $\Delta t$ ms with automatic return to `servoStop[ch]`).
- Commands:
  - `I` -> I2C scanner (`{"status":"OK","pca9685":"CONECTADO_0x40"}`).
  - `S` -> Emergency stop (`PIN_OE = HIGH` + all channels to `servoStop`).
  - `R` -> Enable outputs (`PIN_OE = LOW`).
  - `H` -> Go to vertical `HOME` ($90^\circ$).
  - `G:<m1>,<m2>,<m3>,<m4>,<m5>,<m7>,<tiempo_ms>` -> Coordinated multi-axis move in `tiempo_ms` (Motor 6 computed automatically from `m5` via `SHOULDER_COUPLING`).
  - `T<motor>:<offset_pwm>:<ms>` -> Timed 360° spin with automatic brake to `servoStop` when `ms` expires.
  - `C<motor>:<pulso_stop>:<deg_per_sec>` (also accepts `C<motor>:<pulso_stop>`) -> Hot calibration of neutral pulse and angular speed.
  - `U<+1|-1>` / `J6` -> Configure `SHOULDER_COUPLING` (`+1` same direction, `-1` mirror `180 - deg`).
  - `Q` -> JSON telemetry with current estimated state of the motors.
  - Preserves backward-compatible `M<motor>:<deg>[:<ms>]`, `P<motor>:<pulse>`, `F`, `V`.

S5 — 6-Stage SE(3) Mathematical Model (`src/kinematics_6dof.py` & `config/config_geometria.json`):
- Configurable geometry: `L1 = 115.0`, `L2 = 145.0`, `L3 = 135.0`, `L4 = 55.0`, `d5 = 45.0`, `L6_finger = 75.0` mm.
- Chained $4 \times 4$ homogeneous transforms $T_0^6 = T_0^1(\theta_1) \cdot T_1^2(\theta_2) \cdot T_2^3(\theta_3) \cdot T_3^4(\theta_4) \cdot T_4^5(\theta_5) \cdot T_5^6(\theta_6)$.
- `forward_kinematics(thetas_deg)`: returns individual matrices $[T_0^1 \dots T_5^6]$, global $T_0^6$, TCP `[X, Y, Z]` in mm, orientation `[Pitch, Roll, Yaw]`, gripper opening in mm, and 3D positions of all intermediate joints.
- `inverse_kinematics(x, y, z, pitch_deg, roll_deg, grip_percent)`: analytical Elbow-Up solver + Jacobian ($6 \times 5$) Damped Least Squares refinement, plus `move_linear_xyz` Cartesian waypoint interpolation and Workspace & Collision Guard ($Z \ge 10\text{ mm}$ for elbow, wrist, and TCP; max radius check).

S6 — Real-Time Odometry, Persistent State & Interactive Calibration (`src/state_manager.py`, `src/calibrate_servos.py`, `robot_state.json`):
- Persists angles of motors 1..7, current `(X, Y, Z)`, `(Pitch, Roll, Yaw)`, $T_0^6$, `shoulder_coupling` (`+1` or `-1`), per-motor `stop_pulse` and `deg_per_sec`, and bounded command history.
- Enforces shoulder coupling invariant so channels 4 and 5 never receive contradictory commands.
- Interactive calibration assistant (`src/calibrate_servos.py`) for finding deadband stop pulse (`305`..`310`), verifying Motor 5/6 coupling sign (`+1` vs `-1`), and measuring angular speed (`deg_per_sec`).

S7 — Automated QA Test Suite (`tests/test_kinematics_and_protocol.py`):
- Test 1 (`HOME` identity): at initial vertical posture, $X \approx 0, Y \approx 0, Z = L_1 + L_2 + L_3 + L_4 + d_5 + L_6 = 570.0\text{ mm}$.
- Test 2 (FK <-> IK consistency): 1000 random reachable poses across $360^\circ$; positional error $< 0.5\text{ mm}$.
- Test 3 (Shoulder coupling): no function can emit contradictory commands to PCA9685 channels 4 and 5.
- Test 4 (Table collision guard): requesting $Z = -20\text{ mm}$ raises a controlled exception before sending any serial bytes.

S8 — Hardware Integrator, CLI & AI Agent API (`src/agent_controller.py`):
- Auto-detects Arduino Mega COM port on Windows (`COM4`), compiles/flashes `firmware_mega/firmware_mega.ino` via `arduino-cli`, verifies `I` (`CONECTADO_0x40`), and exposes:
  - `get_current_pose()`
  - `move_to_xyz(x_mm, y_mm, z_mm, pitch_deg, roll_deg, speed_mm_s)`
  - `rotate_wrist(roll_deg_360)`
  - `set_gripper(open_percent)`
  - `pick_and_place(origin_xyz, target_xyz, approach_height_mm)`
  - `emergency_stop()`
  - `zero_jump_boot_sync()`

S9 — Vision Node (Netbook Conectar Igualdad / Ubuntu Server) & Read-First / Zero-Jump Boot (`src/vision_locator.py` & `src/state_manager.py` & `firmware_mega/firmware_mega.ino`):
- In `setup()`, `firmware_mega.ino` MUST keep `digitalWrite(PIN_OE, HIGH)` so the arm boots with PWM outputs physically disabled.
- Supports silent PWM preload (`Z:<m1>,<m2>,<m3>,<m4>,<m5>,<m7>` and `P`/`M` while `OE=HIGH`) and EEPROM odometry persistence (`E` / auto-save).
- `src/vision_locator.py` implements:
  1. Reading the current physical posture from the Ubuntu Server netbook camera (ArUco markers or HSV blue-arm silhouette, HTTP JSON endpoint `/pose`, plus a built-in `--serve` HTTP vision server script for the Ubuntu Server netbook).
  2. Feeding those real angles into $T_0^6$ (`forward_kinematics`) to compute the exact resting $(X, Y, Z)$.
  3. Silently preloading the PWM registers on the PCA9685 while `PIN_OE` remains `HIGH`.
  4. Lowering `PIN_OE` to `LOW` (`R`) ONLY after verifying software-hardware angle match, achieving a 100% jerk-free startup.

S10 — Windows performance environment (`config/env_performance.py`):
- Imported at the top of every vision entry point, sets `OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS=0`, `OPENCV_VIDEOIO_PRIORITY_DSHOW=100`, `OMP_NUM_THREADS=4`, `OPENBLAS_NUM_THREADS=4`, `CUDA_MODULE_LOADING=LAZY`, `YOLO_VERBOSE=False` before NumPy/OpenCV import.

S11 — Zero-lag multi-threaded USB capture (`src/fast_camera.py`):
- `FastUSBCamera` opens `cv2.VideoCapture(index, cv2.CAP_DSHOW)`, forces `MJPG` FOURCC before width/height/FPS (`1280x720` or `1920x1080` at `30/60`), `CAP_PROP_BUFFERSIZE=1`, optional short manual exposure (`CAP_PROP_AUTO_EXPOSURE` / `CAP_PROP_EXPOSURE=-6`).
- A daemon thread always keeps ONLY the newest frame; `get_latest_frame()` returns `(frame, timestamp_ns, fps_real)` in `< 0.2 ms` with no queue backlog.

S12 — Hybrid blue-arm visual analyzer (`src/arm_vision_analyzer.py`):
- Mode 1 geometric: HSV segmentation of the blue printed links + black MG946R joint boxes, PCA/contour principal axes, 2D/3D kinematic skeleton (Base → Shoulder → Elbow → Wrist → Gripper) and live angles `[theta1..theta6]` relative to the table horizontal.
- Mode 2 fiducial: `DICT_4X4_50` ArUco markers on the links take priority with sub-degree trigonometry.
- Mode 3 neural: `KeypointNeuralPredictor` loads `models/arm_keypoints.onnx` (cv2.dnn) or `yolov8n-pose.pt`/`yolo11n-pose.pt` (ultralytics) when available, predicting the 6 joint points.
- Returns one normalized result dict with `thetas_deg`, `motors_deg`, `keypoints_px`, `skeleton`, `cartesian_mm`, `T_0_6`, `confidence`, `latency_ms`.

S13 — Dataset auto-recorder (`src/dataset_recorder.py`):
- Saves `dataset/images/NNNNNN.jpg` at capture resolution and `dataset/labels/NNNNNN.txt` in normalized YOLO keypoints format (class cx cy w h, then 6x x y v), plus a sidecar `.json` with `thetas_deg`, `motors_deg`, `T_0_6`, `cartesian_mm`.

S14 — Live monitoring HUD (`src/live_vision_hud.py`):
- OpenCV window at camera rate showing skeleton, per-joint angles, live TCP `[X, Y, Z]` mm from the 6 matrices, `OE` state (`HIGH [SAFE]` / `LOW [ACTIVE]`), FPS and latency.
- Keys: `SPACE` emergency stop (`S`), `Z` zero/real-state in $T_0^6$, `G` save dataset sample.

S15 — Pulse odometry bridge (`src/pulse_odometry.py`):
- Converts PCA9685 pulse ↔ motor degrees ↔ kinematic thetas for all 8 motors using the calibrated `pulse_min`/`pulse_stop`/`pulse_max` and `home` anchoring, with shoulder coupling (motor 6 = master if coupling +1, else `180 - master`).

## Tasks

| ID | Specs | Route | Status | Commit |
|----|-------|-------|--------|--------|
| T1 | S1-S4 | delegated (Sub-Agent 1: `firmware_mega/firmware_mega.ino`) | done | `59d8592` |
| T2 | S5 | delegated (Sub-Agent 2: `src/kinematics_6dof.py` + `config/config_geometria.json`) | done | `fb4727c` |
| T3 | S6 | delegated (Sub-Agent 3: `src/state_manager.py` + `src/calibrate_servos.py`) | done | `a74ff1e` |
| T4 | S7 | delegated (Sub-Agent 4: `tests/test_kinematics_and_protocol.py`) | done | `ad3436b` |
| T5 | S8 | delegated (Sub-Agent 5: `src/agent_controller.py`) | done | `8f88005` |
| T6 | S1-S8 | inline + independent verifier (`opencode-go/iceman-contributor-free`, all S1-S8 PASS) | done | `ad3436b` |
| T7 | S9 + S1-S8 | delegated (5 Sub-Agents on `opencode-go/*`: Zero-Jump Boot `OE=HIGH`, `src/vision_locator.py`, pitch interpolation in `move_linear_xyz`, QA `test_06`, live `COM4` upload & demo) | done | `HEAD` |
| T8 | S10-S11 | delegated (Fast camera: `config/env_performance.py` + `src/fast_camera.py` + tests) | todo | — |
| T9 | S12 | delegated (Analyzer: `src/arm_vision_analyzer.py` + tests) | todo | — |
| T10 | S13-S14 | delegated (Recorder + HUD: `src/dataset_recorder.py` + `src/live_vision_hud.py` + tests) | todo | — |
| T11 | S15 | delegated (Pulse odometry: `src/pulse_odometry.py` + tests) | todo | — |
| T12 | S10-S15 | inline (camera index scan, 100-frame benchmark, pytest, live HUD smoke) | todo | — |

## Log

L1 — User initial brief: 8-axis 3D-printed arm, Arduino Mega 2560 R3, PCA9685 at 50 Hz over hardware I2C (`SDA 20`, `SCL 21`), 8x MG946R, 5 V/27 A servo rail isolated from the Mega, OE on pin 2.

L2 — 2026-10-29: Workspace moved to `C:\Users\merca\Documentos\brazo-8ejes` due to ACL deny in OneDrive folder.

L3 — 2026-10-29: Toolchain detected: `arduino-cli 1.5.1`, `arduino:avr 1.8.8`, `esp32:esp32 3.3.12`, Python 3.14.3 with `pyserial 3.5`, `numpy 2.3.5`, `pytest 9.0.3`, CH340 on `COM4`.

L4 — 2026-10-29: Firmware boots with OE disabled (`HIGH`) so nothing jerks at power-up.

L5 — 2026-10-29: User supplied motor order 1..7 from gripper to base + 8 AUX.

L6 — 2026-10-30: User supplied the Master Multi-Agent Orchestration Prompt for the 6-DOF Kinematic Control System:
- 5 specialized sub-agents required:
  1. `firmware_mega/firmware_mega.ino` (50 Hz `millis()` non-blocking engine, `SHOULDER_COUPLING = +1` default configurable to `-1`, commands `I`, `S`, `R`, `H`, `G:<m1>,<m2>,<m3>,<m4>,<m5>,<m7>,<tiempo_ms>`, `T<motor>:<offset_pwm>:<ms>`, `C<motor>:<pulso_stop>:<deg_per_sec>`, `Q`).
  2. `src/kinematics_6dof.py` + `config/config_geometria.json` (`RobotArm6DOF`, 6 homogeneous transforms $T_0^1 \dots T_5^6$, `L1=115, L2=145, L3=135, L4=55, d5=45, L6_finger=75`, analytical Elbow-Up IK + 6x5 Jacobian DLS + `move_linear_xyz` + collision guard $Z \ge 10\text{ mm}$).
  3. `src/state_manager.py` + `src/calibrate_servos.py` (`robot_state.json` real-time state matrix, shoulder coupling enforcement, interactive calibration assistant).
  4. `tests/test_kinematics_and_protocol.py` (HOME identity $Z=570\text{ mm}$, 1000-point FK<->IK $<0.5\text{ mm}$, shoulder coupling guard, table collision guard at $Z=-20\text{ mm}$).
  5. `src/agent_controller.py` (COM auto-detection, `arduino-cli` compile/flash, `get_current_pose`, `move_to_xyz`, `rotate_wrist`, `set_gripper`, `pick_and_place`, `emergency_stop`).

L7 — 2026-10-30: User clarified: "es solamantes el planteamiento no esta conectado el mega". Do not flash or block on physical serial hardware; verify firmware compilation via `arduino-cli compile` and run `agent_controller.py` / QA tests in simulation / offline verification mode.

L8 — 2026-10-30: User ran `python -m src.agent_controller --demo` (which revealed `move_linear_xyz` failed on waypoint 0 when starting from singular vertical `HOME` `(0,0,570)` with target `pitch_deg=60.0` instead of interpolating `pitch_deg` from start to target) and supplied the updated Master Prompt adding the Vision Node on the Netbook Conectar Igualdad (Ubuntu Server) + Read-First / Zero-Jump Boot (`src/vision_locator.py`, `setup()` keeping `PIN_OE = HIGH`, silent PWM preload before `R`, EEPROM odometry persistence).
