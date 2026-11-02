# Phase 1: Optical encoder + pre-power audit

## Purpose
Phase 1 proves the arm is optically stable with servos always OFF (`OE=HIGH`). It validates that the virtual optical encoder can read joint positions (via ArUco markers, with geometric fallback) without energizing the servo rail. Only after passing the audit can the system proceed to bring-up.

## Operational vs strict gate
- **Standalone checker gate:** `tools/check_camera_positions.py` defaults to `max_variance_deg2 < 1.5` (`--max-variance`). This checker reports stability; the single-motor tool's mode-aware progression gate is described below.
- **Strict/ArUco-grade target:** `0.15 deg^2` (`--strict`), not required in geometric mode. This is the metrology target when using ArUco markers for high-precision measurements.

For `tools/single_motor_test.py`, the automatic per-window limit is mode-aware: ArUco and partial-ArUco audits use 1.5 deg², while geometric fallback uses 5.0 deg². Geometric fallback is a noisier, supervision-only estimator, so 5.0 deg² tolerates ordinary pixel/angle noise for a bounded single-motor bring-up without treating it as metrology-grade. Its `--max-variance` forces a chosen limit; its `--strict` forces 1.5 deg² in either mode. (`tools/check_camera_positions.py --strict` remains the separate 0.15 deg² metrology target.) The single-motor tool still requires three consecutive passes within its bounded audit-attempt window.

## Audit command
Use `tools/check_camera_positions.py` (or equivalent) to run the pre-power audit:

```powershell
python tools\check_camera_positions.py --frames 30 --max-variance 1.5 --snapshot tools/camera_previews/optical_encoder_audit.jpg
```

### Flags
- `--frames N`: number of frames for the pre-power optical audit (default ~30; ~1.0 s @ 30 FPS)
- `--max-variance FLOAT`: operational variance gate in deg^2 (default 1.5). Values < 1.5 pass the primary gate.
- `--strict`: uses the stricter 0.15 deg^2 target (ArUco-grade). Not required for geometric mode bring-up supervision.
- `--snapshot PATH`: saves an annotated snapshot of the audit frame/state.

## Reading the JSON result
The audit returns (from `VirtualOpticalEncoder.audit_pre_power`):
- `passed` (bool): whether the audit passed the gate
- `primary_max_variance_deg2` (float): max variance over joints 0-3 (primary chain). Must be < `max-variance` (default 1.5)
- `secondary_max_variance_deg2` (float): max variance over joints 4-5 (wrist/distal). Secondary limit is `max(1.5, max_variance_deg * 3.0)` in the implementation
- `oe_recommendation`: `"READY_FOR_PHASE_2"` if passed, else `"HOLD_HIGH"`
- `camera_framing_45deg` / framing assistant: uses ellipse ratio `b/a` from the 45° framing check and `asin(b/a)` to assess framing quality (helper in framing assistant)
- `frames_evaluated`, `mode_used`, `visible_marker_ids`, `resting_thetas_deg`, `resting_motors_deg`, `variances_deg2`, `reason`

## 45° framing assistant
The framing assistant evaluates camera placement at ~45° using the ellipse ratio `b/a` of a fitted ellipse; the angle is derived via `asin(b/a)`. This helps ensure the arm is framed so markers/segments are visible and measurable.

## Known geometric limitations
- In geometric mode (HSV/contour based), the distal wrist segment (`WRIST_P→GRIPPER`, ~60 px) amplifies ~2 px of noise into several degrees. 
- Geometric mode is suitable for **bring-up supervision only**; it is not metrology-grade.
- For real/accurate angle measurements, use **ArUco** with markers IDs 0..6 mounted on the links. With 4+ markers visible, the analyzer switches to ArUco mode and derives angles trigonometrically.
- Marker layout: 0=base, 1=shoulder, 2=elbow, 3=wrist pitch, 4=wrist roll, 5=gripper tip. Print at 100% with 10 mm white border, glue flat.
