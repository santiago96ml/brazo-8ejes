# Bring-up (single motor, top-down)

## Safety pre-checks (mandatory)
- **One motor at a time.** Never energize multiple servos simultaneously during bring-up.
- **5 V current-limited.** Power the servo rail from a current-limited 5 V supply. **Never power the servo rail from the Mega 5V pin.**
- **Common ground.** Ensure common ground between Mega, PCA9685, and servo supply.
- **Servos OFF at start.** `OE=HIGH` until Phase 1 passes and `DecisionGate` authorizes `R`.
- **Horns loose.** Test with servo horns loose. See [docs/CALIBRATION.md](docs/CALIBRATION.md).
- **Firmware:** `firmware_mega/firmware_mega.ino` v2.1.0 is the canonical binary for COM4.

## Bring-up order
Follow this order strictly (one motor at a time):
1. Motor 1 **PINZA** (gripper)
2. Motor 2 **GIRO_PINZA** (wrist roll)
3. Motor 3 **INCLINA_PINZA** (wrist pitch)
4. Motor 4 **CODO** (elbow)
5. Motor 5 & 6 **HOMBRO** (coupled shoulder pair) - treated together
6. Motor 7 **BASE** (waist yaw)

Motor→joint mapping (from README): 1:PINZA, 2:GIRO_PINZA, 3:INCLINA_PINZA, 4:CODO, 5:HOMBRO_IZQ, 6:HOMBRO_DER (mirrored), 7:BASE, 8:AUX.

## Checkout procedure per motor
Use `tools/single_motor_test.py` (conceptual procedure). The sequence is:
1. **Audit** - Run Phase 1 optical audit with servos OFF (`OE=HIGH`). Ensure `passed` with `primary_max_variance_deg2 < 1.5`. Need 3 consecutive passes for `DecisionGate` to allow `R`.
2. **Gate** - `DecisionGate` requires 3 consecutive passes with `var < 1.5` before issuing `R`. If variance is in moderate range, a **40 ms probe pulse** may be used to verify direction.
3. **Probe** - If needed, issue a short probe (40 ms) to verify direction/motion response; verify with optical feedback.
4. **Burst** - Small controlled bursts to test motion in safe range.
5. **Measure** - Measure position/response via optical encoder (ArUco preferred for accuracy).
6. **Brake/stop** - Return to safe state.
7. **S** - End with `S` (estop) which drives all channels to stop pulse and raises `OE` (removes PWM).

Every path ends with `S`.

## Failure statuses
- `ABORT_AUDIT`: Phase 1 audit failed (variance too high, not enough consecutive passes). Do not enable outputs. Check camera framing, lighting, marker visibility.
- `ABORT_PROBE`: Probe pulse did not produce expected motion/direction response. Hold in safe state, verify wiring/power, do not proceed.
- `NO_MOTION`: No motion detected during burst/probe. Check servo power, channel wiring, horn attachment, stop pulse calibration.

## Physical link proportions (reference)
Used by skeleton: L1=115, L2=145, L3=135, L4=55, d5=45, L6=75 mm. `WRIST_R` locked 55 mm after `WRIST_P` (inside the U-fork, before the brass standoffs), `GRIPPER` 120 mm after `WRIST_R`.

## Rollback
- Send `S` immediately.
- Power off the servo rail.
- Inspect wiring, connections, and horn positions before retrying.
- Re-run Phase 1 audit with servos OFF before any new attempt.

## Notes
- Geometric mode is for bring-up supervision only (distal segment amplifies noise). For metrology use ArUco (IDs 0..6).
- Shoulder motors 5 and 6 are coupled (`180 - angle(5)` in README notes).
- Keep sessions to one motor at a time to isolate issues.
