# Calibration

Every MG946R is slightly different, and the "360 degrees" units in this build may be
positional or continuous. Calibrate with the arm unloaded and the servo horns **not**
bolted to the structure until the direction tests pass.

All commands below use the user motor number 1..8 (1 PINZA ... 7 BASE, 8 AUX).
Send them with `tools\armctl.ps1 raw "<command>"` or in the REPL.

## 1. Identify positional vs continuous

1. Connect, then `R` to enable.
2. `P<motor>:307` (neutral). A positional servo holds its position and resists a
   gentle push. A continuous servo stops turning.
3. `T<motor>:20:400` then `T<motor>:-20:400`. A positional servo makes a small move
   and stays there; a continuous servo rotates for 400 ms and stops.

Record the mode per motor and set it once:

```text
N1:P
N2:P
...
```

`M` is refused on a continuous channel with `MODE_CONT_USE_T`; that is intentional.

## 2. Continuous motors: exact stop point

A "307" is nominal, not exact. A continuous servo that creeps at neutral will fight
the rest of the arm.

1. Command `P<motor>:307` and watch the output shaft for 5 s.
2. If it creeps, try 305, 309, 303, 311... in steps of 2 until the shaft is still.
3. Persist it: `C<motor>:<pulse>` then `E`.
4. Repeat `T<motor>:30:500` and `T<motor>:-30:500` to confirm both directions.
5. If one direction is much faster than the other, that is normal for these units;
   keep `|offset| <= 70` near the mechanism limits.

For the shoulder pair, the slave always receives the opposite offset
(`stop - offset`). Confirm on the bench before mounting.

## 3. Positional motors: pulse range and limits

The default mapping is `110` = 0 deg and `500` = 180 deg.

1. `P<motor>:110`. If the servo buzzes against a mechanical stop, raise the value
   until it just stops buzzing (for example 118).
2. Repeat at `P<motor>:500`, lowering it until the buzz disappears.
3. `B<motor>:<min>:<max>` stores that electrical range.
4. `L<motor>:<minDeg>:<maxDeg>` stores the safe mechanical travel, for example
   `L3:15:165` for a wrist that must not fold into the forearm.
5. `O<motor>:90` sets the home angle used by `H`.
6. `E` persists everything.

From then on, `M` and `X` clamp to the limits and the angle mapping uses the
calibrated pulse range.

## 4. Double shoulder (motors 5 and 6)

The two servos face each other and drive the same shoulder. The firmware links them
(`Y6:5`) and inverts the slave (`J6` on by default), so `angle(6) = 180 - angle(5)`.

Bench test with the horns loose:

1. `M5:120`. Motor 5 rotates one way; motor 6 must rotate the opposite way **as seen
   from its own mounting face**.
2. If both rotate the same way, send `J6` (toggles invert) and repeat.
3. `M5:60` and `M5:120` again: both extremes must move together, no binding, no
   buzzing.
4. Only now bolt both horns to the printed bracket.

If the mechanics are later rebuilt with a non-mirrored layout, `Y6:255` unlinks the
pair and each motor can be commanded independently.

## 5. No-jerk power-up

The firmware boots with `OE` HIGH (outputs disabled), loads the stop pulses, and then
prints `READY`. Nothing moves until `R`.

For positional servos the stop pulse is 90 deg, so the first `R` still pulls every
joint toward 90 deg if the arm was parked elsewhere. Two safe routines:

- **Park first**: before switching the PSU off, send `H` (or `X` to 90 on every
  joint) and let the arm settle. The next power-up finds the arm already at 90.
- **Support the arm**: hold the structure while sending `R`, then move with `X` to
  the pose you want.

Never send `R` with the arm resting on a hard stop and all eight servos powered.

## 6. Verification sequence

```text
I                      -> CONECTADO_0x40
V                      -> fw 1.0.0
Q                      -> all channels at their stop pulse, estop 0
M7:100                 -> base moves, ack immediate
Q                      -> base target 100, moving=1, then settles
X4:70,5:110,6:70       -> elbow + shoulder together, all finish at once
T1:35:400              -> continuous gripper motor spins and stops
S                      -> EMERGENCY_STOP, outputs physically disabled
M7:100                 -> ERR ESTOP_LATCHED (expected)
R                      -> ENABLED
E                      -> SAVED
```

If `I` fails, stop here and check `SDA` on pin 20, `SCL` on pin 21, the common
ground and the 10 kOhm pull-up on `OE`.
