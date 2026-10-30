# Mechanical source and mapping notes

The printed structure is **"Roboticarm V1 - Construction Info"** by Juergenlessner
(Thingiverse thing 851345, 2015, CC BY-NC-SA). It is a 6-joint / 7-servo arm: the
shoulder is driven by two servos facing each other, one on each side of the joint.

## What the original design specifies

| Item | Original |
|---|---|
| Servos | 7x TowerPro MG995 |
| Controller | Arduino Mega 2560 |
| PWM driver | PCA9685 over I2C |
| End effector | Robot claw |
| Bearings | 8x 623ZZ |
| Supply | 7.2 V / 5 A |
| Extras | LCD2004, 19-button keypad on a resistor ladder |

Original channel use:

| Channel | Joint |
|---|---|
| 0 | Base turn |
| 1 | Shoulder, **two servos paralleled on one channel** |
| 2 | Elbow |
| 3 | Wrist pitch |
| 4 | Wrist roll |
| 5 | Gripper |

Original pinout (same physical pins as this build): `SCL -> A21`, `SDA -> A20`,
`VCC -> +5V`, `GND -> GND`.

## What this build changes

1. **The two shoulder servos get their own channels** (4 = HOMBRO_IZQ, 5 = HOMBRO_DER)
   instead of sharing one PWM line. Same mechanical result, but each servo keeps its
   own trim, stop point and limits, and neither channel carries twice the current.
   The firmware links them (`Y6:5`, invert on) so the pair still moves as one joint.
2. **Motor numbering follows the harness**, 1 = gripper at the end effector to
   7 = base, with 8 = AUX on the spare channel.
3. **No LCD or 19-button keypad.** The PC host (`armctl`) replaces the local panel;
   the same buttons could be added later on the free Mega pins.
4. **Power**: 5 V / 27 A instead of 7.2 V / 5 A. The current headroom is far better,
   but an MG946R at 5 V delivers less torque than at 6 V. If the arm struggles at
   full extension, a 6 V supply of the same current is the upgrade; never exceed the
   servo rating.

## Wiring difference that matters

In the original, both shoulder servos receive the same pulse, which only works if
they are mounted so that "the same pulse" means "the same physical direction". In
this build each one is calibrated separately, and the mirror rule
`angle(6) = 180 - angle(5)` is applied in firmware. Verify the direction with the
horns loose before bolting the bracket: if motor 6 turns the same way as motor 5,
send `J6` to flip it.

## License

The printed parts are CC BY-NC-SA: attribution required, non-commercial use, and
derivative models shared under the same license. The firmware and host code in this
repository are original work.
