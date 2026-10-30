# 8-axis robotic arm firmware (Arduino Mega 2560 + PCA9685)

Firmware for the 8-axis 3D-printed arm. Drives 8 MG946R servos through a PCA9685
(0x40, 50 Hz) over hardware I2C (SDA=pin20, SCL=pin21). Only `<Wire.h>` and the
AVR core are used.

## Build
```
arduino-cli compile --fqbn arduino:avr:mega firmware/mega2560_arm
```

## Boot
OE (pin 2) is HIGH at boot (outputs disabled). All channels go to their stop pulse.
Send `R` to enable. Host controller sends `R` on connect.

## Motor map (user motor number -> PCA9685 channel)

| Motor | Channel | Name |
|-------|---------|------|
| 1 | 0 | PINZA (gripper) |
| 2 | 1 | GIRO_PINZA (wrist roll) |
| 3 | 2 | INCLINA_PINZA (wrist pitch) |
| 4 | 3 | CODO (elbow) |
| 5 | 4 | HOMBRO_IZQ (left shoulder, master) |
| 6 | 5 | HOMBRO_DER (right shoulder, mirrored slave) |
| 7 | 6 | BASE (waist yaw) |
| 8 | 7 | AUX (spare) |

Commands address the motor number 1..8; firmware converts `channel = motor - 1`.
Valid range 1..8, otherwise `BAD_CH`.

## Shoulder pair (motors 5 and 6)
Motor 6 mirrors motor 5: `angle_M6 = 180 - angle_M5`, or inverted velocity in
continuous mode. Moving either moves both, synchronized. `Y6:5` links them (default),
`J6` toggles invert if the servos face the same way.

## T semantics
`T<motor>:<offset>:<ms>` — signed velocity offset (-150..150) from the stop pulse:
`pulse = constrain(stopPulse + offset, minPulse, maxPulse)`. Absolute pulses use `P`.
On the shoulder pair, the opposite offset is applied to the partner.

## Protocol v1 (examples)
- `I` -> `{"status":"OK","pca9685":"CONECTADO_0x40"}`
- `V` -> `{"status":"VERSION","fw":"1.0.0","board":"MEGA2560_PCA9685","proto":1}`
- `S` -> `{"event":"EMERGENCY_STOP","oe":"HIGH"}`
- `R` -> `{"event":"ENABLED","oe":"LOW"}`
- `F` -> `{"event":"ALL_NEUTRAL"}`
- `M5:90` -> `{"ack":"M_OK","motor":5,"deg":90}`
- `P1:307` -> `{"ack":"P_OK","motor":1,"pulse":307}`
- `T1:40:300` -> `{"ack":"T_OK","motor":1,"offset":40,"ms":300}`
- `C1:307` -> `{"ack":"CALIBRATED","motor":1,"stop":307}`
- `B1:110:500` -> `{"ack":"RANGE","motor":1,"min":110,"max":500}`
- `N1:P` -> `{"ack":"MODE","motor":1,"mode":"P"}`
- `L1:0:180` -> `{"ack":"LIMITS","motor":1,"min":0,"max":180}`
- `Y6:5` -> `{"ack":"MIRROR","motor":6,"src":5}`
- `J6` -> `{"ack":"INVERT","motor":6,"invert":1}`
- `O5:90` -> `{"ack":"HOME_SET","motor":5,"deg":90}`
- `H` -> `{"ack":"HOME"}`
- `X5:90,4:90` -> `{"ack":"X_OK","n":2}`
- `E` -> `{"ack":"SAVED"}`, `D` -> `{"ack":"DEFAULTS"}`
- `W500` -> `{"ack":"WATCHDOG","ms":500}`, `K` -> `{"ack":"K_OK"}`
- `A200` -> `{"ack":"AUTO","ms":200}`, `Q` -> state frame

## EEPROM layout
Magic `ARM1` (4 bytes) + version (1, currently 2) + 8 x `ChannelCal` (packed) + CRC16-CCITT (2).
`ChannelCal`: mode u8, stopPulse u16, minPulse u16, maxPulse u16, minDeg i16, maxDeg i16,
homeDeg u16, invert u8, mirrorSrc u8, speedDps f32.
Total per record 18 bytes.
