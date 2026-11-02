# 6-DOF Button Panel Firmware

This sketch is a separate Arduino Mega 2560 firmware for the physical 12-button panel. It drives seven MG946R servos through a PCA9685 at address `0x40` and 50 Hz. It does not change or replace the serial/kinematics firmware in `firmware_mega/`; that firmware has a separate protocol and remains the canonical controller.

## Hardware map

- PCA9685 `OE` -> Mega pin 2 (active LOW; HIGH disables PWM outputs).
- PCA9685 I2C -> Mega SDA pin 20 and SCL pin 21; address `0x40`.
- Servo channels: `0` PINZA, `1` GIRO_PINZA, `2` MUNECA_PITCH, `3` CODO, `4` HOMBRO_IZQ, `5` HOMBRO_DER (mirrored/coupled), `6` BASE.
- Buttons use `INPUT_PULLUP`; each pair connects its input to GND when pressed.

| Mega pins | Axis | First button | Second button |
|---|---|---|---|
| 22 / 23 | Pinza (ch 0) | Open | Close |
| 24 / 25 | Muñeca pitch (ch 2) | Up | Down |
| 26 / 27 | Base (ch 6) | Left | Right |
| 28 / 29 | Shoulder (ch 4 and 5) | Up | Down |
| 30 / 31 | Elbow (ch 3) | Up | Down |
| 32 / 33 | Gripper rotor (ch 1) | Left | Right |

The firmware requires two matching 10 ms samples to accept a button state, then two accepted samples holding a direction before moving. It ignores buttons for 500 ms after the outputs are enabled. Releasing a button writes that channel's exact `p_stop` value.

## Serial commands

Use `115200` baud; commands end with a newline.

| Command | Behavior |
|---|---|
| `C<ch>:<pulse>` | Set channel stop pulse in RAM (channel `0..6`, pulse `290..325`), and apply it immediately. Example: `C6:306`. |
| `E` | Persist all seven stop pulses to EEPROM with version and CRC16-CCITT. |
| `D` | Restore all stop pulses to `307` in RAM; does not write EEPROM. |
| `Q` | Print JSON state with OE, all stop pulses, and six accepted axis states. |
| `S` | Write stop pulses and disable PCA outputs (OE HIGH). |
| `R` | Write stop pulses and enable outputs (OE LOW); restarts the 500 ms input guard. |
| `I` | Probe the PCA9685 at `0x40`. |
| `V` | Print firmware version. |

To calibrate a channel, adjust its stop pulse until the servo is stationary, for example `C6:306`, then send `E` to save all channels. EEPROM data is accepted only when its magic header, version, checksum, and all pulse ranges are valid; otherwise every channel defaults to `307`.

## Build and upload

Compile for the Mega 2560:

```powershell
& 'C:\Users\merca\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn arduino:avr:mega --warnings all firmware_button_panel
```

Flash the panel firmware to the connected board on `COM11`:

```powershell
& 'C:\Users\merca\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' upload -p COM11 --fqbn arduino:avr:mega firmware_button_panel
```

The canonical protocol firmware stays in `firmware_mega/` because it is used by the kinematics/serial controller; this panel sketch has distinct button handling and must not replace that protocol.
