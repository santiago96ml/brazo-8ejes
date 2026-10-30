# Feature: 8-axis 3D robotic arm control (Mega 2560 + PCA9685 + 8x MG946R)

Branch `feat/arm-8axis` · Delivery: local feature branch, work-unit commits (`exception-ok`: no git remote configured, PR chain not applicable) · Test runner: `python -m unittest discover -s host/tests -v` + `arduino-cli compile --fqbn arduino:avr:mega firmware/mega2560_arm`.

## Specs

S1 — Firmware without external libraries. The sketch must build for `arduino:avr:mega` using only `<Wire.h>` plus the AVR core; no `Adafruit_PWMServoDriver.h`. The PCA9685 (`0x40`) is driven through its own registers: `MODE1 0x00`, `PRESCALE 0xFE`, `LED0_ON_L 0x06`.

S2 — Dual serial input. Commands are accepted on `Serial` (USB, `115200`) and on `Serial1` (pins `18 TX1` / `19 RX1`, `115200`) so an ESP32 can act as a wireless bridge. A reply goes to the port that sent the command; unsolicited events go to both.

S3 — Emergency stop is physical. `PIN_OE = 2`; `HIGH` disables the PCA9685 outputs and is the boot state. The board must boot with OE `HIGH`, then print `{"status":"READY","board":"MEGA2560_PCA9685"}`.

S4 — Required command set (protocol v1, preserved verbatim; `<motor>` is the user motor number 1..8, `<ch>` where shown is the PCA9685 channel):
- `I` — I2C diagnostic. On success prints `{"status":"OK","pca9685":"CONECTADO_0x40"}`; on failure prints an error with the Wire code and the hint `Verificar SDA->Pin20 y SCL->Pin21`.
- `S` — emergency stop: neutral pulses + `{"event":"EMERGENCY_STOP","oe":"HIGH"}`, latched until `R`.
- `R` — enable: `{"event":"ENABLED","oe":"LOW"}`.
- `F` — all neutral `{"event":"ALL_NEUTRAL"}`.
- `M<motor>:<deg>` — angle move with smooth ramp, `{"ack":"M_OK","motor":<n>,"deg":<deg>}`.
- `P<motor>:<pulse>` — raw 12-bit pulse, `{"ack":"P_OK"}`.
- `T<motor>:<offset>:<ms>` — timed continuous rotation: `offset` is a signed velocity offset from the stop pulse, verbatim "`<velocidad_-150_a_150>`", applied for `ms`, then back to the stop pulse, `{"ack":"T_OK"}`. Absolute pulse positioning uses `P`, never `T`.
- `C<motor>:<pulse>` — calibrate the stop/neutral pulse, `{"ack":"CALIBRATED"}`.

S5 — Official mapping, user numbering 1..7 from the end effector to the base (supersedes the first channel map). Commands address the MOTOR number, the firmware converts internally (`channel = motor - 1`):
- Motor 1 -> channel 0 `PINZA` (gripper)
- Motor 2 -> channel 1 `GIRO_PINZA` (wrist roll)
- Motor 3 -> channel 2 `INCLINA_PINZA` (wrist pitch)
- Motor 4 -> channel 3 `CODO` (elbow)
- Motor 5 -> channel 4 `HOMBRO_IZQ` (left shoulder, master)
- Motor 6 -> channel 5 `HOMBRO_DER` (right shoulder, mirrored slave)
- Motor 7 -> channel 6 `BASE` (waist yaw)
- Motor 8 -> channel 7 `AUX` (spare)

S5b — Double shoulder mechanical rule, verbatim: "Nunca debe moverse en el mismo sentido que el Motor 5; siempre trabaja acoplado en espejo (`Angulo_M6 = 180 - Angulo_M5`, o velocidad invertida respecto a `servoStop` si opera en modo 360° continuo)." Moving either motor 5 or 6 moves both, synchronized; direct commands to 6 are accepted but immediately reconciled. `Y<6>:255` keeps the pair linked only when the installer verifies the opposite mounting (`J6` toggles invert if the two servos face the same way).

S6 — 360° and 180° support in the same firmware. Neutral/stop is `307` at 50 Hz; positional travel is `110` (~540 µs, 0°) to `500` (~2440 µs, 180°). Mode `P` = positional (angle), mode `C` = continuous (pulse/time). An `M` command on a `C` channel must be rejected with an error that points to `T`.

S7 — PCA9685 timing: 50 Hz (`PRESCALE 121`), 12-bit, `MODE1` auto-increment `0xA0`, 16 channels available, 8 used.

S8 — Non-blocking motion. Motion is advanced from `loop()`; no `delay()` of user-controlled length inside a command. `S` must take effect immediately even while a long move or a `T` rotation is in progress. `S` while latched only re-acknowledges.

S9 — Safety and calibration: angular limits per channel, watchdog (`W<ms>`, `K` keepalive) that neutralizes and disables on timeout, EEPROM persistence (`E` save, `D` defaults), mirror/slave link for `HOMBRO_DER` motor 6 from `HOMBRO_IZQ` motor 5 (`Y<ch>:<src>`, default `Y6:5`) with invert (`J<ch>`, default enabled), so the paired shoulder tracks the master automatically.

S10 — Host controller in Python 3 with `pyserial`, real-time over USB serial: joint names from config, `status`, `move`, `pulse`, `spin`, `estop`, `enable`, `neutral`, `home`, `pose`, `calibrate`, `monitor`, `repl`, `script`, `ports`, `scan`, `doctor`. Must also work against a simulated device (`--sim`) with no hardware, and support a TCP transport for the ESP32 bridge.

S11 — ESP32/ESP32-C3 bridge firmware: transparent TCP server that forwards bytes to `Serial1` at `115200`, so the host controller can reach the Mega wirelessly with the same protocol.

S12 — Tooling: one script compiles and uploads with `arduino-cli` (auto-detected from the Arduino IDE install or `PATH`), default port `COM4`, FQBN `arduino:avr:mega`.

## Tasks

| ID | Specs | Route | Status | Commit |
|----|-------|-------|--------|--------|
| T1 | S1-S9 | delegated (firmware writer) | todo | — |
| T2 | S10 | delegated (host writer) | todo | — |
| T3 | S11 | delegated (bridge writer) | todo | — |
| T4 | S12 + docs | inline (parent) | todo | — |
| T5 | all | inline + independent verifier (firmware high risk) | todo | — |

## Log

L1 — User brief (condensed; the verbatim requirement fragments live in `## Specs`): 8-axis 3D-printed arm, Arduino Mega 2560 R3, PCA9685 at 50 Hz over hardware I2C (`SDA 20`, `SCL 21`), 8x MG946R, 5 V/27 A servo rail isolated from the Mega, OE on pin 2, commands `M` / `P` / `T` / `C` / `I` / `S` / `R` / `F`, dual `Serial` + `Serial1`, host controller over USB, ESP32 bridge prepared.

L2 — 2026-10-29: Workspace `Proyecto predeterminado` cannot create files or directories from the shell (ACL deny). User chose `C:\Users\merca\Documentos\brazo-8ejes`. Session moved there.

L3 — 2026-10-29: Toolchain detected: arduino-cli 1.5.1 (bundled with Arduino IDE 2), `arduino:avr` 1.8.8, `esp32:esp32` 3.3.12, Python 3.14.3 with pyserial 3.5, CH340 on `COM4`.

L4 — 2026-10-29: Firmware boots with OE disabled (S3) instead of enabling at the end of `setup()` as in the user's base sketch: sending 90° to eight positional servos at power-up is a mechanical risk. The host sends `R` on connect, so `README`/protocol docs and the CLI document this.

L5 — 2026-10-29 (supersedes S5, changes S4/S5b/S6/S9): user supplied the real harness order, motors 1..7 from the gripper to the base, plus motor 8 `AUX` on channel 7. Verbatim rules: "El usuario numera los motores del 1 al 7 comenzando desde el extremo efector (la pinza) hasta la base fija"; "Motor 6 (Canal 5 PCA9685) - HOMBRO_DER (Shoulder Right - Esclavo Invertido)"; "Nunca debe moverse en el mismo sentido que el Motor 5; siempre trabaja acoplado en espejo (Angulo_M6 = 180 - Angulo_M5, o velocidad invertida respecto a servoStop si opera en modo 360° continuo)"; "`T<motor_1_a_7>:<velocidad_-150_a_150>:<ms>` -> Modo 360° continuo seguro". Consequence: commands address motor numbers, `T` is a signed velocity offset, and the shoulder pair is 5/6 (channels 4/5).
