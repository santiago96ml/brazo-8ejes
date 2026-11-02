# 8-axis 3D printed robotic arm

Firmware and host tooling for an Arduino Mega 2560 driving a PCA9685 and eight
TowerPro MG946R servos over USB serial, with an optional ESP32/ESP32-C3 WiFi bridge.

- Firmware: no external libraries, raw `Wire.h` register access to the PCA9685.
- Motion: non-blocking ramps, emergency stop through the `OE` pin, per-joint limits,
  watchdog, EEPROM calibration.
- Host: Python 3 CLI (`armctl`) with real-time telemetry, poses, scripts, a REPL and
  a simulated device for offline work.
- Servo modes: positional 0-180 deg and continuous 360 deg on a per-channel basis.

## Repository layout

```text
firmware_mega/            Official canonical firmware (v2.1.0) — flashed to COM4
firmware/mega2560_arm/    Alternative sketch (motor numbering 1..7 + AUX)
firmware/esp32_bridge/    WiFi <-> UART1 transparent bridge
host/armctl/              Python controller and CLI
host/tests/               unittest suite, runs without hardware
config/                   joints, pulse ranges, poses
examples/                 script sequences
tools/arduino.ps1         build / upload / doctor wrapper around arduino-cli
tools/armctl.ps1          runs the CLI with the right PYTHONPATH
docs/                     wiring, protocol, calibration
odd/tasks/                feature specification and decision log
```

**Note:** `firmware_mega/firmware_mega.ino` (v2.1.0) is the **official canonical firmware binary** flashed to `COM4`. The `firmware/mega2560_arm/` variant uses a different modular structure and may not implement all commands (`Z`, `G`, `U`, etc.) supported by the canonical firmware. Both share the core protocol contract (`I`, `S`, `R`, `Q`, `Z`, `G`, `T`, `C`, `U`, `J6`, `E`, `V`, `F`).

## Quick start

1. Wire everything per [docs/WIRING.md](docs/WIRING.md). The 5 V/27 A servo supply
   never touches the Mega 5V pin.
2. Build the firmware:

   ```powershell
   powershell -ExecutionPolicy Bypass -File tools\arduino.ps1 build
   ```

3. Upload to the board on `COM4`:

   ```powershell
   powershell -ExecutionPolicy Bypass -File tools\arduino.ps1 upload -Port COM4
   ```

   ``arduino.ps1 doctor`` prints the detected CLI, cores, boards and serial ports.

4. Install the host dependency:

   ```powershell
   python -m pip install -r host\requirements.txt
   ```

5. Talk to the arm:

   ```powershell
   tools\armctl.ps1 doctor
   tools\armctl.ps1 status
   tools\armctl.ps1 move base 100
   tools\armctl.ps1 monitor
   tools\armctl.ps1 repl
   ```

   Without hardware, add `--sim` to any command.

## Motor map

The user numbers the motors from the end effector to the base; commands use that
number and the firmware maps it to `channel = motor - 1`.

| Motor | Channel | Joint | Notes |
|---|---|---|---|
| 1 | 0 | PINZA | Gripper |
| 2 | 1 | GIRO_PINZA | Wrist roll |
| 3 | 2 | INCLINA_PINZA | Wrist pitch |
| 4 | 3 | CODO | Elbow |
| 5 | 4 | HOMBRO_IZQ | Shoulder master |
| 6 | 5 | HOMBRO_DER | Shoulder slave, `180 - angle(5)` |
| 7 | 6 | BASE | Waist yaw |
| 8 | 7 | AUX | Spare |

## Safety model

- The board boots with the PCA9685 outputs **disabled** (`OE` HIGH) and prints
  `READY`; the host sends `R` when it connects. Nothing moves at power-up.
- `S` (or `armctl estop`) drives every channel to its stop pulse and raises `OE`,
  which physically removes the PWM. Motion commands are rejected until `R`.
- `W<ms>` enables a watchdog; the host sends `K` keepalives. On timeout the arm
  neutralizes and disables itself.
- Put a 20 A fuse on the 5 V rail and a 10 kOhm pull-up from `OE` to `VCC`.
- Test new mechanisms with the servo horns loose. See [docs/CALIBRATION.md](docs/CALIBRATION.md).

## Host CLI

```text
armctl ports | scan | doctor | status | monitor
armctl move <joint> <deg> [--ms N] [--wait]
armctl pulse <joint> <pulse>
armctl spin <joint> <offset> <ms>        # continuous mode, offset -150..150
armctl estop | enable | neutral | home
armctl pose save|show|goto <name>
armctl mode <joint> P|C
armctl limits <joint> <min> <max> [--save]
armctl calibrate <joint> --stop <pulse> [--min <pulse>] [--max <pulse>] [--save]
armctl repl
armctl script <file>
armctl raw "<protocol line>"
```

Global flags: `--port COM4`, `--baud 115200`, `--tcp HOST:PORT`, `--sim`,
`--config config/arm.json`, `--json`.

On exit the controller leaves the arm enabled and holding by default; the firmware
watchdog (`watchdog_ms` from config, keepalive thread) neutralizes and disables the
board if the process dies. Pass `--disable` to send `F` then `S` on exit.

## Continuous 360 servos

A continuous servo ignores the angle mapping; `M` is rejected. Use a signed offset
from the calibrated stop pulse:

```powershell
tools\armctl.ps1 spin gripper 40 600     # spin one way for 600 ms, then stop
tools\armctl.ps1 spin gripper -40 600
tools\armctl.ps1 pulse gripper 307       # neutral, hold still
```

Find the real stop point (usually 303..311) with [docs/CALIBRATION.md](docs/CALIBRATION.md).

## ESP32 bridge (optional)

Flash `firmware/esp32_bridge` to an ESP32-C3 (or classic ESP32), set the WiFi
credentials in `bridge_config.h`, wire the UART to the Mega pins 18/19 through a
level shifter, then:

```powershell
tools\armctl.ps1 --tcp 192.168.1.50:8266 status
```

The bridge is byte-transparent: same protocol, no changes on the host side.

## Troubleshooting

| Symptom | Check |
|---|---|
| `I` returns an I2C error | `SDA` pin 20, `SCL` pin 21, common ground, 3.3/5 V on `VCC` |
| Board not detected | CH340 driver, another program holding `COM4`, try `arduino.ps1 doctor` |
| Upload fails | Close the serial monitor, confirm `arduino:avr` core is installed |
| All servos buzz at rest | servo rail sag, missing common ground, or stop pulse off by a few counts |
| One servo creeps | calibrate its stop pulse (`C<motor>:<pulse>`) and `E` |
| Shoulder binds | `J6` direction, then recalibrate limits |
| Random resets while moving | 5 V rail collapse: thicker wiring, shorter runs, 6 V supply, fuse size |

## Documentation

- [docs/WIRING.md](docs/WIRING.md) - pinout, power topology, ESP32 UART levels
- [docs/PROTOCOL.md](docs/PROTOCOL.md) - full serial command reference
- [docs/CALIBRATION.md](docs/CALIBRATION.md) - mode detection, stop points, limits, shoulder pairing
- [docs/MECHANICS.md](docs/MECHANICS.md) - source model, original wiring and what changed here
- [odd/tasks/8axis-arm.md](odd/tasks/8axis-arm.md) - specification, tasks and decision log
