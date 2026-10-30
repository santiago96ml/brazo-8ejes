# Wiring

Arduino Mega 2560 R3 + PCA9685 (16-channel, 12-bit) + 8x TowerPro MG946R.

## 1. Control bus (Mega <-> PCA9685)

| PCA9685 | Arduino Mega 2560 | Notes |
|---|---|---|
| GND | GND | Same ground as the 5 V servo supply |
| OE / ED | Digital pin 2 | Active LOW. HIGH disables all outputs |
| SCL | Pin 21 (SCL) | Hardware I2C on the Mega, **not** A5 |
| SDA | Pin 20 (SDA) | Hardware I2C on the Mega, **not** A4 |
| VCC | 5V | Logic supply only |
| V+ (terminal block) | +5 V servo supply | Servo rail only |

The Mega 2560 does not route I2C to A4/A5 or to pins 3/4. Pins 20/21 are the only
hardware I2C pins and they already carry the required pull-ups on the PCA9685 board.

Add a 10 kOhm resistor from `OE` to `VCC`. During a Mega reset, upload or power-up
the pin floats, and the PCA9685 has no guaranteed internal pull-up on `OE`; the
resistor keeps the outputs disabled until the firmware takes control.

## 2. Power topology

```text
 5 V / 27 A PSU
   +V ----[ 20 A fuse ]----[ master switch ]----+---- PCA9685 V+ terminal
                                                 |
                                                 +---- 1000-2200 uF electrolytic
                                                 |     (plus 100 nF ceramic)
   -V -------------------------------------------+---- PCA9685 GND terminal
                                                 |
                                                 +---- Mega GND
                                                 +---- PCA9685 side GND pin

 USB 5 V (PC) ---- Mega USB port        (Mega logic only)
 Mega 5V pin  ---- PCA9685 VCC          (logic only, never the servo rail)
```

Rules:

- The 5 V 27 A positive **never** connects to the Mega 5V pin, VIN, or USB.
- All grounds are tied together at one star point at the PSU negative terminal.
- The PCA9685 `V+` side pin is left unconnected; power enters through the screw
  terminal only.
- The PCA9685 terminal block and its PCB traces are not rated for 27 A. Feed the
  servo rail from a distribution block with 16 AWG wire, or split the servos into
  two banks of four, each fed from the block. Use the PCA9685 only for the PWM
  signals and a short common return.
- A 20 A automotive fuse on the 5 V rail protects the wiring if a servo stalls.
  Size it below the PSU rating and above the sum of the normal holding currents.
- At 5 V an MG946R delivers less than its 6 V rating. Undervoltage during a
  simultaneous move shows up as jitter, buzzing or a Mega reset. Measure the rail
  at the servos while all eight move; if it sags below ~4.6 V, shorten the wiring,
  raise the wire gauge, or move to a 6 V supply rated for the same current.

## 3. Servo channels

| Motor (user numbering) | PCA9685 channel | Joint | Function |
|---|---|---|---|
| 1 | 0 | PINZA | Gripper open/close |
| 2 | 1 | GIRO_PINZA | Wrist roll |
| 3 | 2 | INCLINA_PINZA | Wrist pitch |
| 4 | 3 | CODO | Elbow |
| 5 | 4 | HOMBRO_IZQ | Shoulder master |
| 6 | 5 | HOMBRO_DER | Shoulder slave, mirrored |
| 7 | 6 | BASE | Waist yaw |
| 8 | 7 | AUX | Spare |

Motors 5 and 6 drive the same shoulder from opposite sides. They must always turn in
opposite directions relative to their own mount, otherwise the printed bracket
binds. The firmware keeps `angle(6) = 180 - angle(5)` when the link is enabled
(`Y6:5`, invert on with `J6`). Verify the direction with the couplers loose before
you bolt both horns to the structure.

## 4. Optional ESP32 / ESP32-C3 bridge

| ESP32-C3 SuperMini | Mega 2560 | Notes |
|---|---|---|
| RX (GPIO 4 by default) | TX1, pin 18 | Mega output is 5 V: use a 1k/2k divider or a level shifter |
| TX (GPIO 5 by default) | RX1, pin 19 | 3.3 V is usually read as HIGH by the Mega, a shifter is safer |
| GND | GND | Common ground |
| 5V / VBUS | - | Power the ESP32 from its own USB or a separate 3.3 V source |

Classic ESP32 boards usually use RX 16 / TX 17; set the pins in
`firmware/esp32_bridge/bridge_config.h`.

## 5. Pre-flight checklist

1. PSU unplugged, all eight servos disconnected.
2. Continuity: PSU negative, PCA9685 GND pin, PCA9685 terminal GND and Mega GND are one node.
3. Continuity: PSU positive reaches only the PCA9685 terminal `V+` (through the fuse), never the Mega.
4. 10 kOhm from OE to VCC present.
5. I2C on pins 20/21, OE on pin 2.
6. First power-up with the arm unloaded. The firmware boots with the outputs
   disabled and prints `{"status":"READY",...}`; nothing should move.
7. Send `I` and confirm `CONECTADO_0x40` before sending any move.

## 6. Overheating diagnosis (do this before powering again)

A cable that gets hot is carrying more current than its gauge allows, or the current
is returning through a path that was never sized for it. `P = I^2 * R`: **raising the
supply current limit never makes a cable cooler**, it only allows more current (and
more heat) before the supply gives up. A bench supply's current limit is a ceiling,
not a thermostat.

Ranked causes seen in this build:

1. **Servo current returning through the logic ground.** The PCA9685 side `GND` pin
   and the thin Dupont wires are for logic only (a few mA). The seven servos must
   return through the **screw terminal `GND`** with the same gauge as `V+`. If the
   servo return goes through a Dupont wire, that wire becomes the bottleneck and
   overheats while the motor still works.
2. **Wire gauge too small for the rail.** 7x MG946R at 5 V: ~0.2 A each idle,
   ~0.6-1.0 A each moving, **~2.4 A each stalled**. Seven stalled = ~17 A. Use
   16-18 AWG from the supply to a distribution block, 20-22 AWG per servo pigtail.
3. **Supply far below the load.** A 5 V / 2.7 A supply cannot move seven servos: it
   enters current limit, the rail collapses, every servo stalls, and the wires cook
   at the limit. At 2.5 A the arm "did not move" for exactly this reason. That is not
   a tuning problem, it is a sizing problem.

### Checks with the supply OFF

1. Disconnect the servos. Measure `V+` to `GND` at the PCA9685 terminal: it must read
   open (no continuity). Any continuity is a short - find it before powering.
2. Check polarity: supply `+` to terminal `V+`, supply `-` to terminal `GND`.
3. Identify every wire that got hot and trace where it goes. If it is a Dupont wire
   between the Mega/PCA9685 side header and the supply, it is a return-path error,
   not a load error.
4. Verify the star ground: PSU `-`, terminal `GND`, side `GND`, Mega `GND` are one
   node, and no servo current crosses the Mega board.
5. Only then connect **one** servo and test at 5.0 V with the current limit at 1 A.
   A single MG946R moving freely stays under ~0.8 A. If it hits the limit while
   moving freely, the servo or its wiring is faulty.

### Required supply

- 5 V, **>= 10 A continuous** for normal motion, 15-20 A peak (7x MG946R).
  A 5 V / 2.7 A unit is for one or two servos on the bench, never for the arm.
- 20 A fuse on the rail, 16 AWG main run, distribution block, 1000-2200 uF bulk
  capacitor at the rail.
- Never raise the current limit to force movement. Fix the cause.

### Servo plug orientation (channel 5 melting case)

A servo signal pin is high impedance: it cannot carry amps. If the **signal** wire of a
servo connector heats up, current is flowing where it should not. The two realistic
causes are:

1. **Reversed or shifted plug.** Rotating a 3-pin shell 180 degrees swaps the two outer
   pins (signal <-> GND) while V+ stays in the middle. The servo then returns through
   the PWM pin, which on most PCA9685 boards sits behind a 220 ohm series resistor;
   the resistor and the wire cook while the motor never turns properly. A plug shifted
   by one pin does the same across two adjacent channels.
2. **Damaged servo or lead.** A crushed cable or an internally shorted MG946R draws
   current between GND and signal, heating exactly those two wires and leaving V+
   comparatively cool.

Isolation procedure, one servo at a time, supply OFF between steps:

1. Compare wire colours and shell orientation of the suspect servo against a working
   one. Standard MG946R: brown = GND, red = V+, orange = signal. Clone batches
   sometimes ship black/red/white with a different physical order.
2. With the servo disconnected, measure the PCA9685 channel: `PWM-GND`, `PWM-V+`,
   `V+-GND` in resistance mode and compare with a known-good channel. A short means
   the board trace or the 220 ohm resistor is damaged; that channel is dead.
3. Test the suspect servo alone on a current-limited supply (1 A) with only GND and
   V+ connected, no signal. A healthy servo stays still and draws almost nothing.
   If it draws current or twitches by itself, the servo is internally shorted.
4. Never reuse a heat-damaged lead: the insulation is compromised and the copper is
   annealed. Replace the pigtail.
5. Only after the channel and the servo pass, reconnect one servo at a time and watch
   the current on the supply.
