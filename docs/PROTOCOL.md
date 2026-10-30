# Protocol v1

Line-oriented ASCII over USB serial (`Serial`) or UART1 (`Serial1`, pins 18/19).
`115200 8N1`, one command per line terminated by `\n` (`\r\n` tolerated), maximum
line length 96 bytes. Replies are single-line JSON objects, no pretty printing.

- A reply goes to the port that sent the command.
- Events (`S`, `R`, `F`, watchdog) and automatic state are broadcast to both ports.
- Command letters are case-insensitive; all arguments are decimal integers.
- `<motor>` is the user motor number **1..8** (1 = PINZA ... 7 = BASE, 8 = AUX).
  The firmware converts internally with `channel = motor - 1`.

## Command reference

| Command | Arguments | Effect | Reply |
|---|---|---|---|
| `I` | - | Probe the PCA9685 on the I2C bus | `{"status":"OK","pca9685":"CONECTADO_0x40"}` or `{"status":"ERROR","i2c_code":2,"msg":"Verificar SDA->Pin20 y SCL->Pin21"}` |
| `V` | - | Firmware identity | `{"status":"VERSION","fw":"1.0.0","board":"MEGA2560_PCA9685","proto":1}` |
| `S` | - | Emergency stop: stop pulses, `OE` HIGH, latch | `{"event":"EMERGENCY_STOP","oe":"HIGH"}` |
| `R` | - | Clear the latch, `OE` LOW | `{"event":"ENABLED","oe":"LOW"}` |
| `F` | - | All channels to their stop pulse, stays enabled | `{"event":"ALL_NEUTRAL"}` |
| `M` | `<motor>:<deg>[:<ms>]` | Smooth positional move, 0..180 deg | `{"ack":"M_OK","motor":1,"deg":120}` |
| `P` | `<motor>:<pulse>` | Raw 12-bit pulse, applied immediately | `{"ack":"P_OK","motor":1,"pulse":350}` |
| `T` | `<motor>:<offset>:<ms>` | Timed continuous spin, `offset` -150..150 from the stop pulse | `{"ack":"T_OK","motor":1,"offset":40,"ms":300}` |
| `C` | `<motor>:<pulse>` | Set the stop/neutral pulse and apply it | `{"ack":"CALIBRATED","motor":1,"stop":308}` |
| `B` | `<motor>:<min>:<max>` | Pulse range for the 0..180 deg mapping | `{"ack":"RANGE","motor":1,"min":110,"max":500}` |
| `N` | `<motor>:<P\|C>` | Mode: `P` positional, `C` continuous 360 | `{"ack":"MODE","motor":1,"mode":"P"}` |
| `L` | `<motor>:<minDeg>:<maxDeg>` | Software angle limits | `{"ack":"LIMITS","motor":1,"min":10,"max":170}` |
| `Y` | `<motor>:<src>` | Mirror `<motor>` from `<src>`, `255` unlinks | `{"ack":"MIRROR","motor":6,"src":5}` |
| `J` | `<motor>` | Toggle invert for that channel | `{"ack":"INVERT","motor":6,"invert":1}` |
| `X` | `<motor>:<deg>,<motor>:<deg>,...` | Coordinated move, all axes start and finish together | `{"ack":"X_OK","n":3}` |
| `H` | - | Move every channel to its home angle | `{"ack":"HOME"}` |
| `O` | `<motor>:<deg>` | Set the home angle | `{"ack":"HOME_SET","motor":7,"deg":90}` |
| `E` | - | Persist calibration to EEPROM | `{"ack":"SAVED"}` |
| `D` | - | Restore compiled defaults in RAM (not persisted) | `{"ack":"DEFAULTS"}` |
| `W` | `<ms>` | Watchdog timeout, `0` disables | `{"ack":"WATCHDOG","ms":3000}` |
| `K` | - | Keepalive | `{"ack":"K_OK"}` |
| `Q` | - | One state frame | `{"status":"STATE",...}` |
| `A` | `<ms>` | Automatic state every `ms` (0 off, min 100) | `{"ack":"AUTO","ms":500}` |

## State frame

```json
{"status":"STATE","t":12345,"en":1,"estop":0,"ch":[
  {"c":0,"n":1,"m":"P","d":90,"p":307,"tg":90,"mv":0},
  {"c":1,"n":2,"m":"P","d":90,"p":307,"tg":90,"mv":0}
]}
```

`c` channel, `n` motor, `m` mode, `d` current angle, `p` current pulse,
`tg` target angle, `mv` moving flag (1 while ramping). All eight channels are
always present.

## Error replies

```json
{"ack":"ERR","code":"BAD_CH","cmd":"M9:90"}
```

| Code | Meaning |
|---|---|
| `BAD_CH` | Motor number outside 1..8 |
| `BAD_ARG` | Missing, non-numeric or out-of-range argument |
| `UNKNOWN_CMD` | Letter not implemented |
| `ESTOP_LATCHED` | Motion command while the emergency stop is latched; send `R` |
| `MODE_CONT_USE_T` | `M` sent to a continuous channel; use `T` instead |
| `LINE_TOO_LONG` | Command exceeded 96 bytes and was discarded |

## Behavioral rules

1. **Boot**: `OE` HIGH (outputs disabled), stop pulses loaded, then
   `{"status":"READY","board":"MEGA2560_PCA9685","fw":"1.0.0","oe":"HIGH"}`.
   Nothing moves until `R`.
2. **Ramp**: `M` and `X` run in `loop()` and never block. The ack is sent when the
   motion is accepted, not when it finishes; poll `Q` to follow progress.
   Default speed is 60 deg/s (per channel), `M<motor>:<deg>:<ms>` overrides the
   duration.
3. **Emergency stop** is immediate, including during a ramp or a `T` spin. Motion
   commands are refused until `R`.
4. **Continuous mode**: `T` applies `stop + offset` clamped to `[min,max]`, waits
   `ms`, then restores the stop pulse. `M` is rejected on a `C` channel.
5. **Mirror**: motor 6 follows motor 5 (`angle = 180 - angle` when invert is on).
   Commanding either motor of the pair drives both. `Y6:255` unlinks them,
   `J6` flips the direction.
6. **Watchdog**: with `W` > 0, any received command or `K` resets the timer. On
   timeout the firmware neutralizes, raises `OE`, latches the stop and broadcasts
   `{"event":"WATCHDOG","oe":"HIGH"}`. Recovery needs `R`.
7. **EEPROM**: `E` stores pulse range, stop pulse, limits, mode, invert, mirror
   source and home angle. A CRC failure at boot falls back to compiled defaults.

## Session examples

```text
> I
< {"status":"OK","pca9685":"CONECTADO_0x40"}
> V
< {"status":"VERSION","fw":"1.0.0","board":"MEGA2560_PCA9685","proto":1}
> R
< {"event":"ENABLED","oe":"LOW"}
> M1:40
< {"ack":"M_OK","motor":1,"deg":40}
> X4:70,5:110,6:70
< {"ack":"X_OK","n":3}
> Q
< {"status":"STATE","t":8123,"en":1,"estop":0,"ch":[...]}
> T7:30:500
< {"ack":"T_OK","motor":7,"offset":30,"ms":500}
> S
< {"event":"EMERGENCY_STOP","oe":"HIGH"}
```
