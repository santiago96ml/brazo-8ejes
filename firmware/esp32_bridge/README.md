# ESP32 / ESP32-C3 wireless bridge for the 8-axis arm

Transparent TCP <-> UART bridge. The Mega 2560 exposes the same ASCII/JSON
command protocol on `Serial1` (pins 18 TX1 / 19 RX1) at 115200. This
firmware connects an ESP32 (or ESP32-C3 SuperMini) to that UART and offers a
TCP server so the Python host can send the very same lines over WiFi.

The bridge is byte-transparent: no parsing, no line rewriting, no added
framing, no JSON awareness. Whatever arrives on TCP goes to UART and vice
versa.

## Wiring

```
Mega 2560 (5 V)                           ESP32 / ESP32-C3 (3.3 V)
----------------                           -------------------------
Pin 18 (TX1)  ----[divider/level shifter]---->  RX (default GPIO 4)
Pin 19 (RX1)  <---- direct 3.3 V logic -------  TX (default GPIO 5)
GND           ------------------------------->  GND (common)
```

**Voltage warning:** the Mega is 5 V and the ESP32 is 3.3 V. A divider or
level shifter is REQUIRED on the Mega TX -> ESP32 RX line. The ESP32 TX ->
Mega RX line can be connected directly (3.3 V is read as HIGH by the Mega).

Default pins target the ESP32-C3 SuperMini (RX 4, TX 5, LED 8). Classic
ESP32 (DevKit) users should set in `bridge_config.h`:
`BRIDGE_UART_RX 16`, `BRIDGE_UART_TX 17`, `BRIDGE_LED_PIN 2`.

## Flashing

1. Install the ESP32 Arduino core (3.3.12 tested).
2. Edit `bridge_config.h` (SSID, password, pins).
3. Compile:

   ```
   arduino-cli compile --fqbn esp32:esp32:esp32c3 firmware/esp32_bridge
   ```

   (Classic ESP32: `arduino:esp32:esp32`.)
4. Upload:

   ```
   arduino-cli upload -p COMx --fqbn esp32:esp32:esp32c3 firmware/esp32_bridge
   ```

The IP is printed on the USB serial monitor at 115200.

## Using from the host

```
python -m arm.cli --tcp 192.168.1.50:8266
```

(Replace with the IP printed by the bridge.) The host speaks the identical
protocol; only the transport changes.

## LED legend

- Solid ON: a TCP client is connected.
- Slow blink: WiFi up, no client.
- Fast blink: AP mode (fallback or `BRIDGE_AP_ONLY`).

## Troubleshooting

- No WiFi connect: check SSID/password; after 20 s the bridge falls back to
  AP `ARM-BRIDGE` / `arm12345` (IP 192.168.4.1).
- Garbled replies: baud mismatch; both sides must be 115200.
- Mega not responding: verify TX1/RX1 wiring and common GND; check level
  shifter on Mega TX -> ESP32 RX.
- `BRIDGE_AP_ONLY`: define it in `bridge_config.h` to skip station mode.