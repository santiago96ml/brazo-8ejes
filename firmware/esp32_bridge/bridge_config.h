// User-editable configuration for the ESP32 / ESP32-C3 wireless bridge.
// Copy this file, edit the values below, then flash with arduino-cli.
#pragma once

// ---- WiFi station mode (primary) -----------------------------------------
#define BRIDGE_WIFI_SSID     "YOUR_SSID"
#define BRIDGE_WIFI_PASS     "YOUR_PASSWORD"

// Optional static IP. Leave BRIDGE_USE_STATIC_IP 0 to use DHCP.
#define BRIDGE_USE_STATIC_IP 0
#define BRIDGE_STATIC_IP     "192.168.1.50"
#define BRIDGE_STATIC_GW     "192.168.1.1"
#define BRIDGE_STATIC_MASK   "255.255.255.0"
#define BRIDGE_STATIC_DNS    "8.8.8.8"

// ---- AP fallback -----------------------------------------------------------
// Used when station connect fails (20 s timeout) or BRIDGE_AP_ONLY is set.
#define BRIDGE_AP_SSID       "ARM-BRIDGE"
#define BRIDGE_AP_PASS       "arm12345"   // min 8 chars for WPA2
#define BRIDGE_AP_CHANNEL    1
#define BRIDGE_AP_HIDDEN     0

// Force AP-only mode (no station attempt). Useful for field use without a router.
// #define BRIDGE_AP_ONLY

// ---- Transport -------------------------------------------------------------
#define BRIDGE_TCP_PORT      8266
#define BRIDGE_UART_BAUD     115200

// ---- UART pins -------------------------------------------------------------
// Defaults target the ESP32-C3 SuperMini. Classic ESP32 (DevKit) users should
// set BRIDGE_UART_RX 16 and BRIDGE_UART_TX 17 (and BRIDGE_LED_PIN 2).
#define BRIDGE_UART_RX       4
#define BRIDGE_UART_TX       5

// ---- Status LED ------------------------------------------------------------
// Default 8 = ESP32-C3 SuperMini onboard LED. Classic ESP32: use 2.
#define BRIDGE_LED_PIN       8
#define BRIDGE_LED_ACTIVE_LOW 1   // C3 SuperMini LED is active low

// ---- Banner ---------------------------------------------------------------
// Plain-text banner sent on TCP connect. OFF by default because the host
// parser expects JSON lines from the Mega.
#define BRIDGE_BANNER        0
#define BRIDGE_BANNER_TEXT   "ARM-BRIDGE ready\r\n"

// ---- Pump tuning -----------------------------------------------------------
#define BRIDGE_CHUNK_MAX     256   // max bytes forwarded per loop iteration
#define BRIDGE_WIFI_TIMEOUT_MS 20000
#define BRIDGE_WIFI_RETRY_MS   10000