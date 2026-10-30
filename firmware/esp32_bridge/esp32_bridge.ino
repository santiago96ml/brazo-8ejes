// Transparent TCP <-> UART bridge for the 8-axis arm Mega 2560.
// Forwards bytes verbatim between a single TCP client and Serial1 (115200).
// No parsing, no line rewriting, no framing, no JSON awareness.

#include "bridge_config.h"
#include <WiFi.h>

WiFiServer tcpServer(BRIDGE_TCP_PORT);
WiFiClient tcpClient;
bool apMode = false;
bool wifiConnected = false;
unsigned long lastWifiRetry = 0;
unsigned long lastBlink = 0;
bool blinkState = false;
bool clientWasConnected = false;

// ---- LED helpers -----------------------------------------------------------
static void ledWrite(bool on) {
  if (BRIDGE_LED_ACTIVE_LOW) {
    digitalWrite(BRIDGE_LED_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(BRIDGE_LED_PIN, on ? HIGH : LOW);
  }
}

// ---- WiFi station connect (non-blocking-ish, 20 s cap) ---------------------
static bool connectStation() {
#if defined(BRIDGE_AP_ONLY)
  return false;
#endif
  WiFi.mode(WIFI_STA);
  if (BRIDGE_USE_STATIC_IP) {
    IPAddress ip, gw, mask, dns;
    ip.fromString(BRIDGE_STATIC_IP);
    gw.fromString(BRIDGE_STATIC_GW);
    mask.fromString(BRIDGE_STATIC_MASK);
    dns.fromString(BRIDGE_STATIC_DNS);
    WiFi.config(ip, gw, mask, dns);
  }
  WiFi.begin(BRIDGE_WIFI_SSID, BRIDGE_WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < BRIDGE_WIFI_TIMEOUT_MS) {
    delay(250);
    ledWrite((millis() / 250) % 2 == 0);
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    apMode = false;
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }
  Serial.println("WiFi connect failed, falling back to AP mode");
  return false;
}

static void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(BRIDGE_AP_SSID, BRIDGE_AP_PASS, BRIDGE_AP_CHANNEL, BRIDGE_AP_HIDDEN);
  apMode = true;
  wifiConnected = true;
  Serial.print("AP mode, IP: ");
  Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(BRIDGE_UART_BAUD, SERIAL_8N1, BRIDGE_UART_RX, BRIDGE_UART_TX);

  pinMode(BRIDGE_LED_PIN, OUTPUT);
  ledWrite(false);

  if (!connectStation()) {
    startAP();
  }

  tcpServer.begin();
  tcpServer.setNoDelay(true);
  Serial.print("TCP server listening on port ");
  Serial.println(BRIDGE_TCP_PORT);
}

void loop() {
  // ---- WiFi health / reconnect (non-blocking) ------------------------------
  if (!apMode) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnected = false;
      if (millis() - lastWifiRetry >= BRIDGE_WIFI_RETRY_MS) {
        lastWifiRetry = millis();
        Serial.println("WiFi dropped, retrying...");
        WiFi.disconnect();
        WiFi.begin(BRIDGE_WIFI_SSID, BRIDGE_WIFI_PASS);
      }
    } else {
      wifiConnected = true;
    }
  }

  // ---- Accept / replace client -------------------------------------------
  WiFiClient newClient = tcpServer.available();
  if (newClient) {
    if (tcpClient && tcpClient.connected()) {
      tcpClient.stop();
    }
    tcpClient = newClient;
    tcpClient.setNoDelay(true);
    clientWasConnected = true;
    Serial.println("Client connected");
#if BRIDGE_BANNER
    tcpClient.print(BRIDGE_BANNER_TEXT);
#endif
  }

  // Drop stale/disconnected client handle.
  if (tcpClient && !tcpClient.connected()) {
    tcpClient.stop();
    clientWasConnected = false;
    Serial.println("Client disconnected");
  }

  bool clientConnected = tcpClient && tcpClient.connected();

  // ---- Bidirectional pump (byte-transparent, chunk-capped) ---------------
  if (clientConnected) {
    // TCP -> UART
    int avail = tcpClient.available();
    if (avail > 0) {
      int n = avail;
      if (n > BRIDGE_CHUNK_MAX) n = BRIDGE_CHUNK_MAX;
      uint8_t buf[BRIDGE_CHUNK_MAX];
      int rd = tcpClient.read(buf, n);
      if (rd > 0) {
        Serial1.write(buf, rd);
      }
    }
    // UART -> TCP
    int uavail = Serial1.available();
    if (uavail > 0) {
      int n = uavail;
      if (n > BRIDGE_CHUNK_MAX) n = BRIDGE_CHUNK_MAX;
      uint8_t buf[BRIDGE_CHUNK_MAX];
      int rd = Serial1.readBytes(buf, n);
      if (rd > 0) {
        tcpClient.write(buf, rd);
      }
    }
  } else {
    // Client dropped mid-line: drain any partial bytes already in UART RX
    // buffer so they are not replayed to a later client. (UART input from
    // the Mega that arrived while no client was connected is also dropped,
    // which is the desired transparent behavior.)
    while (Serial1.available()) {
      Serial1.read();
    }
  }

  // ---- Status LED --------------------------------------------------------
  unsigned long now = millis();
  if (apMode) {
    // fast blink
    if (now - lastBlink >= 150) {
      lastBlink = now;
      blinkState = !blinkState;
      ledWrite(blinkState);
    }
  } else if (clientConnected) {
    ledWrite(true); // solid
  } else if (wifiConnected) {
    // slow blink
    if (now - lastBlink >= 750) {
      lastBlink = now;
      blinkState = !blinkState;
      ledWrite(blinkState);
    }
  } else {
    ledWrite(false);
  }

  // Never block the pump. (No delay() here.)
}