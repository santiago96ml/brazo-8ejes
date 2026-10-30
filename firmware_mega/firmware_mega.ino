#include <Wire.h>
#include <EEPROM.h>

// ---------- PCA9685 registers ----------
#define PCA_ADDR 0x40
#define PCA_MODE1 0x00
#define PCA_PRESCALE 0xFE
#define PCA_LED0_ON_L 0x06

#define PIN_OE 2
#define NUM_CH 8

// ---------- Per-channel state ----------
static uint16_t servoMinP[NUM_CH];
static uint16_t servoStopP[NUM_CH];
static uint16_t servoMaxP[NUM_CH];
static float degPerSec[NUM_CH];
static float minDeg[NUM_CH];
static float maxDeg[NUM_CH];
static float homeDeg[NUM_CH];

static float currentDeg[NUM_CH];
static float startDeg[NUM_CH];
static float targetDeg[NUM_CH];
static unsigned long moveStartMs[NUM_CH];
static unsigned long moveDurationMs[NUM_CH];
static bool moving[NUM_CH];

static bool timedActive[NUM_CH];
static unsigned long timedEndMs[NUM_CH];
static uint16_t timedRestoreP[NUM_CH];
static uint16_t currentPulse[NUM_CH];

static int8_t shoulderCoupling = 1;
static bool estopLatched = false;
static bool enabled = true;
static unsigned long lastTickMs = 0;

#define EEPROM_MAGIC_0 0xA6
#define EEPROM_MAGIC_1 0x5A

static char lineBuf0[256];
static char lineBuf1[256];
static uint16_t lineLen0 = 0;
static uint16_t lineLen1 = 0;
static char outBuf[640];

// ---------- helpers ----------
static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static uint16_t degToPulse(uint8_t ch, float deg) {
  float mn = minDeg[ch], mx = maxDeg[ch];
  if (mx <= mn) mx = mn + 1.0f;
  float t = (deg - mn) / (mx - mn);
  t = clampf(t, 0.0f, 1.0f);
  float p = servoMinP[ch] + t * (servoMaxP[ch] - servoMinP[ch]);
  if (p < 0) p = 0;
  if (p > 4095) p = 4095;
  return (uint16_t)p;
}

static float pulseToDeg(uint8_t ch, uint16_t pulse) {
  float mn = minDeg[ch], mx = maxDeg[ch];
  if (mx <= mn) mx = mn + 1.0f;
  float span = (float)(servoMaxP[ch] - servoMinP[ch]);
  if (span < 1) span = 1;
  float t = ((float)pulse - servoMinP[ch]) / span;
  t = clampf(t, 0.0f, 1.0f);
  return mn + t * (mx - mn);
}

static void pcaWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t pcaReadReg(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)PCA_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

static uint8_t pcaProbe() {
  Wire.beginTransmission(PCA_ADDR);
  return Wire.endTransmission();
}

static void pcaSetPulse(uint8_t ch, uint16_t pulse) {
  if (ch >= 16) return;
  if (pulse > 4095) pulse = 4095;
  uint8_t base = PCA_LED0_ON_L + 4 * ch;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(base);
  Wire.write(0);
  Wire.write(0);
  Wire.write(pulse & 0xFF);
  Wire.write((pulse >> 8) & 0x0F);
  Wire.endTransmission();
}

static void pcaInitDevice() {
  pcaWriteReg(PCA_MODE1, 0x00);
  delay(5);
  uint8_t old = pcaReadReg(PCA_MODE1);
  pcaWriteReg(PCA_MODE1, (old & 0x7F) | 0x10); // sleep
  pcaWriteReg(PCA_PRESCALE, 121);              // 50 Hz
  pcaWriteReg(PCA_MODE1, old);
  delay(5);
  pcaWriteReg(PCA_MODE1, old | 0xA0);          // auto-increment + restart
  for (uint8_t i = 0; i < NUM_CH; i++) pcaSetPulse(i, 307);
}

// ---------- defaults ----------
static void defaultsInit() {
  for (uint8_t i = 0; i < NUM_CH; i++) {
    servoMinP[i] = 110;
    servoStopP[i] = 307;
    servoMaxP[i] = 500;
    degPerSec[i] = 60.0f;
    minDeg[i] = 0;
    maxDeg[i] = 180;
    homeDeg[i] = 90;
    currentDeg[i] = 90;
    startDeg[i] = 90;
    targetDeg[i] = 90;
    moveStartMs[i] = 0;
    moveDurationMs[i] = 0;
    moving[i] = false;
    timedActive[i] = false;
    timedEndMs[i] = 0;
    timedRestoreP[i] = 307;
    currentPulse[i] = 307;
  }
  // per-channel degree ranges
  minDeg[0] = 0;   maxDeg[0] = 100;  homeDeg[0] = 50;   // PINZA
  minDeg[1] = 0;   maxDeg[1] = 360;  homeDeg[1] = 0;     // GIRO_PINZA
  minDeg[2] = 0;   maxDeg[2] = 180;  homeDeg[2] = 90;    // INCLINA
  minDeg[3] = 0;   maxDeg[3] = 180;  homeDeg[3] = 90;    // CODO
  minDeg[4] = 15;  maxDeg[4] = 165;  homeDeg[4] = 90;    // HOMBRO_IZQ
  minDeg[5] = 15;  maxDeg[5] = 165;  homeDeg[5] = 90;    // HOMBRO_DER
  minDeg[6] = 0;   maxDeg[6] = 360;  homeDeg[6] = 0;     // BASE
  minDeg[7] = 0;   maxDeg[7] = 180;  homeDeg[7] = 90;    // AUX
  for (uint8_t i = 0; i < NUM_CH; i++) {
    currentDeg[i] = homeDeg[i];
    startDeg[i] = homeDeg[i];
    targetDeg[i] = homeDeg[i];
    currentPulse[i] = servoStopP[i];
  }
}

static void reply(uint8_t port, const char* s) {
  if (port == 0) Serial.println(s); else Serial1.println(s);
}
static void broadcast(const char* s) {
  Serial.println(s);
  Serial1.println(s);
}

static float coupledDeg(float masterDeg) {
  if (shoulderCoupling == 1) return masterDeg;
  return 180.0f - masterDeg;
}

static void cancelTimed(uint8_t ch) {
  timedActive[ch] = false;
}

static void startMove(uint8_t ch, float deg, unsigned long durMs) {
  deg = clampf(deg, minDeg[ch], maxDeg[ch]);
  targetDeg[ch] = deg;
  startDeg[ch] = currentDeg[ch];
  moveStartMs[ch] = millis();
  if (durMs == 0) {
    float delta = fabsf(deg - currentDeg[ch]);
    float sp = degPerSec[ch]; if (sp < 1) sp = 1;
    unsigned long autoMs = (unsigned long)((delta / sp) * 1000.0f);
    if (autoMs < 300) autoMs = 300;
    durMs = autoMs;
  } else {
    if (durMs < 300) durMs = 300;
  }
  moveDurationMs[ch] = durMs;
  moving[ch] = true;
  cancelTimed(ch);
}

static void startShoulderMove(float masterDeg, unsigned long durMs) {
  masterDeg = clampf(masterDeg, minDeg[4], maxDeg[4]);
  float slaveDeg = coupledDeg(masterDeg);
  // compute shared duration
  unsigned long d = durMs;
  if (d == 0) {
    float delta4 = fabsf(masterDeg - currentDeg[4]);
    float delta5 = fabsf(slaveDeg - currentDeg[5]);
    float sp4 = degPerSec[4]; if (sp4 < 1) sp4 = 1;
    float sp5 = degPerSec[5]; if (sp5 < 1) sp5 = 1;
    unsigned long a4 = (unsigned long)((delta4 / sp4) * 1000.0f);
    unsigned long a5 = (unsigned long)((delta5 / sp5) * 1000.0f);
    d = (a4 > a5) ? a4 : a5;
    if (d < 300) d = 300;
  } else {
    if (d < 300) d = 300;
  }
  // set ch4
  targetDeg[4] = masterDeg;
  startDeg[4] = currentDeg[4];
  moveStartMs[4] = millis();
  moveDurationMs[4] = d;
  moving[4] = true;
  cancelTimed(4);
  // set ch5
  targetDeg[5] = slaveDeg;
  startDeg[5] = currentDeg[5];
  moveStartMs[5] = millis();
  moveDurationMs[5] = d;
  moving[5] = true;
  cancelTimed(5);
}

static void syncShoulderNow() {
  // re-sync ch5 to ch4 using coupling
  currentDeg[5] = coupledDeg(currentDeg[4]);
  targetDeg[5] = coupledDeg(targetDeg[4]);
  startDeg[5] = currentDeg[5];
  moving[5] = moving[4];
  moveStartMs[5] = moveStartMs[4];
  moveDurationMs[5] = moveDurationMs[4];
  currentPulse[5] = degToPulse(5, currentDeg[5]);
  if (!estopLatched) pcaSetPulse(5, currentPulse[5]);
}

// ---------- tick ----------
static void tick() {
  unsigned long now = millis();
  // timed restore (checked every loop pass for millisecond accuracy)
  for (uint8_t i = 0; i < NUM_CH; i++) {
    if (timedActive[i] && (long)(now - timedEndMs[i]) >= 0) {
      timedActive[i] = false;
      currentPulse[i] = servoStopP[i];
      if (!estopLatched) pcaSetPulse(i, servoStopP[i]);
    }
  }
  // 50 Hz (20 ms) non-blocking positional interpolation tick
  if (now - lastTickMs < 20) return;
  lastTickMs = now;
  for (uint8_t i = 0; i < NUM_CH; i++) {
    if (moving[i]) {
      unsigned long dur = moveDurationMs[i];
      if (dur == 0) dur = 300;
      unsigned long elapsed = now - moveStartMs[i];
      float t = (float)elapsed / (float)dur;
      if (t >= 1.0f) {
        t = 1.0f;
        moving[i] = false;
        currentDeg[i] = targetDeg[i];
      } else {
        // cosine ease
        float eased = (1.0f - cosf(3.14159265f * t)) / 2.0f;
        currentDeg[i] = startDeg[i] + (targetDeg[i] - startDeg[i]) * eased;
      }
      uint16_t p = degToPulse(i, currentDeg[i]);
      if (p != currentPulse[i]) {
        currentPulse[i] = p;
        if (!estopLatched) pcaSetPulse(i, p);
      }
    }
  }
}

// ---------- command handling ----------
static void handleLine(char* line, uint8_t port);

static void processPort(HardwareSerial& ser, uint8_t port) {
  char* buf = (port == 0) ? lineBuf0 : lineBuf1;
  uint16_t* len = (port == 0) ? &lineLen0 : &lineLen1;
  while (ser.available()) {
    char c = ser.read();
    if (c == '\n') {
      buf[*len] = '\0';
      char tmp[256];
      strncpy(tmp, buf, 255);
      tmp[255] = '\0';
      *len = 0;
      handleLine(tmp, port);
    } else if (c == '\r') {
      // ignore
    } else {
      if (*len < 255) buf[(*len)++] = c;
      else { *len = 0; }
    }
  }
}

static void handleLine(char* line, uint8_t port) {
  // trim trailing
  int n = strlen(line);
  while (n > 0 && (line[n-1] == '\r' || line[n-1] == ' ')) { line[n-1] = '\0'; n--; }
  if (line[0] == '\0') return;
  char cmd = line[0];
  if (cmd >= 'a' && cmd <= 'z') cmd -= 32;

  // single char no-arg
  if (cmd == 'I') {
    uint8_t err = pcaProbe();
    if (err == 0) {
      pcaInitDevice();
      reply(port, "{\"status\":\"OK\",\"pca9685\":\"CONECTADO_0x40\"}");
    } else {
      HardwareSerial& out = (port == 0) ? Serial : Serial1;
      out.print("{\"status\":\"ERROR\",\"i2c_code\":");
      out.print(err);
      out.print(",\"found\":[");
      bool first = true;
      for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          if (!first) out.print(",");
          out.print(addr);
          first = false;
        }
      }
      out.println("],\"msg\":\"Verificar SDA->Pin20, SCL->Pin21, VCC->5V y GND\"}");
    }
    return;
  }
  if (cmd == 'V') {
    reply(port, "{\"status\":\"VERSION\",\"fw\":\"2.0.0\",\"board\":\"MEGA2560_PCA9685\",\"proto\":2}");
    return;
  }
  if (cmd == 'F') {
    for (uint8_t i = 0; i < NUM_CH; i++) {
      moving[i] = false;
      timedActive[i] = false;
      currentPulse[i] = servoStopP[i];
      if (!estopLatched) pcaSetPulse(i, servoStopP[i]);
    }
    broadcast("{\"event\":\"ALL_NEUTRAL\"}");
    return;
  }
  if (cmd == 'S') {
    // emergency stop
    estopLatched = true;
    enabled = false;
    digitalWrite(PIN_OE, HIGH);
    for (uint8_t i = 0; i < NUM_CH; i++) {
      moving[i] = false;
      timedActive[i] = false;
      currentPulse[i] = servoStopP[i];
      pcaSetPulse(i, servoStopP[i]);
    }
    broadcast("{\"event\":\"EMERGENCY_STOP\",\"oe\":\"HIGH\"}");
    return;
  }
  if (cmd == 'R') {
    estopLatched = false;
    enabled = true;
    digitalWrite(PIN_OE, LOW);
    broadcast("{\"event\":\"ENABLED\",\"oe\":\"LOW\"}");
    return;
  }
  if (cmd == 'H') {
    if (estopLatched) { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"ESTOP_LATCHED\"}"); reply(port, outBuf); return; }
    // move all to home in 1500ms
    for (uint8_t i = 0; i < NUM_CH; i++) {
      if (i == 4) startShoulderMove(homeDeg[4], 1500);
      else if (i == 5) { /* handled by shoulder */ }
      else startMove(i, homeDeg[i], 1500);
    }
    reply(port, "{\"ack\":\"HOME\"}");
    return;
  }
  if (cmd == 'Q') {
    HardwareSerial& out = (port == 0) ? Serial : Serial1;
    snprintf(outBuf, sizeof(outBuf), "{\"status\":\"STATE\",\"t\":%lu,\"oe\":%d,\"shoulder_coupling\":%d,\"motors\":[",
             millis(), digitalRead(PIN_OE), shoulderCoupling);
    out.print(outBuf);
    char item[112];
    char sDeg[12], sTgt[12], sDps[12];
    for (uint8_t i = 0; i < NUM_CH; i++) {
      dtostrf(currentDeg[i], 1, 1, sDeg);
      dtostrf(targetDeg[i], 1, 1, sTgt);
      dtostrf(degPerSec[i], 1, 1, sDps);
      snprintf(item, sizeof(item), "{\"m\":%u,\"ch\":%u,\"deg\":%s,\"target\":%s,\"pulse\":%u,\"stop\":%u,\"dps\":%s,\"moving\":%d}",
               i + 1, i, sDeg, sTgt, currentPulse[i], servoStopP[i], sDps, moving[i] ? 1 : 0);
      out.print(item);
      if (i < NUM_CH - 1) out.print(",");
    }
    out.println("]}");
    return;
  }
  // U and J6
  if (cmd == 'U') {
    // U1 or U-1
    int v = atoi(line + 1);
    if (v != 1 && v != -1) { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"BAD_ARG\"}"); reply(port, outBuf); return; }
    shoulderCoupling = (int8_t)v;
    EEPROM.write(0, EEPROM_MAGIC_0);
    EEPROM.write(1, EEPROM_MAGIC_1);
    EEPROM.write(2, (uint8_t)shoulderCoupling);
    syncShoulderNow();
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"COUPLING\",\"shoulder_coupling\":%d}", shoulderCoupling);
    reply(port, outBuf);
    return;
  }
  if (cmd == 'J') {
    // J6 toggle
    if (line[1] == '6' || line[1] == '\0') {
      shoulderCoupling = (shoulderCoupling == 1) ? -1 : 1;
      EEPROM.write(0, EEPROM_MAGIC_0);
      EEPROM.write(1, EEPROM_MAGIC_1);
      EEPROM.write(2, (uint8_t)shoulderCoupling);
      syncShoulderNow();
      snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"COUPLING\",\"shoulder_coupling\":%d}", shoulderCoupling);
      reply(port, outBuf);
      return;
    }
  }

  // G command
  if (cmd == 'G') {
    if (estopLatched) { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"ESTOP_LATCHED\"}"); reply(port, outBuf); return; }
    // parse G:m1,m2,m3,m4,m5,m7,ms  (line starts G: or G)
    char* p = line;
    p++;
    if (*p == ':') p++;
    float vals[7];
    uint8_t vi = 0;
    char* tok = strtok(p, ",");
    while (tok && vi < 7) {
      vals[vi++] = atof(tok);
      tok = strtok(NULL, ",");
    }
    if (vi < 7) { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"BAD_ARG\"}"); reply(port, outBuf); return; }
    float m1 = vals[0], m2 = vals[1], m3 = vals[2], m4 = vals[3], m5 = vals[4], m7 = vals[5];
    unsigned long ms = (unsigned long)vals[6];
    float m6 = coupledDeg(m5);
    if (ms == 0) ms = 1000;
    startMove(0, m1, ms);
    startMove(1, m2, ms);
    startMove(2, m3, ms);
    startMove(3, m4, ms);
    startShoulderMove(m5, ms);
    startMove(6, m7, ms);
    char sM5[12], sM6[12];
    dtostrf(m5, 1, 1, sM5);
    dtostrf(m6, 1, 1, sM6);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"G_OK\",\"m5\":%s,\"m6\":%s,\"ms\":%lu}", sM5, sM6, ms);
    reply(port, outBuf);
    return;
  }

  // T command: T<motor>:<offset>:<ms>
  if (cmd == 'T') {
    if (estopLatched) { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"ESTOP_LATCHED\"}"); reply(port, outBuf); return; }
    // motor digit
    if (line[1] < '1' || line[1] > '8') { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"BAD_CH\"}"); reply(port, outBuf); return; }
    uint8_t motor = line[1] - '0';
    uint8_t ch = motor - 1;
    char* colon = strchr(line + 2, ':');
    if (!colon) { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"BAD_ARG\"}"); reply(port, outBuf); return; }
    *colon = '\0';
    int offset = atoi(line + 2);
    *colon = ':';
    if (offset < -150 || offset > 150) { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"BAD_ARG\"}"); reply(port, outBuf); return; }
    unsigned long ms = strtoul(colon + 1, NULL, 10);
    if (ms == 0) ms = 1000;
    // timed spin
    if (motor == 5 || motor == 6) {
      // drive both ch4 and ch5
      int off4, off5;
      if (motor == 5) { off4 = offset; off5 = shoulderCoupling * offset; }
      else { off5 = offset; off4 = shoulderCoupling * offset; }
      // ch4
      int32_t p4 = (int32_t)servoStopP[4] + off4;
      if (p4 < servoMinP[4]) p4 = servoMinP[4];
      if (p4 > servoMaxP[4]) p4 = servoMaxP[4];
      moving[4] = false; timedActive[4] = true; timedEndMs[4] = millis() + ms; timedRestoreP[4] = servoStopP[4]; currentPulse[4] = (uint16_t)p4;
      if (!estopLatched) pcaSetPulse(4, (uint16_t)p4);
      // ch5
      int32_t p5 = (int32_t)servoStopP[5] + off5;
      if (p5 < servoMinP[5]) p5 = servoMinP[5];
      if (p5 > servoMaxP[5]) p5 = servoMaxP[5];
      moving[5] = false; timedActive[5] = true; timedEndMs[5] = millis() + ms; timedRestoreP[5] = servoStopP[5]; currentPulse[5] = (uint16_t)p5;
      if (!estopLatched) pcaSetPulse(5, (uint16_t)p5);
    } else {
      int32_t p = (int32_t)servoStopP[ch] + offset;
      if (p < servoMinP[ch]) p = servoMinP[ch];
      if (p > servoMaxP[ch]) p = servoMaxP[ch];
      moving[ch] = false;
      timedActive[ch] = true;
      timedEndMs[ch] = millis() + ms;
      timedRestoreP[ch] = servoStopP[ch];
      currentPulse[ch] = (uint16_t)p;
      if (!estopLatched) pcaSetPulse(ch, (uint16_t)p);
    }
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"T_OK\",\"motor\":%u,\"offset\":%d,\"ms\":%lu}", motor, offset, ms);
    reply(port, outBuf);
    return;
  }

  // C command: C<motor>:<stop>[:<dps>]
  if (cmd == 'C') {
    if (line[1] < '1' || line[1] > '8') { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"BAD_CH\"}"); reply(port, outBuf); return; }
    uint8_t motor = line[1] - '0';
    uint8_t ch = motor - 1;
    if (line[2] != ':') { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"BAD_ARG\"}"); reply(port, outBuf); return; }
    char* p = line + 3;
    char* colon = strchr(p, ':');
    uint16_t stop;
    float dps = degPerSec[ch];
    if (colon) {
      *colon = '\0';
      stop = (uint16_t)strtoul(p, NULL, 10);
      *colon = ':';
      dps = atof(colon + 1);
    } else {
      stop = (uint16_t)strtoul(p, NULL, 10);
    }
    servoStopP[ch] = stop;
    if (dps > 0) degPerSec[ch] = dps;
    // if shoulder, sync partner? just set this channel
    currentPulse[ch] = stop;
    if (!estopLatched) pcaSetPulse(ch, stop);
    char sDps[12];
    dtostrf(degPerSec[ch], 1, 1, sDps);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"CALIBRATED\",\"motor\":%u,\"stop\":%u,\"dps\":%s}", motor, stop, sDps);
    reply(port, outBuf);
    return;
  }

  // M command: M<motor>:<deg>[:<ms>]
  if (cmd == 'M') {
    if (estopLatched) { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"ESTOP_LATCHED\"}"); reply(port, outBuf); return; }
    if (line[1] < '1' || line[1] > '8') { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"BAD_CH\"}"); reply(port, outBuf); return; }
    uint8_t motor = line[1] - '0';
    uint8_t ch = motor - 1;
    if (line[2] != ':') { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"BAD_ARG\"}"); reply(port, outBuf); return; }
    char* p = line + 3;
    char* colon = strchr(p, ':');
    float deg = atof(p);
    unsigned long ms = 0;
    if (colon) ms = strtoul(colon + 1, NULL, 10);
    if (motor == 5) {
      startShoulderMove(deg, ms);
    } else if (motor == 6) {
      // convert slave deg to master deg
      float masterDeg = (shoulderCoupling == 1) ? deg : (180.0f - deg);
      startShoulderMove(masterDeg, ms);
    } else {
      startMove(ch, deg, ms);
    }
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"M_OK\",\"motor\":%u,\"deg\":%d}", motor, (int)deg);
    reply(port, outBuf);
    return;
  }

  // P command: P<motor>:<pulse>
  if (cmd == 'P') {
    if (estopLatched) { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"ESTOP_LATCHED\"}"); reply(port, outBuf); return; }
    if (line[1] < '1' || line[1] > '8') { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"BAD_CH\"}"); reply(port, outBuf); return; }
    uint8_t motor = line[1] - '0';
    uint8_t ch = motor - 1;
    if (line[2] != ':') { snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"BAD_ARG\"}"); reply(port, outBuf); return; }
    uint16_t pulse = (uint16_t)strtoul(line + 3, NULL, 10);
    if (motor == 5 || motor == 6) {
      // update both
      if (motor == 5) {
        currentDeg[4] = pulseToDeg(4, pulse);
        currentPulse[4] = pulse;
        moving[4] = false; timedActive[4] = false;
        if (!estopLatched) pcaSetPulse(4, pulse);
        float sd = coupledDeg(currentDeg[4]);
        currentDeg[5] = sd;
        currentPulse[5] = degToPulse(5, sd);
        moving[5] = false; timedActive[5] = false;
        if (!estopLatched) pcaSetPulse(5, currentPulse[5]);
      } else {
        currentDeg[5] = pulseToDeg(5, pulse);
        currentPulse[5] = pulse;
        moving[5] = false; timedActive[5] = false;
        if (!estopLatched) pcaSetPulse(5, pulse);
        float md = (shoulderCoupling == 1) ? currentDeg[5] : (180.0f - currentDeg[5]);
        currentDeg[4] = md;
        currentPulse[4] = degToPulse(4, md);
        moving[4] = false; timedActive[4] = false;
        if (!estopLatched) pcaSetPulse(4, currentPulse[4]);
      }
    } else {
      currentDeg[ch] = pulseToDeg(ch, pulse);
      currentPulse[ch] = pulse;
      moving[ch] = false;
      timedActive[ch] = false;
      if (!estopLatched) pcaSetPulse(ch, pulse);
    }
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"P_OK\",\"motor\":%u,\"pulse\":%u}", motor, pulse);
    reply(port, outBuf);
    return;
  }

  snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"UNKNOWN_CMD\"}");
  reply(port, outBuf);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(PIN_OE, OUTPUT);
  digitalWrite(PIN_OE, HIGH); // safe startup

  Wire.begin();
  defaultsInit();
  {
    if (EEPROM.read(0) == EEPROM_MAGIC_0 && EEPROM.read(1) == EEPROM_MAGIC_1) {
      int8_t sc = (int8_t)EEPROM.read(2);
      if (sc == 1 || sc == -1) shoulderCoupling = sc;
    }
  }
  pcaInitDevice();

  // all channels to stop
  for (uint8_t i = 0; i < NUM_CH; i++) {
    pcaSetPulse(i, servoStopP[i]);
    currentPulse[i] = servoStopP[i];
  }

  // enable outputs
  estopLatched = false;
  enabled = true;
  digitalWrite(PIN_OE, LOW);

  snprintf(outBuf, sizeof(outBuf), "{\"status\":\"READY\",\"board\":\"MEGA2560_PCA9685\",\"fw\":\"2.0.0\",\"shoulder_coupling\":%d}", shoulderCoupling);
  Serial.println(outBuf);
  Serial1.println(outBuf);
}

void loop() {
  processPort(Serial, 0);
  processPort(Serial1, 1);
  tick();
}