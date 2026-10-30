#include "MotionEngine.h"
#include "Config.h"
#include "Calibration.h"
#include "Pca9685.h"

static AxisState axes[NUM_CH];
static bool estopLatched = true;
static bool enabled = false;
static unsigned long watchdogMs = 0;
static unsigned long lastKeepaliveMs = 0;
static unsigned long lastTickMs = 0;

static float clampf(float v, float lo, float hi) { if (v<lo) return lo; if (v>hi) return hi; return v; }

static uint16_t degToPulse(uint8_t ch, float deg) {
  ChannelCal& c = calibCh(ch);
  float mn = c.minDeg, mx = c.maxDeg;
  if (mx <= mn) mx = mn + 1;
  float t = (deg - mn) / (mx - mn);
  t = clampf(t, 0.0f, 1.0f);
  float p = c.minPulse + t * (c.maxPulse - c.minPulse);
  if (p < 0) p = 0; if (p > 4095) p = 4095;
  return (uint16_t)p;
}

void motionInit() {
  for (uint8_t i = 0; i < NUM_CH; i++) {
    axes[i].currentDeg = calibCh(i).homeDeg;
    axes[i].targetDeg = calibCh(i).homeDeg;
    axes[i].currentPulse = calibCh(i).stopPulse;
    axes[i].moving = 0;
    axes[i].moveDurationMs = 0;
    axes[i].timedActive = 0;
    axes[i].timedEndMs = 0;
  }
  estopLatched = true;
  enabled = false;
  lastTickMs = millis();
  lastKeepaliveMs = millis();
  // OE high (disabled) - caller sets pin, but ensure
  digitalWrite(PIN_OE, HIGH);
  pcaSetAll(0); // all off at boot? spec: all channels to stop pulse. But OE high disables outputs. Write pulses anyway.
  for (uint8_t i = 0; i < NUM_CH; i++) pcaSetPulse(i, calibCh(i).stopPulse);
}

void motionEstop() {
  estopLatched = true;
  enabled = false;
  digitalWrite(PIN_OE, HIGH);
  for (uint8_t i = 0; i < NUM_CH; i++) {
    axes[i].moving = 0;
    axes[i].timedActive = 0;
    axes[i].currentPulse = calibCh(i).stopPulse;
    pcaSetPulse(i, calibCh(i).stopPulse);
  }
}

void motionEnable() {
  estopLatched = false;
  enabled = true;
  digitalWrite(PIN_OE, LOW);
}

void motionAllNeutral() {
  for (uint8_t i = 0; i < NUM_CH; i++) {
    axes[i].moving = 0;
    axes[i].timedActive = 0;
    axes[i].targetDeg = axes[i].currentDeg;
    axes[i].currentPulse = calibCh(i).stopPulse;
    if (!estopLatched) pcaSetPulse(i, calibCh(i).stopPulse);
  }
}

bool motionIsEstop() { return estopLatched; }
bool motionIsEnabled() { return enabled; }

void motionMovePos(uint8_t ch, float deg, unsigned long durationMs) {
  if (ch >= NUM_CH) return;
  if (calibCh(ch).mode != MODE_POS) return; // reject continuous handled by router
  // clamp deg to limits
  deg = clampf(deg, calibCh(ch).minDeg, calibCh(ch).maxDeg);
  if (calibCh(ch).invert) {
    // invert angle around midpoint
    float mid = (calibCh(ch).minDeg + calibCh(ch).maxDeg) / 2.0f;
    deg = mid - (deg - mid);
  }
  axes[ch].targetDeg = deg;
  axes[ch].moveTargetDeg = deg;
  axes[ch].moveStartDeg = axes[ch].currentDeg;
  // duration
  float delta = fabsf(deg - axes[ch].currentDeg);
  float speed = calibCh(ch).speedDps;
  if (speed < 1) speed = 1;
  unsigned long autoMs = (unsigned long)((delta / speed) * 1000.0f);
  if (autoMs < 300) autoMs = 300;
  if (durationMs > 0) {
    if (durationMs < 300) durationMs = 300;
    axes[ch].moveDurationMs = durationMs;
  } else {
    axes[ch].moveDurationMs = autoMs;
  }
  axes[ch].moveStartMs = millis();
  axes[ch].moving = 1;
}

void motionMoveRaw(uint8_t ch, uint16_t pulse) {
  if (ch >= NUM_CH) return;
  axes[ch].moving = 0;
  axes[ch].timedActive = 0;
  axes[ch].currentPulse = pulse;
  if (!estopLatched) pcaSetPulse(ch, pulse);
}

static void motionTimedSingle(uint8_t ch, int16_t offset, unsigned long ms) {
  ChannelCal& c = calibCh(ch);
  int32_t p = (int32_t)c.stopPulse + offset;
  if (p < c.minPulse) p = c.minPulse;
  if (p > c.maxPulse) p = c.maxPulse;
  axes[ch].moving = 0;
  axes[ch].currentPulse = (uint16_t)p;
  axes[ch].timedRestorePulse = c.stopPulse;
  axes[ch].timedEndMs = millis() + ms;
  axes[ch].timedActive = 1;
  if (!estopLatched) pcaSetPulse(ch, (uint16_t)p);
}

void motionTimed(uint8_t ch, int16_t offset, unsigned long ms) {
  motionTimedSingle(ch, offset, ms);
  if (calibCh(ch).mirrorSrc < NUM_CH) {
    motionTimedSingle(calibCh(ch).mirrorSrc, (int16_t)(-offset), ms);
  } else {
    for (uint8_t m = 0; m < NUM_CH; m++) {
      if (calibCh(m).mirrorSrc == ch) {
        motionTimedSingle(m, (int16_t)(-offset), ms);
      }
    }
  }
}

void motionCalibrateStop(uint8_t ch, uint16_t pulse) {
  if (ch >= NUM_CH) return;
  calibCh(ch).stopPulse = pulse;
  axes[ch].currentPulse = pulse;
  if (!estopLatched) pcaSetPulse(ch, pulse);
}

void motionSetRange(uint8_t ch, uint16_t mn, uint16_t mx) {
  if (ch >= NUM_CH) return;
  calibCh(ch).minPulse = mn;
  calibCh(ch).maxPulse = mx;
}

void motionSetMode(uint8_t ch, uint8_t mode) {
  if (ch >= NUM_CH) return;
  calibCh(ch).mode = mode;
}

void motionSetLimits(uint8_t ch, int16_t mn, int16_t mx) {
  if (ch >= NUM_CH) return;
  calibCh(ch).minDeg = mn;
  calibCh(ch).maxDeg = mx;
}

void motionSetMirror(uint8_t ch, uint8_t src) {
  if (ch >= NUM_CH) return;
  calibCh(ch).mirrorSrc = src;
}

void motionToggleInvert(uint8_t ch) {
  if (ch >= NUM_CH) return;
  calibCh(ch).invert ^= 1;
}

void motionCoordinated(uint8_t* chs, float* degs, uint8_t n) {
  // compute per-joint durations, use max
  unsigned long maxMs = 300;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t ch = chs[i];
    float deg = degs[i];
    deg = clampf(deg, calibCh(ch).minDeg, calibCh(ch).maxDeg);
    if (calibCh(ch).invert) {
      float mid = (calibCh(ch).minDeg + calibCh(ch).maxDeg)/2.0f;
      deg = mid - (deg - mid);
    }
    degs[i] = deg;
    float delta = fabsf(deg - axes[ch].currentDeg);
    float speed = calibCh(ch).speedDps; if (speed < 1) speed = 1;
    unsigned long ms = (unsigned long)((delta/speed)*1000.0f);
    if (ms < 300) ms = 300;
    if (ms > maxMs) maxMs = ms;
  }
  // set targets with shared duration
  for (uint8_t i = 0; i < n; i++) {
    uint8_t ch = chs[i];
    axes[ch].targetDeg = degs[i];
    axes[ch].moveTargetDeg = degs[i];
    axes[ch].moveStartDeg = axes[ch].currentDeg;
    axes[ch].moveDurationMs = maxMs;
    axes[ch].moveStartMs = millis();
    axes[ch].moving = 1;
  }
}

void motionHomeAll() {
  for (uint8_t i = 0; i < NUM_CH; i++) {
    if (calibCh(i).mode == MODE_POS) {
      motionMovePos(i, (float)calibCh(i).homeDeg, 0);
    }
  }
}

void motionSetHome(uint8_t ch, uint16_t deg) {
  if (ch >= NUM_CH) return;
  calibCh(ch).homeDeg = deg;
}

void motionRestoreDefaults() {
  calibDefaults();
  // reset axes
  for (uint8_t i = 0; i < NUM_CH; i++) {
    axes[i].currentDeg = calibCh(i).homeDeg;
    axes[i].targetDeg = calibCh(i).homeDeg;
    axes[i].currentPulse = calibCh(i).stopPulse;
    axes[i].moving = 0;
    axes[i].timedActive = 0;
  }
}

void motionWatchdog(uint32_t ms) { watchdogMs = ms; lastKeepaliveMs = millis(); }

void motionKeepalive() { lastKeepaliveMs = millis(); }

float motionCurrentDeg(uint8_t ch) { return (ch < NUM_CH) ? axes[ch].currentDeg : 0; }
uint16_t motionCurrentPulse(uint8_t ch) { return (ch < NUM_CH) ? axes[ch].currentPulse : 0; }
float motionTargetDeg(uint8_t ch) { return (ch < NUM_CH) ? axes[ch].targetDeg : 0; }
uint8_t motionMoving(uint8_t ch) { return (ch < NUM_CH) ? axes[ch].moving : 0; }
unsigned long motionMillisNow() { return millis(); }

void motionTick() {
  unsigned long now = millis();
  if (now - lastTickMs < 10) return;
  lastTickMs = now;

  // watchdog
  if (watchdogMs > 0 && (now - lastKeepaliveMs) > watchdogMs) {
    motionEstop();
    // broadcast handled by main? We'll set a flag via static; main checks. 
    // We'll just estop; router broadcasts? Simpler: print here to both.
    Serial.println("{\"event\":\"WATCHDOG\",\"oe\":\"HIGH\"}");
    Serial1.println("{\"event\":\"WATCHDOG\",\"oe\":\"HIGH\"}");
    watchdogMs = 0; // disable until re-enabled? spec: neutralize; keep latched. Disable watchdog to avoid repeat spam.
  }

  // mirrors + motion advance
  for (uint8_t i = 0; i < NUM_CH; i++) {
    // timed continuous restore
    if (axes[i].timedActive && now >= axes[i].timedEndMs) {
      axes[i].timedActive = 0;
      axes[i].currentPulse = axes[i].timedRestorePulse;
      if (!estopLatched) pcaSetPulse(i, axes[i].timedRestorePulse);
    }

    // positional advance
    if (axes[i].moving && calibCh(i).mode == MODE_POS) {
      float start = axes[i].moveStartDeg;
      float target = axes[i].moveTargetDeg;
      unsigned long dur = axes[i].moveDurationMs;
      if (dur == 0) dur = 300;
      unsigned long elapsed = now - axes[i].moveStartMs;
      float t = (float)elapsed / (float)dur;
      if (t >= 1.0f) {
        t = 1.0f;
        axes[i].moving = 0;
      }
      float cur = start + (target - start) * t;
      axes[i].currentDeg = cur;
      axes[i].targetDeg = target;
      uint16_t p = degToPulse(i, cur);
      if (p != axes[i].currentPulse) {
        axes[i].currentPulse = p;
        if (!estopLatched) pcaSetPulse(i, p);
      }
    }
  }

  // mirror channels recompute
  for (uint8_t i = 0; i < NUM_CH; i++) {
    uint8_t src = calibCh(i).mirrorSrc;
    if (src < NUM_CH) {
      // mirror: currentDeg/pulse from src, inverted if calib invert
      float sdeg = axes[src].currentDeg;
      if (calibCh(i).invert) {
        float mid = (calibCh(i).minDeg + calibCh(i).maxDeg)/2.0f;
        // map src range to i range? Simple invert around own midpoint
        float smid = (calibCh(src).minDeg + calibCh(src).maxDeg)/2.0f;
        sdeg = mid - (sdeg - smid);
      }
      axes[i].currentDeg = sdeg;
      axes[i].targetDeg = axes[src].targetDeg;
      axes[i].moving = axes[src].moving;
      uint16_t p = degToPulse(i, sdeg);
      if (p != axes[i].currentPulse) {
        axes[i].currentPulse = p;
        if (!estopLatched) pcaSetPulse(i, p);
      }
    }
  }
}
