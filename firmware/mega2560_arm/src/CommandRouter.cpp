#include "CommandRouter.h"
#include "Config.h"
#include "Calibration.h"
#include "MotionEngine.h"
#include "Pca9685.h"

#define LINE_BUF_SIZE 256

static char buf0[LINE_BUF_SIZE];
static char buf1[LINE_BUF_SIZE];
static uint16_t len0 = 0, len1 = 0;

static char outBuf[320];
static unsigned long autoBroadcastMs = 0;
unsigned long routerAutoBroadcastMs() { return autoBroadcastMs; }

static void reply(uint8_t port, const char* s) {
  if (port == 0) Serial.println(s); else Serial1.println(s);
}
static void broadcast(const char* s) {
  Serial.println(s);
  Serial1.println(s);
}

static void sendErr(uint8_t port, const char* code, const char* cmd) {
  snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"ERR\",\"code\":\"%s\",\"cmd\":\"%s\"}", code, cmd);
  reply(port, outBuf);
}

static bool isMotionBlocked() { return motionIsEstop(); }

// trim trailing \r
static void trimCR(char* s) {
  int n = strlen(s);
  if (n > 0 && s[n-1] == '\r') s[n-1] = '\0';
}

static void handleLine(char* line, uint8_t port) {
  trimCR(line);
  if (line[0] == '\0') return;
  // uppercase first char
  char cmd = line[0];
  if (cmd >= 'a' && cmd <= 'z') cmd -= 32;

  // single-char no-arg
  if (cmd == 'I') {
    if (pcaProbe()) reply(port, "{\"status\":\"OK\",\"pca9685\":\"CONECTADO_0x40\"}");
    else {
      uint8_t c = pcaLastI2cCode();
      snprintf(outBuf, sizeof(outBuf), "{\"status\":\"ERROR\",\"i2c_code\":%u,\"msg\":\"Verificar SDA->Pin20 y SCL->Pin21\"}", c);
      reply(port, outBuf);
    }
    return;
  }
  if (cmd == 'V') { reply(port, "{\"status\":\"VERSION\",\"fw\":\"1.0.0\",\"board\":\"MEGA2560_PCA9685\",\"proto\":1}"); return; }
  if (cmd == 'S') {
    motionEstop();
    broadcast("{\"event\":\"EMERGENCY_STOP\",\"oe\":\"HIGH\"}");
    return;
  }
  if (cmd == 'R') {
    motionEnable();
    broadcast("{\"event\":\"ENABLED\",\"oe\":\"LOW\"}");
    return;
  }
  if (cmd == 'F') {
    motionAllNeutral();
    broadcast("{\"event\":\"ALL_NEUTRAL\"}");
    return;
  }
  if (cmd == 'H') {
    if (isMotionBlocked()) { sendErr(port, "ESTOP_LATCHED", line); return; }
    motionHomeAll();
    reply(port, "{\"ack\":\"HOME\"}");
    return;
  }
  if (cmd == 'E') {
    calibSaveEeprom();
    reply(port, "{\"ack\":\"SAVED\"}");
    return;
  }
  if (cmd == 'D') {
    motionRestoreDefaults();
    reply(port, "{\"ack\":\"DEFAULTS\"}");
    return;
  }
  if (cmd == 'K') {
    motionKeepalive();
    reply(port, "{\"ack\":\"K_OK\"}");
    return;
  }
  if (cmd == 'Q') {
    // state
    snprintf(outBuf, sizeof(outBuf), "{\"status\":\"STATE\",\"t\":%lu,\"en\":%d,\"estop\":%d,\"ch\":[", millis(), motionIsEnabled()?1:0, motionIsEstop()?1:0);
    // append 8 items
    char item[64];
    for (uint8_t i = 0; i < NUM_CH; i++) {
      const char* mstr = (calibCh(i).mode == MODE_POS) ? "P" : "C";
      snprintf(item, sizeof(item), "{\"c\":%u,\"n\":%u,\"m\":\"%s\",\"d\":%d,\"p\":%u,\"tg\":%d,\"mv\":%u}",
        i, i+1, mstr, (int)motionCurrentDeg(i), motionCurrentPulse(i), (int)motionTargetDeg(i), motionMoving(i));
      strncat(outBuf, item, sizeof(outBuf)-strlen(outBuf)-1);
      if (i < NUM_CH-1) strncat(outBuf, ",", sizeof(outBuf)-strlen(outBuf)-1);
    }
    strncat(outBuf, "]}", sizeof(outBuf)-strlen(outBuf)-1);
    reply(port, outBuf);
    return;
  }

  // W<ms> watchdog (no channel)
  if (cmd == 'W') {
    unsigned long ms = strtoul(line + 1, NULL, 10);
    motionWatchdog(ms);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"WATCHDOG\",\"ms\":%lu}", ms);
    reply(port, outBuf);
    return;
  }
  // A<ms> auto broadcast
  if (cmd == 'A') {
    unsigned long ms = strtoul(line + 1, NULL, 10);
    if (ms != 0 && ms < 100) ms = 100;
    autoBroadcastMs = ms;
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"AUTO\",\"ms\":%lu}", ms);
    reply(port, outBuf);
    return;
  }
  // X coordinated
  if (cmd == 'X') {
    if (isMotionBlocked()) { sendErr(port, "ESTOP_LATCHED", line); return; }
    // parse "X0:90,1:45,..."
    uint8_t chs[8]; float degs[8]; uint8_t n = 0;
    char tmp[LINE_BUF_SIZE];
    strncpy(tmp, line + 1, LINE_BUF_SIZE-1);
    tmp[LINE_BUF_SIZE-1] = '\0';
    char* p = tmp;
    while (*p && n < 8) {
      if (*p == ',') { p++; continue; }
      if (*p < '1' || *p > '8') { sendErr(port, "BAD_CH", line); return; }
      uint8_t motor = *p - '0';
      uint8_t ch = motor - 1;
      p++;
      if (*p != ':') { sendErr(port, "BAD_ARG", line); return; }
      p++;
      char* endp;
      float deg = strtod(p, &endp);
      if (endp == p) { sendErr(port, "BAD_ARG", line); return; }
      p = endp;
      if (ch >= NUM_CH) { sendErr(port, "BAD_CH", line); return; }
      if (calibCh(ch).mode != MODE_POS) { sendErr(port, "MODE_CONT_USE_T", line); return; }
      chs[n] = ch; degs[n] = deg; n++;
      if (*p == ',') p++;
    }
    if (n == 0) { sendErr(port, "BAD_ARG", line); return; }
    motionCoordinated(chs, degs, n);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"X_OK\",\"n\":%u}", n);
    reply(port, outBuf);
    return;
  }

  // arg commands with channel (motor number 1..8)
  int idx = 1;
  if (line[idx] < '1' || line[idx] > '8') { sendErr(port, "BAD_CH", line); return; }
  uint8_t motor = line[idx] - '0';
  uint8_t ch = motor - 1;
  idx++;

  if (cmd == 'M') {
    if (isMotionBlocked()) { sendErr(port, "ESTOP_LATCHED", line); return; }
    if (calibCh(ch).mode != MODE_POS) { sendErr(port, "MODE_CONT_USE_T", line); return; }
    // parse :deg or :deg:ms
    if (line[idx] != ':') { sendErr(port, "BAD_ARG", line); return; }
    idx++;
    char* endp;
    float deg = strtod(line + idx, &endp);
    unsigned long ms = 0;
    if (*endp == ':') {
      ms = strtoul(endp + 1, NULL, 10);
    }
    motionMovePos(ch, deg, ms);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"M_OK\",\"motor\":%u,\"deg\":%d}", motor, (int)deg);
    reply(port, outBuf);
    return;
  }
  if (cmd == 'P') {
    if (isMotionBlocked()) { sendErr(port, "ESTOP_LATCHED", line); return; }
    if (line[idx] != ':') { sendErr(port, "BAD_ARG", line); return; }
    idx++;
    uint16_t pulse = (uint16_t)strtoul(line + idx, NULL, 10);
    motionMoveRaw(ch, pulse);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"P_OK\",\"motor\":%u,\"pulse\":%u}", motor, pulse);

    reply(port, outBuf);
    return;
  }
  if (cmd == 'T') {
    if (isMotionBlocked()) { sendErr(port, "ESTOP_LATCHED", line); return; }
    if (line[idx] != ':') { sendErr(port, "BAD_ARG", line); return; }
    idx++;
    char* colon = strchr(line + idx, ':');
    if (!colon) { sendErr(port, "BAD_ARG", line); return; }
    *colon = '\0';
    int offset = atoi(line + idx);
    if (offset < -150 || offset > 150) { sendErr(port, "BAD_ARG", line); return; }
    unsigned long ms = strtoul(colon + 1, NULL, 10);
    motionTimed(ch, (int16_t)offset, ms);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"T_OK\",\"motor\":%u,\"offset\":%d,\"ms\":%lu}", motor, offset, ms);
    reply(port, outBuf);
    return;
  }
  if (cmd == 'C') {
    if (line[idx] != ':') { sendErr(port, "BAD_ARG", line); return; }
    idx++;
    uint16_t pulse = (uint16_t)strtoul(line + idx, NULL, 10);
    motionCalibrateStop(ch, pulse);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"CALIBRATED\",\"motor\":%u,\"stop\":%u}", motor, pulse);

    reply(port, outBuf);
    return;
  }
  if (cmd == 'B') {
    if (line[idx] != ':') { sendErr(port, "BAD_ARG", line); return; }
    idx++;
    char* c2 = strchr(line + idx, ':');
    if (!c2) { sendErr(port, "BAD_ARG", line); return; }
    *c2 = '\0';
    uint16_t mn = (uint16_t)strtoul(line + idx, NULL, 10);
    uint16_t mx = (uint16_t)strtoul(c2 + 1, NULL, 10);
    motionSetRange(ch, mn, mx);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"RANGE\",\"motor\":%u,\"min\":%u,\"max\":%u}", motor, mn, mx);

    reply(port, outBuf);
    return;
  }
  if (cmd == 'N') {
    if (line[idx] != ':') { sendErr(port, "BAD_ARG", line); return; }
    idx++;
    char m = line[idx];
    if (m >= 'a' && m <= 'z') m -= 32;
    uint8_t mode = (m == 'C') ? MODE_CONT : MODE_POS;
    motionSetMode(ch, mode);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"MODE\",\"motor\":%u,\"mode\":\"%c\"}", motor, (mode==MODE_POS)?'P':'C');
    reply(port, outBuf);
    return;
  }
  if (cmd == 'L') {
    if (line[idx] != ':') { sendErr(port, "BAD_ARG", line); return; }
    idx++;
    char* c2 = strchr(line + idx, ':');
    if (!c2) { sendErr(port, "BAD_ARG", line); return; }
    *c2 = '\0';
    int mn = atoi(line + idx);
    int mx = atoi(c2 + 1);
    motionSetLimits(ch, (int16_t)mn, (int16_t)mx);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"LIMITS\",\"motor\":%u,\"min\":%d,\"max\":%d}", motor, mn, mx);
    reply(port, outBuf);
    return;
  }
  if (cmd == 'Y') {
    if (line[idx] != ':') { sendErr(port, "BAD_ARG", line); return; }
    idx++;
    uint8_t srcIn = (uint8_t)strtoul(line + idx, NULL, 10);
    uint8_t src = (srcIn == 255) ? 255 : (srcIn == 0 ? 255 : srcIn - 1);
    // if user passes 0 treat as unlink? spec uses 255 unlink. Keep 255.
    if (srcIn != 255 && (srcIn < 1 || srcIn > 8)) { sendErr(port, "BAD_ARG", line); return; }
    motionSetMirror(ch, src);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"MIRROR\",\"motor\":%u,\"src\":%u}", motor, srcIn);
    reply(port, outBuf);
    return;
  }
  if (cmd == 'J') {
    motionToggleInvert(ch);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"INVERT\",\"motor\":%u,\"invert\":%u}", motor, calibCh(ch).invert);
    reply(port, outBuf);
    return;
  }
  if (cmd == 'O') {
    if (line[idx] != ':') { sendErr(port, "BAD_ARG", line); return; }
    idx++;
    uint16_t deg = (uint16_t)strtoul(line + idx, NULL, 10);
    motionSetHome(ch, deg);
    snprintf(outBuf, sizeof(outBuf), "{\"ack\":\"HOME_SET\",\"motor\":%u,\"deg\":%u}", motor, deg);
    reply(port, outBuf);
    return;
  }
  if (cmd == 'W') {
    // W<ms> no channel; but we parsed ch. Oops: W has no channel. Handle before channel parse.
    // We'll restructure: handle W separately above channel parse. But too late. Adjust: check cmd W before channel.
    // Since we already consumed a digit, wrong. Need fix: move W handling up.
    sendErr(port, "BAD_ARG", line);
    return;
  }
  if (cmd == 'A') {
    sendErr(port, "BAD_ARG", line);
    return;
  }
  if (cmd == 'X') {
    if (isMotionBlocked()) { sendErr(port, "ESTOP_LATCHED", line); return; }
    // parse list after X: ch:deg,ch:deg,...
    // line currently at idx points after channel digit we consumed incorrectly. 
    // X format starts X0:90,1:... so channel parse consumed '0'. Need handle X before channel parse too.
    sendErr(port, "BAD_ARG", line);
    return;
  }

  sendErr(port, "UNKNOWN_CMD", line);
}

// Need to handle W, A, X before channel parsing. Rework: detect in handleLine earlier.
// We'll patch by checking cmd before channel parse. But code above already channel-parsed. 
// To avoid, restructure: handle W/A/X at top. Let's instead add early handling.

void routerInit() {
  len0 = len1 = 0;
  memset(buf0, 0, sizeof(buf0));
  memset(buf1, 0, sizeof(buf1));
}

static void processChar(char c, uint8_t port) {
  char* buf = (port == 0) ? buf0 : buf1;
  uint16_t* len = (port == 0) ? &len0 : &len1;
  if (c == '\n') {
    buf[*len] = '\0';
    // copy to temp
    char tmp[LINE_BUF_SIZE];
    strncpy(tmp, buf, LINE_BUF_SIZE-1);
    tmp[LINE_BUF_SIZE-1] = '\0';
    *len = 0;
    handleLine(tmp, port);
  } else if (c == '\r') {
    // tolerate; ignore unless buffer empty? keep
  } else {
    if (*len < LINE_BUF_SIZE - 1) {
      buf[(*len)++] = c;
    } else {
      // line too long
      *len = 0;
      sendErr(port, "LINE_TOO_LONG", "");
    }
  }
}

void routerPoll() {
  while (Serial.available()) processChar(Serial.read(), 0);
  while (Serial1.available()) processChar(Serial1.read(), 1);
}
