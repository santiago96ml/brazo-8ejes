#include <Arduino.h>
#include <Wire.h>
#include "src/Config.h"
#include "src/Pca9685.h"
#include "src/Calibration.h"
#include "src/MotionEngine.h"
#include "src/CommandRouter.h"

// Auto broadcast state
static unsigned long lastAutoMs = 0;

static void broadcastState() {
  char outBuf[320];
  char item[64];
  snprintf(outBuf, sizeof(outBuf), "{\"status\":\"STATE\",\"t\":%lu,\"en\":%d,\"estop\":%d,\"ch\":[", millis(), motionIsEnabled()?1:0, motionIsEstop()?1:0);
  for (uint8_t i = 0; i < NUM_CH; i++) {
    const char* mstr = (calibCh(i).mode == MODE_POS) ? "P" : "C";
    snprintf(item, sizeof(item), "{\"c\":%u,\"n\":%u,\"m\":\"%s\",\"d\":%d,\"p\":%u,\"tg\":%d,\"mv\":%u}",
      i, i+1, mstr, (int)motionCurrentDeg(i), motionCurrentPulse(i), (int)motionTargetDeg(i), motionMoving(i));
    strncat(outBuf, item, sizeof(outBuf)-strlen(outBuf)-1);
    if (i < NUM_CH-1) strncat(outBuf, ",", sizeof(outBuf)-strlen(outBuf)-1);
  }
  strncat(outBuf, "]}", sizeof(outBuf)-strlen(outBuf)-1);
  Serial.println(outBuf);
  Serial1.println(outBuf);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(PIN_OE, OUTPUT);
  digitalWrite(PIN_OE, HIGH); // boot disabled

  Wire.begin();
  calibInit();
  pcaInitDevice();

  // all channels to stop pulse
  for (uint8_t i = 0; i < NUM_CH; i++) pcaSetPulse(i, calibCh(i).stopPulse);

  motionInit();

  routerInit();

  Serial.println("{\"status\":\"READY\",\"board\":\"MEGA2560_PCA9685\",\"fw\":\"1.0.0\",\"oe\":\"HIGH\"}");
  Serial1.println("{\"status\":\"READY\",\"board\":\"MEGA2560_PCA9685\",\"fw\":\"1.0.0\",\"oe\":\"HIGH\"}");
}

void loop() {
  routerPoll();
  motionTick();

  unsigned long ab = routerAutoBroadcastMs();
  if (ab > 0) {
    if (millis() - lastAutoMs >= ab) {
      lastAutoMs = millis();
      broadcastState();
    }
  }
}
