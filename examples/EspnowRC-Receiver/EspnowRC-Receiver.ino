#include "ESPnowRC.h"

ESPnowRC espnowrc(true); //true = RX

uint16_t pwm[16] = {};

void setup() {
  Serial.begin(115200);

  if(!espnowrc.begin()) {
    delay(100);
    ESP.restart();
  }
}

int cnt = 0;
uint32_t ts = 0;

void loop() {
  if(espnowrc.update(pwm) ||  micros() - ts > 500000) {
    volatile uint32_t now = micros();
    Serial.printf("[RX]\t0:%d\t1:%d\t2:%d\t3:%d\t4:%d\t5:%d\tfs:%d\tbound:%d\tts:%d\tdt:%6d\n"
      , (int)pwm[0]
      , (int)pwm[1]
      , (int)pwm[2]
      , (int)pwm[3]
      , (int)pwm[4]
      , (int)pwm[5]
      , espnowrc.failsafe()
      , espnowrc.is_bound
      , (int)now
      , (int)(now - ts)
    );
    ts = now;
  }
}
