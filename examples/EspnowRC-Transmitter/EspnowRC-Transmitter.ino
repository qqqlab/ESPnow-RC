#include "ESPnowRC.h"

ESPnowRC espnowrc(false); //false = TX

const uint32_t interval = 10000; //1000us = 100Hz 
uint32_t ts = 0;
uint16_t pwm[16] = {};

void setup() {
  Serial.begin(115200);

  if(!espnowrc.begin()) {
    delay(100);
    ESP.restart();
  }

  analogReadResolution(12); //NOTE: ESP32 ADC has 12 bit (0-4095) resolution but is not the best - do some research on this to improve your results
}

void loop() {
  uint32_t now = micros();
  if(now - ts >= interval) {
    ts += interval; //keep exact interval between calls
    if(now - ts >= interval) ts = now; //resync

    //this is just an example for my toy radio, modify as needed
    pwm[0] = analogRead(36); //throttle
    pwm[1] = analogRead(35); //roll
    pwm[2] = analogRead(39); //pitch
    pwm[3] = analogRead(34); //yaw
    pwm[5] = analogRead(32); //a resistor ladder with momentary buttons to ground.
    if(pwm[5] < 3000) pwm[4] = (uint32_t)pwm[5] * 4095 / 3000; //flight mode - remembers the last button pressed (and expand to full range)

    Serial.printf("[TX]\t0:%d\t1:%d\t2:%d\t3:%d\t4:%d\t5:%d\tbound:%d\tlq:%d\tts:%d \n"
      , (int)pwm[0]
      , (int)pwm[1]
      , (int)pwm[2]
      , (int)pwm[3]
      , (int)pwm[4]
      , (int)pwm[5]
      , (int)espnowrc.is_bound
      , (int)espnowrc.tx_linkquality
      , (int)now
    );
    espnowrc.send(pwm);
  }
}
