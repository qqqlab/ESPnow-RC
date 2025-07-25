
//robotmower - tested on ESP32

//=============================================================
//Simple FOC for brushless motors

//NOTE: Simple FOC 2.3.4 compiles with arduino 3.1.3, not with 3.3.0

#include <SimpleFOC.h>

// BLDC motorL & driver instance
BLDCMotor motorL = BLDCMotor(7); //pole_pairs (number of magnets/2)
BLDCDriver3PWM driverL = BLDCDriver3PWM(5, 18, 19, 21); //in_pin1, in_pin2, in_pin3, en_pin1, en_pin2, en_pin3

BLDCMotor motorR = BLDCMotor(7); //pole_pairs (number of magnets/2)
BLDCDriver3PWM driverR = BLDCDriver3PWM(16, 4, 2, 15); //in_pin1, in_pin2, in_pin3, en_pin1, en_pin2, en_pin3


// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { 
  command.motor(&motorL, cmd);
  command.motor(&motorR, cmd); 
}


void motor_init(BLDCMotor *motor, BLDCDriver3PWM * driver) {
  // driver config
  // power supply voltage [V]
  driver->voltage_power_supply = 12;

  driver->init();

  // link the motorL and the driver
  motor->linkDriver(driver);

  // aligning voltage [V]
  motor->voltage_sensor_align = 3;

  // set motion control loop to be used
  motor->controller = MotionControlType::velocity_openloop;

  // default voltage_power_supply
  motor->voltage_limit = 2; // Volts

  // comment out if not needed
  motor->useMonitoring(Serial);

  // initialize motor
  motor->init();
  
  // align encoder and start FOC
  motor->initFOC();

  motor->target = 3; //initial target velocity 1 rad/s
  Serial.printf("Target velocity: %f rad/s\n", motor->target);
  Serial.printf("Voltage limit: %f V\n", motor->voltage_limit);  
}

void motor_setup() {
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  motor_init(&motorL, &driverL);
  motor_init(&motorR, &driverR);
  
  // add target command M
  command.add('M', doMotor, "motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
}

void motor_loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motorL.loopFOC();
  motorR.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motorL.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motorL.move() and set the motorL.target in the code
  motorL.move();
  motorR.move();

  // function intended to be used with serial plotter to monitor motorL variables
  // significantly slowing the execution down!!!!
  // motorL.monitor();

  // user communication
  command.run();
}

void motor_set(BLDCMotor *motor, float thr) {
  //BLDC motor config - change as needed
  float tgt_max = 80;  
  float vlim_max = 4.5;  
  float vlim_min = 1.5;

  if(thr > 1) thr = 1;
  if(thr < -1) thr = -1;
  if(fabs(thr)<0.05) {
    motor->voltage_limit = 0 ;
    motor->target = 0;
  }else{
    motor->voltage_limit = vlim_min + fabs( thr * (vlim_max - vlim_min)) ;
    motor->target = thr * tgt_max;
  }
}


//=============================================================
#include <ESPnowRC.h>

ESPnowRC espnowrc(true); //true = RX

uint16_t pwm[16] = {};

void rc_setup() {
  if(!espnowrc.begin()) {
    delay(100);
    ESP.restart();
  }
}

float rc_stick(float pwm, float mid, float dead) {
    if(pwm < mid - dead) {
      return (pwm - (mid - dead)) / (mid - dead);
    }else if (pwm < mid + dead) {
      return 0;
    }else {
      return (pwm - (mid + dead)) / (4095 - (mid + dead));
    }  
}

void rc_loop() {
  //left stick: throttle
  //right self-centering x-y stick: forward/reverse and turn

  if(espnowrc.update(pwm)) {
    float thr = (float)pwm[0] / 4095; //0.0-1.0
    float dir = rc_stick(pwm[1], 1900, 100);
    float fwd = rc_stick(pwm[2], 1936, 100);

    if( espnowrc.failsafe()) {thr = 0; dir = 0; fwd = 0;}
    //Serial.printf("%f %f %f\n", thr, dir, fwd);

    float r = thr * (fwd + dir);
    float l = thr * (fwd - dir);
    //Serial.printf("%f %f\n", l, r);

    motor_set(&motorR, r);
    motor_set(&motorL, l);
/*
    Serial.printf("[RX]\t0:%d\t1:%d\t2:%d\t3:%d\t4:%d\t5:%d\tfs:%d\tbound:%d\n"
      , (int)pwm[0]
      , (int)pwm[1]
      , (int)pwm[2]
      , (int)pwm[3]
      , (int)pwm[4]
      , (int)pwm[5]
      , espnowrc.failsafe()
      , espnowrc.is_bound
    );
*/

  }
}



//=============================================================
// main

void setup() {
  Serial.begin(115200);

  rc_setup();
  motor_setup();
}

void loop() {
  rc_loop();
  motor_loop();
}