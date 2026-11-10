#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// initalization
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 102
#define SERVOMAX 512
#define NEUTRAL (SERVOMIN+SERVOMAX)/2 

void setup(){
  Serial.begin(9600);
  pwm.begin();

  // 50Hz
  pwm.setPWMFreq(50);
  delay(10);
}

void loop(){
  pwm.setPWM(0,0, SERVOMAX-110); //-60
  pwm.setPWM(1,0, SERVOMAX-80); //-30
  pwm.setPWM(2,0, SERVOMAX-80); //-30
  // max: [SERVOMAX-180, SERVOMAX-150, SERVOMAX-150]
  // neutral: [SERVOMAX-110, SERVOMAX-80, SERVOMAX-80]
  // min: [SERVOMAX-40, SERVOMAX-10, SERVOMAX-10]
}
