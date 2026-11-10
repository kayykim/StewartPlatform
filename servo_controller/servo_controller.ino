#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// initalization
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 102
#define SERVOMAX 512

const int max_tick = SERVOMAX - 10;
const int min_tick = SERVOMAX - 150;
const int neutral_tick = SERVOMAX - 80;

int targetTick = neutral_tick;
int targetChannel = 0;
bool newCommand = false; 

void setup(){
  Serial.begin(9600);
  pwm.begin();

  // 50Hz
  pwm.setPWMFreq(50);
  delay(10);
}

void loop(){
  // check for incoming serial data 
  if(Serial.available() >=3 ){
    int highByte = Serial.read();
    int lowByte = Serial.read();
    int channel = Serial.read();

    int tick = (highByte << 8) | lowByte;
    // Validate channel 
    // Validate tick range
    if(tick and channel){ // 30 deg is the offset of motor1
      targetTick = tick;
      targetChannel = channel;
      newCommand = true;
  }
  // Update servo if new command received 
  if(newCommand){
    //targetPwmTick = ( (targetAngle - SERVOMIN)/(SERVOMAX-SERVOMIN) ) * 180;
    pwm.setPWM(targetChannel, 0, targetTick);
    newCommand = false;
  }
  // max: [SERVOMAX-180, SERVOMAX-150, SERVOMAX-150]
  // neutral: [SERVOMAX-110, SERVOMAX-80, SERVOMAX-80]
  // min: [SERVOMAX-40, SERVOMAX-10, SERVOMAX-10]
}
