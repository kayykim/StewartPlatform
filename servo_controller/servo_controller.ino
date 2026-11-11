#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// initalization
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// #define SERVOMIN 102
// #define SERVOMAX 512

const uint8_t NUM_SERVOS = 3;
// const uint8_t SERVO_PINS[NUM_SERVOS]     = {0, 1, 2};
// const uint16_t MIN_TICK[NUM_SERVOS]      = {SERVOMAX-180, SERVOMAX-150, SERVOMAX-150}; // 332, 362, 362
// const uint16_t MAX_TICK[NUM_SERVOS]      = {SERVOMAX-40, SERVOMAX-10, SERVOMAX-10};    // 472, 502, 502
// const uint16_t NEUTRAL_TICK[NUM_SERVOS]  = {SERVOMAX-110, SERVOMAX-80, SERVOMAX-80};   // 402, 432, 432

bool newCommand = false; 
const bool DEBUG_ECHO = false;          // true = print each applied command
uint8_t rxBuffer[NUM_SERVOS];           // list we store the recieve packet angles
uint16_t tick[NUM_SERVOS];              // list we store the converted ticks
uint8_t rxCount = 0;
bool inPacket = false;

void setup(){
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);     // 50Hz
  delay(10);
  Serial.println("Arduino 3-servo controller ready");
}

void loop(){
  // check for incoming serial data 
  if (Serial.available() >= 6) { // make sure it recieved the entire packet before proceeding
      for (uint8_t i = 0; i < NUM_SERVOS; i++) {
          tick[i] = (uint16_t(Serial.read()) << 8) | Serial.read(); // ex) 01 99 01 B0 01 BB
      }
      newCommand = true;
  } 
  //update servo with new command recieved 
  if(newCommand){
    for(uint8_t i = 0; i < NUM_SERVOS; i++){ // send pwm to three motors 
      pwm.setPWM(i, 0, tick[i]);
      Serial.println(tick[i]);
    }
    newCommand = false;
  }
}