#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// <<<<<<< HEAD
// // initalization
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// #define SERVOMIN 102
// #define SERVOMAX 512

// const int max_tick = SERVOMAX - 10;
// const int min_tick = SERVOMAX - 150;
// const int neutral_tick = SERVOMAX - 80;

// int targetTick = neutral_tick;
// int targetChannel = 0;
// bool newCommand = false; 

// void setup(){
//   Serial.begin(9600);
//   pwm.begin();

//   // 50Hz
//   pwm.setPWMFreq(50);
//   delay(10);
// }

// void loop(){
//   // check for incoming serial data 
//   if(Serial.available() >=3 ){
//     int highByte = Serial.read();
//     int lowByte = Serial.read();
//     int channel = Serial.read();

//     int tick = (highByte << 8) | lowByte;
//     // Validate channel 
//     // Validate tick range
//     if(tick and channel){ // 30 deg is the offset of motor1
//       targetTick = tick;
//       targetChannel = channel;
//       newCommand = true;
//   }
//   // Update servo if new command received 
//   if(newCommand){
//     //targetPwmTick = ( (targetAngle - SERVOMIN)/(SERVOMAX-SERVOMIN) ) * 180;
//     pwm.setPWM(targetChannel, 0, targetTick);
//     newCommand = false;
//   }
//   // max: [SERVOMAX-180, SERVOMAX-150, SERVOMAX-150]
//   // neutral: [SERVOMAX-110, SERVOMAX-80, SERVOMAX-80]
//   // min: [SERVOMAX-40, SERVOMAX-10, SERVOMAX-10]
// =======
// ---------------- Servo configuration (one entry per servo) ----------------
const uint8_t NUM_SERVOS = 3;
const uint8_t SERVO_PINS[NUM_SERVOS]     = {9, 10, 11};
const uint8_t MIN_ANGLE[NUM_SERVOS]      = {5, 5, 5};
const uint8_t MAX_ANGLE[NUM_SERVOS]      = {25, 25, 25};
const uint8_t NEUTRAL_ANGLE[NUM_SERVOS]  = {15, 15, 15};

Servo servos[NUM_SERVOS];

// ---------------- Serial protocol ----------------
// Each command is 4 bytes:  [0xFF] [angle1] [angle2] [angle3]
// 0xFF can never be a valid angle (max 180), so it marks the start of a
// packet and lets the parser re-synchronize if a byte is ever lost.
const uint8_t PACKET_HEADER = 0xFF;
const bool DEBUG_ECHO = false;   // true = print each applied command

uint8_t rxBuffer[NUM_SERVOS];
uint8_t rxCount = 0;
bool inPacket = false;

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(NEUTRAL_ANGLE[i]);
  }

  Serial.println("Arduino 3-servo controller ready");
}

void applyPacket() {
  // Validate every angle first so a bad packet never moves only some servos.
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (rxBuffer[i] < MIN_ANGLE[i] || rxBuffer[i] > MAX_ANGLE[i]) {
      return;  // reject the whole packet
    }
  }

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(rxBuffer[i]);
  }

  if (DEBUG_ECHO) {
    Serial.print("Angles set to: ");
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      Serial.print(rxBuffer[i]);
      Serial.print(i < NUM_SERVOS - 1 ? ", " : "\n");
    }
  }
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    if (b == PACKET_HEADER) {      // start (or restart) a packet
      inPacket = true;
      rxCount = 0;
    } else if (inPacket) {
      rxBuffer[rxCount++] = b;
      if (rxCount == NUM_SERVOS) {
        applyPacket();
        inPacket = false;
        rxCount = 0;
      }
    }
    // Bytes outside a packet are ignored.
  }
// >>>>>>> e0172e42b0256377f752e348fd25ddf4d501b378
// }
