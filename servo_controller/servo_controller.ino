#include <Servo.h>

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
}
