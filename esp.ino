#include <Servo.h>

// Define Servo Objects
Servo servoBase;
Servo servoElbow;
Servo gripper;
// Pins - Adjust these based on your actual wiring
const int servoBasePin = 5;
const int servoElbowPin = 6;
const int gripperPin = 9;

// Sync Byte to ensure we are at the start of a packet
const byte SYNC_BYTE = 0xFF; // 255 in decimal

void setup() {
  // Use a high baud rate for low latency
  Serial.begin(115200);

  // Attach servos to their pins
  // Standard Arduino Servo library uses default pulse widths (544us to 2400us)
  // Pass min/max pulse widths in microseconds to match MG995 (500us to 2400us)
  servoBase.attach(servoBasePin, 500, 2400);
  servoElbow.attach(servoElbowPin, 500, 2400);
  gripper.attach(gripperPin, 500, 2400);

  // Move to a safe "Home" position on startup
  servoBase.write(90);
  servoElbow.write(90);
}


void loop() {

  while (Serial.available() >= 1) {

    byte incoming = Serial.read();
    // Wait until we find SYNC byte
    if (incoming == SYNC_BYTE) {
      // Now wait for remaining bytes
      while (Serial.available() < 4);

      byte angle1 = Serial.read();
      byte angle2 = Serial.read();
      byte g = Serial.read();
      byte checksum = Serial.read();

      if ((angle1 + angle2 + g) % 256 == checksum){
        servoBase.write(angle1);
        servoElbow.write(angle2);
        gripper.write(g);
      }

      break; 
    }
  }
}


// #include <Servo.h>

// Servo servoBase;
// Servo servoElbow;

// const int servoBasePin = 5;
// const int servoElbowPin = 6;

// void setup() {
//   servoBase.attach(servoBasePin, 500, 2400);
//   servoElbow.attach(servoElbowPin, 500, 2400);

//   servoBase.write(0);
//   servoElbow.write(180);
// }

// void loop() {
//   // nothing
// }