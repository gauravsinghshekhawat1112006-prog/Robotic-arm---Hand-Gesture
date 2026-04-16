#include <ESP32Servo.h>

// ================= SERVOS =================
Servo servoBase;
Servo servoElbow;
Servo gripper;

const int servoBasePin = 18;
const int servoElbowPin = 19;
const int gripperPin = 21;

// ================= STEPPER =================
const int PUL = 12;
const int DIR = 13;

const int stepsPerRev = 3200;

// ================= PROTOCOL =================
const byte SYNC_BYTE = 0xFF;

void setup() {
  Serial.begin(115200);

  // ESP32 PWM setup
  servoBase.setPeriodHertz(50);
  servoElbow.setPeriodHertz(50);
  gripper.setPeriodHertz(50);

  servoBase.attach(servoBasePin, 500, 2400);
  servoElbow.attach(servoElbowPin, 500, 2400);
  gripper.attach(gripperPin, 500, 2400);

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

  // Home position
  servoBase.write(90);
  servoElbow.write(90);
  gripper.write(90);
}

// ================= STEPPER FUNCTION =================
void moveStepper(float angle, int dir) {

  digitalWrite(DIR, dir);

  int steps = round((angle / 360.0) * stepsPerRev);

  for(int i = 0; i < steps; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(200);
    digitalWrite(PUL, LOW);
    delayMicroseconds(200);
  }
}

// ================= MAIN LOOP =================
void loop() {

  if (Serial.available()) {

    byte incoming = Serial.read();

    if (incoming == SYNC_BYTE) {

      while (Serial.available() < 6);

      byte a1 = Serial.read();
      byte a2 = Serial.read();
      byte g  = Serial.read();

      byte step_angle = Serial.read();
      byte step_dir   = Serial.read();

      byte checksum = Serial.read();

      if ((a1 + a2 + g + step_angle + step_dir) % 256 == checksum) {

        // --- Execute together ---
        servoBase.write(a1);
        servoElbow.write(a2);
        gripper.write(g);

        moveStepper(step_angle * 2.0, step_dir);
      }
    }
  }
}
