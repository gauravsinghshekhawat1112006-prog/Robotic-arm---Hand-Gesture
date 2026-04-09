#include <Servo.h>
#include <AccelStepper.h>

// Define Servo Objects
Servo servoBase;
Servo servoElbow;
Servo gripper;

// Pins
const int servoBasePin = 5;
const int servoElbowPin = 6;
const int gripperPin = 9;

// Stepper Pins (TB6600)
const int pulPin = 3;
const int dirPin = 4;
const int ENA_PIN = 7;   // FIXED (was conflicting with servo)

// AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, pulPin, dirPin);

// Sync Byte
const byte SYNC_BYTE = 0xFF;

// Step timing
unsigned long lastStepTime = 0;

// Microstepping
#define STEPS_PER_REV 6400
float stepsPerDegree = (float)STEPS_PER_REV / 360.0;

long currentPos = 0;

void setup() {
  Serial.begin(115200);

  servoBase.attach(servoBasePin, 500, 2400);
  servoElbow.attach(servoElbowPin, 500, 2400);
  gripper.attach(gripperPin, 500, 2400);

  pinMode(pulPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  digitalWrite(ENA_PIN, LOW);   // FIXED (enable driver)

  servoBase.write(90);
  servoElbow.write(90);

  stepper.setMaxSpeed(20000);
  stepper.setAcceleration(8000);
}

void loop() {

  stepper.run();

  while (Serial.available() >= 1) {

    byte incoming = Serial.read();

    if (incoming == SYNC_BYTE) {

      while (Serial.available() < 6);

      byte angle1 = Serial.read();
      byte angle2 = Serial.read();
      byte g = Serial.read();
      byte step1 = Serial.read();   // clockwise
      byte step2 = Serial.read();   // anticlockwise
      
      byte checksum = Serial.read();

      if ((angle1 + angle2 + g + step1 + step2) % 256 == checksum){

        servoBase.write(angle1);
        servoElbow.write(angle2);
        gripper.write(g);

        if (millis() - lastStepTime >= 10) {

          if (step1 == 1) {
            currentPos += stepsPerDegree;
            stepper.moveTo(currentPos);
            lastStepTime = millis();
          }

          else if (step2 == 1) {   // FIXED (was == 2)
            currentPos -= stepsPerDegree;
            stepper.moveTo(currentPos);
            lastStepTime = millis();
          }
        }
      }

      break; 
    }
  }
}