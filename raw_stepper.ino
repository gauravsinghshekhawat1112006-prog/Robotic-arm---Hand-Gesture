const int PUL = 3;
const int DIR = 4;

const int stepsPerRev = 3200;

void setup() {
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

  Serial.begin(115200);
  Serial.println("Enter: angle direction");
  Serial.println("Example: 90 1  (1=CW, 0=CCW)");
}


void stepperPosition(float angle, int dir) {
  digitalWrite(DIR, dir);

  int steps = round((angle / 360.0) * stepsPerRev);

  for(int i = 0; i < steps; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(200);
    digitalWrite(PUL, LOW);
    delayMicroseconds(200);
  }
}

void loop() {

  if (Serial.available()) {

    String input = Serial.readStringUntil('\n');
    input.trim();  // remove spaces + \r

    int spaceIndex = input.indexOf(' ');

    if (spaceIndex > 0) {

      String angleStr = input.substring(0, spaceIndex);
      String dirStr   = input.substring(spaceIndex + 1);

      float angle = angleStr.toFloat();
      int dir = dirStr.toInt();

      Serial.print("Moving ");
      Serial.print(angle);
      Serial.print(" degrees | DIR: ");
      Serial.println(dir);

      stepperPosition(angle, dir);

      Serial.println("Done\n");
    } 
    else {
      Serial.println("Invalid input! Use: angle direction");
    }
  }
}
