#include <WiFi.h>
#include "esp_wpa2.h" 
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// --- Wi-Fi credentials ---
#define WIFI_SSID     "IITR_WIFI"   
#define EAP_IDENTITY  "esp32"          
#define EAP_USERNAME  "25116038"          
#define EAP_PASSWORD  "Gauravss@07"    

WebSocketsServer webSocket = WebSocketsServer(81);

// --- Servo objects ---
Servo servoBase;
Servo servoElbow;
Servo gripper;

// --- Pins ---
const int servoBasePin = 18;
const int servoElbowPin = 19;
const int gripperPin = 21;
// Stepper pins 
const int PUL = 13;
const int DIR = 12;

const int stepsPerRev = 3200;

// 🔵 Command from OpenCV
int stepperCommand = 0; // 0 = stop, 1 = CW, 2 = CCW

// -------------------- STEPPER --------------------

void stepperPosition(float angle, int dir) {
  digitalWrite(DIR, dir);

  int steps = (angle / 360.0) * stepsPerRev;

  for(int i = 0; i < steps; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(200);
    digitalWrite(PUL, LOW);
    delayMicroseconds(200);
  }
}

// Continuous motion (gesture control)
void stepperMove(int dir) {

  if(dir == 0) return;  // 🔥 STOP

  if(dir == 1){
    digitalWrite(DIR, HIGH); // CW
  }
  else if(dir == 2){
    digitalWrite(DIR, LOW);  // CCW
  }

  delayMicroseconds(10); // 🔥 direction settle time

  digitalWrite(PUL, HIGH);
  delayMicroseconds(800);  // speed control
  digitalWrite(PUL, LOW);
  delayMicroseconds(800);
}

// -------------------- WEBSOCKET --------------------

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if(type == WStype_TEXT) {

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print("JSON Parse failed: ");
      Serial.println(error.f_str());
      return;
    }

    float x = doc[0]; 
    float y = doc[1];
    float z = doc[2];
    float command = doc[3];
// Move servos
      servoBase.write(x);
      servoElbow.write(y);
      gripper.write(z);

      Serial.println("Servo Updated:");
      Serial.print("Base: "); Serial.print(x);
      Serial.print(" | Elbow: "); Serial.print(y);
      Serial.print(" | Gripper: "); Serial.println(z);


    // Serial.print("X: "); Serial.print(x);
    // Serial.print(" | Y: "); Serial.print(y);
    // Serial.print(" | Z: "); Serial.print(z);
      Serial.print(" | CMD: "); Serial.println(command);

    stepperCommand = (int)command;
  }
}

// -------------------- SETUP --------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting ESP32 WPA2-Enterprise connection...");

  WiFi.mode(WIFI_STA);
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(WIFI_SSID);

  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // --- Servo Setup ---
  servoBase.attach(servoBasePin, 500, 2400);
  servoElbow.attach(servoElbowPin, 500, 2400);
  gripper.attach(gripperPin, 500, 2400);

  // Home position
  servoBase.write(90);
  servoElbow.write(90);
  gripper.write(90);

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  Serial.println("WebSocket server started on port 81");

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
}

// -------------------- LOOP --------------------

void loop() {
  webSocket.loop();

  stepperMove(stepperCommand);

  delay(3);  // 🔥 stabilizes timing
}