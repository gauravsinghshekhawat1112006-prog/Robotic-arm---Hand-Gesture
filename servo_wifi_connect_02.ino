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

// --- WebSocket server ---
WebSocketsServer webSocket = WebSocketsServer(81);

// --- Servo objects ---
Servo servoBase;
Servo servoElbow;
Servo gripper;

// --- Pins ---
const int servoBasePin = 5;
const int servoElbowPin = 6;
const int gripperPin = 9;

// --- WebSocket Event Handler ---
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  
  if (type == WStype_TEXT) {

    StaticJsonDocument<200> doc;

    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print("JSON Parse failed: ");
      Serial.println(error.f_str());
      return;
    }

    // Extract values
    int angle1 = doc[0];
    int angle2 = doc[1];
    int g      = doc[2];
    int checksum = doc[3];

    // Checksum validation
    if ((angle1 + angle2 + g) % 256 == checksum) {

      // Constrain for safety
      angle1 = constrain(angle1, 0, 180);
      angle2 = constrain(angle2, 0, 180);
      g      = constrain(g, 0, 180);

      // Move servos
      servoBase.write(angle1);
      servoElbow.write(angle2);
      gripper.write(g);

      Serial.println("Servo Updated:");
      Serial.print("Base: "); Serial.print(angle1);
      Serial.print(" | Elbow: "); Serial.print(angle2);
      Serial.print(" | Gripper: "); Serial.println(g);

    } else {
      Serial.println("Checksum FAILED ");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting ESP32 WPA2-Enterprise connection...");

  // --- WiFi Setup ---
  WiFi.mode(WIFI_STA);

  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_wpa2_ent_enable();

  WiFi.begin(WIFI_SSID);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
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

  // --- WebSocket Setup ---
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  Serial.println("WebSocket server started on port 81 ");
}

void loop() {
  webSocket.loop(); // keep connection alive
}