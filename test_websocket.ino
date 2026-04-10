#include <WiFi.h>
#include "esp_wpa2.h" 
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// --- Wi-Fi credentials ---
#define WIFI_SSID     "IITR_WIFI"   
#define EAP_IDENTITY  "esp32"          
#define EAP_USERNAME  "25116038"          
#define EAP_PASSWORD  "Gauravss@07"    

// --- WebSocket server on port 81 ---
WebSocketsServer webSocket = WebSocketsServer(81);



void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if(type == WStype_TEXT) {
    // 1. Create a buffer for the JSON document
    // StaticJsonDocument is efficient for small arrays like [x, y, z]
    StaticJsonDocument<200> doc;

    // 2. Deserialize (parse) the payload
    DeserializationError error = deserializeJson(doc, payload);

    // 3. Check for parsing errors (e.g., if the string was corrupted)
    if (error) {
      Serial.print("JSON Parse failed: ");
      Serial.println(error.f_str());
      return;
    }

    // 4. Extract values from the array
    // Since Python sends "[1.0, 2.0, 3.0]", doc is the array itself
    float x = doc[0]; 
    float y = doc[1];
    float z = doc[2];
    float checksum = doc[3];
    // 5. Print or use the values
    Serial.print("X: "); Serial.print(x);
    Serial.print(" | Y: "); Serial.print(y);
    Serial.print(" | Z: "); Serial.println(z);
  }
}
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32 WPA2-Enterprise connection...");

  // --- Wi-Fi setup ---
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

  // --- WebSocket setup ---
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.println("WebSocket server started on port 81");
}
 
void loop() {
  webSocket.loop(); // keep WebSocket alive
}
