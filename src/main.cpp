#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <MQTT.h>
#include <Adafruit_AHTX0.h>
#include <String>
#include <AHT10.h>

#include "creds.h"

WiFiClient wifi;
MQTTClient mqtt;

const int heart_beat_interval = 1*60*1000;  // 1 minute
const int measure_interval = 5*60*1000;     // 1 minute

unsigned long lastHBmillis = heart_beat_interval;
unsigned long lastMeasMillis = measure_interval;

String topic_temp = String(mqtt_user) + String("/temp");
String topic_humid = String(mqtt_user) + String("/humid");

float valueTH = 0;
bool ahtState = true;
AHT10 myAHT10(0x38);

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!mqtt.connect(mqtt_user, mqtt_user, mqtt_pass)) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected!");

  if (!myAHT10.begin(0, 2)) {
    Serial.println("Could not find AHT? Check wiring");
    ahtState = false;
  }
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  mqtt.begin(mqtt_server, mqtt_port, wifi);
  mqtt.onMessage(messageReceived);
  connect();
}

void loop() {
  mqtt.loop();
  delay(10);  // <- fixes some issues with WiFi stability
  if (!mqtt.connected()) connect();

  if (millis() - lastHBmillis > heart_beat_interval) {
    lastHBmillis = millis();
    mqtt.publish(mqtt_user, "heartBeat");
  }

  if (millis() - lastMeasMillis > measure_interval) {
    lastMeasMillis = millis();
    if (ahtState) {
      valueTH = myAHT10.readTemperature(AHT10_FORCE_READ_DATA);
      if (valueTH != AHT10_ERROR) {
        mqtt.publish(topic_temp.c_str(), String(valueTH).c_str());
      } else {
        mqtt.publish(topic_temp.c_str(), "-0.0");
      }
      valueTH = myAHT10.readHumidity(AHT10_USE_READ_DATA);
      if (valueTH != AHT10_ERROR) {
        mqtt.publish(topic_humid.c_str(), String(valueTH).c_str());
      } else {
        mqtt.publish(topic_humid.c_str(), "-0.0");
      }
    }
  }
}