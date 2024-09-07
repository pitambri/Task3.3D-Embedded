#include <ArduinoMqttClient.h>
#include <HCSR04.h>

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

// Wi-Fi and MQTT credentials
char wifiSSID[] = "iPhone";            
char wifiPassword[] = "Pitam@2217"; 
const int trigPin = 7;
const int echoPin = 6;

UltraSonicDistanceSensor distanceSensor(trigPin, echoPin);

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char mqttBroker[] = "broker.hivemq.com";
int mqttPort = 1883;
const char mqttTopic1[] = "SIT210/waves";
const char mqttTopic2[] = "SIT210/pats";

const long measurementInterval = 1000;
unsigned long previousMillis = 0;

// Function to connect to Wi-Fi
void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi SSID: ");
  Serial.println(wifiSSID);
  while (WiFi.begin(wifiSSID, wifiPassword) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }
  Serial.println("Connected to the Wi-Fi");
}

// Function to connect to the MQTT broker
void connectToMQTT() {
  Serial.print("Connecting to MQTT broker ");
  Serial.println(mqttBroker);
  while (!mqttClient.connect(mqttBroker, mqttPort)) {
    Serial.print("MQTT connection failed, error = ");
    Serial.println(mqttClient.connectError());
    delay(5000);
  }
  Serial.println("Connected to MQTT broker.");
}

// Function to measure distance and publish MQTT message
void checkAndPublishWave() {
  float distance = distanceSensor.measureDistanceCm();
  if (distance > 0 && distance < 20) {
    mqttClient.beginMessage(mqttTopic1);
    mqttClient.print("Pitambri: Wave detected at ");
    mqttClient.print("Distance: ");
    mqttClient.print(distance);
    mqttClient.print(" cm");
    mqttClient.endMessage();
    delay(1000);
  }
}

// Function to measure distance and publish MQTT message
void checkAndPublishPat() {
  float distance = distanceSensor.measureDistanceCm();
  if (distance > 20 && distance < 40) {
    mqttClient.beginMessage(mqttTopic2);
    mqttClient.print("Pitambri: Pat detected at ");
    mqttClient.print("Distance: ");
    mqttClient.print(distance);
    mqttClient.print(" cm");
    mqttClient.endMessage();
    delay(1000);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  connectToWiFi();
  connectToMQTT();
}

void loop() {
  mqttClient.poll();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= measurementInterval) {
    previousMillis = currentMillis;
    checkAndPublishWave();
    checkAndPublishPat();
  }
}
