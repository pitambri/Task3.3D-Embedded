#include <ArduinoMqttClient.h>

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h> 
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

// Network credentials
char wifiSSID[] = "iPhone";            
char wifiPassword[] = "Pitam@2217";     

int ledPin = 5;

// MQTT setup
WiFiSSLClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "mqtt-dashboard.com";
int brokerPort = 8883;
const char mqttTopic1[] = "SIT210/waves";
const char mqttTopic2[] = "SIT210/pats";

// Function to connect to WiFi
void connectToWiFi() {
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(wifiSSID);

  while (WiFi.begin(wifiSSID, wifiPassword) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();
}

// Function to connect to the MQTT broker
void connectToMQTTBroker() {
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, brokerPort)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
}

// Function to subscribe to the MQTT topic
void subscribeToTopic1() {
  Serial.print("Subscribing to topic: ");
  Serial.println(mqttTopic1);
  Serial.println();
  mqttClient.subscribe(mqttTopic1);
}
// Function to subscribe to the MQTT topic
void subscribeToTopic2() {
  Serial.print("Subscribing to topic: ");
  Serial.println(mqttTopic2);
  Serial.println();
  mqttClient.subscribe(mqttTopic2);
}

// Function to handle incoming MQTT messages
void handleIncomingMessage() {
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    Serial.print("Message received with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    while (mqttClient.available()) {
      Serial.print((char)mqttClient.read());
    }

    Serial.println();
    Serial.println("LED BEGINS BLINKING");

    if(mqttTopic1)
    {
      // Blink the LED
      for (int i = 0; i < 3; i++)
      {
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
      delay(500);
      }
    }
    if(mqttTopic2)
    {
      // Blink the LED
      for (int i = 0; i < 5; i++)
      {
      digitalWrite(ledPin, HIGH);
      delay(300);
      digitalWrite(ledPin, LOW);
      delay(300);
      }
    }
    

    Serial.println();
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  while (!Serial) {
    ;
  }

  connectToWiFi();
  connectToMQTTBroker();
  subscribeToTopic1();
  subscribeToTopic2();
}

void loop() {
  handleIncomingMessage();
}
