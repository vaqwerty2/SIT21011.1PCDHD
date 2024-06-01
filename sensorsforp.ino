#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <ArduinoBLE.h>

// WiFi settings
const char* ssid = "Vidul";  // Replace with your WiFi SSID
const char* password = "11223344";  // Replace with your WiFi password

// MQTT Broker settings
const char* broker = "broker.emqx.io";
const int port = 1883;
const char* topic_soil = "greenhouse/soil";
const char* topic_water = "greenhouse/water";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// BLE settings
const int soilMoisturePin = A0; // Pin for soil moisture sensor
const int waterSensorPin = A1;  // Pin for MH water sensor

const int SOIL_MOISTURE_THRESHOLD = 500;
const int WATER_SENSOR_THRESHOLD = 500;

BLEService sensorService("180F"); // BLE Service
BLEByteCharacteristic soilMoistureCharacteristic("2A19", BLERead | BLENotify);
BLEByteCharacteristic waterSensorCharacteristic("2A6E", BLERead | BLENotify);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Print the local MAC address
  String macAddress = BLE.address();
  Serial.print("MAC Address: ");
  Serial.println(macAddress);

  BLE.setLocalName("Nano33IoT-Sensors");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(soilMoistureCharacteristic);
  sensorService.addCharacteristic(waterSensorCharacteristic);
  BLE.addService(sensorService);
  BLE.advertise();

  Serial.println("BLE device is now advertising...");

  // Connect to WiFi
  connectToWiFi();
  
  // Connect to MQTT broker
  connectToMQTT();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      // Read the soil moisture sensor
      int soilMoistureValue = analogRead(soilMoisturePin);
      Serial.print("Soil Moisture Value: ");
      Serial.println(soilMoistureValue);

      // Read the water sensor
      int waterSensorValue = analogRead(waterSensorPin);
      Serial.print("Water Sensor Value: ");
      Serial.println(waterSensorValue);

      // Check thresholds and send message if passed
      if (soilMoistureValue < SOIL_MOISTURE_THRESHOLD) {
        soilMoistureCharacteristic.writeValue(1);
        Serial.println("Soil Moisture: LOW");
      } else {
        soilMoistureCharacteristic.writeValue(0);
        Serial.println("Soil Moisture: OK");
      }

      if (waterSensorValue > WATER_SENSOR_THRESHOLD) {
        waterSensorCharacteristic.writeValue(1);
        Serial.println("Water Sensor: HIGH");
      } else {
        waterSensorCharacteristic.writeValue(0);
        Serial.println("Water Sensor: OK");
      }

      // Publish sensor values to MQTT broker
      publishToMQTT(soilMoistureValue, waterSensorValue);

      // Wait for 30 seconds before checking again
      delay(30000);
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  while (WiFi.begin(ssid, password) != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

void connectToMQTT() {
  Serial.print("Connecting to MQTT broker");
  while (!mqttClient.connect(broker, port)) {
    Serial.print(".");
    delay(5000);
  }
  Serial.println("Connected to MQTT broker");
}

void publishToMQTT(int soilMoistureValue, int waterSensorValue) {
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.poll();
  
  String soilPayload = String(soilMoistureValue);
  String waterPayload = String(waterSensorValue);
  
  mqttClient.beginMessage(topic_soil);
  mqttClient.print(soilPayload);
  mqttClient.endMessage();
  
  mqttClient.beginMessage(topic_water);
  mqttClient.print(waterPayload);
  mqttClient.endMessage();
  
  Serial.println("Published sensor values to MQTT");
}
