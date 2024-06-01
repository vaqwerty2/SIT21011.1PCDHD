#include <ArduinoBLE.h>

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

      // Wait for 30 seconds before checking again
      delay(30000);
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
