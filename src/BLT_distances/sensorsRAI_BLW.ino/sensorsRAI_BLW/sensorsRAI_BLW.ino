#include "Adafruit_VL53L0X.h"
#include <ArduinoBLE.h>

// Define the Service and Characteristic UUIDs
BLEService gigaService("19B10000-E8F2-537E-4F6C-D104768A1214");
// Max size increased to 30 characters so all 3 values fit
BLEStringCharacteristic gigaChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 30);

#define TRIG 9
#define ECHO 8

int duration;
float distance_US;

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define SHT_LOX1 5
#define SHT_LOX2 4

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

void setID() {
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);

  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  if (!lox1.begin(LOX1_ADDRESS)) {
    //Serial.println(F("Failed to boot first VL53L0X"));
    while (1);
  }
  delay(10);

  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  if (!lox2.begin(LOX2_ADDRESS)) {
    //Serial.println(F("Failed to boot second VL53L0X"));
    while (1);
  }
}

void read_dual_sensors() {
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
}

void setup() {
  Serial.begin(115200);

  if (!BLE.begin()) {
    //Serial.println("Error starting Bluetooth!");
    while (1);
  }

  BLE.setLocalName("ArduinoGigaR1");
  BLE.setAdvertisedService(gigaService);
  gigaService.addCharacteristic(gigaChar);
  BLE.addService(gigaService);

  BLE.advertise();
  //Serial.println("Bluetooth active, waiting for connection...");

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  setID();

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT_PULLUP);
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    //Serial.print("Connected to: ");
    //Serial.println(central.address());

    while (central.connected()) {
      // 1. Read the VL53L0X sensors
      read_dual_sensors();

      // Handle out-of-range cases (-1 is our error code)
      int dist_vl1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : -1;
      int dist_vl2 = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : -1;

      // 2. Read the ultrasonic sensor
      digitalWrite(TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG, LOW);
      duration = pulseIn(ECHO, HIGH, 30000); // 30 ms timeout

      // Speed of sound in mm/us (gives the distance in millimeters)
      distance_US = duration * 0.34 / 2;

      // 3. Build the comma-separated data packet
      String payload = String(dist_vl1) + "," + String(dist_vl2) + "," + String(distance_US, 1);

      // Print on Arduino to verify
      //Serial.println("Sending: " + payload);

      // 4. Send over Bluetooth
      gigaChar.writeValue(payload);

      delay(200); // Twice per second is a good rate for BLE
    }
   // Serial.println("Disconnected from PC");
  }
}
