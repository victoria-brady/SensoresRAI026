/*
   =======================================================
    Measuring code for two HC-SR04 and four VL53L0X (ToF)
    using Arduino Giga R1
  =======================================================
*/ 
#include "Adafruit_VL53L0X.h"

// Ultrasonic Pins
#define TRIG1 9
#define ECHO1 8
#define TRIG2 12
#define ECHO2 13
float dist1_US, dist2_US;

// ToF I2C Addresses
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33

// ToF Shutdown Pins (XSHUT)
#define SHT_LOX1 5
#define SHT_LOX2 4
#define SHT_LOX3 3
#define SHT_LOX4 2

// Objects and measurement data
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1, measure2, measure3, measure4;

void setID() {
  // Initialize all shutdown pins
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);

  // Reset all sensors
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  delay(10);

  // All unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  // Initialize sensors sequentially to assign unique addresses
  // Wake LOX1, keep others in shutdown
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  if (!lox1.begin(LOX1_ADDRESS)) while(1); 
  delay(10);

  // Wake LOX2
  digitalWrite(SHT_LOX2, HIGH);
  if (!lox2.begin(LOX2_ADDRESS)) while(1);
  delay(10);

  // Wake LOX3
  digitalWrite(SHT_LOX3, HIGH);
  if (!lox3.begin(LOX3_ADDRESS)) while(1);
  delay(10);

  // Wake LOX4
  digitalWrite(SHT_LOX4, HIGH);
  if (!lox4.begin(LOX4_ADDRESS)) while(1);
}

void read_ToF_sensors() {
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  lox4.rangingTest(&measure4, false);

  Serial.print("\nVL1: ");
  if(measure1.RangeStatus != 4) Serial.print(measure1.RangeMilliMeter/10.0); else Serial.print("---");
  
  Serial.print("\tVL2: ");
  if(measure2.RangeStatus != 4) Serial.print(measure2.RangeMilliMeter/10.0); else Serial.print("---");
  
  Serial.print("\tVL3: ");
  if(measure3.RangeStatus != 4) Serial.print(measure3.RangeMilliMeter/10.0); else Serial.print("---");
  
  Serial.print("\tVL4: ");
  if(measure4.RangeStatus != 4) Serial.print(measure4.RangeMilliMeter/10.0); else Serial.print("---");
}

float get_US_distance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH);
  return duration * 0.034 / 2; // Returns distance in cm
}

void setup() {
  Serial.begin(115200);
  //while (!Serial) delay(1); //Waits serial monitor opening

  setID(); // Assign I2C addresses

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT_PULLUP);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT_PULLUP);
  
  Serial.println(F("System Ready..."));
}

void loop() {
  read_ToF_sensors();
  
  dist1_US = get_US_distance(TRIG1, ECHO1);
  dist2_US = get_US_distance(TRIG2, ECHO2);
  
  Serial.print("\tUS1: " + String(dist1_US));
  Serial.print("\tUS2: " + String(dist2_US) + "\n");
  
  delay(100);
}