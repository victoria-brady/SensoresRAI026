#include "Adafruit_VL53L0X.h"
//un ultrasonico y un lase VL53L0X
#define TRIG1 7 
#define ECHO1 6// name pin

int duration1;
float distance1;


Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("VL: "); Serial.print(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }

  digitalWrite(TRIG1, 0);
  delayMicroseconds(2);
  digitalWrite(TRIG1, 1);
  delayMicroseconds(10);
  digitalWrite(TRIG1, 0);
  duration1 = pulseIn(ECHO1,1);
  distance1 = duration1*0.34/2;

  Serial.print("\t US1: " + String(distance1) + "\t);    
  delay(100);
}



