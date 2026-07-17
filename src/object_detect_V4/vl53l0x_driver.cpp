#include "vl53l0x_driver.h"

Adafruit_VL53L0X loxArray[NUM_VL53L0X];

void init_VL53L0X() {
  // 1. Apagar todos los sensores (Reset)
  for (int i = 0; i < NUM_VL53L0X; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(10);
  
  // 2. Encenderlos de a uno y cambiar su dirección I2C
  for (int i = 0; i < NUM_VL53L0X; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(10); // Tiempo para que el sensor despierte
    
    uint8_t new_address = VL53L0X_BASE_ADDR + i;
    
    if (!loxArray[i].begin(new_address)) {
      Serial.print(F("Fallo al iniciar el VL53L0X #"));
      Serial.println(i);
      // No bloqueamos con while(1) para permitir que el resto del sistema corra
    } else {
      Serial.print(F("VL53L0X #"));
      Serial.print(i);
      Serial.print(F(" iniciado en ID 0x"));
      Serial.println(new_address, HEX);
    }
  }
}

int read_VL53L0X(int index) {
  if (index < 0 || index >= NUM_VL53L0X) return -1;
  
  VL53L0X_RangingMeasurementData_t measure;
  loxArray[index].rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) {  // 4 indica fuera de rango
    return measure.RangeMilliMeter;
  }
  return -1; // -1 indica error o fuera de rango
}