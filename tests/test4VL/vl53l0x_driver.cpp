#include "vl53l0x_driver.h"

// Arreglo de objetos para los sensores
Adafruit_VL53L0X sensors[NUMSENSORS];

void init_vl53l0x() {
  // 1. Apagar todos los sensores (Reset)
  for (uint8_t i = 0; i < NUMSENSORS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(10); // Esperar a que todos se apaguen por completo

  // 2. Encenderlos uno por uno y asignarles una nueva dirección I2C
  for (uint8_t i = 0; i < NUMSENSORS; i++) {
    // Encender el sensor actual
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(10); // Tiempo para que el sensor despierte

    // La dirección será secuencial: 0x30, 0x31, 0x32...
    uint8_t new_address = VL53L0X_BASE_ADDR + i;
    
    // Iniciar el sensor con su nueva dirección
    if (!sensors[i].begin(new_address)) {
      Serial.print(F("Error crítico: Fallo al iniciar el VL53L0X #"));
      Serial.println(i + 1);
      while (1); // Bucle infinito de seguridad si falla un sensor
    }
    delay(10);
  }
}

void read_vl53l0x(uint16_t* distances) {
  VL53L0X_RangingMeasurementData_t measure;

  for (uint8_t i = 0; i < NUMSENSORS; i++) {
    sensors[i].rangingTest(&measure, false); // true para debug

    if (measure.RangeStatus != 4) { // Status 4 = Fuera de rango
      distances[i] = measure.RangeMilliMeter;
    } else {
      distances[i] = 0xFFFF; // Marcador de error / fuera de rango
    }
  }
}