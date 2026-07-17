#include "config.h"
#include "vl53l0x_driver.h"
#include "hcsr04_driver.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Esperar conexión por USB
  
  Serial.println(F("\n--- Iniciando Sistema de 12 Sensores ---"));
  
  init_VL53L0X();
  init_HCSR04();
  
  Serial.println(F("--- Setup Completado ---\n"));
}

void loop() {
  Serial.println(F("========================================"));
  
  // 1. Leer e imprimir Sensores de Infrarrojo (VL53L0X)
  Serial.print(F("VL53L0X (mm) -> "));
  for (int i = 0; i < NUM_VL53L0X; i++) {
    int dist_ir = read_VL53L0X(i);
    Serial.print(F("S")); 
    Serial.print(i); 
    Serial.print(F(": "));
    
    if (dist_ir != -1) {
      Serial.print(dist_ir);
    } else {
      Serial.print(F("ERR"));
    }
    Serial.print(F("\t"));
  }
  Serial.println();

  // 2. Leer e imprimir Sensores de Ultrasonido (HC-SR04)
  Serial.print(F("HC-SR04 (cm) -> "));
  for (int i = 0; i < NUM_HCSR04; i++) {
    float dist_us = read_HCSR04(i);
    Serial.print(F("S")); 
    Serial.print(i); 
    Serial.print(F(": "));
    
    if (dist_us > 0) {
      Serial.print(dist_us, 1);
    } else {
      Serial.print(F("ERR"));
    }
    Serial.print(F("\t"));
  }
  Serial.println();
  
  // Intervalo no bloqueante sugerido de 250ms entre lecturas completas
  delay(250); 
}