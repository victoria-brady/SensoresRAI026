#include "config.h"
#include "vl53l0x_driver.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  Serial.println(F("===================================="));
  Serial.println(F("Iniciando Sistema de Sensores..."));
  Serial.println(F("===================================="));

  // Inicializar pines del sensor ultrasónico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLUP);

  // Inicializar driver de los VL53L0X
  init_vl53l0x();

  Serial.println(F("Todos los sensores inicializados correctamente."));
}

void loop() {
  // --- 1. Lectura de los Sensores VL53L0X ---
  uint16_t vl53l0x_distances[NUMSENSORS];
  read_vl53l0x(vl53l0x_distances);

  // Imprimir lecturas de los ToF
  for (uint8_t i = 0; i < NUMSENSORS; i++) {
    Serial.print(F("VL_"));
    Serial.print(i + 1);
    Serial.print(F(": "));
    
    if (vl53l0x_distances[i] != 0xFFFF) {
      Serial.print(vl53l0x_distances[i]);
      Serial.print(F(" mm"));
    } else {
      Serial.print(F("Out_of_range"));
    }
    Serial.print(F("\t"));
  }

  // --- 2. Retardo del ciclo ---
  delay(100);
}