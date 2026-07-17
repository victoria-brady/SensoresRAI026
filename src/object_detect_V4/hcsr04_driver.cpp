#include "hcsr04_driver.h"

void init_HCSR04() {
  for (int i = 0; i < NUM_HCSR04; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    digitalWrite(TRIG_PINS[i], LOW);
    pinMode(ECHO_PINS[i], INPUT);
  }
  Serial.println(F("Sensores HC-SR04 iniciados."));
}

float read_HCSR04(int index) {
  if (index < 0 || index >= NUM_HCSR04) return -1.0;
  
  digitalWrite(TRIG_PINS[index], LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PINS[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PINS[index], LOW);
  
  // Usamos el timeout para evitar el bloqueo crónico
  long duration = pulseIn(ECHO_PINS[index], HIGH, US_TIMEOUT);
  
  if (duration == 0) {
    return -1.0; // Fuera de rango o falló la conexión
  }
  
  // Cálculo corregido con constante acústica a nivel del mar
  float distance = (duration * 0.0343) / 2.0;
  return distance;
}