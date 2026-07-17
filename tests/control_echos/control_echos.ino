#include "config.h"

// Variables volátiles para indicar si la interrupción se disparó
volatile bool echoReceived[NUM_HCSR04] = {false, false, false, false, false, false};

// Mbed OS requiere funciones ISR estáticas individuales (no aceptan parámetros)
void isr_echo_0() { echoReceived[0] = true; }
void isr_echo_1() { echoReceived[1] = true; }
void isr_echo_2() { echoReceived[2] = true; }
void isr_echo_3() { echoReceived[3] = true; }
void isr_echo_4() { echoReceived[4] = true; }
void isr_echo_5() { echoReceived[5] = true; }

// Array de punteros a las funciones ISR para facilitar la inicialización
void (*isr_functions[NUM_HCSR04])() = {
  isr_echo_0, isr_echo_1, isr_echo_2, 
  isr_echo_3, isr_echo_4, isr_echo_5
};

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Esperar a que abra el monitor serie

  Serial.println("\n--- INICIANDO TEST DE CONFLICTOS EXTI (ECHO) ---");
  Serial.println("Si la placa se congela o parpadea el LED rojo, el ULTIMO pin impreso tiene conflicto.");
  delay(2000);

  for (int i = 0; i < NUM_HCSR04; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    digitalWrite(TRIG_PINS[i], LOW);
    
    // Configurar pin de ECHO con PULLUP para proteger de los 5V mediante el diodo
    pinMode(ECHO_PINS[i], INPUT_PULLUP);
    
    Serial.print("Intentando habilitar interrupcion en pin ECHO: ");
    Serial.println(ECHO_PINS[i]);
    delay(100); // Pequeña pausa para asegurar que el mensaje se envíe por Serial
    
    // Si hay conflicto de línea EXTI, Mbed OS se bloqueará exactamente en esta línea:
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[i]), isr_functions[i], CHANGE);
    
    Serial.println(" -> EXITO: Sin conflictos en este pin.");
  }

  Serial.println("\n--- TEST DE CONFLICTOS SUPERADO ---");
  Serial.println("Iniciando prueba de disparos (Trigger)...");
  delay(1000);
}

void loop() {
  for (int i = 0; i < NUM_HCSR04; i++) {
    // 1. Limpiar bandera
    echoReceived[i] = false;

    // 2. Enviar pulso de Trigger
    digitalWrite(TRIG_PINS[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PINS[i], LOW);

    // 3. Esperar un momento a que el sonido vaya y vuelva (50ms máximo)
    delay(50);

    // 4. Evaluar si la interrupción funcionó
    Serial.print("Sensor US");
    Serial.print(i + 1);
    Serial.print(" (Echo Pin ");
    Serial.print(ECHO_PINS[i]);
    Serial.print("): ");

    if (echoReceived[i]) {
      Serial.println("OK - Interrupcion detectada.");
    } else {
      Serial.println("FALLO - No se detecto el Echo. (Revisa conexion o sensor dañado)");
    }
  }
  
  Serial.println("----------------------------------------");
  delay(2000); // Repetir la prueba cada 2 segundos
}