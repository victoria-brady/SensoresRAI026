// InterruptManager.cpp
#include "HardwareInterruptManager.hpp"

// Pines definidos según el análisis previo de colisiones EXTI
#define ECHO_M1 12
#define ECHO_M2 28
#define ECHO_M3 24
#define ECHO_M4 36
#define ECHO_M5 48
#define ECHO_M6 9
#define KILL_SWITCH_PIN 29

// Variables volátiles para almacenar tiempos (accedidas por ISR y código principal)
volatile unsigned long startTimes[6] = {0};
volatile unsigned long echoDurations[6] = {0};
volatile uint8_t killSwitchState = 0;

// ISR para Kill Switch
void ISR_KillSwitch() {
    // Si usas PULLDOWN y conectas a 3.3V (aunque recomendamos PULLUP a GND)
    killSwitchState = (digitalRead(KILL_SWITCH_PIN) == LOW) ? 1 : 0;
}

// Plantilla ISR para un sensor (ejemplo M1)
void ISR_Echo_M1() {
    if (digitalRead(ECHO_M1) == HIGH) {
        startTimes[0] = micros(); // Inicia el pulso
    } else {
        echoDurations[0] = micros() - startTimes[0]; // Termina el pulso
    }
}

void ISR_Echo_M2() {
    if (digitalRead(ECHO_M2) == HIGH) {
        startTimes[1] = micros(); // Inicia el pulso
    } else {
        echoDurations[1] = micros() - startTimes[1]; // Termina el pulso
    }
}

void ISR_Echo_M3() {
    if (digitalRead(ECHO_M3) == HIGH) {
        startTimes[2] = micros(); // Inicia el pulso
    } else {
        echoDurations[2] = micros() - startTimes[2]; // Termina el pulso
    }
}

void ISR_Echo_M4() {
    if (digitalRead(ECHO_M4) == HIGH) {
        startTimes[3] = micros(); // Inicia el pulso
    } else {
        echoDurations[3] = micros() - startTimes[3]; // Termina el pulso
    }
}

void ISR_Echo_M5() {
    if (digitalRead(ECHO_M5) == HIGH) {
        startTimes[4] = micros(); // Inicia el pulso
    } else {
        echoDurations[4] = micros() - startTimes[4]; // Termina el pulso
    }
}

void ISR_Echo_M6() {
    if (digitalRead(ECHO_M6) == HIGH) {
        startTimes[5] = micros(); // Inicia el pulso
    } else {
        echoDurations[5] = micros() - startTimes[5]; // Termina el pulso
    }
}

void initInterrupts() {
    pinMode(KILL_SWITCH_PIN, INPUT_PULLDOWN); // O INPUT_PULLUP según hardware
    attachInterrupt(digitalPinToInterrupt(KILL_SWITCH_PIN), ISR_KillSwitch, CHANGE);

    pinMode(ECHO_M1, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ECHO_M1), ISR_Echo_M1, CHANGE);

    pinMode(ECHO_M2, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ECHO_M2), ISR_Echo_M2, CHANGE);

    pinMode(ECHO_M3, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ECHO_M3), ISR_Echo_M3, CHANGE);

    pinMode(ECHO_M4, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ECHO_M4), ISR_Echo_M4, CHANGE);

    pinMode(ECHO_M5, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ECHO_M5), ISR_Echo_M5, CHANGE);

    pinMode(ECHO_M6, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ECHO_M6), ISR_Echo_M6, CHANGE);

}

// Funciones para extraer los datos de forma segura
unsigned long getEchoTime(uint8_t sensorIndex) {
    if (sensorIndex > 5) return 0;
    
    // Copiar el valor atómicamente si es posible o simplemente retornarlo
    noInterrupts();
    unsigned long duration = echoDurations[sensorIndex];
    echoDurations[sensorIndex] = 0; // Limpiar para el siguiente ciclo
    interrupts();
    return duration;
}

volatile uint8_t getKillSwitchState() {
    return killSwitchState;
}