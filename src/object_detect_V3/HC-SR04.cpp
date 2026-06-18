#include "HC-SR04.h"
#include "config.h"

volatile unsigned long us_inicio[NUM_US]   = {0, 0};
volatile unsigned long us_duracion[NUM_US] = {0, 0};
volatile bool          us_listo[NUM_US]    = {false, false};
uint8_t us_fallos[NUM_US]                  = {0, 0};

const uint8_t trigs[NUM_US] = {TRIG_1, TRIG_2};

void ISR_ECHO_1() {
    if (digitalRead(ECHO_1) == HIGH) us_inicio[0] = micros();
    else if (us_inicio[0] > 0) {
        us_duracion[0] = micros() - us_inicio[0];
        us_listo[0] = true;
        us_inicio[0] = 0;
    }
}

void ISR_ECHO_2() {
    if (digitalRead(ECHO_2) == HIGH) us_inicio[1] = micros();
    else if (us_inicio[1] > 0) {
        us_duracion[1] = micros() - us_inicio[1];
        us_listo[1] = true;
        us_inicio[1] = 0;
    }
}

void initAllUS() {
    pinMode(TRIG_1, OUTPUT); digitalWrite(TRIG_1, LOW);
    pinMode(TRIG_2, OUTPUT); digitalWrite(TRIG_2, LOW);
    pinMode(ECHO_1, INPUT_PULLUP);
    pinMode(ECHO_2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ECHO_1), ISR_ECHO_1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_2), ISR_ECHO_2, CHANGE);
}

void triggerAllUS() {
    for (int i = 0; i < NUM_US; i++) {
        us_listo[i] = false;
        digitalWrite(trigs[i], LOW);  delayMicroseconds(2);
        digitalWrite(trigs[i], HIGH); delayMicroseconds(10);
        digitalWrite(trigs[i], LOW);
    }
}

uint16_t getRawUS(uint8_t id) {
    if (!us_listo[id]) {
        us_fallos[id]++;
        if (us_fallos[id] >= MAX_FALLOS) us_fallos[id] = 0;
        return 0;
    }
    us_fallos[id] = 0;
    uint16_t mm = (uint16_t)((us_duracion[id] * 343UL) / 2000UL);
    if (mm < US_MIN || mm > US_MAX) return 0;
    return mm;
}