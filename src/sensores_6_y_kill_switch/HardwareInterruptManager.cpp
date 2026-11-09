// InterruptManager.cpp
#include "HardwareInterruptManager.hpp"

// Pins defined according to the earlier EXTI collision analysis
#define ECHO_M1 12
#define ECHO_M2 28
#define ECHO_M3 24
#define ECHO_M4 36
#define ECHO_M5 48
#define ECHO_M6 9
#define KILL_SWITCH_PIN 29

// Volatile variables to store timestamps (accessed by ISRs and main code)
volatile unsigned long startTimes[6] = {0};
volatile unsigned long echoDurations[6] = {0};
volatile uint8_t killSwitchState = 0;

// ISR for the Kill Switch
void ISR_KillSwitch() {
    // If using PULLDOWN and wiring to 3.3V (although PULLUP to GND is recommended)
    killSwitchState = (digitalRead(KILL_SWITCH_PIN) == LOW) ? 1 : 0;
}

// ISR template for one sensor (example M1)
void ISR_Echo_M1() {
    if (digitalRead(ECHO_M1) == HIGH) {
        startTimes[0] = micros(); // Pulse starts
    } else {
        echoDurations[0] = micros() - startTimes[0]; // Pulse ends
    }
}

void ISR_Echo_M2() {
    if (digitalRead(ECHO_M2) == HIGH) {
        startTimes[1] = micros(); // Pulse starts
    } else {
        echoDurations[1] = micros() - startTimes[1]; // Pulse ends
    }
}

void ISR_Echo_M3() {
    if (digitalRead(ECHO_M3) == HIGH) {
        startTimes[2] = micros(); // Pulse starts
    } else {
        echoDurations[2] = micros() - startTimes[2]; // Pulse ends
    }
}

void ISR_Echo_M4() {
    if (digitalRead(ECHO_M4) == HIGH) {
        startTimes[3] = micros(); // Pulse starts
    } else {
        echoDurations[3] = micros() - startTimes[3]; // Pulse ends
    }
}

void ISR_Echo_M5() {
    if (digitalRead(ECHO_M5) == HIGH) {
        startTimes[4] = micros(); // Pulse starts
    } else {
        echoDurations[4] = micros() - startTimes[4]; // Pulse ends
    }
}

void ISR_Echo_M6() {
    if (digitalRead(ECHO_M6) == HIGH) {
        startTimes[5] = micros(); // Pulse starts
    } else {
        echoDurations[5] = micros() - startTimes[5]; // Pulse ends
    }
}

void initInterrupts() {
    pinMode(KILL_SWITCH_PIN, INPUT_PULLDOWN); // Or INPUT_PULLUP depending on hardware
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

// Functions to safely extract the data
unsigned long getEchoTime(uint8_t sensorIndex) {
    if (sensorIndex > 5) return 0;

    // Copy the value atomically if possible, or simply return it
    noInterrupts();
    unsigned long duration = echoDurations[sensorIndex];
    echoDurations[sensorIndex] = 0; // Clear for the next cycle
    interrupts();
    return duration;
}

volatile uint8_t getKillSwitchState() {
    return killSwitchState;
}
