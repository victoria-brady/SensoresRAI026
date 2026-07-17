#ifndef CONFIG_H
#define CONFIG_H

// --- Configuración Global ---
#define NUM_VL53L0X 6
#define NUM_HCSR04  6

// --- Configuración VL53L0X (I2C) ---
const int XSHUT_PINS[NUM_VL53L0X] = {2, 3, 4, 5, 6, 7};
const uint8_t VL53L0X_BASE_ADDR = 0x30; 

// --- Configuración HC-SR04 (Ultrasonido) ---
// Asegurar el uso de pines digitales libres de conflictos
const int TRIG_PINS[NUM_HCSR04] = {8, 13, 26, 34, 40, 52};
const int ECHO_PINS[NUM_HCSR04] = {9, 12, 24, 28, 36, 50};

// D9   9
// D12  11
// D24  12
// D28  15
// D36  06
// D50  11

// Timeout de 30ms (~5 metros) para no bloquear el procesador si no hay eco
#define US_TIMEOUT 30000 

#endif