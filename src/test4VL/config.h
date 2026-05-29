#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// CONFIGURACIÓN GLOBAL
// ==========================================

// Define la cantidad de sensores VL53L0X
#define NUMSENSORS 4 

// Configuración de Ultrasonido
#define TRIG_PIN 9
#define ECHO_PIN 8

// Configuración de los VL53L0X
#define VL53L0X_BASE_ADDR 0x30 // Dirección I2C base

// Arreglo de pines XSHUT (Debe tener tantos elementos como NUMSENSORS)
// Si cambias NUMSENSORS a 3, deberás agregar un pin extra aquí, por ejemplo: {5, 4, 3}
const uint8_t XSHUT_PINS[NUMSENSORS] = {5, 4};

#endif // CONFIG_H