#ifndef VL53L0X_DRIVER_H
#define VL53L0X_DRIVER_H

#include "config.h"
#include "Adafruit_VL53L0X.h"

// Inicializa todos los sensores definidos en NUMSENSORS asignando direcciones I2C dinámicas
void init_vl53l0x();

// Lee los sensores y guarda los resultados en el array proporcionado
// Si un sensor está fuera de rango, devuelve 0xFFFF
void read_vl53l0x(uint16_t* distances);

#endif // VL53L0X_DRIVER_H