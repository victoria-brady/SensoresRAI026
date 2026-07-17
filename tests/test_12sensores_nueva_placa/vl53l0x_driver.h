#ifndef VL53L0X_DRIVER_H
#define VL53L0X_DRIVER_H

#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include "config.h"

// Inicializa todos los sensores VL53L0X
void init_VL53L0X();

// Lee un sensor específico y devuelve la distancia en mm (-1 si hay error)
int read_VL53L0X(int index);

#endif