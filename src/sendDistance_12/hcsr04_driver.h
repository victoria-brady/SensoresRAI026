#ifndef HCSR04_DRIVER_H
#define HCSR04_DRIVER_H

#include <Arduino.h>
#include "config.h"

// Configura los pines de entrada y salida
void init_HCSR04();

// Dispara y lee el sensor, devolviendo la distancia en cm (-1.0 si falla)
float read_HCSR04(int index);

#endif