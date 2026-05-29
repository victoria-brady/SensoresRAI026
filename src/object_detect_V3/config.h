#pragma once
#include <Arduino.h>

// ─── Configuración General ───────────────────────────────────────────────────
#define LOOP_MS       50
#define FILTER_N       5
#define MAX_FALLOS     5
#define NUM_VL         4
#define NUM_US         2

// ─── Límites del Filtro Estadístico (Tolerancia a picos) ─────────────────────
#define MAX_STD_DEV_VL 50  // mm máximos de desvío tolerado para láser
#define MAX_STD_DEV_US 100 // mm máximos de desvío tolerado para ultrasonido

// ─── Pines VL53L0X ───────────────────────────────────────────────────────────
#define XSHUT_1   2
#define XSHUT_2   3
#define XSHUT_3   4
#define XSHUT_4   5

// ─── Pines HC-SR04 ───────────────────────────────────────────────────────────
#define TRIG_1    8
#define ECHO_1    9
#define TRIG_2   13
#define ECHO_2   12

// ─── Direcciones I2C VL53L0X ─────────────────────────────────────────────────
#define ADDR_1   0x30
#define ADDR_2   0x31
#define ADDR_3   0x32
#define ADDR_4   0x33

// ─── Límites de validez ──────────────────────────────────────────────────────
#define VL_MIN      20
#define VL_MAX    1200
#define VL_ERR    8190
#define US_MIN     200
#define US_MAX    3500