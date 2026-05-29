#include "filter.h"
#include "config.h"
#include <string.h>

uint16_t buf_vl[NUM_VL][MEDIAN_N] = {0};
uint16_t buf_us[NUM_US][MEDIAN_N] = {0};
uint8_t  buf_idx_vl[NUM_VL]       = {0};
uint8_t  buf_idx_us[NUM_US]       = {0};

// Función estática privada para calcular media y desvío estándar
static uint16_t calculateStdDevFilter(uint16_t *buf, float max_std_dev) {
    float sum = 0;
    uint8_t n = 0;

    // 1. Filtrar errores (0) y sumar los valores válidos
    for (uint8_t i = 0; i < FILTER_N; i++) {
        if (buf[i] > 0) {
            sum += buf[i];
            n++;
        }
    }

    // Si todo el buffer está lleno de errores (0)
    if (n == 0) return 0;

    // Si solo hay un valor válido en el historial, no hay desvío posible
    if (n == 1) return (uint16_t)sum;

    // 2. Calcular la media (promedio)
    float mean = sum / n;

    // 3. Calcular la varianza
    float variance = 0;
    for (uint8_t i = 0; i < FILTER_N; i++) {
        if (buf[i] > 0) {
            variance += (buf[i] - mean) * (buf[i] - mean);
        }
    }
    variance /= n;

    // 4. Calcular desviación estándar
    float std_dev = sqrt(variance);

    // 5. Evaluar si los datos son caóticos (pico de error)
    if (std_dev > max_std_dev) {
        return 0; // Amortiguar a 0 como solicitaste
    }

    // 6. Si los datos son estables, retornar la media filtrada
    return (uint16_t)mean;
}

uint16_t applyFilterVL(uint8_t id, uint16_t rawValue) {
    // Sobrescribir el dato más antiguo en el buffer circular
    buf_vl[id][buf_idx_vl[id] % FILTER_N] = rawValue;
    buf_idx_vl[id]++;
    
    // Aplicar filtro con el límite específico para el VL53L0X
    return calculateStdDevFilter(buf_vl[id], MAX_STD_DEV_VL);
}

uint16_t applyFilterUS(uint8_t id, uint16_t rawValue) {
    // Sobrescribir el dato más antiguo en el buffer circular
    buf_us[id][buf_idx_us[id] % FILTER_N] = rawValue;
    buf_idx_us[id]++;
    
    // Aplicar filtro con el límite específico para el HC-SR04
    return calculateStdDevFilter(buf_us[id], MAX_STD_DEV_US);
}