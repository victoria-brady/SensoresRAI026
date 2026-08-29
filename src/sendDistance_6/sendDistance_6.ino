// ═══════════ sendDistance_6.ino ═══════════
#include <Wire.h>
#include "sensores.hpp"

#define NUM_MODULES 6
#define LOOP_PERIOD_MS 25 // Ejecución constante a 40 Hz

// Configuración de Hardware (M1 a M6)
SensorModule modules[NUM_MODULES] = {
    SensorModule(13, 12, 30, 0x30), // M1: Front (0°)
    SensorModule(34, 28, 32, 0x31), // M2: Diag Left (-45°)
    SensorModule(26, 24,  2, 0x32), // M3: Diag Right (45°)
    SensorModule(40, 36,  3, 0x33), // M4: Side Left (-90°)
    SensorModule(52, 50,  4, 0x34), // M5: Side Right (90°)
    SensorModule( 8,  9,  5, 0x35)  // M6: Rear (180°)
};

unsigned long lastLoopTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Wire.begin();
    Wire.setClock(400000); // Bus I2C en Modo 400Hz,100Hz es estándar
    delay(50);

    // Apagar todos los VL53L0X vía XSHUT antes de inicializar
    const uint8_t xshutPins[NUM_MODULES] = {30, 32, 2, 3, 4, 5};
    for (uint8_t i = 0; i < NUM_MODULES; i++) {
        pinMode(xshutPins[i], OUTPUT);
        digitalWrite(xshutPins[i], LOW);
    }
    delay(50);

    // Inicializar módulos de sensores
    for (uint8_t i = 0; i < NUM_MODULES; i++) {
        modules[i].initUS();
        modules[i].initVL(&Wire);
    }
}

void loop() {
    unsigned long now = millis();
    if (now - lastLoopTime < LOOP_PERIOD_MS) return;
    lastLoopTime = now;

    int16_t fusedReads[NUM_MODULES];

    // Actualizar sensores y obtener la distancia fusionada por módulo
    for (uint8_t i = 0; i < NUM_MODULES; i++) {
        modules[i].updateAndFuse();
        fusedReads[i] = modules[i].getFusedDistance();

        // 0 = Sin lectura válida este ciclo (evita enviar valores negativos a la Jetson)
        if (fusedReads[i] < 0) {
            fusedReads[i] = 0;
        }
    }

    // Trama formateada con 6 datos fusionados: S<M1>,<M2>,<M3>,<M4>,<M5>,<M6>E
    char serialBuffer[64];
    snprintf(serialBuffer, sizeof(serialBuffer),
             "S%d,%d,%d,%d,%d,%dE",
             fusedReads[0], fusedReads[1], fusedReads[2],
             fusedReads[3], fusedReads[4], fusedReads[5]);

    Serial.println(serialBuffer);
}