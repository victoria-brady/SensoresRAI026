#include <Wire.h>
#include "config.h"
#include "VL53L0X.h"
#include "HC-SR04.h"
#include "filter.h"
#include "PacketPush.h"
#include "my_timer.h"

void procesarSensores() {
    SensorPacket paquete;
    
    // 1. Disparar ultrasonidos (para ser capturados en 2do plano por ISR)
    triggerAllUS();
    
    // 2. Capturar y filtrar láseres
    for (int i = 0; i < NUM_VL; i++) {
        paquete.vl[i] = applyFilterVL(i, getRawVL53L0X(i));
    }
    
    // 3. Capturar y filtrar ultrasonidos (datos del ciclo anterior)
    for (int i = 0; i < NUM_US; i++) {
        paquete.us[i] = applyFilterUS(i, getRawUS(i));
    }
    
    // 4. Enviar datos estructurados
    sendSerialPacket(paquete);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Wire.begin();
    Wire.setClock(400000);
    
    initAllUS();
    initAllVL53L0X();
}

void loop() {
    // El timer se encarga de llamar a "procesarSensores" cada LOOP_MS
    runTaskPeriodically(procesarSensores, LOOP_MS);
}