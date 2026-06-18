#include "VL53L0X.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X vl[NUM_VL];
bool vl_ok[NUM_VL] = {false, false, false, false};
const uint8_t vl_xshut[NUM_VL] = {XSHUT_1, XSHUT_2, XSHUT_3, XSHUT_4};
const uint8_t vl_addr[NUM_VL]  = {ADDR_1,  ADDR_2,  ADDR_3,  ADDR_4};
uint8_t vl_fallos[NUM_VL]      = {0, 0, 0, 0};

bool initSingleVL(uint8_t id) {
    digitalWrite(vl_xshut[id], HIGH);
    delay(10);
    if (!vl[id].begin(VL_ERR, false, &Wire)) {
        digitalWrite(vl_xshut[id], LOW);
        return false;
    }
    vl[id].setAddress(vl_addr[id]);
    vl_fallos[id] = 0;
    return true;
}

void initAllVL53L0X() {
    for (int i = 0; i < NUM_VL; i++) {
        pinMode(vl_xshut[i], OUTPUT);
        digitalWrite(vl_xshut[i], LOW);
    }
    delay(20);
    for (int i = 0; i < NUM_VL; i++) {
        vl_ok[i] = initSingleVL(i);
    }
}

uint16_t getRawVL53L0X(uint8_t id) {
    if (!vl_ok[id]) {
        vl_ok[id] = initSingleVL(id);
        return 0;
    }

    VL53L0X_RangingMeasurementData_t m;
    vl[id].rangingTest(&m, false);
    uint16_t raw = m.RangeMilliMeter;

    bool invalido = (m.RangeStatus == 4) || (raw >= VL_ERR) || (raw < VL_MIN) || (raw > VL_MAX);

    if (invalido) {
        vl_fallos[id]++;
        if (vl_fallos[id] >= MAX_FALLOS) {
            digitalWrite(vl_xshut[id], LOW);
            delay(5);
            vl_ok[id] = initSingleVL(id);
            vl_fallos[id] = 0;
        }
        return 0;
    }
    
    vl_fallos[id] = 0;
    return raw;
}