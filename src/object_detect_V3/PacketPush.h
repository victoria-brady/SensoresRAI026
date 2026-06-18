#pragma once
#include <stdint.h>
#include "config.h"

struct SensorPacket {
    uint16_t vl[NUM_VL];
    uint16_t us[NUM_US];
};

void sendSerialPacket(const SensorPacket& data);