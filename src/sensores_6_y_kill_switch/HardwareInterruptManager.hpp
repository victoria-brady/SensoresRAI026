#ifndef HardwareInterruptManager_HPP
#define HardwareInterruptManager_HPP

// InterruptManager.hpp
#pragma once
#include <Arduino.h>

void initInterrupts();
unsigned long getEchoTime(uint8_t sensorIndex);
volatile uint8_t getKillSwitchState();

#endif  // HardwareInterruptManager_HPP