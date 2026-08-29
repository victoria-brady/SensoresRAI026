// ═══════════ sensores.hpp ═══════════
#ifndef SENSORES_HPP
#define SENSORES_HPP

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define MEDIAN_WINDOW 5  // Ventana del filtro de mediana para ultrasonido

class SensorModule {
private:
  uint8_t m_trigPin;
  uint8_t m_echoPin;
  uint8_t m_xshutPin;
  uint8_t m_i2cAddr;

  Adafruit_VL53L0X m_vlSensor;
  bool m_vlOk;
  bool m_usOk;

  int16_t m_usBuffer[MEDIAN_WINDOW];
  uint8_t m_bufferIdx;

  int16_t m_lastVlCm;
  int16_t m_lastUsCm;
  int16_t m_fusedCm;

  // Compensación física: Ultrasonido está 6 cm más adelante que el VL
  static constexpr int16_t US_OFFSET_CM = 6;

  int16_t computeMedian(int16_t* buf, uint8_t size);

public:
  explicit SensorModule(uint8_t trigPin = 0, uint8_t echoPin = 0, uint8_t xshutPin = 0, uint8_t i2cAddr = 0)
    : m_trigPin(trigPin), m_echoPin(echoPin), m_xshutPin(xshutPin), m_i2cAddr(i2cAddr),
      m_vlOk(false), m_usOk(false), m_bufferIdx(0),
      m_lastVlCm(-1), m_lastUsCm(-1), m_fusedCm(0) {
    for (uint8_t i = 0; i < MEDIAN_WINDOW; i++) m_usBuffer[i] = -1;
  }

  // Boot no bloqueante para el VL53L0X
  bool initVL(TwoWire* wireInstance = &Wire);
  int16_t readVL();
  int16_t readUS();
  int16_t updateAndFuse();

  void initUS() {
    pinMode(m_trigPin, OUTPUT);
    digitalWrite(m_trigPin, LOW);
    pinMode(m_echoPin, INPUT_PULLUP);
    m_usOk = true;
  }

  int16_t getVLDistance() const { return m_lastVlCm; }
  int16_t getUSDistance() const { return m_lastUsCm; }
  int16_t getFusedDistance() const { return m_fusedCm; }

  bool isVLOk() const { return m_vlOk; }
  bool isUSOk() const { return m_usOk; }
};

#endif  // SENSORES_HPP