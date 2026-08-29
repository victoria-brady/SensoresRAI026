#ifndef SENSORES_HPP
#define SENSORES_HPP

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define MEDIAN_WINDOW 5  // Ventana del filtro de mediana para ultrasonido

class SensorModule {
private:
  uint8_t m_id;
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
  SensorModule();
  SensorModule(uint8_t id, uint8_t trigPin, uint8_t echoPin, uint8_t xshutPin, uint8_t i2cAddr);

  // Boot no bloqueante para el VL53L0X
  bool initVL(TwoWire* wireInstance = &Wire);
  void initUS();

  int16_t readVL();
  int16_t readUS();
  int16_t updateAndFuse();

  int16_t getVLDistance() const {
    return m_lastVlCm;
  }
  int16_t getUSDistance() const {
    return m_lastUsCm;
  }
  int16_t getFusedDistance() const {
    return m_fusedCm;
  }

  bool isVLOk() const {
    return m_vlOk;
  }
  bool isUSOk() const {
    return m_usOk;
  }
};

#endif  // SENSORES_HPP