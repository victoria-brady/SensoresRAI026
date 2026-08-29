#include "sensores.hpp"

SensorModule::SensorModule()
  : m_id(0), m_trigPin(0), m_echoPin(0), m_xshutPin(0), m_i2cAddr(0),
    m_vlOk(false), m_usOk(false), m_bufferIdx(0),
    m_lastVlCm(-1), m_lastUsCm(-1), m_fusedCm(0) {
  for (uint8_t i = 0; i < MEDIAN_WINDOW; i++) m_usBuffer[i] = -1;
}

SensorModule::SensorModule(uint8_t id, uint8_t trigPin, uint8_t echoPin, uint8_t xshutPin, uint8_t i2cAddr)
  : m_id(id), m_trigPin(trigPin), m_echoPin(echoPin), m_xshutPin(xshutPin), m_i2cAddr(i2cAddr),
    m_vlOk(false), m_usOk(false), m_bufferIdx(0),
    m_lastVlCm(-1), m_lastUsCm(-1), m_fusedCm(0) {
  for (uint8_t i = 0; i < MEDIAN_WINDOW; i++) m_usBuffer[i] = -1;
}

void SensorModule::initUS() {
  pinMode(m_trigPin, OUTPUT);
  digitalWrite(m_trigPin, LOW);
  pinMode(m_echoPin, INPUT_PULLUP);
  m_usOk = true;
}

bool SensorModule::initVL(TwoWire* wireInstance) {
  pinMode(m_xshutPin, OUTPUT);
  digitalWrite(m_xshutPin, HIGH);
  delay(20);

  // Timeout de inicialización de 150 ms para evitar bloqueos del sistema
  unsigned long t0 = millis();
  m_vlOk = false;

  while (millis() - t0 < 150) {
    if (m_vlSensor.begin(m_i2cAddr, false, wireInstance)) {
      m_vlSensor.setAddress(m_i2cAddr);
      m_vlOk = true;
      break;
    }
    delay(10);
  }

  if (!m_vlOk) {
    digitalWrite(m_xshutPin, LOW);  // Apagar si falla
  }
  return m_vlOk;
}

int16_t SensorModule::readVL() {
  if (!m_vlOk) {
    m_lastVlCm = -1;
    return -1;
  }

  VL53L0X_RangingMeasurementData_t measure;
  m_vlSensor.rangingTest(&measure, false);

  // Aceptar cualquier lectura excepto Error 4 (Out of Range insalvable)
  if (measure.RangeStatus == 4) {
    m_lastVlCm = -1;
  } else {
    m_lastVlCm = static_cast<int16_t>(measure.RangeMilliMeter / 10);
  }
  return m_lastVlCm;
}

int16_t SensorModule::readUS() {
  if (!m_usOk) return -1;

  // Disparo TRIG de 10us
  digitalWrite(m_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(m_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(m_trigPin, LOW);

  // Timeout de 20.000 us (~340 cm max) para no congelar el loop
  unsigned long duration = pulseIn(m_echoPin, HIGH, 20000UL);
  int16_t rawCm = (duration == 0) ? -1 : static_cast<int16_t>(duration / 58);

  // Aplicar filtro de mediana para descartar picos por telas o ángulos oblicuos
  m_usBuffer[m_bufferIdx % MEDIAN_WINDOW] = rawCm;
  m_bufferIdx++;

  int16_t filteredUs = computeMedian(m_usBuffer, MEDIAN_WINDOW);

  // Compensación de offset: Alinear marco de referencia con el sensor VL (+6 cm)
  if (filteredUs > 0) {
    m_lastUsCm = filteredUs + US_OFFSET_CM;
  } else {
    m_lastUsCm = -1;
  }

  return m_lastUsCm;
}

int16_t SensorModule::computeMedian(int16_t* buf, uint8_t size) {
  int16_t temp[MEDIAN_WINDOW];
  uint8_t count = 0;

  for (uint8_t i = 0; i < size; i++) {
    if (buf[i] > 0) temp[count++] = buf[i];
  }

  if (count == 0) return -1;

  // Inserción simple para ordenar array pequeño
  for (uint8_t i = 1; i < count; i++) {
    int16_t key = temp[i];
    int8_t j = i - 1;
    while (j >= 0 && temp[j] > key) {
      temp[j + 1] = temp[j];
      j--;
    }
    temp[j + 1] = key;
  }

  return temp[count / 2];
}

int16_t SensorModule::updateAndFuse() {
  int16_t vl = readVL();
  int16_t us = readUS();

  // 1. Detección de Anomalía de Cercanía Extrema:
  // Si el VL detecta pegado (<= 10 cm) y el US rebota en falso (> 200 cm), invalidar US.
  if (vl > 0 && vl <= 10 && us > 200) {
    us = -1;
  }

  // 2. Matriz de Priorización y Fusión
  if (vl > 0 && us > 0) {
    if (vl <= 120) {
      // Rango corto/medio: Prioridad ToF (alta precisión óptica)
      m_fusedCm = vl;
    } else {
      // Rango largo (> 120 cm): Prioridad Ultrasonido
      m_fusedCm = us;
    }
  } else if (vl > 0) {
    m_fusedCm = vl;  // Fallback automático a VL
  } else if (us > 0) {
    m_fusedCm = us;  // Fallback automático a Ultrasonido
  } else {
    m_fusedCm = 0;  // Sin lectura válida
  }

  return m_fusedCm;
}