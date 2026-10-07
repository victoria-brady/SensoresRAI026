// ═══════════ sensores.cpp ═══════════
#include "sensores.hpp"

bool SensorModule::initVL(TwoWire* wireInstance) {
  pinMode(m_xshutPin, OUTPUT);
  digitalWrite(m_xshutPin, HIGH);
  delay(20);

  // Timeout de inicialización de 150 ms para evitar bloqueos del sistema
  unsigned long t0 = millis();
  m_vlOk = false;

  while (millis() - t0 < 150) {
      // Inicializar comunicándose con la dirección de fábrica (0x29)
      if (m_vlSensor.begin(0x29, false, wireInstance)) {
          // Cambiar a la dirección definitiva solo tras una conexión exitosa
          if (m_i2cAddr != 0x29) {
              m_vlSensor.setAddress(m_i2cAddr);
          }
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
  m_lastVlCm = (measure.RangeStatus == 4)
                 ? -1
                 : static_cast<int16_t>(measure.RangeMilliMeter / 10);
  return m_lastVlCm;
}

void SensorModule::triggerUS() {
  if (!m_usOk) return;
  // Disparo TRIG de 10us. Esto no bloquea la lectura, solo inicia el sonido físico.
  digitalWrite(m_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(m_trigPin, LOW);
}

void SensorModule::setUltrasonicTime(unsigned long durationMicroseconds) {
  m_lastUsDuration = durationMicroseconds;
}

int16_t SensorModule::processUSTime() {
  if (!m_usOk) return -1;

  // Filtrar lecturas fuera de rango o tiempos vacíos
  int16_t rawCm = (m_lastUsDuration == 0 || m_lastUsDuration > 9000UL) 
                  ? -1 
                  : static_cast<int16_t>(m_lastUsDuration / 58);

  // Aplicar filtro de mediana para descartar picos por telas o ángulos oblicuos
  m_usBuffer[m_bufferIdx % MEDIAN_WINDOW] = rawCm;
  m_bufferIdx++;

  int16_t filteredUs = computeMedian(m_usBuffer, MEDIAN_WINDOW);

  // Compensación de offset: Alinear marco de referencia con el sensor VL (+6 cm)
  m_lastUsCm = (filteredUs > 0) ? filteredUs + US_OFFSET_CM : -1;

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
  int16_t us = processUSTime();

  // 1. Detección de Anomalía de Cercanía Extrema:
  // Si el VL detecta pegado (<= 10 cm) y el US rebota en falso (> 200 cm), invalidar US.
  if (vl > 0 && vl <= 10 && us > 200) {
    us = -1;
  }

  // 2. Matriz de Priorización y Fusión
  if (vl > 0 && us > 0) {
    m_fusedCm = (vl <= 120) ? vl : us;  // Corto/medio: ToF | Largo: Ultrasonido
  } else if (vl > 0) {
    m_fusedCm = vl;  // Fallback automático a VL
  } else if (us > 0) {
    m_fusedCm = us;  // Fallback automático a Ultrasonido
  } else {
    m_fusedCm = 0;  // Sin lectura válida
  }

  return m_fusedCm;
}