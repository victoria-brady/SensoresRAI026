// ═══════════ sensores.cpp ═══════════
#include "sensores.hpp"

bool SensorModule::initVL(TwoWire* wireInstance) {
  pinMode(m_xshutPin, OUTPUT);
  digitalWrite(m_xshutPin, HIGH);
  delay(20);

  // 150 ms init timeout to avoid system lockups
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
    digitalWrite(m_xshutPin, LOW);  // Power off on failure
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

  // Accept any reading except Error 4 (unrecoverable out of range)
  m_lastVlCm = (measure.RangeStatus == 4)
                 ? -1
                 : static_cast<int16_t>(measure.RangeMilliMeter / 10);
  return m_lastVlCm;
}

int16_t SensorModule::readUS() {
  if (!m_usOk) return -1;

  // 10 us TRIG pulse
  digitalWrite(m_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(m_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(m_trigPin, LOW);

  // Timeout tuned to 9000 us (~155 cm max) to avoid blocking the loop
  unsigned long duration = pulseIn(m_echoPin, HIGH, 9000UL);
  int16_t rawCm = (duration == 0) ? -1 : static_cast<int16_t>(duration / 58);

  // Apply median filter to discard spikes from fabrics or oblique angles
  m_usBuffer[m_bufferIdx % MEDIAN_WINDOW] = rawCm;
  m_bufferIdx++;

  int16_t filteredUs = computeMedian(m_usBuffer, MEDIAN_WINDOW);

  // Offset compensation: align reference frame with the VL sensor (+6 cm)
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

  // Simple insertion sort for a small array
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

  // 1. Extreme-proximity anomaly detection:
  // If the VL detects something very close (<= 10 cm) and the US bounces falsely (> 200 cm), invalidate the US.
  if (vl > 0 && vl <= 10 && us > 200) {
    us = -1;
  }

  // 2. Prioritization and fusion matrix
  if (vl > 0 && us > 0) {
    m_fusedCm = (vl <= 120) ? vl : us;  // Short/medium: ToF | Long: Ultrasonic
  } else if (vl > 0) {
    m_fusedCm = vl;  // Automatic fallback to VL
  } else if (us > 0) {
    m_fusedCm = us;  // Automatic fallback to Ultrasonic
  } else {
    m_fusedCm = 0;  // No valid reading
  }

  return m_fusedCm;
}
