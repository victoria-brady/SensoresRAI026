/*
 * sensores_RAI.ino
 * Robot perro RAI — Object Detection / Sensor Array
 *
 * Lee 4x VL53L0X + 2x HC-SR04 y manda el estado de 6 zonas
 * a la Jetson Orin Nano por USB Serial cada 50ms.
 *
 * Salida: "Z1,Z2,Z3,Z4,Z5,Z6\n"  donde cada valor es 0, 1 o 2
 *   Ejemplo: "0,2,1,0,0,0\n"
 *   0 = libre
 *   1 = objeto lejos  (1000mm - 2000mm)
 *   2 = objeto cerca  (menos de 1000mm)
 *
 * Zonas:
 *   Z1 = VL1  diagonal izq-adelante   (XSHUT=D2)
 *   Z2 = US1 + VL1 + VL2 fusionados   centro-adelante (TRIG=D8, ECHO=D9)
 *   Z3 = VL2  diagonal der-adelante   (XSHUT=D3)
 *   Z4 = VL3  diagonal izq-atrás      (XSHUT=D4)
 *   Z5 = US2 + VL3 + VL4 fusionados   centro-atrás    (TRIG=D13, ECHO=D12)
 *   Z6 = VL4  diagonal der-atrás      (XSHUT=D5)
 *
 * Fusión conservadora:
 *   - VL lee bien ropa, personas, superficies suaves
 *   - US lee bien vidrio, ventanas, paredes
 *   - Basta que UN sensor detecte para reportar obstáculo
 *   - Los DOS deben coincidir en libre para reportar libre
 *
 * Robustez:
 *   - HC-SR04 por interrupciones, nunca bloquea el loop
 *   - VL53L0X lectura no bloqueante
 *   - Filtro de mediana por sensor
 *   - Watchdog: sensor que falla N veces se reinicia solo
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ═══════════════════════════════════════════════════════════════════════════════
// CONFIGURACIÓN — ajustar según pruebas reales
// ═══════════════════════════════════════════════════════════════════════════════
#define DIST_CERCA_MM      1000   // Menos de esto → nivel 2
#define DIST_LEJOS_MM      2000   // Menos de esto → nivel 1, sino → 0

#define LOOP_MS              50   // 20Hz
#define MEDIAN_N              5   // Muestras para filtro de mediana
#define MAX_FALLOS            5   // Fallos antes de reiniciar sensor

// ─── Pines VL53L0X ───────────────────────────────────────────────────────────
#define XSHUT_1   2
#define XSHUT_2   3
#define XSHUT_3   4
#define XSHUT_4   5

// ─── Pines HC-SR04 ───────────────────────────────────────────────────────────
#define TRIG_1    8
#define ECHO_1    9    // Interrupción
#define TRIG_2   13
#define ECHO_2   12   // Interrupción

// ─── Direcciones I2C únicas ──────────────────────────────────────────────────
#define ADDR_1   0x30
#define ADDR_2   0x31
#define ADDR_3   0x32
#define ADDR_4   0x33

// ─── Límites de validez ──────────────────────────────────────────────────────
#define VL_MIN      20
#define VL_MAX    1200
#define VL_ERR    8190
#define US_MIN     200
#define US_MAX    3500

// ═══════════════════════════════════════════════════════════════════════════════
// VARIABLES GLOBALES
// ═══════════════════════════════════════════════════════════════════════════════

// ─── VL53L0X ─────────────────────────────────────────────────────────────────
Adafruit_VL53L0X vl[4];
bool    vl_ok[4]      = {false, false, false, false};
uint8_t vl_xshut[4]  = {XSHUT_1, XSHUT_2, XSHUT_3, XSHUT_4};
uint8_t vl_addr[4]   = {ADDR_1,  ADDR_2,  ADDR_3,  ADDR_4};
uint8_t vl_fallos[4] = {0, 0, 0, 0};

// ─── HC-SR04 (interrupciones) ────────────────────────────────────────────────
volatile unsigned long us_inicio[2]   = {0, 0};
volatile unsigned long us_duracion[2] = {0, 0};
volatile bool          us_listo[2]    = {false, false};
uint8_t us_fallos[2] = {0, 0};

// ─── Buffers mediana ─────────────────────────────────────────────────────────
uint16_t buf_vl[4][MEDIAN_N];
uint16_t buf_us[2][MEDIAN_N];
uint8_t  buf_idx = 0;

// ─── Temporización ───────────────────────────────────────────────────────────
unsigned long ultimo_ms = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// INTERRUPCIONES HC-SR04
// ═══════════════════════════════════════════════════════════════════════════════
void ISR_ECHO_1() {
  if (digitalRead(ECHO_1) == HIGH) {
    us_inicio[0] = micros();
  } else if (us_inicio[0] > 0) {
    us_duracion[0] = micros() - us_inicio[0];
    us_listo[0]    = true;
    us_inicio[0]   = 0;
  }
}

void ISR_ECHO_2() {
  if (digitalRead(ECHO_2) == HIGH) {
    us_inicio[1] = micros();
  } else if (us_inicio[1] > 0) {
    us_duracion[1] = micros() - us_inicio[1];
    us_listo[1]    = true;
    us_inicio[1]   = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Wire.begin();
  Wire.setClock(400000);

  // Pines ultrasónicos
  pinMode(TRIG_1, OUTPUT); digitalWrite(TRIG_1, LOW);
  pinMode(TRIG_2, OUTPUT); digitalWrite(TRIG_2, LOW);
  pinMode(ECHO_1, INPUT);
  pinMode(ECHO_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_1), ISR_ECHO_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_2), ISR_ECHO_2, CHANGE);

  // Apagar todos los VL primero
  for (int i = 0; i < 4; i++) {
    pinMode(vl_xshut[i], OUTPUT);
    digitalWrite(vl_xshut[i], LOW);
  }
  delay(20);

  // Encender de a uno y asignar dirección I2C única
  for (int i = 0; i < 4; i++) {
    vl_ok[i] = iniciarVL(i);
  }

  // Inicializar buffers de mediana en cero
  memset(buf_vl, 0, sizeof(buf_vl));
  memset(buf_us, 0, sizeof(buf_us));
}

// ═══════════════════════════════════════════════════════════════════════════════
// LOOP — nunca se bloquea
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  unsigned long ahora = millis();
  if (ahora - ultimo_ms < LOOP_MS) return;
  ultimo_ms = ahora;

  // 1. Disparar ultrasonidos (no bloqueante, ISR captura el echo)
  dispararUS(0, TRIG_1);
  dispararUS(1, TRIG_2);

  // 2. Leer VL53L0X
  uint16_t dvl[4];
  for (int i = 0; i < 4; i++) dvl[i] = leerVL(i);

  // 3. Leer HC-SR04 (resultado de ISR del ciclo anterior)
  uint16_t dus[2];
  for (int i = 0; i < 2; i++) dus[i] = leerUS(i);

  // 4. Calcular nivel por zona
  uint8_t z[6];
  z[0] = nivel(dvl[0]);                      // Z1 = VL1 izq-adelante
  z[1] = fusionar(dus[0], dvl[0], dvl[1]);   // Z2 = centro-adelante (fusión)
  z[2] = nivel(dvl[1]);                      // Z3 = VL2 der-adelante
  z[3] = nivel(dvl[2]);                      // Z4 = VL3 izq-atrás
  z[4] = fusionar(dus[1], dvl[2], dvl[3]);   // Z5 = centro-atrás (fusión)
  z[5] = nivel(dvl[3]);                      // Z6 = VL4 der-atrás

  // 5. Mandar a la Jetson: "Z1,Z2,Z3,Z4,Z5,Z6\n"
  Serial.print(z[0]); Serial.print(',');
  Serial.print(z[1]); Serial.print(',');
  Serial.print(z[2]); Serial.print(',');
  Serial.print(z[3]); Serial.print(',');
  Serial.print(z[4]); Serial.print(',');
  Serial.println(z[5]);

  buf_idx++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// VL53L0X — inicialización y lectura con watchdog
// ═══════════════════════════════════════════════════════════════════════════════
bool iniciarVL(uint8_t i) {
  digitalWrite(vl_xshut[i], HIGH);
  delay(10);
  if (!vl[i].begin(VL_ERR, false, &Wire)) {
    digitalWrite(vl_xshut[i], LOW);
    return false;
  }
  vl[i].setAddress(vl_addr[i]);
  vl_fallos[i] = 0;
  return true;
}

uint16_t leerVL(uint8_t i) {
  if (!vl_ok[i]) {
    vl_ok[i] = iniciarVL(i);
    return 0;
  }

  VL53L0X_RangingMeasurementData_t m;
  vl[i].rangingTest(&m, false);
  uint16_t raw = m.RangeMilliMeter;

  // RangeStatus 4 = phase fail (vidrio, transparente, sin objeto)
  bool invalido = (m.RangeStatus == 4)
               || (raw >= VL_ERR)
               || (raw < VL_MIN)
               || (raw > VL_MAX);

  if (invalido) {
    vl_fallos[i]++;
    if (vl_fallos[i] >= MAX_FALLOS) {
      // Reiniciar sensor sin trabar el resto
      digitalWrite(vl_xshut[i], LOW);
      delay(5);
      vl_ok[i]     = iniciarVL(i);
      vl_fallos[i] = 0;
    }
    // Devolver último valor válido del buffer
    return buf_vl[i][(buf_idx + MEDIAN_N - 1) % MEDIAN_N];
  }

  vl_fallos[i] = 0;
  buf_vl[i][buf_idx % MEDIAN_N] = raw;
  return mediana(buf_vl[i]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// HC-SR04 — disparo y lectura desde ISR
// ═══════════════════════════════════════════════════════════════════════════════
void dispararUS(uint8_t i, uint8_t trig) {
  us_listo[i] = false;
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
}

uint16_t leerUS(uint8_t i) {
  if (!us_listo[i]) {
    us_fallos[i]++;
    if (us_fallos[i] >= MAX_FALLOS) us_fallos[i] = 0;
    return 0;
  }
  us_fallos[i] = 0;

  uint16_t mm = (uint16_t)((us_duracion[i] * 343UL) / 2000UL);
  if (mm < US_MIN || mm > US_MAX) return 0;

  buf_us[i][buf_idx % MEDIAN_N] = mm;
  return mediana(buf_us[i]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// LÓGICA DE ZONAS
// ═══════════════════════════════════════════════════════════════════════════════

// Distancia en mm → nivel 0/1/2
uint8_t nivel(uint16_t mm) {
  if (mm == 0)             return 0;
  if (mm < DIST_CERCA_MM)  return 2;
  if (mm < DIST_LEJOS_MM)  return 1;
  return 0;
}

// Fusiona zona central: toma el nivel más alto (más peligroso) de los tres
// → US ve vidrio aunque VL falle, VL ve ropa aunque US falle
uint8_t fusionar(uint16_t dus, uint16_t dvl_a, uint16_t dvl_b) {
  return max(nivel(dus), max(nivel(dvl_a), nivel(dvl_b)));
}

// ═══════════════════════════════════════════════════════════════════════════════
// FILTRO DE MEDIANA — ignora ceros (lecturas inválidas)
// ═══════════════════════════════════════════════════════════════════════════════
uint16_t mediana(uint16_t *buf) {
  uint16_t tmp[MEDIAN_N];
  uint8_t n = 0;

  for (uint8_t i = 0; i < MEDIAN_N; i++) {
    if (buf[i] > 0) tmp[n++] = buf[i];
  }
  if (n == 0) return 0;

  for (uint8_t i = 0; i < n - 1; i++)
    for (uint8_t j = 0; j < n - i - 1; j++)
      if (tmp[j] > tmp[j+1]) {
        uint16_t t = tmp[j]; tmp[j] = tmp[j+1]; tmp[j+1] = t;
      }

  return tmp[n / 2];
}
