/*
 * sendDistance_6_Interrupts.ino
 * Robot perro RAI — Object Detection / Sensor Array (3x VL53L0X + 3x HC-SR04)
 *
 * Arquitectura Optimizada por Interrupciones:
 * - 3x VL53L0X (XSHUT: 2, 3, 4)
 * - 3x HC-SR04 (ECHO: 12, 9, 24)
 * - CERO salidas de debug. Única salida Serial: "S<vl1>,<vl2>,<vl3>,<us1>,<us2>,<us3>E"
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>

// ═══════════════════════════════════════════════════════════════════════════════
// CONFIGURACIÓN — RED
// ═══════════════════════════════════════════════════════════════════════════════
#define WIFI_SSID     "RED_DEL_ROBOT"
#define WIFI_PASS     "PASSWORD"

#define DEST_IP        "192.168.1.50"
#define DEST_PORT      43899
#define LOCAL_UDP_PORT 43898

#define ENABLE_WIFI    0
#define WIFI_RETRY_MS  3000

// ═══════════════════════════════════════════════════════════════════════════════
// CONFIGURACIÓN — SENSORES
// ═══════════════════════════════════════════════════════════════════════════════
#define NUM_US  3
#define NUM_VL  3

#define LOOP_MS        50
#define MEDIAN_N       3
#define MAX_FALLOS_VL  5
#define US_TIMEOUT_US  35000UL
#define US_SETTLE_MS   10

// ─── Pines HC-SR04 (ECHO obligatorios: 12, 9, 24) ─────────────────────────────
const uint8_t HC_TRIG[NUM_US] = {13, 8, 26}; // TRIG correspondientes según placa
const uint8_t HC_ECHO[NUM_US] = {12, 9, 24};

// ─── Pines VL53L0X (XSHUT obligatorios: 2, 3, 4) ──────────────────────────────
const uint8_t VL_XSHUT[NUM_VL] = {2, 3, 4};
const uint8_t VL_ADDR[NUM_VL]  = {0x30, 0x31, 0x32};

// ═══════════════════════════════════════════════════════════════════════════════
// VARIABLES GLOBALES E INTERRUPCIONES
// ═══════════════════════════════════════════════════════════════════════════════
Adafruit_VL53L0X vl_sensors[NUM_VL];
bool    vl_ok[NUM_VL]      = {false};
uint8_t vl_fallos[NUM_VL]  = {0};

bool us_ok[NUM_US] = {false};

int16_t buf_vl[NUM_VL][MEDIAN_N];
int16_t buf_us[NUM_US][MEDIAN_N];
uint8_t buf_idx = 0;

unsigned long ultimo_ms = 0;

WiFiUDP       Udp;
IPAddress     destIp;
bool          wifi_iniciado  = false;
unsigned long ultimo_intento = 0;

// Variables Volátiles para las Rutinas de Interrupción (ISR)
volatile unsigned long echo_start[NUM_US]    = {0, 0, 0};
volatile unsigned long echo_duration[NUM_US] = {0, 0, 0};
volatile bool          new_data_us[NUM_US]   = {false, false, false};

// ─── Rutinas de Servicio de Interrupción (ISR) ───────────────────────────────
void isr_echo0() {
  if (digitalRead(HC_ECHO[0]) == HIGH) {
    echo_start[0] = micros();
  } else {
    echo_duration[0] = micros() - echo_start[0];
    new_data_us[0] = true;
  }
}

void isr_echo1() {
  if (digitalRead(HC_ECHO[1]) == HIGH) {
    echo_start[1] = micros();
  } else {
    echo_duration[1] = micros() - echo_start[1];
    new_data_us[1] = true;
  }
}

void isr_echo2() {
  if (digitalRead(HC_ECHO[2]) == HIGH) {
    echo_start[2] = micros();
  } else {
    echo_duration[2] = micros() - echo_start[2];
    new_data_us[2] = true;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  // Esperar a que Serial inicie sin imprimir mensajes bloqueantes
  while (!Serial && millis() < 3000);
  
  // HC-SR04 Setup
  for (int i = 0; i < NUM_US; i++) {
    pinMode(HC_TRIG[i], OUTPUT);
    digitalWrite(HC_TRIG[i], LOW);
    pinMode(HC_ECHO[i], INPUT_PULLUP);
    us_ok[i] = true;
  }

  // Vincular Interrupciones por Hardware (Cualquier cambio de estado en el pin)
  attachInterrupt(digitalPinToInterrupt(HC_ECHO[0]), isr_echo0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HC_ECHO[1]), isr_echo1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HC_ECHO[2]), isr_echo2, CHANGE);

  // VL53L0X Setup
  Wire.begin();
  Wire.setClock(100000);
  delay(50);

  for (int i = 0; i < NUM_VL; i++) {
    pinMode(VL_XSHUT[i], OUTPUT);
    digitalWrite(VL_XSHUT[i], LOW);
    vl_ok[i] = false;
  }
  delay(50);

  for (int i = 0; i < NUM_VL; i++) {
    vl_ok[i] = iniciarVL(i);
  }

  // Inicializar buffers en -1
  for (int i = 0; i < NUM_VL; i++)
    for (int j = 0; j < MEDIAN_N; j++) buf_vl[i][j] = -1;
  for (int i = 0; i < NUM_US; i++)
    for (int j = 0; j < MEDIAN_N; j++) buf_us[i][j] = -1;

#if ENABLE_WIFI
  destIp.fromString(DEST_IP);
  conectarWiFi();
#endif

  ultimo_ms = millis();
}

bool iniciarVL(uint8_t i) {
  digitalWrite(VL_XSHUT[i], HIGH);
  delay(50);

  unsigned long start_time = millis();
  bool sensor_ok = false;
  while (millis() - start_time < 200) {
    if (vl_sensors[i].begin(VL_ADDR[i], false, &Wire)) {
      vl_sensors[i].setAddress(VL_ADDR[i]);
      sensor_ok = true;
      break;
    }
    delay(20);
  }

  if (!sensor_ok) {
    digitalWrite(VL_XSHUT[i], LOW);
  }
  delay(50);
  return sensor_ok;
}

void conectarWiFi() {
  ultimo_intento = millis();
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Udp.begin(LOCAL_UDP_PORT);
    wifi_iniciado = true;
  } else {
    wifi_iniciado = false;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// LOOP PRINCIPAL
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  unsigned long ahora = millis();
  if (ahora - ultimo_ms < LOOP_MS) return;
  ultimo_ms = ahora;

  int16_t dvl[NUM_VL];
  for (int i = 0; i < NUM_VL; i++) {
    dvl[i] = leerVL(i);
  }

  int16_t dus[NUM_US];
  for (int i = 0; i < NUM_US; i++) {
    dus[i] = leerUS(i);
    if (i < NUM_US - 1) delay(US_SETTLE_MS);
  }

  // Contrato con el brainstem: 0 = "sin lectura válida este ciclo". El sketch
  // usa -1 internamente para inválido (mediana/buffers), pero un '-' en la
  // trama hace que el parser del brainstem descarte el PAQUETE ENTERO — y eso
  // pasa justo cuando un sensor falla (VL en pared especular con RangeStatus!=0,
  // o US sin eco), que es el peor momento para quedarse ciego de los 6.
  // Clampear a 0 mantiene vivos a los demás aunque uno falle.
  for (int i = 0; i < NUM_VL; i++) if (dvl[i] < 0) dvl[i] = 0;
  for (int i = 0; i < NUM_US; i++) if (dus[i] < 0) dus[i] = 0;

  // Trama estructurada estricta sin metadata adicional
  char msg[64];
  int len = snprintf(msg, sizeof(msg), "S%d,%d,%d,%d,%d,%dE",
                      dvl[0], dvl[1], dvl[2],
                      dus[0], dus[1], dus[2]);

  Serial.println(msg);

#if ENABLE_WIFI
  if (wifi_iniciado && WiFi.status() == WL_CONNECTED) {
    Udp.beginPacket(destIp, DEST_PORT);
    Udp.write((const uint8_t*)msg, len);
    Udp.endPacket();
  } else if (millis() - ultimo_intento >= WIFI_RETRY_MS) {
    conectarWiFi();
  }
#endif

  buf_idx++;
}

int16_t leerVL(uint8_t i) {
  if (!vl_ok[i]) {
    vl_ok[i] = iniciarVL(i);
    return -1;
  }

  VL53L0X_RangingMeasurementData_t m;
  vl_sensors[i].rangingTest(&m, false);

  int16_t raw;
  if (m.RangeStatus != 0) {
    raw = -1;
    vl_fallos[i]++;
    if (vl_fallos[i] >= MAX_FALLOS_VL) {
      digitalWrite(VL_XSHUT[i], LOW);
      delay(5);
      vl_ok[i]     = iniciarVL(i);
      vl_fallos[i] = 0;
    }
  } else {
    raw = (int16_t)m.RangeMilliMeter;
    vl_fallos[i] = 0;
  }

  buf_vl[i][buf_idx % MEDIAN_N] = raw;
  return mediana(buf_vl[i]);
}

int16_t leerUS(uint8_t i) {
  if (!us_ok[i]) return -1;

  // Limpiar bandera de interrupción antes del disparo
  new_data_us[i] = false;

  // Señal TRIG
  digitalWrite(HC_TRIG[i], LOW);
  delayMicroseconds(2);
  digitalWrite(HC_TRIG[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_TRIG[i], LOW);

  // Espera no bloqueante mediada por hardware
  unsigned long timeout = micros() + US_TIMEOUT_US;
  while (!new_data_us[i] && micros() < timeout) {
    // La interrupción se encargará de levantar 'new_data_us[i]'
  }

  int16_t mm = -1; // Valor por defecto si hay timeout (ej. objeto fuera de rango)
  
  if (new_data_us[i]) {
    // Velocidad del sonido (343 m/s)
    mm = (int16_t)((echo_duration[i] * 343UL) / 2000UL);
  }

  buf_us[i][buf_idx % MEDIAN_N] = mm;
  return mediana(buf_us[i]);
}

int16_t mediana(int16_t *buf) {
  int16_t tmp[MEDIAN_N];
  uint8_t n = 0;

  for (uint8_t i = 0; i < MEDIAN_N; i++) {
    if (buf[i] >= 0) tmp[n++] = buf[i];
  }
  if (n == 0) return -1;

  for (uint8_t i = 0; i < n - 1; i++)
    for (uint8_t j = 0; j < n - i - 1; j++)
      if (tmp[j] > tmp[j + 1]) {
        int16_t t = tmp[j]; tmp[j] = tmp[j + 1]; tmp[j + 1] = t;
      }

  return tmp[n / 2];
}