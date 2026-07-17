/*
 * object_detect_V2.ino
 * Robot perro RAI — Object Detection / Sensor Array
 *
 * Lee 4x VL53L0X + 2x HC-SR04 y manda las distancias crudas cada 50ms:
 *   - por WiFi (UDP) a una IP:puerto configurables (la PC en la red del robot,
 *     o en el futuro la Jetson Orin Nano). Es lo que consume el brainstem.
 *   - por USB Serial en paralelo (debug / fallback).
 *
 * Placa: Arduino GIGA R1 WiFi (módulo WiFi integrado, librería <WiFi.h>).
 *
 * Salida: "SVL1,VL2,VL3,VL4,US1,US2E"
 *   S   = inicio del mensaje
 *   E   = fin del mensaje
 *   Valores en mm, 0 = sin lectura válida ese ciclo
 *   Ejemplo: "S342,891,0,1205,654,0E"
 *
 * El brainstem escucha estos paquetes en UDP :43899 (ver
 * basicoperation/brainstem/core/brainstem_config.h::kSensorUdpPort y
 * basicoperation/shared/sensor_layout.h para el contrato del formato).
 *
 * Sensores y posiciones:
 *   VL1 = diagonal izq-adelante   (XSHUT=D2)
 *   VL2 = diagonal der-adelante   (XSHUT=D3)
 *   VL3 = diagonal izq-atrás      (XSHUT=D4)
 *   VL4 = diagonal der-atrás      (XSHUT=D5)
 *   US1 = centro-adelante         (TRIG=D8, ECHO=D9)
 *   US2 = centro-atrás            (TRIG=D13, ECHO=D12)
 *
 * Robustez:
 *   - HC-SR04 por interrupciones (D9 y D12) — nunca bloquea el loop
 *   - VL53L0X lectura no bloqueante
 *   - Filtro de mediana de 5 muestras por sensor
 *   - Watchdog: sensor que falla 5 veces se reinicia solo sin trabar el resto
 *   - Protocolo S...E: la Jetson siempre sabe dónde empieza y termina el mensaje
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>        // Arduino GIGA R1 WiFi — módulo integrado

// ═══════════════════════════════════════════════════════════════════════════════
// CONFIGURACIÓN — RED (EDITAR ESTO)
// ═══════════════════════════════════════════════════════════════════════════════
// Red WiFi del robot a la que se conecta el Arduino.
#define WIFI_SSID     "RED_DEL_ROBOT"     // ← SSID de la red del robot
#define WIFI_PASS     "PASSWORD"          // ← password

// Destino de los paquetes UDP: la IP de TU compu (o la Jetson) dentro de la red
// del robot, y el puerto donde escucha el brainstem (kSensorUdpPort = 43899).
#define DEST_IP       "192.168.1.50"      // ← IP de la compu que corre el brainstem
#define DEST_PORT     43899               // = brainstem kSensorUdpPort
#define LOCAL_UDP_PORT 43898              // puerto local del Arduino (cualquiera libre)

// ENABLE_WIFI:
//   0 = el Arduino sólo manda por USB Serial (DEFAULT). Usalo cuando el Arduino
//       va cableado por USB a una laptop: esa laptop corre el bridge
//       serial_to_udp_bridge.py que reenvía a la compu/Jetson con el brainstem.
//       No hace falta SSID ni WiFi en el Arduino.
//   1 = el Arduino se conecta a la WiFi del robot y manda UDP directo (sin
//       laptop intermedia). Sólo para el futuro montaje on-board; requiere
//       WIFI_SSID/WIFI_PASS válidos o el reintento de conexión frena el loop.
#define ENABLE_WIFI    0
#define WIFI_RETRY_MS  3000  // cada cuánto reintentar si se cae el WiFi

// ═══════════════════════════════════════════════════════════════════════════════
// CONFIGURACIÓN — SENSORES
// ═══════════════════════════════════════════════════════════════════════════════
#define LOOP_MS       50   // objetivo 20Hz (ver nota de latencia abajo)
#define MEDIAN_N       3   // Muestras del filtro de mediana. Bajado de 5 a 3:
                           // a 5, un obstáculo súbito tardaba ~3 ciclos en
                           // dominar la mediana (lag peligroso para el reflejo).
                           // A 3 reacciona en ~2 ciclos y sigue filtrando ruido.
#define MAX_FALLOS     5   // Fallos consecutivos antes de reiniciar sensor

// ─── Pines VL53L0X ───────────────────────────────────────────────────────────
#define XSHUT_1   2   // VL1 diagonal atras izquierda
#define XSHUT_2   3   // VL2 diagonal atras derecha
#define XSHUT_3   4   // VL3 diagonal adelante izquierda
#define XSHUT_4   5   // VL4 diagonal adelante  derecha

// ─── Pines HC-SR04 ───────────────────────────────────────────────────────────
#define TRIG_1    8   // US1 centro-adelante
#define ECHO_1    9   // Interrupción
#define TRIG_2   13   // US2 centro-atrás
#define ECHO_2   12   // Interrupción

// ─── Direcciones I2C únicas para cada VL53L0X ────────────────────────────────
#define ADDR_1   0x30
#define ADDR_2   0x31
#define ADDR_3   0x32
#define ADDR_4   0x33

// ─── Límites de validez de lecturas ──────────────────────────────────────────
#define VL_MIN      0    // mm mínimo válido para VL53L0X
#define VL_MAX    1200    // mm máximo válido para VL53L0X
#define VL_ERR    8190    // valor de error que devuelve Adafruit
#define US_MIN     0    // mm mínimo válido para HC-SR04
#define US_MAX    3500    // mm máximo válido para HC-SR04

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

// ─── Buffers para filtro de mediana ──────────────────────────────────────────
uint16_t buf_vl[4][MEDIAN_N];
uint16_t buf_us[2][MEDIAN_N];
uint8_t  buf_idx = 0;

// ─── Temporización ───────────────────────────────────────────────────────────
unsigned long ultimo_ms = 0;

// ─── WiFi / UDP ────────────────────────────────────────────────────────────────
WiFiUDP   Udp;
IPAddress destIp;
bool          wifi_iniciado   = false;  // Udp.begin() ya corrió
unsigned long ultimo_intento  = 0;      // último intento de (re)conexión WiFi

// ═══════════════════════════════════════════════════════════════════════════════
// INTERRUPCIONES HC-SR04
// El ECHO sube cuando llega el pulso y baja cuando termina.
// La ISR mide exactamente cuánto duró sin bloquear el loop.
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
  Wire.setClock(400000);  // I2C rápido (400kHz)

  // Pines ultrasónicos
  pinMode(TRIG_1, OUTPUT); digitalWrite(TRIG_1, LOW);
  pinMode(TRIG_2, OUTPUT); digitalWrite(TRIG_2, LOW);
  pinMode(ECHO_1, INPUT_PULLUP);
  pinMode(ECHO_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ECHO_1), ISR_ECHO_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_2), ISR_ECHO_2, CHANGE);

  // Apagar todos los VL primero para asignar direcciones limpias
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

  // WiFi: parsear IP destino y lanzar la primera conexión (no bloqueante en el
  // loop — si falla se reintenta cada WIFI_RETRY_MS sin trabar los sensores).
#if ENABLE_WIFI
  destIp.fromString(DEST_IP);
  conectarWiFi();
#endif
}

// ═══════════════════════════════════════════════════════════════════════════════
// WiFi — conexión con timeout corto (no bloquea el loop indefinidamente)
// ═══════════════════════════════════════════════════════════════════════════════
void conectarWiFi() {
  ultimo_intento = millis();
  Serial.print("WiFi: conectando a "); Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Esperamos hasta 8s a asociarnos. Si no engancha, seguimos igual y el loop
  // reintenta más tarde — los sensores nunca dejan de leerse.
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Udp.begin(LOCAL_UDP_PORT);
    wifi_iniciado = true;
    Serial.print("WiFi OK. IP local: "); Serial.println(WiFi.localIP());
    Serial.print("Enviando UDP a "); Serial.print(DEST_IP);
    Serial.print(":"); Serial.println(DEST_PORT);
  } else {
    wifi_iniciado = false;
    Serial.println("WiFi: sin conexion (reintento luego, sigo por Serial)");
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// LOOP PRINCAL — nunca se bloquea
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  unsigned long ahora = millis();
  if (ahora - ultimo_ms < LOOP_MS) return;
  ultimo_ms = ahora;

  // 1. Disparar ultrasonidos
  //    No bloquea — la ISR captura el echo en segundo plano
  dispararUS(0, TRIG_1);
  dispararUS(1, TRIG_2);

  // 2. Leer VL53L0X — distancias crudas filtradas en mm
  uint16_t dvl[4];
  for (int i = 0; i < 4; i++) dvl[i] = leerVL(i);

  // 3. Leer HC-SR04 — resultado capturado por ISR en ciclo anterior
  uint16_t dus[2];
  for (int i = 0; i < 2; i++) dus[i] = leerUS(i);

  // 4. Armar el paquete con protocolo S...E
  //    Formato: SVL1,VL2,VL3,VL4,US1,US2E   (0 = sin lectura válida ese ciclo)
  char msg[48];
  int len = snprintf(msg, sizeof(msg), "S%u,%u,%u,%u,%u,%uE",
                        // VL1 izq-adelante
                     dvl[2],   // VL3 izq-atrás
                     dus[0],
                     dvl[3],
                     dvl[1],   // VL2 der-adelante
                     dus[1],   // US1 centro-adelante
                     dvl[0]);  // US2 centro-atrás

  // 4a. Serial (debug / fallback) — println agrega el \n que el parser tolera.
  Serial.println(msg);

  // 4b. WiFi UDP — el datagrama que consume el brainstem (:43899).
#if ENABLE_WIFI
  if (wifi_iniciado && WiFi.status() == WL_CONNECTED) {
    Udp.beginPacket(destIp, DEST_PORT);
    Udp.write((const uint8_t*)msg, len);
    Udp.endPacket();
  } else if (millis() - ultimo_intento >= WIFI_RETRY_MS) {
    // Caído o nunca conectó: reintentar sin bloquear el ritmo de sensado.
    conectarWiFi();
  }
#endif

  buf_idx++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// VL53L0X — inicialización y lectura con watchdog
// ═══════════════════════════════════════════════════════════════════════════════

// Enciende un VL, espera que arranque y le asigna su dirección I2C única
bool iniciarVL(uint8_t i) {
  digitalWrite(vl_xshut[i], HIGH);
  delay(10);
  if (!vl[i].begin(VL_ERR, false, &Wire)) {
    digitalWrite(vl_xshut[i], LOW);  // Apagar si falló
    return false;
  }
  vl[i].setAddress(vl_addr[i]);
  // Modo HIGH_SPEED: baja el timing budget de ~33ms a ~20ms por medición.
  // Con 4 sensores eso recorta el loop de ~130ms a ~80ms (de ~7Hz a ~12Hz),
  // que es lo que más impacta en la latencia del reflejo de frenado.
  // Si tu versión de la librería no tiene configSensor, comentá esta línea.
  vl[i].configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
  vl_fallos[i] = 0;
  return true;
}

// Lee un VL53L0X con filtro de mediana y watchdog
uint16_t leerVL(uint8_t i) {
  // Si el sensor estaba caído, intentar reiniciarlo
  if (!vl_ok[i]) {
    vl_ok[i] = iniciarVL(i);
    return 0;
  }

  VL53L0X_RangingMeasurementData_t m;
  vl[i].rangingTest(&m, false);
  uint16_t raw = m.RangeMilliMeter;

  // RangeStatus 4 = phase fail (vidrio, transparente, o sin objeto)
  bool invalido = (m.RangeStatus == 4)
               || (raw >= VL_ERR)
               || (raw < VL_MIN)
               || (raw > VL_MAX);

  if (invalido) {
    vl_fallos[i]++;
    if (vl_fallos[i] >= MAX_FALLOS) {
      // Reiniciar sensor sin trabar el resto del sistema
      digitalWrite(vl_xshut[i], LOW);
      delay(5);
      vl_ok[i]     = iniciarVL(i);
      vl_fallos[i] = 0;
    }
    // Devolver 0 — sin lectura válida este ciclo
    return 0;
  }

  vl_fallos[i] = 0;
  buf_vl[i][buf_idx % MEDIAN_N] = raw;
  return mediana(buf_vl[i]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// HC-SR04 — disparo no bloqueante + lectura desde ISR
// ═══════════════════════════════════════════════════════════════════════════════

// Manda el pulso de 10us al TRIG. La ISR mide el ECHO.
void dispararUS(uint8_t i, uint8_t trig) {
  us_listo[i] = false;
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
}

// Lee la distancia calculada por la ISR del ciclo anterior
uint16_t leerUS(uint8_t i) {
  if (!us_listo[i]) {
    // No llegó el echo — timeout o sensor desconectado
    us_fallos[i]++;
    if (us_fallos[i] >= MAX_FALLOS) us_fallos[i] = 0;
    return 0;
  }
  us_fallos[i] = 0;

  // Convertir duración del pulso a mm
  // Velocidad del sonido: 343 m/s → ida y vuelta ÷ 2
  uint16_t mm = (uint16_t)((us_duracion[i] * 343UL) / 2000UL);

  if (mm < US_MIN || mm > US_MAX) return 0;

  buf_us[i][buf_idx % MEDIAN_N] = mm;
  return mediana(buf_us[i]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// FILTRO DE MEDIANA — ignora ceros (lecturas inválidas)
// ═══════════════════════════════════════════════════════════════════════════════
uint16_t mediana(uint16_t *buf) {
  uint16_t tmp[MEDIAN_N];
  uint8_t n = 0;

  // Copiar solo valores válidos
  for (uint8_t i = 0; i < MEDIAN_N; i++) {
    if (buf[i] > 0) tmp[n++] = buf[i];
  }
  if (n == 0) return 0;

  // Ordenar de menor a mayor (burbuja, N es pequeño)
  for (uint8_t i = 0; i < n - 1; i++)
    for (uint8_t j = 0; j < n - i - 1; j++)
      if (tmp[j] > tmp[j+1]) {
        uint16_t t = tmp[j]; tmp[j] = tmp[j+1]; tmp[j+1] = t;
      }

  // Retornar el valor del medio
  return tmp[n / 2];
}
