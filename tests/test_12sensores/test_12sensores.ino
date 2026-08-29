#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ─────────────────────────────────────────────────────────────────────────────
// CONFIGURACIÓN - 6 HC-SR04 + 6 VL53L0X
// ─────────────────────────────────────────────────────────────────────────────

#define NUM_US  6
#define NUM_VL  6

// ─── Pines HC-SR04 (Ultrasónicos) ────────────────────────────────────────────
const uint8_t HC_TRIG[NUM_US] = {13, 8,  52, 40, 34, 26};
const uint8_t HC_ECHO[NUM_US] = {12, 9,  50, 36, 28, 24};

// ─── Pines VL53L0X (ToF) - XSHUT ─────────────────────────────────────────────
const uint8_t VL_XSHUT[NUM_VL] = {2, 3, 4, 5, 30, 32};

// ─── Direcciones I2C de los VL53L0X (dinámicamente asignadas) ─────────────────
const uint8_t VL_ADDR[NUM_VL] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};

// ─────────────────────────────────────────────────────────────────────────────
// VARIABLES - HC-SR04
// ─────────────────────────────────────────────────────────────────────────────
int16_t hc_distance[NUM_US]                = {-1};

// ─────────────────────────────────────────────────────────────────────────────
// VARIABLES - VL53L0X
// ─────────────────────────────────────────────────────────────────────────────

Adafruit_VL53L0X vl_sensors[NUM_VL];
bool vl_ok[NUM_VL] = {false};
int16_t vl_distance[NUM_VL] = {-1};

// ─────────────────────────────────────────────────────────────────────────────
// INICIALIZACIÓN
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n╔═══════════════════════════════════════════╗");
    Serial.println("║   TEST 12 SENSORES - RAI                  ║");
    Serial.println("║   6x HC-SR04 + 6x VL53L0X                ║");
    Serial.println("╚═══════════════════════════════════════════╝\n");
    
    // ──────────────────────────────────────────────────────────────────────────
    // INICIALIZAR HC-SR04
    // ──────────────────────────────────────────────────────────────────────────
    Serial.println("[HC-SR04] Inicializando...");
    for (int i = 0; i < NUM_US; i++) {
        pinMode(HC_TRIG[i], OUTPUT);
        digitalWrite(HC_TRIG[i], LOW);
        pinMode(HC_ECHO[i], INPUT_PULLUP);
    }

    Serial.println("[HC-SR04] ✓ OK\n");
    
    // ──────────────────────────────────────────────────────────────────────────
    // INICIALIZAR VL53L0X
    // ──────────────────────────────────────────────────────────────────────────
   // ──── INICIALIZAR VL53L0X CON TIMEOUT ────
    Serial.println("[VL53L0X] Inicializando 6 sensores ToF...\n");

    Wire.begin();
    Wire.setClock(100000);  // ← IMPORTANTE: Reducir I2C a 100kHz
    delay(50);

    // Apagar todos
    for (int i = 0; i < NUM_VL; i++) {
        pinMode(VL_XSHUT[i], OUTPUT);
        digitalWrite(VL_XSHUT[i], LOW);
        vl_ok[i] = false;
    }
    delay(50);

    // Encender con TIMEOUT manual
    for (int i = 0; i < NUM_VL; i++) {
        digitalWrite(VL_XSHUT[i], HIGH);
        delay(50);  // ← AUMENTAR a 50ms
        
        // Intentar begin() con límite de tiempo
        unsigned long start_time = millis();
        bool sensor_ok = false;
        
        while (millis() - start_time < 200) {  // ← 200ms timeout
            if (vl_sensors[i].begin(VL_ADDR[i], false, &Wire)) {
                vl_sensors[i].setAddress(VL_ADDR[i]);
                // Permite lecturas con retornos de láser más débiles en entornos ruidosos
                vl_sensors[i].setLimitCheckEnable(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, true);
                vl_sensors[i].setLimitCheckValue(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
                sensor_ok = true;
                break;
            }
            delay(20);
        }
        
        if (sensor_ok) {
            Serial.print("         [VL");
            Serial.print(i);
            Serial.print("] @0x");
            Serial.print(VL_ADDR[i], HEX);
            Serial.println(" ✓ OK");
            vl_ok[i] = true;
        } else {
            Serial.print("         [VL");
            Serial.print(i);
            Serial.print("] @0x");
            Serial.print(VL_ADDR[i], HEX);
            Serial.println(" ✗ TIMEOUT");
            digitalWrite(VL_XSHUT[i], LOW);
            vl_ok[i] = false;
        }
        
        delay(50);  // ← Delay entre sensores
    }
    
    Serial.println("\n╔═══════════════════════════════════════════╗");
    Serial.println("║   LEYENDO SENSORES...                     ║");
    Serial.println("╚═══════════════════════════════════════════╝\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// LECTURA HC-SR04
// ─────────────────────────────────────────────────────────────────────────────
void readAllUS_Polling() {
    for (int i = 0; i < NUM_US; i++) {
        // Disparar
        digitalWrite(HC_TRIG[i], LOW);
        delayMicroseconds(2);
        digitalWrite(HC_TRIG[i], HIGH);
        delayMicroseconds(10);
        digitalWrite(HC_TRIG[i], LOW);
        
        // Esperar y medir
        unsigned long timeout = micros() + 35000;  // 35ms timeout
        while (digitalRead(HC_ECHO[i]) == LOW && micros() < timeout);
        
        unsigned long start = micros();
        while (digitalRead(HC_ECHO[i]) == HIGH && micros() < timeout);
        unsigned long duration = micros() - start;
        
        // Calcular distancia
        int distance_cm = duration / 58;
        hc_distance[i] =  distance_cm;
        
        delay(10);  // Espacio entre lecturas
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LECTURA VL53L0X
// ─────────────────────────────────────────────────────────────────────────────

void readAllVL() {
    for (int i = 0; i < NUM_VL; i++) {
        if (!vl_ok[i]) {
            vl_distance[i] = -1;
            continue;
        }
        
        VL53L0X_RangingMeasurementData_t measurement;
        vl_sensors[i].rangingTest(&measurement, false);
    
        if (measurement.RangeStatus == 4) { //4 = out of range posta
            vl_distance[i] = -1;
        } else {
            vl_distance[i] = measurement.RangeMilliMeter / 10; // Distancia en cm
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
    // Leer sensores
    readAllUS_Polling();
    readAllVL();
    
    // ──────────────────────────────────────────────────────────────────────────
    // IMPRIMIR EN UNA LÍNEA
    // ──────────────────────────────────────────────────────────────────────────
    Serial.print("HC[");
    for (int i = 0; i < NUM_US; i++) {
        if (hc_distance[i] == -1) {
            Serial.print("X");
        } else {
            Serial.print(hc_distance[i]);
        }
        if (i < NUM_US - 1) Serial.print(",");
    }
    Serial.print("] | VL[");
    for (int i = 0; i < NUM_VL; i++) {
        if (vl_distance[i] == -1) {
            Serial.print("X");
        } else {
            Serial.print(vl_distance[i]);
        }
        if (i < NUM_VL - 1) Serial.print(",");
    }
    Serial.println("]");
    
    delay(50);  // ~20 Hz
}
