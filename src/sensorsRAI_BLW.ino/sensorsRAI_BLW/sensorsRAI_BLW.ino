#include "Adafruit_VL53L0X.h"
#include <ArduinoBLE.h>

// Definir el Servicio y la Característica UUID
BLEService gigaService("19B10000-E8F2-537E-4F6C-D104768A1214"); 
// Aumentamos el tamaño máximo a 30 caracteres para que entren los 3 datos
BLEStringCharacteristic gigaChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 30);

#define TRIG 9
#define ECHO 8

int duration;
float distance_US;

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define SHT_LOX1 5
#define SHT_LOX2 4

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

void setID() {
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);

  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  if (!lox1.begin(LOX1_ADDRESS)) {
    //Serial.println(F("Failed to boot first VL53L0X"));
    while (1);
  }
  delay(10);

  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  if (!lox2.begin(LOX2_ADDRESS)) {
    //Serial.println(F("Failed to boot second VL53L0X"));
    while (1);
  }
}

void read_dual_sensors() {
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
}

void setup() {
  Serial.begin(115200);

  if (!BLE.begin()) {
    //Serial.println("¡Error al iniciar Bluetooth!");
    while (1);
  }

  BLE.setLocalName("ArduinoGigaR1");
  BLE.setAdvertisedService(gigaService);
  gigaService.addCharacteristic(gigaChar);
  BLE.addService(gigaService);

  BLE.advertise();
  //Serial.println("Bluetooth activo, esperando conexión...");
  
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  setID();

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT_PULLUP);
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    //Serial.print("Conectado a: ");
    //Serial.println(central.address());

    while (central.connected()) {
      // 1. Leemos los sensores VL53L0X
      read_dual_sensors();
      
      // Manejamos casos donde estén fuera de rango (-1 será nuestro código de error)
      int dist_vl1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : -1;
      int dist_vl2 = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : -1;

      // 2. Leemos el ultrasónico
      digitalWrite(TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG, LOW);
      duration = pulseIn(ECHO, HIGH, 30000); //30ms de timeout
      
      // Velocidad del sonido en mm/us (te dará la distancia en milímetros)
      distance_US = duration * 0.34 / 2; 

      // 3. Armamos el paquete de datos separado por comas
      String datos = String(dist_vl1) + "," + String(dist_vl2) + "," + String(distance_US, 1);
      
      // Imprimimos en Arduino para verificar
      //Serial.println("Enviando: " + datos);

      // 4. Enviamos por Bluetooth
      gigaChar.writeValue(datos);
      
      delay(200); // 2 veces por segundo es un buen ritmo para BLE
    }
   // Serial.println("Desconectado de la PC");
  }
}