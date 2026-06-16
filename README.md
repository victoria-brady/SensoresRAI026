# SensoresRAI026 — Array de sensores de proximidad para RAI

Firmware y herramientas del **array de sensores de distancia** del robot perro
cuadrúpedo **RAI** (ITBA, RAI 2026).

El sistema lee **4 sensores láser ToF VL53L0X** + **2 sensores ultrasónicos
HC-SR04** sobre un **Arduino GIGA R1 WiFi**, filtra las lecturas y emite las
distancias (en mm) por **USB Serial**. Una PC con el Arduino enchufado corre un
puente Python que reenvía esos datos por **UDP** al *brainstem* del robot, que
los usa para el reflejo de detección/frenado ante obstáculos.

```
   ┌──────────────┐   USB Serial    ┌─────────────────────────┐    UDP :43899   ┌────────────┐
   │ Arduino GIGA │  115200 baud    │ PC con el Arduino:       │   datagramas    │ brainstem  │
   │  R1 WiFi     │ ───────────────▶│ serial_to_udp_bridge.py │ ───────────────▶│ del robot  │
   │ (firmware)   │  CSV de 6 dist. │ (lee Serial, manda UDP) │                 │            │
   └──────────────┘                 └─────────────────────────┘                 └────────────┘
        ▲
   4× VL53L0X (I2C) + 2× HC-SR04 (interrupción)
```

---

## Versión activa

> **La versión activa y de referencia es [`src/object_detect_V3/`](src/object_detect_V3/).**

Es la última iteración (junio 2026) y la única con arquitectura **modular**
(drivers, filtro, scheduler y empaquetado en archivos separados). Las versiones
anteriores se conservan como histórico — ver [Historial de versiones](#historial-de-versiones).

> ⚠️ **Gap conocido de integración.** El firmware V3 emite el CSV *sin* el marco
> `S...E`, pero el puente `serial_to_udp_bridge.py` todavía descarta toda línea
> que no empiece con `S` y termine con `E` (protocolo de la V2). Tal cual están
> hoy, **V3 + bridge no se hablan**: el bridge filtra el 100% de los paquetes.
> Ver [Integración Serial → UDP](#integración-serial--udp).

---

## Hardware

| Componente            | Cantidad | Bus / Pines                                  |
|-----------------------|----------|----------------------------------------------|
| Arduino GIGA R1 WiFi  | 1        | Placa principal                              |
| VL53L0X (láser ToF)   | 4        | I2C @ 400 kHz · XSHUT en D2, D3, D4, D5      |
| HC-SR04 (ultrasónico) | 2        | TRIG D8 / ECHO D9 · TRIG D13 / ECHO D12      |

- Cada VL53L0X recibe una dirección I2C única en el arranque (`0x30`–`0x33`),
  encendiéndolos de a uno por su pin XSHUT.
- Los HC-SR04 se miden por **interrupción** sobre el pin ECHO (CHANGE), así el
  loop nunca se bloquea esperando el eco.
- El diseño de la placa (KiCad, 6 sensores) está en [`PCB/`](PCB/).

---

## Estructura del repositorio

```
src/
  object_detect_V3/        ← VERSIÓN ACTIVA (firmware modular)
  object_detect_V2/        firmware monolítico con WiFi/UDP opcional (histórico)
  object_detect_V1/        primer firmware monolítico, solo Serial (histórico)
  sensorsRAI_V1/           prototipo dual (2 VL + 1 US), bloqueante (histórico)
  sensorsRAI_V2/           duplicado de object_detect_V2 (artefacto de rename)
  sensorsRAI_BLW.ino/      prototipo por Bluetooth LE (histórico)
  sensorRAI_BLR.py         cliente Python del prototipo BLE (histórico)
  serial_to_udp_bridge.py  puente Serial → UDP para el brainstem
tests/                     sketches de validación de hardware (sensores sueltos)
PCB/                       proyecto KiCad de la placa (6 sensores)
docs/                      links a datasheets, Drive y planos de conexión
```

---

## Firmware activo: `object_detect_V3`

Arquitectura modular. Cada `.ino`/`.cpp` tiene una responsabilidad:

| Archivo                  | Responsabilidad                                                        |
|--------------------------|-----------------------------------------------------------------------|
| `object_detect_V3.ino`   | `setup()` + `loop()`; orquesta el ciclo de lectura vía scheduler       |
| `config.h`               | Toda la configuración: pines, direcciones I2C, Nº de sensores, límites |
| `VL53L0X.cpp/.h`         | Init secuencial con dirección I2C única + *watchdog* por sensor        |
| `HC-SR04.cpp/.h`         | Disparo no bloqueante + medición del eco por ISR                       |
| `filter.cpp/.h`          | Filtro de media con rechazo por desviación estándar                    |
| `PacketPush.cpp/.h`      | `struct SensorPacket` + envío del CSV por Serial                       |
| `my_timer.cpp/.h`        | Scheduler cooperativo genérico (`runTaskPeriodically`)                 |

### Cómo funciona

Cada `LOOP_MS` (50 ms ≈ 20 Hz) el scheduler ejecuta `procesarSensores()`, que:

1. Dispara los dos ultrasonidos (el eco lo captura la ISR en segundo plano).
2. Lee y filtra los 4 láseres.
3. Lee y filtra los 2 ultrasonidos (datos del eco del ciclo anterior).
4. Envía el paquete por Serial.

**Filtro (`filter.cpp`):** mantiene un buffer circular de `FILTER_N` muestras por
sensor, ignora los ceros (lecturas inválidas) y devuelve la **media**. Si la
**desviación estándar** del buffer supera `MAX_STD_DEV_*` (lecturas caóticas /
pico de ruido), amortigua a `0` en vez de propagar basura.

**Watchdog (`VL53L0X.cpp`):** un sensor que entrega `MAX_FALLOS` lecturas
inválidas seguidas se apaga y reinicia solo, sin trabar al resto del array.

### Parámetros (`config.h`)

| Parámetro                  | Valor    | Significado                                       |
|----------------------------|----------|---------------------------------------------------|
| `LOOP_MS`                  | 50       | Período del ciclo (~20 Hz)                         |
| `FILTER_N`                 | 5        | Muestras del filtro por sensor                     |
| `MAX_FALLOS`               | 5        | Fallos seguidos antes de reiniciar un sensor       |
| `NUM_VL` / `NUM_US`        | 4 / 2    | Cantidad de láseres / ultrasonidos                 |
| `MAX_STD_DEV_VL` / `_US`   | 500 / 500| Desvío estándar máx. tolerado (mm) antes de cortar |
| `VL_MIN`..`VL_MAX`         | 0..9000  | Rango válido del láser (`VL_ERR` = 8190)           |
| `US_MIN`..`US_MAX`         | 0..9000  | Rango válido del ultrasonido                        |

### Formato de salida

Línea de texto ASCII por ciclo: **6 enteros (mm) separados por coma**, terminada
en `\n`, a **115200 baud**. `0` = sin lectura válida ese ciclo.

El orden de los campos **no** es FL,FC,FR,... sino el que define
[`PacketPush.cpp`](src/object_detect_V3/PacketPush.cpp):

| Campo CSV | Fuente  | Posición física              |
|-----------|---------|------------------------------|
| 1         | `vl[2]` | **BR** — atrás-derecha       |
| 2         | `us[0]` | **FC** — adelante-centro     |
| 3         | `vl[3]` | **BL** — atrás-izquierda     |
| 4         | `vl[1]` | **FR** — adelante-derecha    |
| 5         | `us[1]` | **BC** — atrás-centro        |
| 6         | `vl[0]` | **FL** — adelante-izquierda  |

Ejemplo: `342,654,0,891,0,1205`

### Compilar y cargar

1. Arduino IDE con soporte para **Arduino GIGA R1** (core *Arduino Mbed OS GIGA*).
2. Instalar la librería **Adafruit_VL53L0X** (Library Manager).
3. Abrir `src/object_detect_V3/object_detect_V3.ino` (el IDE toma todos los `.cpp/.h`
   de la carpeta) y cargar a la placa.
4. Verificar las lecturas en el Monitor Serie a **115200 baud**.

---

## Integración Serial → UDP

[`src/serial_to_udp_bridge.py`](src/serial_to_udp_bridge.py) corre en la PC que
tiene el Arduino conectado por USB: lee las líneas del Serial y las reenvía como
datagramas UDP al *brainstem* (puerto **43899** = `kSensorUdpPort`). Así el
Arduino no necesita WiFi.

```bash
pip install pyserial

# Ver puertos disponibles
python src/serial_to_udp_bridge.py --list

# Reenviar COM5 → brainstem en otra PC de la red del robot
python src/serial_to_udp_bridge.py --port COM5 --dest-ip 192.168.1.50
```

| Flag          | Default     | Descripción                                       |
|---------------|-------------|---------------------------------------------------|
| `--list`      | —           | Lista los puertos serie y sale                    |
| `--port`      | (requerido) | Puerto del Arduino (`COM5`, `/dev/ttyACM0`, …)    |
| `--baud`      | 115200      | Debe coincidir con `Serial.begin()` del firmware  |
| `--dest-ip`   | 127.0.0.1   | IP de la PC que corre el brainstem                |
| `--dest-port` | 43899       | Puerto UDP del brainstem                          |
| `--echo`      | —           | Imprime cada paquete reenviado (debug)            |

> **Windows:** el puerto COM es exclusivo. Cerrá el Monitor Serie del Arduino IDE
> antes de correr el bridge o vas a ver *"Access is denied"*.

> ⚠️ **Para que V3 funcione con el bridge** hay que cerrar el gap del marco `S...E`:
> o bien **re-activar** el frame en `PacketPush.cpp` (las líneas `Serial.print('S')`
> / `Serial.println('E')` están comentadas), o bien **relajar** `looks_valid()` en
> el bridge para que acepte CSV pelado. Hoy el bridge espera el formato de la V2.

---

## Historial de versiones

| Versión                | Sensores        | Transporte            | Filtro            | Estado                              |
|------------------------|-----------------|-----------------------|-------------------|-------------------------------------|
| **`object_detect_V3`** | 4 VL + 2 US     | USB Serial            | media + σ (std)   | **Activa** (modular)                |
| `object_detect_V2`     | 4 VL + 2 US     | Serial + WiFi/UDP opc.| mediana (N=3)     | Histórico — compatible con el bridge |
| `object_detect_V1`     | 4 VL + 2 US     | USB Serial            | mediana (N=5)     | Histórico                            |
| `sensorsRAI_V1`        | 2 VL + 1 US     | Serial (texto legible)| —                 | Prototipo bloqueante                 |
| `sensorsRAI_BLW` + `sensorRAI_BLR.py` | 2 VL + 1 US | Bluetooth LE | — | Prototipo abandonado                 |
| `sensorsRAI_V2`        | —               | —                     | —                 | Duplicado de V2 (artefacto de rename)|

- **V1 → V2:** se agregó WiFi/UDP opcional (`ENABLE_WIFI`), modo `HIGH_SPEED` del
  láser, y se bajó la mediana de 5 a 3 muestras (menor latencia ante un obstáculo
  súbito). La V2 mantiene el protocolo `S...E` que el bridge espera.
- **V2 → V3:** refactor a módulos, scheduler cooperativo, y cambio de filtro
  (mediana → media con rechazo por desviación estándar). Se quitó el WiFi: V3 es
  solo Serial (la salida a UDP queda a cargo del bridge en la PC).

---

## Tests (`tests/`)

Sketches independientes para validar hardware suelto antes de integrar:

- `HCSR04_single/` — un único HC-SR04.
- `vl53l0x_dual/` — dos VL53L0X (ejemplo Adafruit de direccionamiento I2C).
- `test4VL/` — driver propio para N VL53L0X. *Nota:* `config.h` declara
  `NUMSENSORS 4` pero `XSHUT_PINS` solo lista 2 pines `{5, 4}` — completar antes
  de usar con 4 sensores.

---

## Documentación

- [`docs/Arduino Giga/`](docs/Arduino%20Giga/) — datasheet y manual del GIGA R1.
- [`docs/Sensores/`](docs/Sensores/) — planos de conexión y carpeta de Drive.
- [`PCB/`](PCB/) — proyecto KiCad de la placa de 6 sensores.
