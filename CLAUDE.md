# CLAUDE.md

Guía para agentes que trabajen en este repositorio. Resumen operativo; ante una
duda de detalle, la fuente de verdad es el código en `src/object_detect_V3/`.

## Qué es esto

Firmware + herramientas del **array de sensores de proximidad** del robot perro
cuadrúpedo **RAI** (ITBA, RAI 2026). Un **Arduino GIGA R1 WiFi** lee 4 láseres
ToF VL53L0X (I2C) + 2 ultrasónicos HC-SR04, filtra y emite distancias en mm por
USB Serial. En la PC, `serial_to_udp_bridge.py` reenvía esos datos por UDP al
*brainstem* del robot (puerto 43899), que dispara el reflejo de frenado.

Cadena de datos: **Arduino —(USB Serial 115200)→ bridge Python —(UDP :43899)→ brainstem**.

## Versión activa — empezá acá

**`src/object_detect_V3/`** es la versión de referencia (junio 2026, modular).
Todo trabajo de firmware nuevo va sobre V3. El resto de `src/` es histórico
(ver tabla abajo) — no lo edites salvo pedido explícito.

`main` y `serial_integration` tienen contenido idéntico hoy (la rama de trabajo
suele ser `serial_integration`).

## Mapa del código (V3)

| Archivo                | Responsabilidad |
|------------------------|-----------------|
| `object_detect_V3.ino` | `setup()`/`loop()`; `procesarSensores()` orquesta el ciclo vía scheduler |
| `config.h`             | **Única fuente de config**: pines, direcciones I2C, `NUM_*`, filtro, límites |
| `VL53L0X.cpp/.h`       | Init con dirección I2C única por XSHUT + watchdog (reinicia sensor tras `MAX_FALLOS`) |
| `HC-SR04.cpp/.h`       | Disparo no bloqueante; el eco se mide en ISR sobre ECHO (CHANGE) |
| `filter.cpp/.h`        | Buffer circular `FILTER_N`; media ignorando ceros; corta a 0 si σ > `MAX_STD_DEV_*` |
| `PacketPush.cpp/.h`    | `struct SensorPacket {vl[NUM_VL]; us[NUM_US];}` + serialización CSV |
| `my_timer.cpp/.h`      | `runTaskPeriodically(cb, ms)` — scheduler cooperativo (máx. 5 tareas) |

Flujo de un ciclo (`procesarSensores`): disparar US → leer+filtrar VL → leer+filtrar US
(eco del ciclo previo) → `sendSerialPacket`.

## Formato de salida (contrato con el bridge/brainstem)

CSV de 6 enteros en mm, `\n` al final, 115200 baud. `0` = sin lectura válida.
El orden lo fija `PacketPush.cpp` y **no** es secuencial por índice:

| Campo | Fuente  | Posición |
|-------|---------|----------|
| 1 | `vl[2]` | BR atrás-derecha |
| 2 | `us[0]` | FC adelante-centro |
| 3 | `vl[3]` | BL atrás-izquierda |
| 4 | `vl[1]` | FR adelante-derecha |
| 5 | `us[1]` | BC atrás-centro |
| 6 | `vl[0]` | FL adelante-izquierda |

Si tocás este orden, actualizá en simultáneo el parser del brainstem y esta tabla.

## Comandos

Firmware (Arduino IDE, core *Arduino Mbed OS GIGA*, librería *Adafruit_VL53L0X*):
abrir `src/object_detect_V3/object_detect_V3.ino` y cargar. No hay build por CLI
configurado en el repo. Para verificar, Monitor Serie a 115200.

Bridge (Python, requiere `pip install pyserial`):
```bash
python src/serial_to_udp_bridge.py --list                       # listar puertos
python src/serial_to_udp_bridge.py --port COM5 --dest-ip <ip>   # reenviar a UDP :43899
```

## Trampas / cosas a saber (IMPORTANTE)

- **Gap V3 ↔ bridge (frame `S...E`).** `serial_to_udp_bridge.py::looks_valid()`
  exige que cada línea empiece con `S` y termine con `E` (protocolo de la **V2**).
  Pero en V3 `PacketPush.cpp` esas escrituras (`Serial.print('S')` /
  `Serial.println('E')`) están **comentadas** → emite CSV pelado. Consecuencia:
  el bridge descarta el 100% de los paquetes de V3. Para integrar: re-activar el
  frame en `PacketPush.cpp` **o** relajar `looks_valid()`. Tenelo presente antes
  de afirmar que la cadena completa "funciona".
- **V3 no tiene WiFi.** La salida directa por UDP desde el Arduino solo existe en
  V2 (`ENABLE_WIFI`). En V3 el UDP lo hace el bridge en la PC.
- **`config.h` es la única perilla.** Cambiá pines / Nº de sensores / umbrales ahí,
  no en los `.cpp`. Los arrays (`vl_xshut`, `vl_addr`, etc.) están dimensionados a
  `NUM_VL`/`NUM_US`; si cambiás esas constantes, revisá los inicializadores.
- **`sensorsRAI_V2/` es un duplicado** de `object_detect_V2` (artefacto de un
  rename). No lo confundas con una versión propia.
- **`tests/test4VL/config.h`** declara `NUMSENSORS 4` pero `XSHUT_PINS` solo tiene
  2 pines — bug latente del sketch de prueba.
- **Idioma:** todo el proyecto (código, comentarios, commits, docs) está en
  **español**. Mantené ese idioma al escribir código y documentación.

## Historial de versiones (`src/`)

| Carpeta                | Resumen | Estado |
|------------------------|---------|--------|
| `object_detect_V3/`    | Modular; filtro media+σ; scheduler; solo Serial | **Activa** |
| `object_detect_V2/`    | Monolítico; WiFi/UDP opcional; mediana N=3; HIGH_SPEED; protocolo `S...E` | Histórico (compatible con bridge) |
| `object_detect_V1/`    | Monolítico; solo Serial; mediana N=5; `S...E` | Histórico |
| `sensorsRAI_V1/`       | Prototipo 2 VL + 1 US, bloqueante (`pulseIn`), salida legible | Prototipo |
| `sensorsRAI_BLW.ino/` + `sensorRAI_BLR.py` | Prototipo Bluetooth LE (ArduinoBLE / bleak) | Abandonado |
| `sensorsRAI_V2/`       | Duplicado de `object_detect_V2` | Artefacto |

## Estructura del repo

- `src/` — firmware (versiones) + bridge Python.
- `tests/` — sketches de validación de sensores sueltos (`HCSR04_single`, `vl53l0x_dual`, `test4VL`).
- `PCB/` — proyecto KiCad de la placa de 6 sensores (con backups y PDFs).
- `docs/` — links a datasheet del GIGA R1, planos de conexión y carpeta de Drive.

## Alcance y convenciones de trabajo

- No reescribas firmware histórico (V1/V2/`sensorsRAI_*`) salvo pedido explícito.
- Hardware embebido: en el `loop()`/ISR nada de bloqueos largos ni `delay()` en el
  camino caliente (rompe el ritmo de ~20 Hz y la latencia del reflejo de frenado).
- Si cambiás el formato de salida, el período (`LOOP_MS`) o el baudrate, son
  cambios de **contrato**: documentalos y avisá del impacto en bridge y brainstem.
