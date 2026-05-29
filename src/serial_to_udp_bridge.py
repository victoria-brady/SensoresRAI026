#!/usr/bin/env python3
"""serial_to_udp_bridge.py

Puente Serial -> UDP para el array de sensores del robot RAI.

El Arduino GIGA (object_detect_V2.ino) manda una linea "S...E" por USB Serial
cada 50 ms. Este script, corriendo en la laptop que tiene el Arduino enchufado,
lee esas lineas y las reenvia como datagramas UDP al brainstem (que escucha en
kSensorUdpPort = 43899).

Asi el Arduino no necesita WiFi: el IP/puerto destino se configuran aca.

Uso:
    # mismo equipo que corre el brainstem (loopback):
    python3 serial_to_udp_bridge.py --port COM5

    # brainstem en otra maquina / Jetson en la red del robot:
    python3 serial_to_udp_bridge.py --port /dev/ttyACM0 --dest-ip 192.168.1.50

Flags:
    --port      Puerto serie del Arduino. Windows: COM5 / COM6 ...
                Linux/WSL: /dev/ttyACM0 / /dev/ttyUSB0
    --baud      Baudios (debe coincidir con Serial.begin del sketch). Default 115200.
    --dest-ip   IP donde corre el brainstem. Default 127.0.0.1 (mismo equipo).
    --dest-port Puerto UDP del brainstem. Default 43899 (= kSensorUdpPort).
    --echo      Imprime cada paquete reenviado (debug).

Requiere: pip install pyserial
"""

import argparse
import socket
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("Falta pyserial. Instalalo con:  pip install pyserial")


def looks_valid(line: str) -> bool:
    """Acepta solo lineas bien formadas 'S...E' (descarta ruido / parciales)."""
    return len(line) >= 3 and line[0] == "S" and line[-1] == "E"


def main() -> None:
    ap = argparse.ArgumentParser(description="Puente Serial->UDP de sensores RAI")
    ap.add_argument("--port", required=True, help="puerto serie del Arduino")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dest-ip", default="127.0.0.1",
                    help="IP del brainstem (default loopback)")
    ap.add_argument("--dest-port", type=int, default=43899,
                    help="puerto UDP del brainstem (= kSensorUdpPort)")
    ap.add_argument("--echo", action="store_true", help="imprime cada paquete")
    args = ap.parse_args()

    dest = (args.dest_ip, args.dest_port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[bridge] reenviando {args.port}@{args.baud} -> UDP "
          f"{args.dest_ip}:{args.dest_port}")

    sent = 0
    while True:
        try:
            # timeout corto: si el Arduino no manda, igual reintentamos limpio.
            with serial.Serial(args.port, args.baud, timeout=1) as ser:
                print(f"[bridge] serie abierto: {args.port}")
                while True:
                    raw = ser.readline()  # hasta '\n' (Serial.println lo agrega)
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="ignore").strip()
                    if not looks_valid(line):
                        continue
                    sock.sendto(line.encode("ascii"), dest)
                    sent += 1
                    if args.echo or sent == 1 or sent % 200 == 0:
                        print(f"[bridge] #{sent} {line}")
        except serial.SerialException as e:
            print(f"[bridge] serie caido ({e}); reintento en 2s...")
            time.sleep(2)
        except KeyboardInterrupt:
            print("\n[bridge] cerrando.")
            return


if __name__ == "__main__":
    main()
