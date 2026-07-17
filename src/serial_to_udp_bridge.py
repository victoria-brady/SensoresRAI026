#!/usr/bin/env python3
"""serial_to_udp_bridge.py

Puente Serial -> UDP para el array de sensores del robot RAI.

Corre en la PC (Windows) que tiene el Arduino GIGA enchufado por USB. El Arduino
(object_detect_V2.ino) manda una linea "S...E" por Serial cada 50 ms; este
script las lee y las reenvia como datagramas UDP a la PC que corre el brainstem
(que escucha en kSensorUdpPort = 43899). Asi el Arduino no necesita WiFi.

IMPORTANTE (Windows): el puerto COM es exclusivo. Cerra el Monitor Serie del
Arduino IDE antes de correr este script, o vas a ver "Access is denied".

Uso tipico (Windows, brainstem en otra PC de la red del robot):
    python serial_to_udp_bridge.py --list                       # ver puertos COM
    python serial_to_udp_bridge.py --port COM5 --dest-ip 192.168.1.50

Flags:
    --list      Lista los puertos serie disponibles y sale.
    --port      Puerto serie del Arduino. Windows: COM5 / COM6 ...
                Linux/WSL: /dev/ttyACM0 / /dev/ttyUSB0
    --baud      Baudios (debe coincidir con Serial.begin del sketch). Default 115200.
    --dest-ip   IP de la PC que corre el brainstem. Default 127.0.0.1 (mismo equipo).
    --dest-port Puerto UDP del brainstem. Default 43899 (= kSensorUdpPort).
    --echo      Imprime cada paquete reenviado (debug).

Requiere: pip install pyserial   (usar el Python de Windows, no el de WSL)
"""

import argparse
import socket
import sys
import time

try:
    import serial  # pyserial
    from serial.tools import list_ports
except ImportError:
    sys.exit("Falta pyserial. Instalalo con (Python de Windows):  pip install pyserial")


def listar_puertos() -> None:
    puertos = list(list_ports.comports())
    if not puertos:
        print("No se detectaron puertos serie. ¿Esta enchufado el Arduino?")
        return
    print("Puertos serie detectados:")
    for p in puertos:
        print(f"  {p.device:10s}  {p.description}")


def looks_valid(line: str) -> bool:
    """Acepta solo lineas bien formadas 'S...E' (descarta ruido / parciales)."""
    return len(line) >= 3 and line[0] == "S" and line[-1] == "E"


def main() -> None:
    ap = argparse.ArgumentParser(description="Puente Serial->UDP de sensores RAI")
    ap.add_argument("--list", action="store_true", help="lista puertos COM y sale")
    ap.add_argument("--port", help="puerto serie del Arduino (ej. COM5)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dest-ip", default="127.0.0.1",
                    help="IP del brainstem (default loopback)")
    ap.add_argument("--dest-port", type=int, default=43899,
                    help="puerto UDP del brainstem (= kSensorUdpPort)")
    ap.add_argument("--echo", action="store_true", help="imprime cada paquete")
    args = ap.parse_args()

    if args.list:
        listar_puertos()
        return

    if not args.port:
        print("Falta --port. Corre con --list para ver los puertos disponibles.")
        listar_puertos()
        sys.exit(2)

    dest = (args.dest_ip, args.dest_port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[bridge] reenviando {args.port}@{args.baud} -> UDP "
          f"{args.dest_ip}:{args.dest_port}")
    print("[bridge] (cerra el Monitor Serie del Arduino IDE si ves 'Access is denied')")

    sent = 0
    rate_t0 = time.time()
    rate_n0 = 0
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

                    # Log throttleado. Imprimir CADA paquete satura la consola de
                    # Windows, y si la consola se traba (seleccionar texto en cmd
                    # la congela) el print bloquea el loop: el serial se encola y
                    # los datagramas salen despues en rafaga, viejos — un delay
                    # que se confunde con "el sensor frena tarde". Con --echo se
                    # ve todo; sin --echo, solo el primer paquete.
                    if args.echo:
                        print(f"[bridge] #{sent} {line}")
                    elif sent == 1:
                        print(f"[bridge] #1 (primer paquete OK) {line}")

                    # Tasa cada 5 s. Compara este pkt/s con el 'dt=' que imprime
                    # el brainstem (sensor_thread): si el bridge manda 15 pkt/s
                    # pero el brainstem ve ~5, hay perdida/retraso en la red.
                    now = time.time()
                    if now - rate_t0 >= 5.0:
                        rate = (sent - rate_n0) / (now - rate_t0)
                        print(f"[bridge] {rate:.1f} pkt/s (total {sent})")
                        rate_t0 = now
                        rate_n0 = sent
        except serial.SerialException as e:
            print(f"[bridge] no pude abrir/leer {args.port} ({e}); reintento en 2s...")
            time.sleep(2)
        except KeyboardInterrupt:
            print("\n[bridge] cerrando.")
            return


if __name__ == "__main__":
    main()
