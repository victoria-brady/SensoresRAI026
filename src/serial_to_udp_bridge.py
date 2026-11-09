#!/usr/bin/env python3
"""serial_to_udp_bridge.py

Serial -> UDP bridge for the RAI robot sensor array.

Runs on the PC (Windows) that has the Arduino GIGA plugged in over USB. The Arduino
(object_detect_V2.ino) sends an "S...E" line over Serial every 50 ms; this
script reads them and forwards them as UDP datagrams to the PC running the brainstem
(which listens on kSensorUdpPort = 43899). This way the Arduino does not need WiFi.

IMPORTANT (Windows): the COM port is exclusive. Close the Arduino IDE Serial
Monitor before running this script, or you will see "Access is denied".

Typical usage (Windows, brainstem on another PC on the robot network):
    python serial_to_udp_bridge.py --list                       # list COM ports
    python serial_to_udp_bridge.py --port COM5 --dest-ip 192.168.1.50

Flags:
    --list      Lists the available serial ports and exits.
    --port      Arduino serial port. Windows: COM5 / COM6 ...
                Linux/WSL: /dev/ttyACM0 / /dev/ttyUSB0
    --baud      Baud rate (must match Serial.begin in the sketch). Default 115200.
    --dest-ip   IP of the PC running the brainstem. Default 127.0.0.1 (same machine).
    --dest-port UDP port of the brainstem. Default 43899 (= kSensorUdpPort).
    --echo      Prints every forwarded packet (debug).

Requires: pip install pyserial   (use the Windows Python, not the WSL one)
"""

import argparse
import socket
import sys
import time

try:
    import serial  # pyserial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is missing. Install it with (Windows Python):  pip install pyserial")


def list_available_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports detected. Is the Arduino plugged in?")
        return
    print("Serial ports detected:")
    for p in ports:
        print(f"  {p.device:10s}  {p.description}")


def looks_valid(line: str) -> bool:
    """Accepts only well-formed 'S...E' lines (discards noise / partial frames)."""
    return len(line) >= 3 and line[0] == "S" and line[-1] == "E"


def main() -> None:
    ap = argparse.ArgumentParser(description="RAI sensors Serial->UDP bridge")
    ap.add_argument("--list", action="store_true", help="list COM ports and exit")
    ap.add_argument("--port", help="Arduino serial port (e.g. COM5)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dest-ip", default="127.0.0.1",
                    help="brainstem IP (default loopback)")
    ap.add_argument("--dest-port", type=int, default=43899,
                    help="brainstem UDP port (= kSensorUdpPort)")
    ap.add_argument("--echo", action="store_true", help="print every packet")
    args = ap.parse_args()

    if args.list:
        list_available_ports()
        return

    if not args.port:
        print("Missing --port. Run with --list to see the available ports.")
        list_available_ports()
        sys.exit(2)

    dest = (args.dest_ip, args.dest_port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[bridge] forwarding {args.port}@{args.baud} -> UDP "
          f"{args.dest_ip}:{args.dest_port}")
    print("[bridge] (close the Arduino IDE Serial Monitor if you see 'Access is denied')")

    sent = 0
    rate_t0 = time.time()
    rate_n0 = 0
    while True:
        try:
            # short timeout: if the Arduino sends nothing, we still retry cleanly.
            with serial.Serial(args.port, args.baud, timeout=1) as ser:
                print(f"[bridge] serial port opened: {args.port}")
                while True:
                    raw = ser.readline()  # up to '\n' (Serial.println adds it)
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="ignore").strip()
                    if not looks_valid(line):
                        continue
                    sock.sendto(line.encode("ascii"), dest)
                    sent += 1

                    # Throttled log. Printing EVERY packet floods the Windows
                    # console, and if the console stalls (selecting text in cmd
                    # freezes it) the print blocks the loop: serial data queues up
                    # and datagrams go out later in a burst, stale -- a delay
                    # that gets mistaken for "the sensor brakes late". With --echo
                    # everything is shown; without --echo, only the first packet.
                    if args.echo:
                        print(f"[bridge] #{sent} {line}")
                    elif sent == 1:
                        print(f"[bridge] #1 (first packet OK) {line}")

                    # Rate every 5 s. Compare this pkt/s with the 'dt=' printed
                    # by the brainstem (sensor_thread): if the bridge sends 15 pkt/s
                    # but the brainstem sees ~5, there is network loss/delay.
                    now = time.time()
                    if now - rate_t0 >= 5.0:
                        rate = (sent - rate_n0) / (now - rate_t0)
                        print(f"[bridge] {rate:.1f} pkt/s (total {sent})")
                        rate_t0 = now
                        rate_n0 = sent
        except serial.SerialException as e:
            print(f"[bridge] could not open/read {args.port} ({e}); retrying in 2s...")
            time.sleep(2)
        except KeyboardInterrupt:
            print("\n[bridge] shutting down.")
            return


if __name__ == "__main__":
    main()
