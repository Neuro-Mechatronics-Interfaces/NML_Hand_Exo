"""
example_bluetooth_exo.py
------------------------
Demonstrates connecting to the NML Hand Exoskeleton over Bluetooth (HC-05 module).

Hardware setup
~~~~~~~~~~~~~~
1. Pair the HC-05 module with your PC via the OS Bluetooth settings.
   Default PIN: 1234
2. After pairing, a virtual COM port is assigned (e.g. COM8 on Windows,
   /dev/rfcomm0 on Linux).  Check Device Manager → Ports (COM & LPT).
3. Make sure the HC-05 baud rate matches HC05_BAUD_RATE in config.h (115200).
   If you have not configured the module yet, use AT commands:
       AT+UART=115200,0,0

Usage
~~~~~
    # List detected Bluetooth COM ports
    python example_bluetooth_exo.py --list

    # Connect and print device info
    python example_bluetooth_exo.py --port COM8

    # Explicit baud rate (must match HC-05 AT+UART setting)
    python example_bluetooth_exo.py --port COM8 --baud 115200
"""
from __future__ import annotations

import argparse
import sys

from serial.tools import list_ports

from nml_hand_exo.interface import HandExo, SerialComm


def bluetooth_ports() -> list[str]:
    """Return serial ports whose OS metadata identifies Bluetooth."""
    matches = []
    for port in list_ports.comports():
        text = f"{port.description} {port.hwid}".lower()
        if "bluetooth" in text or "bthenum" in text or "rfcomm" in text:
            matches.append(port.device)
    return matches


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="NML Hand Exo — Bluetooth (HC-05) example",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", metavar="PORT",
                        help="Bluetooth virtual COM port (e.g. COM8 or /dev/rfcomm0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (must match HC-05 AT+UART setting)")
    parser.add_argument("--list", action="store_true",
                        help="List detected Bluetooth COM ports and exit")
    args = parser.parse_args(argv)

    if args.list:
        bt_ports = bluetooth_ports()
        if bt_ports:
            print("Detected Bluetooth COM ports:")
            for p in bt_ports:
                print(f"  {p}")
        else:
            print("No Bluetooth COM ports detected. Make sure the HC-05 is paired.")
        return 0

    if not args.port:
        print("ERROR: Supply --port PORT or use --list to discover available ports.",
              file=sys.stderr)
        return 1

    comm = SerialComm(port=args.port, baudrate=args.baud)
    exo = HandExo(comm, verbose=True)

    try:
        print(f"Connecting to exo via Bluetooth on {args.port} @ {args.baud} baud...")
        exo.connect()
        print("Connected.\n")

        print("Firmware version :", exo.version())
        print("Exo mode         :", exo.get_exo_mode())
        print("Home positions   :", exo.get_home())
        print("Device info      :", exo.info())
        print("Motor angles     :", exo.get_motor_angle())
        print("Absolute angles  :", exo.get_absolute_motor_angle())
        print("Gesture          :", exo.get_gesture())
        print("Gesture state    :", exo.get_gesture_state())
        print("Motor torques    :", exo.get_motor_torque())

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    finally:
        exo.close()
        print("Connection closed.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
