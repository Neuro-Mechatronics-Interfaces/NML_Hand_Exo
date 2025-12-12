#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NML Hand Exo — Simple Serial CLI
---------------------------------
Windows-first command line tool to talk to the Exo controller over a serial port.

Examples:
  # List serial ports
  python tools/hand_exo_cli.py --list-ports

  # Connect, print device info, and exit
  python tools/hand_exo_cli.py --connect COM5 --baud 57600 --info

  # Home all joints
  python tools/hand_exo_cli.py --connect COM5 --home

  # Home a specific target (e.g., "thumb" or axis id)
  python tools/hand_exo_cli.py --connect COM5 --home thumb

  # Send a gesture command
  python tools/hand_exo_cli.py --connect COM5 --send "gc:pinch:a"

  # Send multiple commands in order
  python tools/hand_exo_cli.py --connect COM5 --send "led:1:on" --send "led:1:off"

  # Monitor the serial output for 10 seconds after sending a command
  python tools/hand_exo_cli.py --connect COM5 --send "sys:info" --read-for 10

  # Continuous monitor (Ctrl-C to stop)
  python tools/hand_exo_cli.py --connect COM5 --monitor
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable, Optional, List

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def list_serial_ports(verbose: bool = True) -> List[list_ports.ListPortInfo]:
    ports = list(list_ports.comports())
    if verbose:
        if not ports:
            print("No serial ports found.")
        else:
            print("Available serial ports:")
            for p in ports:
                vid = f"{p.vid:04X}" if p.vid is not None else "----"
                pid = f"{p.pid:04X}" if p.pid is not None else "----"
                print(f"  {p.device:>8}  | {p.description}  | VID:PID={vid}:{pid}  | {p.hwid}")
    return ports


_EOL_MAP = {
    "lf": b"\n",
    "cr": b"\r",
    "crlf": b"\r\n",
}


def _eol_bytes(eol: str) -> bytes:
    e = eol.lower()
    if e not in _EOL_MAP:
        raise ValueError(f"Invalid EOL '{eol}'. Choose from: lf, cr, crlf.")
    return _EOL_MAP[e]


def _open_serial(port: str, baud: int, timeout: float) -> serial.Serial:
    # On Windows, pyserial handles "COM10" etc. automatically; no need for "\\.\COM10"
    ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
    # small settle delay for Windows drivers
    time.sleep(0.1)
    return ser


def _send_line(ser: serial.Serial, line: str, eol: str, encoding: str = "utf-8") -> None:
    data = line.encode(encoding, errors="replace") + _eol_bytes(eol)
    ser.write(data)
    ser.flush()


def _read_lines_for(ser: serial.Serial, seconds: float, encoding: str = "utf-8") -> None:
    """Read and print lines for up to `seconds`. Non-blocking-ish."""
    end = time.time() + max(0.0, seconds)
    buf = b""
    while time.time() < end:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except Exception:
            chunk = b""
        if not chunk:
            # sleep a touch to reduce CPU
            time.sleep(0.01)
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            # print the line, add back the newline
            print(line.decode(encoding, errors="replace"))


def _monitor(ser: serial.Serial, encoding: str = "utf-8") -> None:
    """Continuous monitor; Ctrl-C to exit."""
    print("Monitoring serial output... (Ctrl-C to stop)")
    try:
        while True:
            try:
                line = ser.readline()  # uses timeout
            except Exception:
                line = b""
            if line:
                print(line.decode(encoding, errors="replace"), end="")
    except KeyboardInterrupt:
        print("\nStopped.")


def run_actions(
    port: str,
    baud: int,
    timeout: float,
    eol: str,
    encoding: str,
    do_info: bool,
    info_cmd: str,
    home_target: Optional[str],
    sends: Iterable[str],
    read_for: float,
    monitor: bool,
) -> int:
    try:
        with _open_serial(port, baud, timeout) as ser:
            print(f"Connected to {ser.port} @ {ser.baudrate} baud")

            # --info
            if do_info:
                cmd = info_cmd or "sys:info"
                print(f"> {cmd}")
                _send_line(ser, cmd, eol, encoding=encoding)
                if read_for <= 0:
                    read_for = 2.0  # default read window for info
                _read_lines_for(ser, read_for, encoding=encoding)

            # --home [target]
            if home_target is not None:
                target = home_target or "all"
                cmd = f"home {target}"
                print(f"> {cmd}")
                _send_line(ser, cmd, eol, encoding=encoding)
                if read_for <= 0:
                    read_for = 3.0
                _read_lines_for(ser, read_for, encoding=encoding)

            # --send "..."
            for s in sends:
                s = s.strip("\r\n")
                if not s:
                    continue
                print(f"> {s}")
                _send_line(ser, s, eol, encoding=encoding)
                if read_for > 0:
                    _read_lines_for(ser, read_for, encoding=encoding)

            # --monitor (after other actions)
            if monitor:
                _monitor(ser, encoding=encoding)

            return 0
    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        return 2
    except ValueError as e:
        print(f"Value error: {e}", file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="NML Hand Exo — simple serial CLI",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # discovery / connection
    p.add_argument("--list-ports", action="store_true", help="List available serial ports and exit")
    p.add_argument("--connect", metavar="PORT", help="Serial port to connect to (e.g., COM5, /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=57600, help="Baud rate")
    p.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout (seconds)")
    p.add_argument("--eol", choices=("lf", "crlf", "cr"), default="lf", help="Line ending to append to commands")
    p.add_argument("--encoding", default="utf-8", help="Text encoding for I/O")

    # actions
    p.add_argument("--info", action="store_true", help="Request device info (sends --info-cmd, defaults to 'sys:info')")
    p.add_argument("--info-cmd", default="sys:info", help="Command used by --info")
    p.add_argument("--home", nargs="?", const="all", help="Home target (omit value to home 'all')")
    p.add_argument("--send", action="append", default=[], help="Send a raw command string (can be used multiple times)")
    p.add_argument("--read-for", type=float, default=0.0, help="After each command, read for N seconds")
    p.add_argument("--monitor", action="store_true", help="Continuous read (Ctrl-C to stop)")

    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    # list ports + early exit
    if args.list_ports and not args.connect:
        list_serial_ports(verbose=True)
        return 0

    # if they asked for actions without a port
    if not args.connect:
        if args.list_ports:
            # they asked to list only, so we already handled it
            return 0
        print("No port specified. Use --connect PORT (e.g., COM5) or --list-ports to discover ports.", file=sys.stderr)
        return 1

    # show available ports (without spamming) to aid selection on Windows
    list_serial_ports(verbose=False)

    return run_actions(
        port=args.connect,
        baud=args.baud,
        timeout=args.timeout,
        eol=args.eol,
        encoding=args.encoding,
        do_info=bool(args.info),
        info_cmd=args.info_cmd,
        home_target=args.home,
        sends=args.send or [],
        read_for=max(0.0, args.read_for),
        monitor=bool(args.monitor),
    )


if __name__ == "__main__":
    sys.exit(main())
