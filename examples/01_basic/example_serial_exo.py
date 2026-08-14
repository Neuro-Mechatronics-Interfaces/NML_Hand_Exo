"""Connect to an NML Hand Exo over USB serial and print device status."""

from __future__ import annotations

import argparse

from nml_hand_exo.interface import HandExo, SerialComm


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial port, for example COM12")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    exo = HandExo(
        SerialComm(port=args.port, baudrate=args.baudrate),
        verbose=args.verbose,
    )
    try:
        exo.connect()
        print(f"Firmware: {exo.version()}")
        print(f"Mode: {exo.get_exo_mode()}")
        print(f"Device: {exo.info()}")
        print(f"Motor angles: {exo.get_motor_angle('all')}")
    finally:
        exo.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
