"""Connect to an NML Hand Exo through a TCP serial bridge."""

from __future__ import annotations

import argparse

from nml_hand_exo.interface import HandExo, TCPComm


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="bridge IP address or hostname")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)

    exo = HandExo(
        TCPComm(args.host, port=args.port, timeout=args.timeout),
        verbose=args.verbose,
    )
    try:
        exo.connect()
        print(f"Firmware: {exo.version()}")
        print(f"Device: {exo.info()}")
    finally:
        exo.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
