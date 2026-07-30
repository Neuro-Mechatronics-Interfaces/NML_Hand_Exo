from __future__ import annotations

import argparse
import os
import sys


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="handexo")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("gui", help="launch the NML Hand Exo PyQt GUI")
    subparsers.add_parser("emg-centroid", help="launch the EMG centroid decoder GUI")

    args = parser.parse_args(argv)
    if args.command == "gui":
        this_src = os.path.dirname(os.path.abspath(__file__))
        if sys.path[0] != this_src:
            sys.path.insert(0, this_src)
        from nml_hand_exo.applications.hand_exo_gui import main as gui_main

        gui_main()
        return 0
    if args.command == "emg-centroid":
        this_src = os.path.dirname(os.path.abspath(__file__))
        if sys.path[0] != this_src:
            sys.path.insert(0, this_src)
        from nml_hand_exo.applications.emg_centroid_decoder_gui import main as emg_main

        emg_main()
        return 0
    parser.error(f"unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
