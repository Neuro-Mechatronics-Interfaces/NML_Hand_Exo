from __future__ import annotations

import argparse

from . import __version__


def main(argv: list[str] | None = None) -> int:
    """Run the installed ``handexo`` application launcher."""
    parser = argparse.ArgumentParser(
        prog="handexo",
        description="Launch NML Hand Exoskeleton applications.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("gui", help="launch the exoskeleton operator GUI")
    subparsers.add_parser(
        "emg-centroid", help="launch the legacy centroid decoder GUI"
    )
    subparsers.add_parser(
        "emg-intent",
        aliases=["emg-intent-decoder"],
        help="launch intent discovery, validation, and continuous decoding",
    )

    args = parser.parse_args(argv)
    if args.command == "gui":
        from .applications.hand_exo_gui import main as application_main
    elif args.command == "emg-centroid":
        from .applications.emg_centroid_decoder_gui import main as application_main
    elif args.command in {"emg-intent", "emg-intent-decoder"}:
        from .applications.emg_intent_decoder_gui import main as application_main
    else:  # pragma: no cover - argparse enforces the available commands.
        parser.error(f"unknown command: {args.command}")

    return int(application_main() or 0)


if __name__ == "__main__":
    raise SystemExit(main())
