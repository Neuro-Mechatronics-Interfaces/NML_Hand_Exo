"""Console bootstrap for the standalone NML task-cue application."""
from __future__ import annotations


import argparse

from nml_hand_exo import __version__


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="nml-task-cue",
        description="Launch the event-marked participant task-cue application.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.parse_args(argv)

    from nml_hand_exo.applications.task_cue_gui import main as gui_main

    return int(gui_main())


if __name__ == "__main__":
    raise SystemExit(main())
