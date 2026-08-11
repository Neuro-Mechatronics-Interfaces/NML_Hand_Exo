"""Console bootstrap for the standalone NML task-cue application."""
from __future__ import annotations

def main() -> int:
    from nml_hand_exo.applications.task_cue_gui import main as gui_main

    return int(gui_main())


if __name__ == "__main__":
    raise SystemExit(main())
