#!/usr/bin/env python3
"""
Convenience launcher for the NML Hand Exoskeleton operator GUI.

Equivalent to running `handexo gui`, but works from a plain checkout even if
the package has not been `pip install -e .`'d -- this script puts src/ on
sys.path itself. Double-click on Windows (with .py files associated to
python.exe) or run from a terminal:

    python launch_gui.py
"""

import sys
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from nml_hand_exo.applications.hand_exo_gui import main

if __name__ == "__main__":
    main()
