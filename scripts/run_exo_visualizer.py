from __future__ import annotations

import sys
from pathlib import Path


def _bootstrap_local_src() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    src_path = repo_root / "src"
    src_path_str = str(src_path)
    if src_path_str in sys.path:
        sys.path.remove(src_path_str)
    sys.path.insert(0, src_path_str)


_bootstrap_local_src()

from nml_hand_exo.applications.exo_state_visualizer import main


if __name__ == "__main__":
    raise SystemExit(main())
