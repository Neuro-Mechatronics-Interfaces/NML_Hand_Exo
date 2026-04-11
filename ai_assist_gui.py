from __future__ import annotations

import sys
from pathlib import Path


def _neurobridge_src_candidates() -> list[Path]:
    repo_root = Path(__file__).resolve().parent
    return [
        repo_root / "external" / "NeuroBridge" / "src",
        repo_root.parent / "NeuroBridge" / "src",
    ]


def _bootstrap_neurobridge_src() -> Path | None:
    for src_path in _neurobridge_src_candidates():
        if not src_path.exists():
            continue
        src_path_str = str(src_path)
        if src_path_str in sys.path:
            sys.path.remove(src_path_str)
        sys.path.insert(0, src_path_str)
        return src_path
    return None


def main() -> int:
    runtime_src = _bootstrap_neurobridge_src()
    if runtime_src is None:
        print("Unable to find external/NeuroBridge/src.")
        print("Run from the NML_Hand_Exo repository root.")
        return 1

    try:
        from nml_hand_exo.applications.ai_assist_gui import main as gui_main
    except Exception as exc:
        print("Unable to import NeuroBridge GUI runtime.")
        print("Run first: python scripts/setup_ai_submodule_env.py")
        print(f"Import error: {exc}")
        return 1

    return int(gui_main())


if __name__ == "__main__":
    raise SystemExit(main())
