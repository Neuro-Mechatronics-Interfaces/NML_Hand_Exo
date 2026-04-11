from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def _neurobridge_src_candidates() -> list[Path]:
    repo_root = Path(__file__).resolve().parents[1]
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


def _resolve_gui_main():
    runtime_src = _bootstrap_neurobridge_src()
    if runtime_src is None:
        raise ModuleNotFoundError("Unable to find external/NeuroBridge/src")

    from nml_hand_exo.applications.ai_assist_gui import main as gui_main

    return gui_main


def _should_detach() -> bool:
    if os.name != "nt":
        return False
    if os.environ.get("NML_AI_ASSIST_FOREGROUND") == "1":
        return False
    return "--foreground" not in sys.argv[1:]


def _detach_gui_process() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    pythonw = repo_root / ".venv" / "Scripts" / "pythonw.exe"
    executable = str(pythonw if pythonw.exists() else Path(sys.executable))
    forwarded_args = [arg for arg in sys.argv[1:] if arg != "--foreground"]
    child_env = os.environ.copy()
    child_env["NML_AI_ASSIST_FOREGROUND"] = "1"
    creation_flags = 0
    if os.name == "nt":
        creation_flags = getattr(subprocess, "DETACHED_PROCESS", 0) | getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)

    subprocess.Popen(
        [executable, str(Path(__file__).resolve()), *forwarded_args],
        cwd=str(repo_root),
        env=child_env,
        creationflags=creation_flags,
        close_fds=True,
    )
    return 0


def main() -> int:
    if _should_detach():
        return _detach_gui_process()

    try:
        gui_main = _resolve_gui_main()
    except Exception as exc:
        print("Unable to import NeuroBridge GUI runtime.")
        print("Run first: python scripts/setup_ai_submodule_env.py")
        print(f"Import error: {exc}")
        return 1

    return int(gui_main())


if __name__ == "__main__":
    raise SystemExit(main())
