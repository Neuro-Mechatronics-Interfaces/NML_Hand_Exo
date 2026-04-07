from __future__ import annotations

import argparse
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


def _resolve_neurobridge_runner():
    _bootstrap_neurobridge_src()
    try:
        from neurobridge_assist.runner import main as neurobridge_main

        return neurobridge_main
    except ModuleNotFoundError:
        runtime_src = _bootstrap_neurobridge_src()
        if runtime_src is not None:
            from neurobridge_assist.runner import main as neurobridge_main

            return neurobridge_main
        raise


def main() -> int:
    try:
        neurobridge_main = _resolve_neurobridge_runner()
    except Exception as exc:
        print("Unable to import NeuroBridge runtime.")
        print("Run first: python scripts/setup_ai_submodule_env.py")
        print(f"Import error: {exc}")
        return 1

    parser = argparse.ArgumentParser(description="Run NeuroBridge AI agent from Python script.")
    parser.add_argument("--bundle", default="nml_default")
    parser.add_argument("--command", default="")
    parser.add_argument("--conversation-mode", action="store_true")
    args, passthrough = parser.parse_known_args()

    argv: list[str] = ["--bundle", args.bundle]
    if args.command.strip():
        argv += ["--command", args.command.strip()]
    else:
        argv += ["--conversation-mode"]

    argv += passthrough
    return int(neurobridge_main(argv))


if __name__ == "__main__":
    raise SystemExit(main())
