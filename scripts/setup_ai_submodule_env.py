from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def run(cmd: list[str], cwd: Path) -> None:
    print("+", " ".join(cmd))
    subprocess.run(cmd, cwd=str(cwd), check=True)


def resolve_neurobridge_project(repo_root: Path, submodule_path: Path) -> Path:
    candidates = [submodule_path]
    for candidate in candidates:
        if (candidate / "pyproject.toml").exists() and (candidate / "src" / "neurobridge_assist").exists():
            return candidate
    raise FileNotFoundError(
        "No compatible NeuroBridge runtime found. Checked: "
        + ", ".join(str(path) for path in candidates)
        + ". Expected pyproject.toml and src/neurobridge_assist/."
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Set up local venv for NeuroBridge submodule AI workflow.")
    parser.add_argument("--submodule-path", default="external/NeuroBridge")
    parser.add_argument(
        "--allow-sibling-neurobridge",
        action="store_true",
        help="Allow fallback to ../NeuroBridge when submodule runtime is unavailable.",
    )
    parser.add_argument(
        "--install-local-nml",
        action="store_true",
        help="Also install local NML_Hand_Exo package in editable mode (off by default).",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    venv_dir = repo_root / ".venv"
    submodule_path = (repo_root / args.submodule_path).resolve()

    if not venv_dir.exists():
        run([sys.executable, "-m", "venv", str(venv_dir)], cwd=repo_root)

    venv_python = venv_dir / "Scripts" / "python.exe"
    if not venv_python.exists():
        raise FileNotFoundError(f"Missing venv python executable: {venv_python}")

    if not submodule_path.exists():
        print(f"[WARN] Submodule path not found: {submodule_path}")

    neurobridge_project: Path | None = None
    try:
        neurobridge_project = resolve_neurobridge_project(repo_root, submodule_path)
    except FileNotFoundError:
        if args.allow_sibling_neurobridge:
            sibling_path = (repo_root.parent / "NeuroBridge").resolve()
            if (sibling_path / "pyproject.toml").exists() and (sibling_path / "src" / "neurobridge_assist").exists():
                neurobridge_project = sibling_path
                print(f"[WARN] Using sibling NeuroBridge fallback: {sibling_path}")
    if neurobridge_project is None:
        raise FileNotFoundError(
            "No compatible NeuroBridge runtime found at the submodule path. "
            "Run scripts/setup_neurobridge_submodule.bat or pass --allow-sibling-neurobridge."
        )

    run([str(venv_python), "-m", "pip", "install", "--upgrade", "pip"], cwd=repo_root)
    run([str(venv_python), "-m", "pip", "install", "-e", f"{neurobridge_project}[ai]"], cwd=repo_root)
    if args.install_local_nml:
        run([str(venv_python), "-m", "pip", "install", "-e", "."], cwd=repo_root)

    print("Environment ready.")
    print(f"NeuroBridge runtime path: {neurobridge_project}")
    if args.install_local_nml:
        print("Local NML package installed: yes")
    else:
        print("Local NML package installed: no (NeuroBridge runtime only)")
    print("Run: .venv\\Scripts\\python.exe scripts\\run_ai_agent.py --bundle nml_default --command status")
    print("Run visualizer: .venv\\Scripts\\python.exe scripts\\run_exo_visualizer.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
