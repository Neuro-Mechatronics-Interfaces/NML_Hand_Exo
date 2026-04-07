from __future__ import annotations

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def _resolve_neurobridge_visualizer_path() -> Path:
    repo_root = Path(__file__).resolve().parents[3]
    candidates = [
        repo_root / "external" / "NeuroBridge" / "src" / "nml_hand_exo" / "applications" / "exo_state_visualizer.py",
        repo_root.parent / "NeuroBridge" / "src" / "nml_hand_exo" / "applications" / "exo_state_visualizer.py",
    ]
    for path in candidates:
        if path.exists():
            return path
    raise ModuleNotFoundError(
        "NeuroBridge visualizer entrypoint is not available. Run setup and ensure NeuroBridge source is present."
    )


def _load_neurobridge_main():
    module_path = _resolve_neurobridge_visualizer_path()
    spec = spec_from_file_location("_neurobridge_exo_state_visualizer", module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load NeuroBridge visualizer module from {module_path}")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    main = getattr(module, "main", None)
    if main is None:
        raise ImportError(f"NeuroBridge visualizer module has no 'main' callable: {module_path}")
    return main


def main() -> int:
    neurobridge_main = _load_neurobridge_main()
    return int(neurobridge_main())


if __name__ == "__main__":
    raise SystemExit(main())
