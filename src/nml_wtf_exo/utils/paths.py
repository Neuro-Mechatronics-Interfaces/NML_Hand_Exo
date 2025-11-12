# nml/utils/paths.py
"""
Centralized absolute path resolution for the nml project.

- Primary output: PATHS (dict[str, str]) with absolute paths.
- Uses environment overrides when present:
    NML_PROJECT_ROOT, NML_RESOURCES_DIR
- Assumes this file lives at <package_root>/utils/paths.py

Typical keys:
  package_root            : repo root directory
  gui_dir                 : <package_root>/gui
  ui_dir                  : <package_root>/gui/ui
  test_dir                : <package_root>/test
  resources_dir           : <package_root>/resources
  keyboard_layout_json    : <package_root>/resources/keyboard_layout.json
  virtual_keyboard_png    : <package_root>/resources/virtual-keyboard.png
  phrases_txt             : <package_root>/resources/phrases.txt
"""

from __future__ import annotations
import os
import sys
import platform
from pathlib import Path
from typing import Dict

__all__ = [
    "PATHS",
    "get_paths",
    "update_path",
    "refresh_paths",
]

def _env_path(name: str) -> Path | None:
    val = os.environ.get(name)
    return Path(val).expanduser().resolve() if val else None

def _default_root() -> Path:
    """
    Infer project root by walking up from this file to the parent of 'nml'.
    """
    here = Path(__file__).resolve()
    # .../utils/paths.py -> .../utils -> .../ -> <package_root>
    utils_dir = here.parent
    root = utils_dir.parent
    return root

# def _default_root() -> Path:
#     """
#     Infer project root by walking up from this file.
#     Priority:
#       1) parent dir containing a VCS/packaging marker ('.git', 'pyproject.toml').
#       2) fallback: parent of 'nml' (legacy behavior).
#     """
#     here = Path(__file__).resolve()
#     # legacy baseline: .../nml/utils/paths.py -> .../nml -> parent
#     legacy_root = here.parent.parent.parent

#     markers = {".git"}
#     for p in here.parents:
#         try:
#             contents = {x.name for x in p.iterdir()}
#         except Exception:
#             continue
#         if markers & contents:
#             return p

#     return legacy_root

def _find_lsl_dll(verbose: bool = False) -> str | None:
    """
    Locate the platform-specific pylsl binary (e.g., lsl.dll, liblsl.so, liblsl.dylib)
    inside the current virtual environment.

    Returns
    -------
    str | None
        Absolute path to the liblsl shared library, or None if not found.
    """
    # --- Determine the active environment root ---
    # sys.prefix points to the root of the current interpreter (venv or global)
    venv_root = Path(sys.prefix).resolve()

    # --- Expected lib name by platform ---
    system = platform.system()
    if system in ("Windows", "Microsoft"):
        libname = "lsl.dll"
        subpath = Path("Lib/site-packages/pylsl/lib")
    elif system == "Darwin":
        libname = "liblsl.dylib"
        subpath = Path("lib/python*/site-packages/pylsl/lib")  # will glob later
    elif system == "Linux":
        libname = "liblsl.so"
        subpath = Path("lib/python*/site-packages/pylsl/lib")  # will glob later
    else:
        if verbose:
            print(f"[paths] Unsupported OS: {system}")
        return None

    # --- Build candidate directories ---
    candidates: list[Path] = []

    # (1) canonical Windows-style venv structure
    win_candidate = venv_root / "Lib" / "site-packages" / "pylsl" / "lib" / libname
    candidates.append(win_candidate)

    # (2) POSIX-style layouts
    for base in (venv_root / "lib").glob("python*/site-packages/pylsl/lib"):
        candidates.append(base / libname)

    # (3) fallback: inspect sys.path entries (covers editable installs)
    for entry in sys.path:
        p = Path(entry)
        if p.name == "pylsl" or (p.parent.name == "pylsl" and p.name == "lib"):
            cand = (p / "lib" / libname) if p.name == "pylsl" else (p / libname)
            candidates.append(cand)

    # --- Return first match ---
    for cand in candidates:
        if cand.is_file():
            return str(cand.resolve())

    if verbose:
        print("[paths] liblsl not found in candidates:")
        for c in candidates:
            print("  ", c)

    return None

def _first_existing(candidates: list[Path], fallback: Path) -> Path:
    for p in candidates:
        try:
            if p.is_file() or p.is_dir():
                return p.resolve()
        except Exception:
            # Some paths may not be accessible; ignore and continue
            pass
    return fallback.resolve()

def _build_paths() -> Dict[str, str]:
    # Resolve base locations with environment overrides
    package_root = _env_path("NML_PROJECT_ROOT") or _default_root()
    gui_dir = package_root / "gui"
    ui_dir = gui_dir / "ui"
    test_dir = package_root / "test"

    # Resources (allow ENV override)
    resources_dir = _env_path("NML_RESOURCES_DIR") or (package_root / "resources")

    # Common files
    keyboard_layout_json = resources_dir / "keyboard_layout.json"
    virtual_keyboard_png = resources_dir / "virtual-keyboard.png"
    phrases_txt = resources_dir / "phrases.txt"

    # In case someone launches from an alternate CWD, try a few fallbacks:
    cwd = Path.cwd()
    keyboard_layout_json = _first_existing(
        [
            keyboard_layout_json,
            cwd / "resources" / "keyboard_layout.json",
            ui_dir / "resources" / "keyboard_layout.json",
        ],
        keyboard_layout_json,
    )
    virtual_keyboard_png = _first_existing(
        [
            virtual_keyboard_png,
            cwd / "resources" / "virtual-keyboard.png",
            ui_dir / "resources" / "virtual-keyboard.png",
        ],
        virtual_keyboard_png,
    )
    phrases_txt = _first_existing(
        [
            phrases_txt,
            cwd / "resources" / "phrases.txt",
        ],
        phrases_txt,
    )

    # Normalize to absolute strings
    as_abs = lambda p: str(Path(p).resolve())

    return {
        "package_root": as_abs(package_root),
        "logs_dir": as_abs(package_root / "logs"), 
        "landmarks_dir": as_abs(package_root / "landmarks"), 
        "gui_dir": as_abs(gui_dir),
        "ui_dir": as_abs(ui_dir),
        "test_dir": as_abs(test_dir),
        "configs_dir": as_abs(package_root / "config"), 
        "resources_dir": as_abs(resources_dir),
        "keyboard_layout_json": as_abs(keyboard_layout_json),
        "virtual_keyboard_png": as_abs(virtual_keyboard_png),
        "phrases_txt": as_abs(phrases_txt),
        "lsl_dll": _find_lsl_dll()
    }

# Build once at import
PATHS: Dict[str, str] = _build_paths()

def get_paths() -> Dict[str, str]:
    """Return a shallow copy of the resolved absolute PATHS dict."""
    return dict(PATHS)

def update_path(key: str, value: str | os.PathLike) -> None:
    """
    Override a single entry in PATHS (stored as absolute string).
    Useful for tests or dynamic reconfiguration.
    """
    PATHS[key] = str(Path(value).expanduser().resolve())

def refresh_paths() -> None:
    """
    Recompute all entries (e.g., after changing env vars).
    """
    global PATHS
    PATHS = _build_paths()

if __name__ == "__main__":
    import pprint
    pprint.pprint(PATHS)
