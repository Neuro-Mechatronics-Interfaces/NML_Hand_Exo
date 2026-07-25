"""Canonical repository data paths used by the Python applications."""

from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
CALIBRATION_PROFILES_DIR = (
    REPOSITORY_ROOT / "examples" / "calibration" / "profiles"
)
ROM_OUTPUT_DIR = REPOSITORY_ROOT / "output_data"
UDP_BINDINGS_DIR = REPOSITORY_ROOT / "examples" / "udp_bindings"
