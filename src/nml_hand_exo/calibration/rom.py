"""Pure calculations and output naming for range-of-motion assessments."""

from __future__ import annotations

from pathlib import Path

from nml_hand_exo._paths import ROM_OUTPUT_DIR


def build_motor_orientation(
    profile: dict | None, motor_names: list[str]
) -> dict[str, dict[str, float | bool]]:
    """Return home/flip metadata for every requested motor."""
    orientation = {}
    profile_motors = (profile or {}).get("motors", {})
    for name in motor_names:
        motor = profile_motors.get(name, {})
        orientation[name] = {
            "home": float(motor.get("home", 0.0)),
            "flip": bool(motor.get("flip", False)),
        }
    return orientation


def normalize_angle(absolute: float, home: float, flip: bool) -> float:
    """Normalize an encoder angle to zero-at-home, positive-flexion space."""
    return home - absolute if flip else absolute - home


def determine_run_number(
    participant: str,
    date_str: str,
    output_dir: str | Path = ROM_OUTPUT_DIR,
) -> int:
    """Return the next run number for a participant and date."""
    directory = Path(output_dir)
    prefix = f"{participant}_rom_{date_str}_"
    run = 1
    if not directory.is_dir():
        return run
    for path in directory.iterdir():
        if not path.is_file() or not path.name.startswith(prefix):
            continue
        if path.suffix.lower() != ".csv":
            continue
        run_text = path.name.removeprefix(prefix).removesuffix(path.suffix)
        try:
            run = max(run, int(run_text) + 1)
        except ValueError:
            continue
    return run
