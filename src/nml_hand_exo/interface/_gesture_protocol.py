"""Pure shared helpers for the continuous gesture-angle protocol."""

from __future__ import annotations

import math


ANGLE_ADDRESSABLE_GESTURES = (
    "thumb",
    "thumbadd",
    "thumbrot",
    "thumbflex",
    "index",
    "middle",
    "ring",
    "pinky",
    "wrist",
)


def normalize_udp_gesture_angle_command(command: str) -> tuple[str, str] | None:
    """Validate and canonicalize a UDP gesture-angle command.

    Non-gesture commands return ``None``. Gesture-angle commands reject
    malformed, unsupported, non-finite, or out-of-range targets instead of
    relying on the firmware's clamping behavior.
    """
    if not command.startswith("set_gesture_angle:"):
        return None
    parts = command.split(":")
    if len(parts) != 3:
        raise ValueError("expected set_gesture_angle:<gesture>:<0-100>")
    gesture = parts[1].strip().lower()
    if gesture not in ANGLE_ADDRESSABLE_GESTURES:
        raise ValueError("gesture is not angle-addressable")
    try:
        percent = float(parts[2])
    except ValueError as exc:
        raise ValueError("percent must be a finite number") from exc
    if not math.isfinite(percent) or not 0.0 <= percent <= 100.0:
        raise ValueError("percent must be in the range 0-100")
    return f"set_gesture_angle:{gesture}:{percent:g}", gesture
