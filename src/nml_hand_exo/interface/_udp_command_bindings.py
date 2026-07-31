"""Pure helpers for integer UDP command-binding profiles.

The GUI owns the socket and serial connection.  This module deliberately has
no Qt or serial dependencies so profile parsing and command expansion can be
tested without hardware.
"""

from __future__ import annotations

import json
import re
from itertools import product


UDP_BINDING_SCHEMA_VERSION = 3
UDP_CONNECTION_PORT_THRESHOLD = 64
UDP_CONNECTION_PORT_MAX = 65535
UDP_HEARTBEAT_REQUEST_VALUE = 1023
DEFAULT_TORQUE_MA = 100.0
DEFAULT_REPEAT_MS = 100

# Torque-pulse defaults (schema v3).  A discrete torque command plays a
# bell-shaped current pulse toward a target endpoint instead of holding a flat
# current, and REST (value 0) reverts the applied pulse then eases to home.
DEFAULT_PULSE_DURATION_MS = 1000
DEFAULT_PULSE_STEP_MS = 20
DEFAULT_EASE_DURATION_MS = 800
DEFAULT_PULSE_SHAPE = "raised_cosine"
PULSE_SHAPES = ("raised_cosine",)

_SIGNED_INTEGER_RE = re.compile(r"^[+-]?\d+$")
_MOTOR_PLACEHOLDER_RE = re.compile(r"\{([A-Za-z][A-Za-z0-9_]*)\}")

_DIGITS = (
    (1, "thumb", "thumbflex"),
    (2, "index", "index"),
    (3, "middle", "middle"),
    (4, "ring", "ring"),
    (5, "pinky", "pinky"),
)


def parse_udp_integer(payload: str) -> int | None:
    """Return a UDP integer payload, accepting plain text or JSON values.

    Accepted JSON forms are an integer primitive and an object containing an
    integer ``value`` field.  Booleans and floating-point values are rejected.
    """
    text = payload.strip()
    if _SIGNED_INTEGER_RE.fullmatch(text):
        return int(text)
    if not text or text[0] not in "{[-0123456789":
        return None
    try:
        decoded = json.loads(text)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    if isinstance(decoded, dict):
        decoded = decoded.get("value")
    if isinstance(decoded, bool) or not isinstance(decoded, int):
        return None
    return decoded


def default_bindings(control_mode: str = "torque") -> list[dict]:
    """Create the standard REST plus signed per-digit binding list."""
    mode = control_mode.strip().lower()
    if mode not in ("torque", "position"):
        raise ValueError("control_mode must be 'torque' or 'position'")

    bindings = [
        {
            "value": 0,
            "command": (
                "stop:all" if mode == "torque" else "set_gesture:grasp:open"
            ),
            "description": "REST / no torque",
        }
    ]
    current_text = f"{DEFAULT_TORQUE_MA:g}"
    for value, digit, motor_name in _DIGITS:
        if mode == "torque":
            positive = f"set_current:{{{motor_name}}}:{current_text}"
            negative = f"set_current:{{{motor_name}}}:-{current_text}"
        else:
            # Firmware exposes one flex/extend gesture per digit, named for the
            # digit itself, so +N/-N map straight through without the pinch
            # approximations this used to need.
            positive = f"set_gesture:{digit}:flex"
            negative = f"set_gesture:{digit}:extend"
        bindings.extend(
            (
                {
                    "value": value,
                    "command": positive,
                    "description": f"{digit.title()} flexion",
                },
                {
                    "value": -value,
                    "command": negative,
                    "description": f"{digit.title()} extension",
                },
            )
        )
    return sorted(bindings, key=lambda binding: int(binding["value"]))


def make_default_binding_profile(
    name: str = "Default Finger Torque", control_mode: str = "torque"
) -> dict:
    """Return a complete, serializable binding profile."""
    mode = control_mode.strip().lower()
    return {
        "schema_version": UDP_BINDING_SCHEMA_VERSION,
        "name": name.strip() or "Unnamed UDP Binding Map",
        "control_mode": mode,
        "target": "Both",
        "repeat_ms": DEFAULT_REPEAT_MS,
        "pulse_shape": DEFAULT_PULSE_SHAPE,
        "pulse_duration_ms": DEFAULT_PULSE_DURATION_MS,
        "pulse_step_ms": DEFAULT_PULSE_STEP_MS,
        "ease_duration_ms": DEFAULT_EASE_DURATION_MS,
        "bindings": default_bindings(mode),
    }


def index_middle_pinch_bindings() -> list[dict]:
    """Posture bindings mapping UDP 2 -> index pinch, 3 -> middle pinch.

    Value 0 is REST and opens both pinches so a "let go" packet fully releases
    the hand.  These are the two gestures the UDP source cares about; other
    integers are simply unbound.
    """
    return [
        {
            "value": 0,
            "command": "set_gesture:pinch_index:open\nset_gesture:pinch_middle:open",
            "description": "REST / open both pinches",
        },
        {
            "value": 2,
            "command": "set_gesture:pinch_index:close",
            "description": "Index pinch",
        },
        {
            "value": 3,
            "command": "set_gesture:pinch_middle:close",
            "description": "Middle pinch",
        },
    ]


def make_index_middle_pinch_profile(
    name: str = "Index/Middle Pinch (Posture)",
) -> dict:
    """Posture-mode profile focused on the index and middle pinch gestures."""
    return {
        "schema_version": UDP_BINDING_SCHEMA_VERSION,
        "name": name.strip() or "Index/Middle Pinch (Posture)",
        "control_mode": "position",
        "target": "Both",
        "repeat_ms": DEFAULT_REPEAT_MS,
        "pulse_shape": DEFAULT_PULSE_SHAPE,
        "pulse_duration_ms": DEFAULT_PULSE_DURATION_MS,
        "pulse_step_ms": DEFAULT_PULSE_STEP_MS,
        "ease_duration_ms": DEFAULT_EASE_DURATION_MS,
        "bindings": index_middle_pinch_bindings(),
    }


def normalize_binding_profile(data: dict, fallback_name: str = "") -> dict:
    """Validate and normalize a binding profile loaded from JSON."""
    if not isinstance(data, dict):
        raise ValueError("Binding profile must be a JSON object")
    mode = str(data.get("control_mode", "torque")).strip().lower()
    if mode not in ("torque", "position"):
        raise ValueError("control_mode must be 'torque' or 'position'")

    repeat_ms = _require_plain_int(
        data.get("repeat_ms", DEFAULT_REPEAT_MS), "repeat_ms"
    )
    if not 20 <= repeat_ms <= 5000:
        raise ValueError("repeat_ms must be between 20 and 5000")

    pulse_shape = str(
        data.get("pulse_shape", DEFAULT_PULSE_SHAPE)
    ).strip().lower() or DEFAULT_PULSE_SHAPE
    if pulse_shape not in PULSE_SHAPES:
        raise ValueError(
            f"pulse_shape must be one of {', '.join(PULSE_SHAPES)}"
        )
    pulse_duration_ms = _require_plain_int(
        data.get("pulse_duration_ms", DEFAULT_PULSE_DURATION_MS),
        "pulse_duration_ms",
    )
    if not 50 <= pulse_duration_ms <= 10000:
        raise ValueError("pulse_duration_ms must be between 50 and 10000")
    pulse_step_ms = _require_plain_int(
        data.get("pulse_step_ms", DEFAULT_PULSE_STEP_MS), "pulse_step_ms"
    )
    if not 5 <= pulse_step_ms <= 1000:
        raise ValueError("pulse_step_ms must be between 5 and 1000")
    ease_duration_ms = _require_plain_int(
        data.get("ease_duration_ms", DEFAULT_EASE_DURATION_MS),
        "ease_duration_ms",
    )
    if not 0 <= ease_duration_ms <= 10000:
        raise ValueError("ease_duration_ms must be between 0 and 10000")

    raw_bindings = data.get("bindings")
    if not isinstance(raw_bindings, list) or not raw_bindings:
        raise ValueError("bindings must be a non-empty list")
    bindings = []
    seen_values = set()
    for row, raw in enumerate(raw_bindings, start=1):
        if not isinstance(raw, dict):
            raise ValueError(f"Binding row {row} must be an object")
        value = _require_plain_int(raw.get("value"), f"Binding row {row} value")
        if abs(value) > UDP_CONNECTION_PORT_THRESHOLD:
            raise ValueError(
                f"Binding value {value} is reserved for connection-port status; "
                f"binding values must be between "
                f"-{UDP_CONNECTION_PORT_THRESHOLD} and "
                f"{UDP_CONNECTION_PORT_THRESHOLD}"
            )
        if value in seen_values:
            raise ValueError(f"Duplicate binding value: {value}")
        command = str(raw.get("command", "")).strip()
        if not command:
            raise ValueError(f"Binding row {row} command is empty")
        seen_values.add(value)
        bindings.append(
            {
                "value": value,
                "command": command,
                "description": str(raw.get("description", "")).strip(),
            }
        )

    target = str(data.get("target", "Both")).strip()
    if target not in ("Both", "Left Only", "Right Only"):
        target = "Both"
    return {
        "schema_version": UDP_BINDING_SCHEMA_VERSION,
        "name": str(data.get("name") or fallback_name or "Unnamed UDP Binding Map").strip(),
        "control_mode": mode,
        "target": target,
        "repeat_ms": repeat_ms,
        "pulse_shape": pulse_shape,
        "pulse_duration_ms": pulse_duration_ms,
        "pulse_step_ms": pulse_step_ms,
        "ease_duration_ms": ease_duration_ms,
        "bindings": sorted(bindings, key=lambda binding: binding["value"]),
    }


def binding_lookup(profile: dict) -> dict[int, dict]:
    """Return an integer-keyed lookup after validating a profile."""
    normalized = normalize_binding_profile(profile)
    return {binding["value"]: binding for binding in normalized["bindings"]}


def expand_command_templates(
    command_text: str, motor_targets: dict[str, list[int]]
) -> list[str]:
    """Expand active-side motor placeholders into explicit-ID commands.

    Multiple serial commands can be placed on separate lines.  A placeholder
    may resolve to two IDs in Dual/Both mode, producing one command per side.
    """
    templates = [line.strip() for line in command_text.splitlines() if line.strip()]
    if not templates:
        raise ValueError("Binding command is empty")
    commands: list[str] = []
    for template in templates:
        placeholders = list(dict.fromkeys(_MOTOR_PLACEHOLDER_RE.findall(template)))
        if not placeholders:
            commands.append(template)
            continue
        target_sets = []
        for placeholder in placeholders:
            ids = motor_targets.get(placeholder.lower(), [])
            if not ids:
                raise ValueError(
                    f"No active motor matches placeholder {{{placeholder}}}"
                )
            target_sets.append(ids)
        for resolved_ids in product(*target_sets):
            expanded = template
            for placeholder, dxl_id in zip(placeholders, resolved_ids):
                expanded = expanded.replace(f"{{{placeholder}}}", str(int(dxl_id)))
            commands.append(expanded)
    return commands


def _require_plain_int(value, field_name: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{field_name} must be an integer")
    if isinstance(value, int):
        return value
    if isinstance(value, str) and _SIGNED_INTEGER_RE.fullmatch(value.strip()):
        return int(value)
    raise ValueError(f"{field_name} must be an integer")
