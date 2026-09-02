"""Host-side command stream contract and state tracker.

The values in this module describe commands observed at the host transport.
They are deliberately named ``requested_*`` because firmware gesture expansion,
clamping, watchdogs, and Dynamixel register writes can change what is ultimately
applied.  Unknown values remain NaN in the LSL stream.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Iterable, Mapping


COMMAND_STREAM_NAME = "NMLHandExoCommandV1"
COMMAND_STREAM_TYPE = "NMLHandExoCommand"
COMMAND_STREAM_SCHEMA = "nml.hand_exo.command.v1"
COMMAND_EVENT_STREAM_NAME = "NMLHandExoEventsV1"
COMMAND_EVENT_STREAM_TYPE = "NMLHandExoEvents"
COMMAND_EVENT_STREAM_SCHEMA = "nml.hand_exo.events.v1"

CONTROL_MODE_CODES = {
    "unknown": 0,
    "position": 1,
    "current_position": 2,
    "velocity": 3,
    "current": 4,
}

COMMAND_SOURCE_CODES = {
    "unknown": 0,
    "gui": 1,
    "udp": 2,
    "emg": 3,
    "safety": 4,
    "calibration": 5,
    "instrumentation": 6,
}

GLOBAL_COMMAND_FIELDS: tuple[tuple[str, str, str], ...] = (
    ("frame.sequence", "count", "sequence"),
    ("global.control_mode_request_code", "code", "control_mode_request"),
    ("global.command_source_code", "code", "command_source"),
    ("global.watchdog_timeout_ms", "ms", "watchdog_timeout"),
    ("global.total_current_limit_request_mA", "mA", "total_current_limit_request"),
    ("global.hold_current_request_mA", "mA", "hold_current_request"),
    ("global.assistance_level", "normalized", "assistance_level"),
    ("global.snapshot_valid", "boolean", "snapshot_valid"),
)

MOTOR_COMMAND_FIELDS: tuple[tuple[str, str], ...] = (
    ("requested_relative_angle_deg", "degrees"),
    ("requested_velocity_rpm", "rpm"),
    ("requested_current_mA", "mA"),
    ("requested_current_limit_mA", "mA"),
    ("requested_profile_velocity_raw", "raw_unit"),
    ("host_enabled_request", "boolean"),
    ("direct_command_active", "boolean"),
)


@dataclass(frozen=True)
class CommandMotor:
    motor_id: int
    name: str
    side: str

    @property
    def prefix(self) -> str:
        side = self.side.strip().upper()[:1] or "U"
        clean = re.sub(r"[^a-zA-Z0-9]+", "_", self.name).strip("_") or "motor"
        return f"{side}.{clean}.id{int(self.motor_id)}"


def infer_side(display_name: str, motor_id: int) -> str:
    text = str(display_name).strip()
    if text.upper().startswith("L:") or 1 <= int(motor_id) <= 9:
        return "L"
    if text.upper().startswith("R:") or 11 <= int(motor_id) <= 19:
        return "R"
    return "U"


def bare_name(display_name: str) -> str:
    text = str(display_name).strip()
    return text[2:] if len(text) > 2 and text[1] == ":" else text


def command_channel_specs(motors: Iterable[CommandMotor]) -> list[dict[str, object]]:
    descriptors = tuple(motors)
    ids = [int(motor.motor_id) for motor in descriptors]
    if len(ids) != len(set(ids)):
        raise ValueError("Command stream cannot contain duplicate motor IDs")
    specs: list[dict[str, object]] = [
        {"label": label, "unit": unit, "quantity": quantity}
        for label, unit, quantity in GLOBAL_COMMAND_FIELDS
    ]
    for motor in descriptors:
        for quantity, unit in MOTOR_COMMAND_FIELDS:
            specs.append(
                {
                    "label": f"{motor.prefix}.{quantity}",
                    "unit": unit,
                    "quantity": quantity,
                    "motor_id": int(motor.motor_id),
                    "motor_name": motor.name,
                    "side": motor.side,
                }
            )
    return specs


def is_recordable_command(command: str) -> bool:
    """Return whether a wire command can change device or recording state."""

    name = normalize_command(command).partition(":")[0].lower()
    if not name:
        return False
    if name.startswith("get_") or name in {"info", "help", "version", "current_status"}:
        return False
    return True


def normalize_command(command: str) -> str:
    return str(command).strip().rstrip(";\r\n").strip()


def _finite_float(value: str) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


class CommandStateTracker:
    """Maintain a fixed-width snapshot of observed host command requests."""

    def __init__(self) -> None:
        self._motors: tuple[CommandMotor, ...] = ()
        self._prefix_by_id: dict[int, str] = {}
        self._sequence = 0
        self._values: dict[str, float | int | bool | None] = {}
        self.configure_motors(())

    @property
    def motors(self) -> tuple[CommandMotor, ...]:
        return self._motors

    def configure_motors(self, motors: Iterable[CommandMotor]) -> None:
        descriptors = tuple(motors)
        specs = command_channel_specs(descriptors)
        self._motors = descriptors
        self._prefix_by_id = {int(motor.motor_id): motor.prefix for motor in descriptors}
        # A connection/motor-set transition starts a new host-request state.
        # Carrying goals across reconnects would falsely imply they remain active.
        self._values = {str(spec["label"]): None for spec in specs}
        self._values["global.control_mode_request_code"] = CONTROL_MODE_CODES["unknown"]
        self._values["global.command_source_code"] = COMMAND_SOURCE_CODES["unknown"]
        self._values["global.snapshot_valid"] = True

    def channel_specs(self) -> list[dict[str, object]]:
        return command_channel_specs(self._motors)

    def snapshot(self) -> dict[str, float | int | bool | None]:
        self._sequence += 1
        values = dict(self._values)
        values["frame.sequence"] = self._sequence
        return values

    def observe(self, event: Mapping[str, object]) -> dict[str, object]:
        """Parse one observed command event and update transmitted requests.

        State changes only on ``status='sent'``. A later acknowledgement or
        failure is retained in the event stream but does not invent a firmware
        register state or attempt an unsafe rollback.
        """

        command = normalize_command(str(event.get("command", "")))
        status = str(event.get("status", "unknown")).strip().lower() or "unknown"
        source = str(event.get("source", "unknown")).strip().lower() or "unknown"
        normalized = dict(event)
        normalized.update(
            {
                "command": command,
                "status": status,
                "source": source,
                "recordable": is_recordable_command(command),
            }
        )
        if status != "sent" or not normalized["recordable"]:
            return normalized

        self._values["global.command_source_code"] = COMMAND_SOURCE_CODES.get(source, 0)
        parts = command.split(":")
        name = parts[0].lower()

        if name == "set_control_mode" and len(parts) >= 3:
            self._values["global.control_mode_request_code"] = CONTROL_MODE_CODES.get(
                parts[-1].strip().lower(), 0
            )
            self._mark_mode_transition_disabled()
        elif name == "set_motor_mode" and len(parts) >= 2:
            self._values["global.control_mode_request_code"] = CONTROL_MODE_CODES.get(
                parts[1].strip().lower(), 0
            )
            self._mark_mode_transition_disabled()
        elif name == "set_command_timeout" and len(parts) >= 2:
            self._set_global_number("global.watchdog_timeout_ms", parts[1])
        elif name == "set_total_current_lim" and len(parts) >= 2:
            self._set_global_number("global.total_current_limit_request_mA", parts[1])
        elif name == "set_hold_current" and len(parts) >= 2:
            self._set_global_number("global.hold_current_request_mA", parts[1])

        if name in {"enable", "disable", "stop"} and len(parts) >= 2:
            for motor_id in self._target_ids(parts[1]):
                if name == "enable":
                    self._set_motor(motor_id, "host_enabled_request", True)
                elif name == "disable":
                    self._set_motor(motor_id, "host_enabled_request", False)
                    self._set_motor(motor_id, "direct_command_active", False)
                else:
                    self._set_motor(motor_id, "requested_velocity_rpm", 0.0)
                    self._set_motor(motor_id, "requested_current_mA", 0.0)
                    self._set_motor(motor_id, "direct_command_active", False)
        elif name in {"set_angle", "hold_position"} and len(parts) >= 3:
            self._set_motor_number(parts[1], "requested_relative_angle_deg", parts[2])
            self._set_motor_flag(parts[1], "direct_command_active", True)
            if name == "hold_position" and len(parts) >= 4:
                self._set_motor_number(parts[1], "requested_current_limit_mA", parts[3])
        elif name == "release_hold" and len(parts) >= 2:
            self._set_motor_flag(parts[1], "direct_command_active", False)
            self._set_motor_flag(parts[1], "host_enabled_request", False)
        elif name == "set_velocity" and len(parts) >= 3:
            self._set_motor_number(parts[1], "requested_velocity_rpm", parts[2])
            self._set_motor_flag(parts[1], "direct_command_active", True)
        elif name == "set_current" and len(parts) >= 3:
            self._set_motor_number(parts[1], "requested_current_mA", parts[2])
            self._set_motor_flag(parts[1], "direct_command_active", True)
        elif name == "set_current_lim" and len(parts) >= 3:
            for motor_id in self._target_ids(parts[1]):
                self._set_motor_number(str(motor_id), "requested_current_limit_mA", parts[2])
        elif name == "set_goal_velocity" and len(parts) >= 3:
            for motor_id in self._target_ids(parts[1]):
                self._set_motor_number(str(motor_id), "requested_profile_velocity_raw", parts[2])
        return normalized

    def _target_ids(self, target: str) -> list[int]:
        if str(target).strip().lower() == "all":
            return sorted(self._prefix_by_id)
        try:
            motor_id = int(target)
        except (TypeError, ValueError):
            return []
        return [motor_id] if motor_id in self._prefix_by_id else []

    def _set_global_number(self, label: str, value: str) -> None:
        parsed = _finite_float(value)
        if parsed is not None:
            self._values[label] = parsed

    def _mark_mode_transition_disabled(self) -> None:
        # The documented firmware contract leaves torque disabled after a
        # global mode transition; later explicit enable commands update this.
        for motor_id in self._prefix_by_id:
            self._set_motor(motor_id, "host_enabled_request", False)
            self._set_motor(motor_id, "direct_command_active", False)

    def _set_motor(self, motor_id: int, quantity: str, value: float | bool) -> None:
        prefix = self._prefix_by_id.get(int(motor_id))
        if prefix is not None:
            self._values[f"{prefix}.{quantity}"] = value

    def _set_motor_number(self, target: str, quantity: str, value: str) -> None:
        parsed = _finite_float(value)
        if parsed is None:
            return
        for motor_id in self._target_ids(target):
            self._set_motor(motor_id, quantity, parsed)

    def _set_motor_flag(self, target: str, quantity: str, value: bool) -> None:
        for motor_id in self._target_ids(target):
            self._set_motor(motor_id, quantity, value)
