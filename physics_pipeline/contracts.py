"""Versioned stream and processed-data contracts for the physics pipeline."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import re
from typing import Iterable


EXO_STATE_SCHEMA = "nml.hand_exo.state.v1"
EXO_STATE_STREAM_NAME = "NMLHandExoStateV1"
EXO_STATE_STREAM_TYPE = "NMLHandExoState"
EXO_COMMAND_SCHEMA = "nml.hand_exo.command.v1"
EXO_COMMAND_STREAM_NAME = "NMLHandExoCommandV1"
EXO_COMMAND_STREAM_TYPE = "NMLHandExoCommand"
EXO_EVENT_SCHEMA = "nml.hand_exo.events.v1"
EXO_EVENT_STREAM_NAME = "NMLHandExoEventsV1"
EXO_EVENT_STREAM_TYPE = "NMLHandExoEvents"
HAND_KINEMATICS_SCHEMA = "nml.hand_kinematics.v1"
PHYSICS_SESSION_SCHEMA = "nml.physics_session.v1"


@dataclass(frozen=True)
class NumericChannelSpec:
    """One fixed-position numeric channel in an LSL stream."""

    label: str
    unit: str
    quantity: str
    motor_id: int | None = None
    motor_name: str | None = None
    side: str | None = None

    def metadata(self) -> dict[str, str]:
        values = asdict(self)
        return {key: str(value) for key, value in values.items() if value is not None}


@dataclass(frozen=True)
class ExoMotorDescriptor:
    """Unambiguous description of one firmware-managed Dynamixel motor."""

    motor_id: int
    name: str
    side: str

    @property
    def prefix(self) -> str:
        side = self.side.strip().upper()[:1] or "U"
        clean = re.sub(r"[^a-zA-Z0-9]+", "_", self.name).strip("_") or "motor"
        return f"{side}.{clean}.id{int(self.motor_id)}"


MOTOR_STATE_FIELDS: tuple[tuple[str, str], ...] = (
    ("relative_angle_deg", "degrees"),
    ("absolute_angle_deg", "degrees"),
    ("position_ticks", "ticks"),
    ("velocity_rpm", "rpm"),
    ("present_current_mA", "mA"),
    # This is explicitly an estimate derived from present current and the
    # configured motor torque constant.  It is not human joint torque.
    ("estimated_motor_torque_from_current_Nm", "N.m"),
    ("telemetry_valid", "boolean"),
)


FRAME_STATE_FIELDS: tuple[tuple[str, str], ...] = (
    ("frame.sequence", "count"),
    ("frame.firmware_timestamp_ms", "ms"),
    ("frame.fast_read_flags", "code"),
)


def build_exo_state_channels(
    motors: Iterable[ExoMotorDescriptor],
) -> tuple[NumericChannelSpec, ...]:
    """Return the stable channel order for ``NMLHandExoStateV1``."""

    descriptors = tuple(motors)
    ids = [int(motor.motor_id) for motor in descriptors]
    if len(ids) != len(set(ids)):
        raise ValueError("Exoskeleton state stream cannot contain duplicate motor IDs")
    channels: list[NumericChannelSpec] = [
        NumericChannelSpec(label=label, unit=unit, quantity=label)
        for label, unit in FRAME_STATE_FIELDS
    ]
    for motor in descriptors:
        for quantity, unit in MOTOR_STATE_FIELDS:
            channels.append(
                NumericChannelSpec(
                    label=f"{motor.prefix}.{quantity}",
                    unit=unit,
                    quantity=quantity,
                    motor_id=int(motor.motor_id),
                    motor_name=motor.name,
                    side=motor.side,
                )
            )
    return tuple(channels)


def infer_motor_side(display_name: str, motor_id: int) -> str:
    """Infer side only from explicit GUI labels or the documented DXL ID map."""

    text = str(display_name).strip()
    if text.upper().startswith("L:"):
        return "left"
    if text.upper().startswith("R:"):
        return "right"
    mid = int(motor_id)
    if 1 <= mid <= 9:
        return "left"
    if 11 <= mid <= 19:
        return "right"
    return "unknown"


def bare_motor_name(display_name: str) -> str:
    text = str(display_name).strip()
    return text[2:] if len(text) > 2 and text[1] == ":" else text
