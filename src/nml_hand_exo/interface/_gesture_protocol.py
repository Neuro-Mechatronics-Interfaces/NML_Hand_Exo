"""Pure shared helpers for the continuous gesture-angle protocol."""

from __future__ import annotations

import math
import struct


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

# Shared UDP pose-ack contract. Keep this in the SDK rather than copying the
# binary layout between the standalone receiver and the Qt GUI.
UDP_GESTURE_JOINTS = ("thumb", "index", "middle", "ring", "pinky", "wrist")

# Fixed positional order of the firmware `set_finger_angles` batch command
# (firmware >= 0.6.4). It MUST match kFingerOrder in
# src/cpp/nml_hand_exo/utils.cpp: thumb, index, middle, ring, pinky, then the
# optional wrist. Each field is a SIGNED INTEGER in [-100, 100] anchored at the
# joint's calibrated rest posture: -100 extend, 0 rest, +100 flex. An empty
# field holds the joint unchanged.
SET_FINGER_ANGLES_ORDER = ("thumb", "index", "middle", "ring", "pinky", "wrist")
SET_FINGER_ANGLES_PREFIX = "set_finger_angles"
SET_FINGER_ANGLES_ACK_PREFIX = "OK: finger_angles"
#: Inclusive magnitude limit of a set_finger_angles field. -100 is the extend
#: posture, +100 the flex posture, 0 the calibrated rest posture.
SET_FINGER_ANGLES_MAX = 100

# Continuous UDP ack frame (schema nml.continuous.v2). One is returned for every
# accepted continuous datagram, carrying the uint32 sequence being acked and the
# signed int8 value dispatched to each joint. Distinct magic from the NGA2
# gesture pose ack so a consumer can tell the two apart on a shared port.
CONTINUOUS_ACK_MAGIC = b"NGA3"
CONTINUOUS_ACK_HEADER = "<4sIB"   # magic, uint32 sequence, uint8 joint count
CONTINUOUS_ACK_RECORD = "b"       # one signed int8 per joint
COMMAND_PASSTHROUGH_ACK = 1000
POSE_QUERY = "get_gesture_angles:all"
POSE_ACK_MAGIC = b"NGA2"
POSE_ACK_HEADER = "<4shB"
POSE_ACK_RECORD = "Bf"
POSE_UNAVAILABLE = 255


def pack_pose_ack(
    value: int,
    joints: tuple[str, ...],
    pose: dict[str, dict[str, int | float | None]],
) -> bytes:
    """Pack one NGA2 pose datagram accompanying an ASCII integer ACK."""
    values: list[int | float] = []
    for joint in joints:
        record = pose.get(joint) or {}
        fraction = int(record.get("fraction", POSE_UNAVAILABLE)) & 0xFF
        angle = record.get("angle_delta_deg")
        values.extend((fraction, float("nan") if angle is None else float(angle)))
    return struct.pack(
        POSE_ACK_HEADER + POSE_ACK_RECORD * len(joints),
        POSE_ACK_MAGIC,
        int(value),
        len(joints),
        *values,
    )


def unpack_pose_ack(
    data: bytes, joints: tuple[str, ...] = UDP_GESTURE_JOINTS
) -> tuple[int, dict[str, dict[str, int | float | None]]] | None:
    """Parse an NGA2 pose datagram, rejecting truncated or mismatched frames."""
    header_len = struct.calcsize(POSE_ACK_HEADER)
    if not data or len(data) < header_len or not data.startswith(POSE_ACK_MAGIC):
        return None
    _, value, count = struct.unpack_from(POSE_ACK_HEADER, data)
    record_len = struct.calcsize("<" + POSE_ACK_RECORD)
    if len(data) < header_len + count * record_len or count != len(joints):
        return None
    pose: dict[str, dict[str, int | float | None]] = {}
    offset = header_len
    for joint in joints:
        fraction, angle = struct.unpack_from("<" + POSE_ACK_RECORD, data, offset)
        pose[joint] = {
            "fraction": fraction,
            "angle_delta_deg": angle if math.isfinite(angle) else None,
        }
        offset += record_len
    return value, pose


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


def clamp_finger_value(value: float) -> int:
    """Round and clamp one continuous value to a set_finger_angles field.

    The wire contract is a signed integer in ``[-100, 100]``: -100 is a joint's
    extend posture, 0 its rest posture, +100 its flex posture. The continuous
    stream carries ``[-1, 1]`` (positive flex, negative extend, zero rest), so
    the natural mapping is ``round(value * 100)``. Values already in ``[-100,
    100]`` pass through unchanged after rounding.
    """
    scaled = round(float(value))
    return max(-SET_FINGER_ANGLES_MAX, min(SET_FINGER_ANGLES_MAX, scaled))


def format_set_finger_angles(
    values: dict[str, int | float | None],
    order: tuple[str, ...] = SET_FINGER_ANGLES_ORDER,
) -> str:
    """Build a firmware ``set_finger_angles`` batch command (firmware >= 0.6.4).

    ``values`` maps joint name to a SIGNED integer in ``[-100, 100]`` on that
    joint's rest-anchored axis: -100 is its extend posture, 0 its rest posture,
    +100 its flex posture. A joint missing from the mapping, or mapped to
    ``None``, becomes an EMPTY field, which the firmware holds unchanged. Values
    are validated and rounded to an integer here rather than relying on the
    firmware, and trailing held joints are dropped to keep the datagram short (an
    omitted trailing field holds the same as an empty one).

    Args:
        values: Joint name -> signed value in ``[-100, 100]`` (or ``None`` to
            hold). Non-integer numbers are rounded to the nearest integer.
        order: Positional joint order; defaults to the wire order the firmware
            expects and must not be reordered lightly.

    Returns:
        str: e.g. ``"set_finger_angles:70:-40::0:-40"`` (ring held with an empty
        field). At least one joint must carry a value.

    Raises:
        ValueError: If no joint has a value, or a value is non-numeric,
            non-finite, or outside ``[-100, 100]``.
    """
    fields: list[str] = []
    have_value = False
    for joint in order:
        value = values.get(joint)
        if value is None:
            fields.append("")
            continue
        if isinstance(value, bool):
            raise ValueError(
                f"set_finger_angles value for {joint!r} must be numeric, got {value!r}"
            )
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"set_finger_angles value for {joint!r} must be numeric, got {value!r}"
            ) from exc
        if not math.isfinite(number):
            raise ValueError(
                f"set_finger_angles value for {joint!r} must be finite, got {value!r}"
            )
        rounded = round(number)
        if not -SET_FINGER_ANGLES_MAX <= rounded <= SET_FINGER_ANGLES_MAX:
            raise ValueError(
                f"set_finger_angles value for {joint!r} must be in "
                f"[-{SET_FINGER_ANGLES_MAX}, {SET_FINGER_ANGLES_MAX}], got {number:g}"
            )
        fields.append(str(int(rounded)))
        have_value = True
    if not have_value:
        raise ValueError("set_finger_angles needs at least one joint with a value")
    # Trailing empty (held) fields carry no information: an omitted field holds
    # exactly like an empty one, so drop them to shorten the datagram.
    while fields and fields[-1] == "":
        fields.pop()
    return SET_FINGER_ANGLES_PREFIX + ":" + ":".join(fields)


def pack_continuous_ack(
    sequence: int,
    joints: tuple[str, ...],
    values: dict[str, int | None],
) -> bytes:
    """Pack one ``NGA3`` continuous-ack datagram (schema nml.continuous.v2).

    Returned upstream for every accepted continuous datagram. Carries the
    uint32 ``sequence`` being acknowledged and the signed int8 value dispatched
    to each joint in ``joints`` order. A held joint (``None``) is encoded as its
    signed value if known; callers pass the value actually sent, so a held joint
    that was left unchanged is reported as 0 (rest) by convention.

    Args:
        sequence: The uint32 packet sequence being acked.
        joints: Joint order for the per-joint bytes.
        values: Joint name -> signed value in ``[-100, 100]`` actually sent.

    Returns:
        bytes: ``magic + uint32 seq + uint8 count + int8[count]``.
    """
    packed_values = []
    for joint in joints:
        raw = values.get(joint)
        signed = 0 if raw is None else int(raw)
        packed_values.append(max(-128, min(127, signed)))
    return struct.pack(
        CONTINUOUS_ACK_HEADER + CONTINUOUS_ACK_RECORD * len(joints),
        CONTINUOUS_ACK_MAGIC,
        int(sequence) & 0xFFFFFFFF,
        len(joints),
        *packed_values,
    )


def unpack_continuous_ack(
    data: bytes, joints: tuple[str, ...] = SET_FINGER_ANGLES_ORDER
) -> tuple[int, dict[str, int]] | None:
    """Parse an ``NGA3`` continuous-ack datagram.

    Returns ``(sequence, {joint: signed_value})`` or ``None`` for a frame that
    is truncated, carries the wrong magic, or whose joint count does not match
    ``joints``.
    """
    header_len = struct.calcsize(CONTINUOUS_ACK_HEADER)
    if not data or len(data) < header_len or not data.startswith(CONTINUOUS_ACK_MAGIC):
        return None
    _, sequence, count = struct.unpack_from(CONTINUOUS_ACK_HEADER, data)
    record_len = struct.calcsize("<" + CONTINUOUS_ACK_RECORD)
    if len(data) < header_len + count * record_len or count != len(joints):
        return None
    result: dict[str, int] = {}
    offset = header_len
    for joint in joints:
        (value,) = struct.unpack_from("<" + CONTINUOUS_ACK_RECORD, data, offset)
        result[joint] = value
        offset += record_len
    return sequence, result
