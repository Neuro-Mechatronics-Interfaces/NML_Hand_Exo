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
