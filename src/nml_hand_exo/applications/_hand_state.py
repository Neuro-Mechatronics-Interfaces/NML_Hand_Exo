"""Telemetry-to-display mapping; has no motor-command or device-I/O authority."""
import math


def motor_flexion_fraction(relative_angle, calibration):
    """Map a relative encoder angle to the firmware's home-to-flexion axis.

    Mirrors NMLHandExo::getGestureOrigin/getGestureSpan (firmware >= 0.6.0),
    including its 2:1 opposite-span override and 2 degree minimum travel.
    This is a schematic curl fraction, not an anatomical joint measurement.
    """
    if relative_angle is None or not calibration:
        return None
    try:
        relative, home, lo, hi = map(float, (
            relative_angle, calibration["home"],
            calibration["limit_min"], calibration["limit_max"],
        ))
        flip = calibration["flip"]
        if not isinstance(flip, bool) or not all(map(math.isfinite, (relative, home, lo, hi))) or lo >= hi:
            return None
    except (KeyError, TypeError, ValueError):
        return None
    origin = min(hi, max(lo, home))
    up, down = hi - origin, origin - lo
    preferred, opposite = (-down, up) if flip else (up, -down)
    span = opposite if abs(opposite) > abs(preferred) * 2 and abs(opposite) - abs(preferred) >= 2 else preferred
    if abs(span) < 2:
        return None
    absolute = home + (-relative if flip else relative)
    return min(1.0, max(0.0, (absolute - origin) / span))


def motor_display_sample(relative_angle, calibration, motor_id, metadata=None):
    """Keep the position's provenance and original sample clocks with its curl."""
    metadata = metadata or {}
    source = metadata.get("motor_field_sources", {}).get(motor_id, {}).get("position", "unknown")
    fraction = None if source == "unavailable" else motor_flexion_fraction(relative_angle, calibration)
    return {
        "fraction": fraction,
        "source": source if fraction is not None else "unavailable",
        "sample_timestamp_ms": metadata.get("motor_sample_timestamp_ms", {}).get(motor_id),
        "utc_timestamp_ms": metadata.get("motor_utc_timestamp_ms", {}).get(motor_id),
        "host_received_monotonic_s": metadata.get("host_poll_completed_monotonic_s"),
    }
