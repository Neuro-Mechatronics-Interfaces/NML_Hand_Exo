"""Qt-free bell-shaped torque-pulse helpers for the UDP receiver.

The UDP source sends discrete integer gesture states.  Rather than holding a
flat current while a state is active, torque-mode bindings play a bell-shaped
current pulse toward a target endpoint: the current ramps smoothly from zero up
to a per-motor peak and back to zero over a fixed duration.  A later "let go"
action reverts the applied pulse and then eases each joint back to its homed
angle with set-position commands.

This module has no Qt or serial dependencies so the profile maths can be unit
tested without hardware (mirrors :mod:`_udp_metrics`).  All timing is passed in
explicitly in milliseconds; the caller owns the clock and the QTimer.
"""

from __future__ import annotations

import math


def raised_cosine_amplitude(elapsed_ms: float, duration_ms: float) -> float:
    """Return the Hann-window amplitude in ``[0, 1]`` for a pulse.

    ``amp(t) = 0.5 * (1 - cos(2*pi*t/T))`` — zero at both ends, peaking at 1.0
    at ``t = T/2``.  Outside ``(0, T)`` the amplitude is zero so a finished or
    not-yet-started pulse applies no current.
    """
    if duration_ms <= 0:
        return 0.0
    if elapsed_ms <= 0.0 or elapsed_ms >= duration_ms:
        return 0.0
    return 0.5 * (1.0 - math.cos(2.0 * math.pi * elapsed_ms / duration_ms))


def smoothstep(t: float) -> float:
    """Return the smoothstep easing ``3t^2 - 2t^3`` with ``t`` clamped to [0, 1]."""
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t * t * (3.0 - 2.0 * t)


class TorquePulse:
    """A bell-shaped current pulse over a fixed duration.

    Parameters
    ----------
    peaks:
        Mapping of Dynamixel motor ID to its signed peak current in mA.  The
        sign carries flexion/extension direction; the Hann envelope scales the
        magnitude over time.
    duration_ms:
        Total pulse duration in milliseconds.
    start_ms:
        Monotonic timestamp (ms) marking ``t = 0`` for the pulse.
    shape:
        Envelope name; only ``"raised_cosine"`` is currently supported.
    """

    def __init__(
        self,
        peaks: dict[int, float],
        duration_ms: float,
        start_ms: float,
        shape: str = "raised_cosine",
    ):
        if shape != "raised_cosine":
            raise ValueError(f"Unsupported pulse shape: {shape}")
        self.peaks = {int(k): float(v) for k, v in peaks.items()}
        self.duration_ms = float(duration_ms)
        self.start_ms = float(start_ms)
        self.shape = shape

    def elapsed(self, now_ms: float) -> float:
        return float(now_ms) - self.start_ms

    def is_done(self, now_ms: float) -> bool:
        return self.elapsed(now_ms) >= self.duration_ms

    def sample(self, now_ms: float) -> tuple[dict[int, float], bool]:
        """Return ``({motor_id: current_ma}, done)`` for the current instant.

        When the pulse is finished, every target's current is zero and ``done``
        is ``True`` so the caller can emit a final torque-off and stop stepping.
        """
        done = self.is_done(now_ms)
        if done:
            return ({motor_id: 0.0 for motor_id in self.peaks}, True)
        amp = raised_cosine_amplitude(self.elapsed(now_ms), self.duration_ms)
        return ({motor_id: peak * amp for motor_id, peak in self.peaks.items()}, False)
