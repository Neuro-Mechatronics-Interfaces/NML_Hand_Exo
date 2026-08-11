"""Read-only contact-evidence estimator for Phase-1 hardware characterization.

The estimator reports evidence only. Its output is intentionally not connected
to motor commands, arming, mode changes, or current targets.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class ShadowContactState(str, Enum):
    FREE = "free"
    CANDIDATE = "candidate"
    CONTACT = "contact"
    LIMIT = "limit"
    STALE = "stale"


@dataclass(frozen=True)
class ShadowContactConfig:
    closing_intent_threshold: float = 0.25
    current_on_mA: float = 80.0
    current_off_mA: float = 55.0
    velocity_on_deg_s: float = 8.0
    velocity_off_deg_s: float = 12.0
    contact_dwell_ms: int = 150
    stale_after_ms: int = 150
    limit_margin_deg: float = 5.0
    current_filter_tau_ms: float = 80.0
    velocity_filter_tau_ms: float = 60.0


@dataclass(frozen=True)
class ShadowContactResult:
    state: ShadowContactState
    filtered_current_mA: float
    filtered_velocity_deg_s: float
    evidence: bool
    near_limit: bool
    dwell_ms: int


class ShadowContactEstimator:
    """Stateful, hysteretic estimator for one finger digit."""

    def __init__(self, config: ShadowContactConfig | None = None):
        self.config = config or ShadowContactConfig()
        self._filtered_current = 0.0
        self._filtered_velocity = 0.0
        self._last_update_ms: int | None = None
        self._candidate_since_ms: int | None = None
        self._contact = False

    @staticmethod
    def _filter(previous: float, sample: float, dt_ms: int, tau_ms: float) -> float:
        if tau_ms <= 0 or dt_ms <= 0:
            return float(sample)
        alpha = 1.0 - math.exp(-float(dt_ms) / float(tau_ms))
        return previous + alpha * (float(sample) - previous)

    def update(
        self,
        *,
        now_ms: int,
        sample_ms: int,
        intent: float,
        current_mA: float,
        velocity_deg_s: float,
        angle_deg: float,
        lower_limit_deg: float,
        upper_limit_deg: float,
        closing_intent_sign: float = 1.0,
        closing_motion_sign: float = 1.0,
    ) -> ShadowContactResult:
        now = int(now_ms)
        sample_time = int(sample_ms)
        dt = 0 if self._last_update_ms is None else max(0, now - self._last_update_ms)
        self._last_update_ms = now
        self._filtered_current = self._filter(
            self._filtered_current, abs(float(current_mA)), dt,
            self.config.current_filter_tau_ms,
        )
        self._filtered_velocity = self._filter(
            self._filtered_velocity, float(velocity_deg_s), dt,
            self.config.velocity_filter_tau_ms,
        )

        age_ms = max(0, now - sample_time)
        closing = float(intent) * float(closing_intent_sign)
        close_direction = 1.0 if float(closing_motion_sign) >= 0 else -1.0
        distance_to_limit = (
            float(upper_limit_deg) - float(angle_deg)
            if close_direction > 0
            else float(angle_deg) - float(lower_limit_deg)
        )
        near_limit = distance_to_limit <= self.config.limit_margin_deg

        if age_ms > self.config.stale_after_ms:
            self._candidate_since_ms = None
            self._contact = False
            return ShadowContactResult(
                ShadowContactState.STALE,
                self._filtered_current,
                self._filtered_velocity,
                False,
                near_limit,
                0,
            )
        if near_limit and closing >= self.config.closing_intent_threshold:
            self._candidate_since_ms = None
            self._contact = False
            return ShadowContactResult(
                ShadowContactState.LIMIT,
                self._filtered_current,
                self._filtered_velocity,
                False,
                True,
                0,
            )

        current_threshold = (
            self.config.current_off_mA if self._contact else self.config.current_on_mA
        )
        velocity_threshold = (
            self.config.velocity_off_deg_s if self._contact
            else self.config.velocity_on_deg_s
        )
        evidence = (
            closing >= self.config.closing_intent_threshold
            and self._filtered_current >= current_threshold
            and abs(self._filtered_velocity) <= velocity_threshold
        )
        if evidence:
            if self._candidate_since_ms is None:
                self._candidate_since_ms = now
            dwell = max(0, now - self._candidate_since_ms)
            if dwell >= self.config.contact_dwell_ms:
                self._contact = True
            state = (
                ShadowContactState.CONTACT
                if self._contact else ShadowContactState.CANDIDATE
            )
        else:
            self._candidate_since_ms = None
            self._contact = False
            dwell = 0
            state = ShadowContactState.FREE

        return ShadowContactResult(
            state,
            self._filtered_current,
            self._filtered_velocity,
            evidence,
            near_limit,
            dwell,
        )
