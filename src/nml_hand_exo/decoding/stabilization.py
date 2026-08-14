from __future__ import annotations

from dataclasses import dataclass, replace
import math

from .contracts import DecoderDecision


@dataclass
class IntentOutputStabilizer:
    """Smooth continuous intent and reject single-window direction reversals."""

    ema_alpha: float = 0.25
    max_step: float = 0.18
    enter_threshold: float = 0.08
    open_enter_threshold: float | None = None
    close_enter_threshold: float | None = None
    release_threshold: float = 0.04
    switch_samples: int = 3
    output_gain: float = 1.0
    response_exponent: float = 1.0
    value: float = 0.0
    direction: int = 0
    pending_direction: int = 0
    pending_count: int = 0

    def reset(self) -> None:
        self.value = 0.0
        self.direction = 0
        self.pending_direction = 0
        self.pending_count = 0

    def update(
        self,
        decision: DecoderDecision,
        *,
        open_label: str,
        close_label: str,
        rest_label: str = "rest",
    ) -> DecoderDecision:
        projection = (
            decision.signed_intent
            if decision.raw_signed_projection is None
            else decision.raw_signed_projection
        )
        projection = float(projection)
        if not math.isfinite(projection):
            projection = 0.0
        magnitude = min(
            1.0,
            max(0.0, float(self.output_gain))
            * abs(projection) ** max(0.05, float(self.response_exponent)),
        )
        raw = 0.0 if decision.rejected else math.copysign(magnitude, projection)
        if decision.rejected:
            self.reset()
            return replace(decision, state=rest_label, signed_intent=0.0)

        open_threshold = max(
            0.0,
            float(
                self.enter_threshold
                if self.open_enter_threshold is None
                else self.open_enter_threshold
            ),
        )
        close_threshold = max(
            0.0,
            float(
                self.enter_threshold
                if self.close_enter_threshold is None
                else self.close_enter_threshold
            ),
        )
        entered_direction = (
            1
            if projection >= close_threshold
            else -1
            if projection <= -open_threshold
            else 0
        )
        requested = entered_direction
        # Apply an actual center deadband when inactive, then use the smaller
        # release threshold as hysteresis once a direction has activated.
        if self.direction == 0 and requested == 0:
            raw = 0.0
        elif self.direction > 0 and entered_direction >= 0:
            if projection <= self.release_threshold:
                raw = 0.0
                requested = 0
                self.direction = 0
            else:
                requested = 1
        elif self.direction < 0 and entered_direction <= 0:
            if projection >= -self.release_threshold:
                raw = 0.0
                requested = 0
                self.direction = 0
            else:
                requested = -1
        if self.direction and requested == -self.direction:
            if self.pending_direction == requested:
                self.pending_count += 1
            else:
                self.pending_direction = requested
                self.pending_count = 1
            if self.pending_count < max(1, int(self.switch_samples)):
                raw = 0.0
                requested = 0
            else:
                self.direction = requested
                self.pending_direction = 0
                self.pending_count = 0
        else:
            self.pending_direction = 0
            self.pending_count = 0
            if requested:
                self.direction = requested
            elif abs(raw) <= self.release_threshold:
                self.direction = 0

        target = self.ema_alpha * raw + (1.0 - self.ema_alpha) * self.value
        delta = max(-self.max_step, min(self.max_step, target - self.value))
        self.value = max(-1.0, min(1.0, self.value + delta))
        if (
            requested == 0
            and self.pending_direction == 0
            and abs(self.value) <= self.release_threshold
        ):
            self.value = 0.0
            self.direction = 0
        if self.direction > 0 and self.value < 0.0:
            self.value = 0.0
        elif self.direction < 0 and self.value > 0.0:
            self.value = 0.0

        state = (
            close_label
            if self.value > 0.0
            else open_label
            if self.value < 0.0
            else rest_label
        )
        return replace(decision, state=state, signed_intent=float(self.value))
