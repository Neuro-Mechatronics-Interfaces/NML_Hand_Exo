from __future__ import annotations

from dataclasses import dataclass, replace

from .contracts import DecoderDecision


@dataclass
class IntentOutputStabilizer:
    """Smooth continuous intent and reject single-window direction reversals."""

    ema_alpha: float = 0.35
    max_step: float = 0.18
    enter_threshold: float = 0.12
    release_threshold: float = 0.06
    switch_samples: int = 3
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
        raw = 0.0 if decision.rejected else max(-1.0, min(1.0, float(decision.signed_intent)))
        if decision.rejected:
            self.reset()
            return replace(decision, state=rest_label, signed_intent=0.0)

        requested = 1 if raw >= self.enter_threshold else -1 if raw <= -self.enter_threshold else 0
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
