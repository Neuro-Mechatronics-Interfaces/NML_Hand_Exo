from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class OrientationSample:
    roll_deg: float | None = None
    pitch_deg: float | None = None
    gyro_norm: float | None = None
    accel_norm: float | None = None

    @property
    def is_available(self) -> bool:
        return self.roll_deg is not None and self.pitch_deg is not None


@dataclass(frozen=True)
class DecoderDecision:
    state: str
    signed_intent: float
    confidence: float
    rejected: bool
    reason: str = ""
    probabilities: dict[str, float] = field(default_factory=dict)
    open_activation: float = 0.0
    close_activation: float = 0.0


@dataclass(frozen=True)
class PairEvaluation:
    open_label: str
    close_label: str
    balanced_accuracy: float
    balanced_accuracy_std: float
    rest_false_activation_rate: float
    reject_false_activation_rate: float
    direction_confusion_rate: float
    composite_score: float
    folds: int
