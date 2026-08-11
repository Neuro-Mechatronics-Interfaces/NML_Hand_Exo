from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .contracts import DecoderDecision, OrientationSample
from .models import ShrinkageLDAIntentModel
from .orientation import ContinuousRestAdapter


@dataclass
class IntentDecoderPipeline:
    rest_label: str = "rest"
    open_label: str = "open"
    close_label: str = "close"
    confidence_threshold: float = 0.60
    effort_deadband: float = 0.05
    require_orientation: bool = False
    adapter: ContinuousRestAdapter = field(default_factory=ContinuousRestAdapter)
    model: ShrinkageLDAIntentModel = field(default_factory=ShrinkageLDAIntentModel)
    _open_effort_axis: np.ndarray | None = field(default=None, init=False, repr=False)
    _close_effort_axis: np.ndarray | None = field(default=None, init=False, repr=False)
    _open_rest_anchor: float = field(default=0.0, init=False, repr=False)
    _open_mvc_anchor: float = field(default=1.0, init=False, repr=False)
    _close_rest_anchor: float = field(default=0.0, init=False, repr=False)
    _close_mvc_anchor: float = field(default=1.0, init=False, repr=False)

    def fit(
        self,
        features: np.ndarray,
        labels: np.ndarray,
        roll_deg: np.ndarray,
        pitch_deg: np.ndarray,
    ) -> "IntentDecoderPipeline":
        self.adapter.fit(features, labels, roll_deg, pitch_deg, self.rest_label)
        corrected = self.adapter.transform(features, roll_deg, pitch_deg)
        self.model.fit(corrected, labels)
        required = {self.rest_label, self.open_label, self.close_label}
        if not required.issubset(set(self.model.classes)):
            raise ValueError(f"Decoder is missing required classes: {sorted(required - set(self.model.classes))}")
        self._fit_continuous_effort(corrected, labels)
        return self

    def _fit_continuous_effort(
        self, corrected_features: np.ndarray, labels: np.ndarray
    ) -> None:
        """Anchor two one-vs-rest LDA projections at rest=0 and MVC=1.

        The rest anchor is the 95th percentile in the MVC direction rather
        than the median. This makes ordinary resting variation exactly zero;
        the selected gesture's median recorded contraction remains 1.0.
        """
        if self.model.scaler is None or self.model.classifier is None:
            raise RuntimeError("Intent model has not been fit")
        scaled = self.model.scaler.transform(
            np.asarray(corrected_features, dtype=np.float64)
        )
        y = np.asarray(labels, dtype=object)
        classes = list(self.model.classes)
        coefficients = np.asarray(self.model.classifier.coef_, dtype=np.float64)
        rest_index = classes.index(self.rest_label)

        def fit_direction(label: str) -> tuple[np.ndarray, float, float]:
            class_index = classes.index(label)
            axis = coefficients[class_index] - coefficients[rest_index]
            norm = float(np.linalg.norm(axis))
            if not np.isfinite(norm) or norm <= 1e-12:
                raise ValueError(f"Cannot construct a rest-to-MVC LDA axis for {label}")
            axis = axis / norm
            projection = scaled @ axis
            rest_values = projection[y == self.rest_label]
            mvc_values = projection[y == label]
            if float(np.median(mvc_values)) < float(np.median(rest_values)):
                axis = -axis
                projection = -projection
                rest_values = projection[y == self.rest_label]
                mvc_values = projection[y == label]
            rest_anchor = float(np.quantile(rest_values, 0.95))
            mvc_anchor = float(np.median(mvc_values))
            if not np.isfinite(rest_anchor) or not np.isfinite(mvc_anchor):
                raise ValueError(f"Invalid continuous-effort anchors for {label}")
            if mvc_anchor <= rest_anchor + 1e-9:
                raise ValueError(
                    f"Rest variation overlaps the recorded MVC anchor for {label}"
                )
            return axis, rest_anchor, mvc_anchor

        (
            self._open_effort_axis,
            self._open_rest_anchor,
            self._open_mvc_anchor,
        ) = fit_direction(self.open_label)
        (
            self._close_effort_axis,
            self._close_rest_anchor,
            self._close_mvc_anchor,
        ) = fit_direction(self.close_label)

    def project_continuous(
        self,
        features: np.ndarray,
        roll_deg: np.ndarray,
        pitch_deg: np.ndarray,
    ) -> dict[str, np.ndarray]:
        """Project one or more samples onto the normalized continuous control axis."""
        values = np.asarray(features, dtype=np.float64)
        if values.ndim == 1:
            values = values.reshape(1, -1)
        roll = np.asarray(roll_deg, dtype=np.float64).reshape(-1)
        pitch = np.asarray(pitch_deg, dtype=np.float64).reshape(-1)
        if len(values) != len(roll) or len(values) != len(pitch):
            raise ValueError("features, roll_deg, and pitch_deg must contain matching rows")
        if self.model.scaler is None:
            raise RuntimeError("Intent model has not been fit")

        corrected = self.adapter.transform(values, roll, pitch)
        probabilities = self.model.predict_proba(corrected)
        labels = list(self.model.classes)
        scaled = self.model.scaler.transform(corrected)

        def activation(axis, rest_anchor, mvc_anchor):
            if axis is None:
                raise RuntimeError("Continuous-effort calibration has not been fit")
            projection = scaled @ axis
            return np.clip(
                (projection - rest_anchor) / (mvc_anchor - rest_anchor),
                0.0,
                1.0,
            )

        open_activation = activation(
            self._open_effort_axis,
            self._open_rest_anchor,
            self._open_mvc_anchor,
        )
        close_activation = activation(
            self._close_effort_axis,
            self._close_rest_anchor,
            self._close_mvc_anchor,
        )
        rest_probability = probabilities[:, labels.index(self.rest_label)]
        open_probability = probabilities[:, labels.index(self.open_label)]
        close_probability = probabilities[:, labels.index(self.close_label)]
        close_direction = close_probability >= open_probability
        effort = np.where(close_direction, close_activation, open_activation)
        signed = np.where(close_direction, effort, -effort)
        confidence = rest_probability + np.maximum(open_probability, close_probability)
        rejected = confidence < self.confidence_threshold
        signed[(effort <= self.effort_deadband) | rejected] = 0.0
        return {
            "signed_intent": signed.astype(np.float64, copy=False),
            "open_activation": open_activation.astype(np.float64, copy=False),
            "close_activation": close_activation.astype(np.float64, copy=False),
            "confidence": confidence.astype(np.float64, copy=False),
            "rejected": rejected.astype(bool, copy=False),
            "probabilities": probabilities,
        }

    def predict(self, feature: np.ndarray, orientation: OrientationSample) -> DecoderDecision:
        if self.require_orientation and not orientation.is_available:
            return DecoderDecision(
                state=self.rest_label,
                signed_intent=0.0,
                confidence=0.0,
                rejected=True,
                reason="required orientation unavailable",
            )
        roll = np.asarray([np.nan if orientation.roll_deg is None else orientation.roll_deg])
        pitch = np.asarray([np.nan if orientation.pitch_deg is None else orientation.pitch_deg])
        projected = self.project_continuous(
            np.asarray(feature).reshape(1, -1), roll, pitch
        )
        probabilities = projected["probabilities"][0]
        labels = self.model.classes
        probability_map = {label: float(probabilities[index]) for index, label in enumerate(labels)}
        # Confidence measures support for the calibrated control manifold:
        # rest plus the stronger directional class. This stays high through a
        # legitimate rest-to-MVC transition while falling for reject samples
        # and simultaneous open/close ambiguity.
        confidence = float(projected["confidence"][0])
        if bool(projected["rejected"][0]):
            return DecoderDecision(
                state=self.rest_label,
                signed_intent=0.0,
                confidence=confidence,
                rejected=True,
                reason="low confidence",
                probabilities=probability_map,
            )
        open_activation = float(projected["open_activation"][0])
        close_activation = float(projected["close_activation"][0])
        signed = float(projected["signed_intent"][0])
        if signed == 0.0:
            state = self.rest_label
        else:
            state = self.close_label if signed > 0 else self.open_label
        return DecoderDecision(
            state=state,
            signed_intent=signed,
            confidence=confidence,
            rejected=False,
            probabilities=probability_map,
            open_activation=open_activation,
            close_activation=close_activation,
        )
