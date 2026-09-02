"""Grouped validation for intent and state prediction models."""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Callable

import numpy as np
from sklearn.metrics import balanced_accuracy_score, confusion_matrix, mean_absolute_error, r2_score
from sklearn.model_selection import GroupKFold, LeaveOneGroupOut

from .models import StateConditionedIntentModel


@dataclass(frozen=True)
class ClassificationFold:
    held_out_groups: tuple[str, ...]
    balanced_accuracy: float
    labels: tuple[str, ...]
    confusion: tuple[tuple[int, ...], ...]

    def to_dict(self) -> dict:
        return asdict(self)


def evaluate_intent_grouped(
    emg_features: np.ndarray,
    labels: np.ndarray,
    groups: np.ndarray,
    state: np.ndarray | None = None,
    *,
    folds: int | None = 5,
) -> dict:
    x = np.asarray(emg_features, dtype=np.float64)
    y = np.asarray(labels).astype(str)
    g = np.asarray(groups).astype(str)
    if not (len(x) == len(y) == len(g)):
        raise ValueError("features, labels, and groups must contain matching samples")
    if len(np.unique(g)) < 2:
        raise ValueError("Grouped evaluation requires at least two groups")
    if state is not None and len(np.asarray(state)) != len(x):
        raise ValueError("state and features must contain matching samples")
    splitter = (
        LeaveOneGroupOut()
        if folds is None
        else GroupKFold(n_splits=min(int(folds), len(np.unique(g))))
    )
    all_labels = tuple(sorted(np.unique(y)))
    fold_results = []
    for train, test in splitter.split(x, y, g):
        measured_state = None if state is None else np.asarray(state, dtype=np.float64)
        model = StateConditionedIntentModel().fit(
            x[train], y[train], None if measured_state is None else measured_state[train]
        )
        prediction = model.predict(
            x[test], None if measured_state is None else measured_state[test]
        )
        fold_results.append(
            ClassificationFold(
                held_out_groups=tuple(sorted(set(g[test]))),
                balanced_accuracy=float(balanced_accuracy_score(y[test], prediction)),
                labels=all_labels,
                confusion=tuple(
                    tuple(int(value) for value in row)
                    for row in confusion_matrix(y[test], prediction, labels=all_labels)
                ),
            )
        )
    scores = np.asarray([item.balanced_accuracy for item in fold_results])
    return {
        "schema": "nml.grouped_intent_evaluation.v1",
        "fold_count": len(fold_results),
        "balanced_accuracy_mean": float(np.mean(scores)),
        "balanced_accuracy_std": float(np.std(scores)),
        "folds": [item.to_dict() for item in fold_results],
    }


def regression_metrics(reference: np.ndarray, prediction: np.ndarray) -> dict:
    y = np.asarray(reference, dtype=np.float64)
    yhat = np.asarray(prediction, dtype=np.float64)
    if y.shape != yhat.shape or y.ndim != 2:
        raise ValueError("reference and prediction must have matching (samples, outputs) shape")
    return {
        "mae_per_output": mean_absolute_error(y, yhat, multioutput="raw_values").tolist(),
        "mae_mean": float(mean_absolute_error(y, yhat)),
        "r2_per_output": r2_score(y, yhat, multioutput="raw_values").tolist(),
        "r2_variance_weighted": float(r2_score(y, yhat, multioutput="variance_weighted")),
    }
