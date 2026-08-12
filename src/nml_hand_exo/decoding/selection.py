from __future__ import annotations

from itertools import combinations

import numpy as np

from .contracts import PairEvaluation
from .models import ShrinkageLDAIntentModel
from .orientation import ContinuousRestAdapter


def recording_id(group: object) -> str:
    """Return the source recording prefix stored in an imported trial group."""
    return str(group).split(":", 1)[0]


def _balanced_recording_folds(labels: np.ndarray, groups: np.ndarray, folds: int):
    """Yield folds that never split windows or trials from one recording."""
    recordings = np.asarray([recording_id(group) for group in groups], dtype=object)
    unique = np.unique(recordings)
    recording_labels = {}
    for recording in unique:
        present = sorted(
            set(str(value) for value in labels[recordings == recording]) - {"rest"}
        )
        recording_labels[recording] = "+".join(present) if present else "rest_only"
    assignment = {}
    rng = np.random.default_rng(17)
    for label in sorted(set(recording_labels.values())):
        candidates = np.asarray(
            [recording for recording in unique if recording_labels[recording] == label],
            dtype=object,
        )
        rng.shuffle(candidates)
        for index, recording in enumerate(candidates):
            assignment[recording] = index % folds
    for fold in range(folds):
        test = np.asarray([assignment[recording] == fold for recording in recordings])
        yield np.flatnonzero(~test), np.flatnonzero(test)


def _balanced_accuracy(actual: np.ndarray, predicted: np.ndarray, classes: list[str]) -> float:
    recalls = []
    for label in classes:
        mask = actual == label
        recalls.append(float(np.mean(predicted[mask] == label)) if np.any(mask) else 0.0)
    return float(np.mean(recalls))


def rank_intent_pairs(
    features: np.ndarray,
    labels: np.ndarray,
    groups: np.ndarray,
    roll_deg: np.ndarray,
    pitch_deg: np.ndarray,
    rest_label: str = "rest",
    reject_label: str = "reject",
    folds: int = 5,
    use_orientation: bool = True,
) -> list[PairEvaluation]:
    X = np.asarray(features, dtype=np.float64)
    y = np.asarray(labels, dtype=object)
    group_values = np.asarray(groups, dtype=object)
    roll = np.asarray(roll_deg, dtype=np.float64)
    pitch = np.asarray(pitch_deg, dtype=np.float64)
    if not use_orientation:
        roll = np.full(roll.shape, np.nan, dtype=np.float64)
        pitch = np.full(pitch.shape, np.nan, dtype=np.float64)
    candidates = sorted(set(str(value) for value in y) - {rest_label, reject_label})
    results = []
    for first, second in combinations(candidates, 2):
        evaluated_classes = [rest_label, first, second]
        if len(set(group_values[y == reject_label])) >= 2:
            evaluated_classes.append(reject_label)
        keep = np.isin(y, evaluated_classes)
        Xsub, ysub = X[keep], y[keep]
        gsub, rsub, psub = group_values[keep], roll[keep], pitch[keep]
        class_recording_counts = [
            len({recording_id(group) for group in gsub[ysub == label]})
            for label in evaluated_classes
        ]
        usable_folds = min(int(folds), *class_recording_counts)
        if usable_folds < 2:
            continue
        accuracy, rest_false, reject_false, direction_error = [], [], [], []
        for train, test in _balanced_recording_folds(ysub, gsub, usable_folds):
            adapter = ContinuousRestAdapter().fit(
                Xsub[train], ysub[train], rsub[train], psub[train], rest_label
            )
            train_features = adapter.transform(Xsub[train], rsub[train], psub[train])
            test_features = adapter.transform(Xsub[test], rsub[test], psub[test])
            model = ShrinkageLDAIntentModel().fit(train_features, ysub[train])
            prediction = model.predict(test_features)
            accuracy.append(_balanced_accuracy(ysub[test], prediction, evaluated_classes))
            rest = ysub[test] == rest_label
            rest_false.append(float(np.mean(prediction[rest] != rest_label)))
            reject = ysub[test] == reject_label
            if np.any(reject):
                reject_false.append(float(np.mean(np.isin(prediction[reject], [first, second]))))
            active = ~rest
            direction_error.append(float(np.mean(
                ((ysub[test] == first) & (prediction == second))
                | ((ysub[test] == second) & (prediction == first))
            )) / max(float(np.mean(active)), 1e-12))
        mean_accuracy = float(np.mean(accuracy))
        mean_rest_false = float(np.mean(rest_false))
        mean_reject_false = float(np.mean(reject_false)) if reject_false else 0.0
        mean_direction_error = float(np.mean(direction_error))
        composite = (
            mean_accuracy
            - 0.5 * mean_rest_false
            - 0.5 * mean_reject_false
            - 0.5 * mean_direction_error
        )
        results.append(PairEvaluation(
            open_label=first,
            close_label=second,
            balanced_accuracy=mean_accuracy,
            balanced_accuracy_std=float(np.std(accuracy)),
            rest_false_activation_rate=mean_rest_false,
            reject_false_activation_rate=mean_reject_false,
            direction_confusion_rate=mean_direction_error,
            composite_score=composite,
            folds=usable_folds,
        ))
    return sorted(results, key=lambda item: item.composite_score, reverse=True)
