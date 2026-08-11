from __future__ import annotations

from itertools import combinations

import numpy as np

from .contracts import PairEvaluation
from .models import ShrinkageLDAIntentModel
from .orientation import ContinuousRestAdapter


def _balanced_group_folds(labels: np.ndarray, groups: np.ndarray, folds: int):
    unique = np.unique(groups)
    group_labels = {group: str(labels[np.flatnonzero(groups == group)[0]]) for group in unique}
    assignment = {}
    rng = np.random.default_rng(17)
    for label in sorted(set(group_labels.values())):
        candidates = np.asarray([g for g in unique if group_labels[g] == label], dtype=object)
        rng.shuffle(candidates)
        for index, group in enumerate(candidates):
            assignment[group] = index % folds
    for fold in range(folds):
        test = np.asarray([assignment[group] == fold for group in groups])
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
) -> list[PairEvaluation]:
    X = np.asarray(features, dtype=np.float64)
    y = np.asarray(labels, dtype=object)
    group_values = np.asarray(groups, dtype=object)
    roll = np.asarray(roll_deg, dtype=np.float64)
    pitch = np.asarray(pitch_deg, dtype=np.float64)
    candidates = sorted(set(str(value) for value in y) - {rest_label, reject_label})
    results = []
    for first, second in combinations(candidates, 2):
        evaluated_classes = [rest_label, first, second]
        if len(set(group_values[y == reject_label])) >= 2:
            evaluated_classes.append(reject_label)
        keep = np.isin(y, evaluated_classes)
        Xsub, ysub = X[keep], y[keep]
        gsub, rsub, psub = group_values[keep], roll[keep], pitch[keep]
        class_group_counts = [len(set(gsub[ysub == label])) for label in evaluated_classes]
        usable_folds = min(int(folds), *class_group_counts)
        if usable_folds < 2:
            continue
        accuracy, rest_false, reject_false, direction_error = [], [], [], []
        for train, test in _balanced_group_folds(ysub, gsub, usable_folds):
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
