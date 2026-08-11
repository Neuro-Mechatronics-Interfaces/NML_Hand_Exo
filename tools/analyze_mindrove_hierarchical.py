"""Evaluate orientation-conditioned EMG decoding from alternating XDF blocks.

Expected protocol: every recording starts with rest, then alternates 5-second
rest/gesture blocks.  File names identify the gesture.  The default MindRove
layout is channel 0 metadata, channels 1..8 EMG, and channels 9..11 accel.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from sklearn.decomposition import PCA
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.covariance import LedoitWolf
from sklearn.metrics import balanced_accuracy_score, confusion_matrix, f1_score
from sklearn.preprocessing import StandardScaler


GESTURES = {
    "block_indexext": "index_extend",
    "block_indexflex": "index_flex",
    "block_thumbpinchclose": "pinch_close",
    "block_thumbpinchopen": "pinch_open",
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_root", type=Path)
    parser.add_argument("--out-json", type=Path, required=True)
    parser.add_argument("--events-out", type=Path, required=True)
    parser.add_argument("--block-seconds", type=float, default=5.0)
    parser.add_argument("--trim-seconds", type=float, default=0.5)
    parser.add_argument("--window-seconds", type=float, default=0.25)
    parser.add_argument("--hop-seconds", type=float, default=0.125)
    parser.add_argument("--folds", type=int, default=5)
    return parser.parse_args()


def load_xdf(path: Path):
    try:
        import pyxdf
    except ImportError as exc:
        raise RuntimeError("Install pyxdf to read XDF recordings") from exc
    streams, _ = pyxdf.load_xdf(str(path), verbose=False)
    candidates = [s for s in streams if np.asarray(s["time_series"]).ndim == 2]
    if not candidates:
        raise RuntimeError(f"No numeric stream found in {path}")
    return max(candidates, key=lambda s: np.asarray(s["time_series"]).shape[0])


def orientation_from_accel(accel: np.ndarray) -> tuple[float, float]:
    ax, ay, az = (float(v) for v in accel)
    roll = np.arctan2(ay, az)
    pitch = np.arctan2(-ax, np.hypot(ay, az))
    return roll, pitch


def feature_window(emg: np.ndarray) -> np.ndarray:
    # Spatial common-mode removal followed by per-channel log-RMS.
    x = emg - emg.mean(axis=1, keepdims=True)
    return np.log1p(np.sqrt(np.mean(x * x, axis=0) + 1e-12))


def build_dataset(args):
    X, y, groups, roll, pitch, sessions = [], [], [], [], [], []
    events = []
    recordings = []
    for path in sorted(args.data_root.rglob("*.xdf")):
        gesture = GESTURES.get(path.stem)
        if gesture is None:
            continue
        stream = load_xdf(path)
        values = np.asarray(stream["time_series"], dtype=float)
        timestamps = np.asarray(stream["time_stamps"], dtype=float)
        if values.shape[1] < 12:
            raise RuntimeError(f"Expected at least 12 channels in {path.name}")
        fs = float(stream["info"]["nominal_srate"][0])
        duration = float(timestamps[-1] - timestamps[0])
        complete_blocks = int(duration // args.block_seconds)
        recordings.append({
            "file": path.name,
            "gesture": gesture,
            "samples": int(len(values)),
            "sample_rate_hz": fs,
            "duration_s": duration,
            "complete_blocks": complete_blocks,
        })
        for block in range(complete_blocks):
            start_s = block * args.block_seconds
            end_s = start_s + args.block_seconds
            label = "rest" if block % 2 == 0 else gesture
            events.append({
                "file": path.name,
                "block_index": block,
                "label": label,
                "phase_type": "rest" if label == "rest" else "gesture",
                "start_time_s": start_s,
                "end_time_s": end_s,
                "duration_s": args.block_seconds,
                "start_timestamp": float(timestamps[0] + start_s),
                "end_timestamp": float(timestamps[0] + end_s),
                "analysis_start_s": start_s + args.trim_seconds,
                "analysis_end_s": end_s - args.trim_seconds,
            })

        win = max(1, int(round(args.window_seconds * fs)))
        hop = max(1, int(round(args.hop_seconds * fs)))
        relative = timestamps - timestamps[0]
        for start in range(0, len(values) - win + 1, hop):
            center_s = float(np.mean(relative[start : start + win]))
            block = int(center_s // args.block_seconds)
            within = center_s - block * args.block_seconds
            if block >= complete_blocks:
                continue
            if within < args.trim_seconds or within > args.block_seconds - args.trim_seconds:
                continue
            label = "rest" if block % 2 == 0 else gesture
            window = values[start : start + win]
            X.append(feature_window(window[:, 1:9]))
            accel = np.mean(window[:, 9:12], axis=0)
            r, p = orientation_from_accel(accel)
            roll.append(r)
            pitch.append(p)
            y.append(label)
            groups.append(f"{path.name}:{block}")
            sessions.append(path.name)
    if not X:
        raise RuntimeError(f"No recognized XDF files found below {args.data_root}")
    return (
        np.asarray(X), np.asarray(y, dtype=object), np.asarray(groups, dtype=object),
        np.asarray(roll), np.asarray(pitch), np.asarray(sessions, dtype=object),
        recordings, events,
    )


def balanced_group_folds(y: np.ndarray, groups: np.ndarray, n_folds: int):
    rng = np.random.default_rng(7)
    unique_groups = np.unique(groups)
    group_labels = {g: str(y[np.flatnonzero(groups == g)[0]]) for g in unique_groups}
    assignment = {}
    for label in sorted(set(group_labels.values())):
        class_groups = np.asarray([g for g in unique_groups if group_labels[g] == label], dtype=object)
        rng.shuffle(class_groups)
        for index, group in enumerate(class_groups):
            assignment[group] = index % n_folds
    for fold in range(n_folds):
        test = np.asarray([assignment[g] == fold for g in groups])
        yield np.flatnonzero(~test), np.flatnonzero(test)


def orientation_basis(roll, pitch, mode):
    if mode == "roll":
        return np.column_stack([np.ones(len(roll)), np.sin(roll), np.cos(roll)])
    if mode == "roll_pitch":
        return np.column_stack([
            np.ones(len(roll)), np.sin(roll), np.cos(roll),
            np.sin(pitch), np.cos(pitch),
        ])
    raise ValueError(mode)


def fit_rest_adapter(X, y, roll, pitch, mode):
    rest = y == "rest"
    design = orientation_basis(roll[rest], pitch[rest], mode)
    ridge = np.eye(design.shape[1]) * 1e-3
    ridge[0, 0] = 0.0
    coef = np.linalg.solve(design.T @ design + ridge, design.T @ X[rest])
    fitted = design @ coef
    residual = X[rest] - fitted
    spread = residual.std(axis=0)
    scale = spread + 0.25 * np.median(spread) + 1e-6
    return coef, scale


def apply_rest_adapter(X, roll, pitch, mode, coef, scale):
    baseline = orientation_basis(roll, pitch, mode) @ coef
    return (X - baseline) / scale


def fit_lda(X, y):
    scaler = StandardScaler().fit(X)
    classes = np.unique(y)
    lda = LinearDiscriminantAnalysis(
        solver="lsqr", shrinkage="auto", priors=np.full(len(classes), 1.0 / len(classes))
    ).fit(scaler.transform(X), y)
    return scaler, lda


def predict_lda(model, X):
    scaler, lda = model
    return lda.predict(scaler.transform(X))


def fit_conditional_prototypes(X, y, roll, pitch):
    """Fit a smooth orientation-dependent centroid for every class.

    A shared shrinkage covariance keeps the comparison LDA-like while allowing
    each gesture prototype to move continuously with roll and pitch.
    """
    design = orientation_basis(roll, pitch, "roll_pitch")
    classes = sorted(set(y))
    coefficients = {}
    residuals = []
    ridge = np.eye(design.shape[1]) * 1e-3
    ridge[0, 0] = 0.0
    for label in classes:
        mask = y == label
        class_design = design[mask]
        coef = np.linalg.solve(
            class_design.T @ class_design + ridge,
            class_design.T @ X[mask],
        )
        coefficients[label] = coef
        residuals.append(X[mask] - class_design @ coef)
    covariance = LedoitWolf().fit(np.vstack(residuals))
    return classes, coefficients, covariance.precision_


def predict_conditional_prototypes(model, X, roll, pitch):
    classes, coefficients, precision = model
    design = orientation_basis(roll, pitch, "roll_pitch")
    distances = []
    for label in classes:
        residual = X - design @ coefficients[label]
        distances.append(np.einsum("ij,jk,ik->i", residual, precision, residual))
    return np.asarray(classes, dtype=object)[np.argmin(np.column_stack(distances), axis=1)]


def orientation_interaction_features(X, roll, pitch):
    """Allow each EMG channel's decision weight to vary with orientation."""
    orientation = np.column_stack([
        np.sin(roll), np.cos(roll), np.sin(pitch), np.cos(pitch)
    ])
    interactions = [X * orientation[:, index : index + 1] for index in range(orientation.shape[1])]
    return np.column_stack([X, orientation, *interactions])


def discrete_expert_predict(Xtr, ytr, rtr, Xte, rte):
    fallback = fit_lda(Xtr, ytr)
    thresholds = np.quantile(rtr, [1 / 3, 2 / 3])
    train_bins = np.digitize(rtr, thresholds)
    test_bins = np.digitize(rte, thresholds)
    experts = {}
    required = set(np.unique(ytr))
    for bin_id in range(3):
        mask = train_bins == bin_id
        if set(np.unique(ytr[mask])) == required and np.sum(mask) >= 5 * len(required):
            experts[bin_id] = fit_lda(Xtr[mask], ytr[mask])
    prediction = np.empty(len(Xte), dtype=object)
    for bin_id in range(3):
        mask = test_bins == bin_id
        if np.any(mask):
            prediction[mask] = predict_lda(experts.get(bin_id, fallback), Xte[mask])
    return prediction


def fit_soft_roll_experts(X, y, roll):
    classes = np.unique(y)
    centers = np.quantile(roll, [0.2, 0.5, 0.8])
    bandwidth = max(float(np.quantile(np.abs(roll[:, None] - centers[None, :]), 0.55)), np.deg2rad(5.0))
    fallback = fit_lda(X, y)
    experts = []
    for center in centers:
        distance = np.abs(roll - center)
        # Overlapping neighborhoods keep transitions smooth and ensure each
        # expert sees enough complete gesture blocks.
        cutoff = np.quantile(distance, 0.72)
        mask = distance <= cutoff
        experts.append(fit_lda(X[mask], y[mask]) if set(np.unique(y[mask])) == set(classes) else fallback)
    return classes, centers, bandwidth, experts


def predict_soft_roll_experts(model, X, roll):
    classes, centers, bandwidth, experts = model
    weights = np.exp(-0.5 * ((roll[:, None] - centers[None, :]) / bandwidth) ** 2)
    weights /= np.maximum(weights.sum(axis=1, keepdims=True), 1e-12)
    probabilities = np.zeros((len(X), len(classes)), dtype=float)
    for expert_index, (scaler, lda) in enumerate(experts):
        local = lda.predict_proba(scaler.transform(X))
        aligned = np.zeros_like(probabilities)
        for source_index, label in enumerate(lda.classes_):
            aligned[:, np.flatnonzero(classes == label)[0]] = local[:, source_index]
        probabilities += weights[:, expert_index : expert_index + 1] * aligned
    return classes[np.argmax(probabilities, axis=1)]


def evaluate(X, y, groups, roll, pitch, n_folds):
    classes = sorted(set(y))
    models = (
        "raw", "discrete_roll_experts", "soft_roll_experts", "continuous_roll",
        "continuous_roll_pitch", "continuous_class_prototypes",
        "continuous_interaction_lda", "rest_corrected_interaction_lda",
    )
    folds = []
    aggregate = {name: np.zeros((len(classes), len(classes)), dtype=int) for name in models}
    for fold, (train, test) in enumerate(balanced_group_folds(y, groups, n_folds), start=1):
        for model_name in models:
            if model_name == "raw":
                pred = predict_lda(fit_lda(X[train], y[train]), X[test])
            elif model_name == "discrete_roll_experts":
                pred = discrete_expert_predict(
                    X[train], y[train], roll[train], X[test], roll[test]
                )
            elif model_name == "soft_roll_experts":
                pred = predict_soft_roll_experts(
                    fit_soft_roll_experts(X[train], y[train], roll[train]),
                    X[test], roll[test],
                )
            elif model_name == "continuous_class_prototypes":
                conditional = fit_conditional_prototypes(
                    X[train], y[train], roll[train], pitch[train]
                )
                pred = predict_conditional_prototypes(
                    conditional, X[test], roll[test], pitch[test]
                )
            elif model_name == "continuous_interaction_lda":
                Xtr = orientation_interaction_features(
                    X[train], roll[train], pitch[train]
                )
                Xte = orientation_interaction_features(
                    X[test], roll[test], pitch[test]
                )
                pred = predict_lda(fit_lda(Xtr, y[train]), Xte)
            elif model_name == "rest_corrected_interaction_lda":
                coef, scale = fit_rest_adapter(
                    X[train], y[train], roll[train], pitch[train], "roll_pitch"
                )
                corrected_train = apply_rest_adapter(
                    X[train], roll[train], pitch[train], "roll_pitch", coef, scale
                )
                corrected_test = apply_rest_adapter(
                    X[test], roll[test], pitch[test], "roll_pitch", coef, scale
                )
                Xtr = orientation_interaction_features(
                    corrected_train, roll[train], pitch[train]
                )
                Xte = orientation_interaction_features(
                    corrected_test, roll[test], pitch[test]
                )
                pred = predict_lda(fit_lda(Xtr, y[train]), Xte)
            else:
                mode = "roll" if model_name == "continuous_roll" else "roll_pitch"
                coef, scale = fit_rest_adapter(
                    X[train], y[train], roll[train], pitch[train], mode
                )
                Xtr = apply_rest_adapter(X[train], roll[train], pitch[train], mode, coef, scale)
                Xte = apply_rest_adapter(X[test], roll[test], pitch[test], mode, coef, scale)
                pred = predict_lda(fit_lda(Xtr, y[train]), Xte)
            cm = confusion_matrix(y[test], pred, labels=classes)
            aggregate[model_name] += cm
            folds.append({
                "fold": fold,
                "model": model_name,
                "balanced_accuracy": float(balanced_accuracy_score(y[test], pred)),
                "macro_f1": float(f1_score(y[test], pred, average="macro")),
                "test_groups": int(len(set(groups[test]))),
            })
    summary = {}
    for model_name in models:
        rows = [r for r in folds if r["model"] == model_name]
        summary[model_name] = {
            "balanced_accuracy_mean": float(np.mean([r["balanced_accuracy"] for r in rows])),
            "balanced_accuracy_std": float(np.std([r["balanced_accuracy"] for r in rows])),
            "macro_f1_mean": float(np.mean([r["macro_f1"] for r in rows])),
            "confusion_matrix": aggregate[model_name].tolist(),
        }
    return classes, folds, summary


def pairwise_results(X, y, groups, roll, pitch, n_folds):
    output = []
    for gesture in sorted(set(y) - {"rest"}):
        keep = np.isin(y, ["rest", gesture])
        result = {"gesture": gesture}
        for model_name, mode in (("raw", None), ("continuous_roll_pitch", "roll_pitch")):
            values = []
            for train, test in balanced_group_folds(y[keep], groups[keep], n_folds):
                Xsub, ysub = X[keep], y[keep]
                rsub, psub = roll[keep], pitch[keep]
                if mode is None:
                    Xtr, Xte = Xsub[train], Xsub[test]
                else:
                    coef, scale = fit_rest_adapter(
                        Xsub[train], ysub[train], rsub[train], psub[train], mode
                    )
                    Xtr = apply_rest_adapter(Xsub[train], rsub[train], psub[train], mode, coef, scale)
                    Xte = apply_rest_adapter(Xsub[test], rsub[test], psub[test], mode, coef, scale)
                pred = predict_lda(fit_lda(Xtr, ysub[train]), Xte)
                values.append(float(balanced_accuracy_score(ysub[test], pred)))
            result[model_name] = {
                "balanced_accuracy_mean": float(np.mean(values)),
                "balanced_accuracy_std": float(np.std(values)),
            }
        output.append(result)
    return output


def gesture_pair_results(X, y, groups, roll, pitch, n_folds):
    gestures = sorted(set(y) - {"rest"})
    output = []
    for first_index, first in enumerate(gestures):
        for second in gestures[first_index + 1 :]:
            keep = np.isin(y, [first, second])
            Xsub, ysub = X[keep], y[keep]
            gsub, rsub, psub = groups[keep], roll[keep], pitch[keep]
            result = {"classes": [first, second]}
            scores = {name: [] for name in ("raw", "discrete_roll_experts", "continuous_roll_pitch")}
            for train, test in balanced_group_folds(ysub, gsub, n_folds):
                scores["raw"].append(float(balanced_accuracy_score(
                    ysub[test], predict_lda(fit_lda(Xsub[train], ysub[train]), Xsub[test])
                )))
                scores["discrete_roll_experts"].append(float(balanced_accuracy_score(
                    ysub[test], discrete_expert_predict(
                        Xsub[train], ysub[train], rsub[train], Xsub[test], rsub[test]
                    )
                )))
                # With no rest samples in this comparison, learn the orientation
                # baseline from both classes. This tests nuisance regression,
                # not a rest-reference adapter.
                design = orientation_basis(rsub[train], psub[train], "roll_pitch")
                ridge = np.eye(design.shape[1]) * 1e-3
                ridge[0, 0] = 0.0
                coef = np.linalg.solve(
                    design.T @ design + ridge, design.T @ Xsub[train]
                )
                residual_train = Xsub[train] - design @ coef
                residual_test = Xsub[test] - orientation_basis(
                    rsub[test], psub[test], "roll_pitch"
                ) @ coef
                scores["continuous_roll_pitch"].append(float(balanced_accuracy_score(
                    ysub[test], predict_lda(
                        fit_lda(residual_train, ysub[train]), residual_test
                    )
                )))
            for model_name, values in scores.items():
                result[model_name] = {
                    "balanced_accuracy_mean": float(np.mean(values)),
                    "balanced_accuracy_std": float(np.std(values)),
                }
            output.append(result)
    return output


def control_triplet_results(X, y, groups, roll, pitch, n_folds):
    configurations = {
        "index_flex_extend": ["rest", "index_flex", "index_extend"],
        "pinch_open_close": ["rest", "pinch_open", "pinch_close"],
    }
    output = []
    for name, labels in configurations.items():
        keep = np.isin(y, labels)
        Xsub, ysub = X[keep], y[keep]
        gsub, rsub, psub = groups[keep], roll[keep], pitch[keep]
        scores = {model: [] for model in (
            "raw", "discrete_roll_experts", "soft_roll_experts", "continuous_roll",
            "continuous_roll_pitch", "continuous_class_prototypes",
            "continuous_interaction_lda", "rest_corrected_interaction_lda",
        )}
        aggregate = {
            model: np.zeros((len(labels), len(labels)), dtype=int) for model in scores
        }
        for train, test in balanced_group_folds(ysub, gsub, n_folds):
            predictions = {
                "raw": predict_lda(fit_lda(Xsub[train], ysub[train]), Xsub[test]),
                "discrete_roll_experts": discrete_expert_predict(
                    Xsub[train], ysub[train], rsub[train], Xsub[test], rsub[test]
                ),
                "soft_roll_experts": predict_soft_roll_experts(
                    fit_soft_roll_experts(
                        Xsub[train], ysub[train], rsub[train]
                    ),
                    Xsub[test], rsub[test],
                ),
                "continuous_class_prototypes": predict_conditional_prototypes(
                    fit_conditional_prototypes(
                        Xsub[train], ysub[train], rsub[train], psub[train]
                    ),
                    Xsub[test], rsub[test], psub[test],
                ),
                "continuous_interaction_lda": predict_lda(
                    fit_lda(
                        orientation_interaction_features(
                            Xsub[train], rsub[train], psub[train]
                        ),
                        ysub[train],
                    ),
                    orientation_interaction_features(
                        Xsub[test], rsub[test], psub[test]
                    ),
                ),
            }
            coef, scale = fit_rest_adapter(
                Xsub[train], ysub[train], rsub[train], psub[train], "roll_pitch"
            )
            corrected_train = apply_rest_adapter(
                Xsub[train], rsub[train], psub[train], "roll_pitch", coef, scale
            )
            corrected_test = apply_rest_adapter(
                Xsub[test], rsub[test], psub[test], "roll_pitch", coef, scale
            )
            predictions["rest_corrected_interaction_lda"] = predict_lda(
                fit_lda(
                    orientation_interaction_features(
                        corrected_train, rsub[train], psub[train]
                    ),
                    ysub[train],
                ),
                orientation_interaction_features(
                    corrected_test, rsub[test], psub[test]
                ),
            )
            for model_name, mode in (
                ("continuous_roll", "roll"),
                ("continuous_roll_pitch", "roll_pitch"),
            ):
                coef, scale = fit_rest_adapter(
                    Xsub[train], ysub[train], rsub[train], psub[train], mode
                )
                train_corrected = apply_rest_adapter(
                    Xsub[train], rsub[train], psub[train], mode, coef, scale
                )
                test_corrected = apply_rest_adapter(
                    Xsub[test], rsub[test], psub[test], mode, coef, scale
                )
                predictions[model_name] = predict_lda(
                    fit_lda(train_corrected, ysub[train]), test_corrected
                )
            for model_name, pred in predictions.items():
                scores[model_name].append(float(balanced_accuracy_score(ysub[test], pred)))
                aggregate[model_name] += confusion_matrix(
                    ysub[test], pred, labels=labels
                )
        result = {"name": name, "classes": labels}
        for model_name, values in scores.items():
            result[model_name] = {
                "balanced_accuracy_mean": float(np.mean(values)),
                "balanced_accuracy_std": float(np.std(values)),
                "confusion_matrix": aggregate[model_name].tolist(),
            }
        output.append(result)
    return output


def projection_payload(X, y, roll, pitch, sessions):
    coef, scale = fit_rest_adapter(X, y, roll, pitch, "roll_pitch")
    corrected = apply_rest_adapter(X, roll, pitch, "roll_pitch", coef, scale)
    scaler = StandardScaler().fit(corrected)
    pca = PCA(n_components=2, random_state=0).fit(scaler.transform(corrected))
    coords = pca.transform(scaler.transform(corrected))
    classes = sorted(set(y))
    lda = LinearDiscriminantAnalysis(
        solver="lsqr", shrinkage="auto", priors=np.full(len(classes), 1.0 / len(classes))
    ).fit(coords, y)
    xmin, xmax = np.quantile(coords[:, 0], [0.005, 0.995])
    ymin, ymax = np.quantile(coords[:, 1], [0.005, 0.995])
    gx = np.linspace(xmin, xmax, 72)
    gy = np.linspace(ymin, ymax, 48)
    grid = np.asarray([[x, yy] for yy in gy for x in gx])
    region = lda.predict(grid)
    selected = []
    rng = np.random.default_rng(11)
    for label in classes:
        idx = np.flatnonzero(y == label)
        selected.extend(rng.choice(idx, size=min(170, len(idx)), replace=False).tolist())
    return {
        "explained_variance": [float(v) for v in pca.explained_variance_ratio_],
        "training_balanced_accuracy_2d": float(balanced_accuracy_score(y, lda.predict(coords))),
        "x_range": [float(xmin), float(xmax)],
        "y_range": [float(ymin), float(ymax)],
        "grid_shape": [len(gy), len(gx)],
        "regions": [classes.index(str(v)) for v in region],
        "points": [
            {
                "x": round(float(coords[i, 0]), 4),
                "y": round(float(coords[i, 1]), 4),
                "class": str(y[i]),
                "roll_deg": round(float(np.degrees(roll[i])), 2),
                "pitch_deg": round(float(np.degrees(pitch[i])), 2),
                "session": str(sessions[i]),
            }
            for i in selected
        ],
    }


def main():
    args = parse_args()
    X, y, groups, roll, pitch, sessions, recordings, events = build_dataset(args)
    classes, folds, model_summary = evaluate(
        X, y, groups, roll, pitch, args.folds
    )
    payload = {
        "protocol": {
            "first_block": "rest",
            "block_seconds": args.block_seconds,
            "trim_seconds": args.trim_seconds,
            "window_seconds": args.window_seconds,
            "hop_seconds": args.hop_seconds,
            "validation": "balanced class-wise assignment of complete 5-second blocks to held-out folds",
        },
        "recordings": recordings,
        "classes": classes,
        "n_windows": int(len(y)),
        "n_groups": int(len(set(groups))),
        "class_windows": {label: int(np.sum(y == label)) for label in classes},
        "orientation_degrees": {
            "roll_percentiles_5_50_95": [float(v) for v in np.degrees(np.quantile(roll, [0.05, 0.5, 0.95]))],
            "pitch_percentiles_5_50_95": [float(v) for v in np.degrees(np.quantile(pitch, [0.05, 0.5, 0.95]))],
        },
        "models": model_summary,
        "folds": folds,
        "pairwise": pairwise_results(X, y, groups, roll, pitch, args.folds),
        "gesture_pairwise": gesture_pair_results(
            X, y, groups, roll, pitch, args.folds
        ),
        "control_triplets": control_triplet_results(
            X, y, groups, roll, pitch, args.folds
        ),
        "projection": projection_payload(X, y, roll, pitch, sessions),
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    args.events_out.parent.mkdir(parents=True, exist_ok=True)
    with args.events_out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(events[0]))
        writer.writeheader()
        writer.writerows(events)
    print(json.dumps({
        "n_windows": payload["n_windows"],
        "n_groups": payload["n_groups"],
        "orientation_degrees": payload["orientation_degrees"],
        "models": payload["models"],
        "pairwise": payload["pairwise"],
        "gesture_pairwise": payload["gesture_pairwise"],
        "control_triplets": payload["control_triplets"],
    }, indent=2))


if __name__ == "__main__":
    main()
