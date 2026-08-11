"""Visualize supervised LDA separability in an imported EMG intent session."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.preprocessing import StandardScaler

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from nml_hand_exo.decoding import IntentCaptureSession


def display_label(label: str) -> str:
    return str(label).removeprefix("attempt_")


def fit_lda(features: np.ndarray, labels: np.ndarray):
    scaler = StandardScaler()
    standardized = scaler.fit_transform(features)
    class_count = len(np.unique(labels))
    component_count = min(2, class_count - 1, standardized.shape[1])
    lda = LinearDiscriminantAnalysis(
        n_components=component_count,
        solver="eigen",
        shrinkage="auto",
    )
    projected = lda.fit_transform(standardized, labels)
    if projected.shape[1] == 1:
        projected = np.column_stack([projected[:, 0], np.zeros(projected.shape[0])])
    explained = np.zeros(2, dtype=np.float64)
    ratio = np.asarray(getattr(lda, "explained_variance_ratio_", []), dtype=np.float64)
    explained[: min(2, ratio.size)] = ratio[:2] * 100.0
    loadings = np.asarray(lda.scalings_[:, :2], dtype=np.float64)
    if loadings.shape[1] == 1:
        loadings = np.column_stack([loadings[:, 0], np.zeros(loadings.shape[0])])
    return projected, explained, loadings


def plot_space(
    projected: np.ndarray,
    labels: np.ndarray,
    explained: np.ndarray,
    destination: Path,
    title: str,
    max_points_per_class: int,
):
    rng = np.random.default_rng(42)
    classes = np.unique(labels)
    colors = plt.get_cmap("tab10", len(classes))
    origin = np.mean(projected, axis=0)
    fig, ax = plt.subplots(figsize=(11, 8))
    centroids = {}
    for index, label in enumerate(classes):
        members = np.flatnonzero(labels == label)
        if members.size > max_points_per_class:
            members = rng.choice(members, max_points_per_class, replace=False)
        points = projected[members]
        centroid = np.mean(projected[labels == label], axis=0)
        centroids[str(label)] = centroid.tolist()
        ax.scatter(
            points[:, 0], points[:, 1], s=13, alpha=0.24,
            color=colors(index), edgecolors="none", label=display_label(label),
        )
        ax.annotate(
            "",
            xy=centroid,
            xytext=origin,
            arrowprops={"arrowstyle": "->", "color": colors(index), "lw": 1.8},
        )
        ax.scatter(
            centroid[0], centroid[1], s=90, marker="X",
            color=colors(index), edgecolor="black", linewidth=0.6,
        )
    ax.axhline(0.0, color="0.75", lw=0.8)
    ax.axvline(0.0, color="0.75", lw=0.8)
    ax.set_xlabel(f"LD1 ({explained[0]:.1f}% discriminant variance)")
    ax.set_ylabel(f"LD2 ({explained[1]:.1f}% discriminant variance)")
    ax.set_title(title)
    ax.grid(alpha=0.16)
    ax.legend(loc="best", fontsize=8, ncols=2)
    fig.tight_layout()
    fig.savefig(destination, dpi=180)
    plt.close(fig)
    return centroids


def plot_loadings(loadings: np.ndarray, destination: Path):
    channels = [f"EMG {index + 1}" for index in range(loadings.shape[0])]
    positions = np.arange(len(channels))
    width = 0.38
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.bar(positions - width / 2, loadings[:, 0], width, label="LD1")
    ax.bar(positions + width / 2, loadings[:, 1], width, label="LD2")
    ax.axhline(0.0, color="black", lw=0.8)
    ax.set_xticks(positions, channels)
    ax.set_ylabel("Standardized LDA loading")
    ax.set_title("EMG Channel Contributions to LDA Axes")
    ax.legend()
    ax.grid(axis="y", alpha=0.2)
    fig.tight_layout()
    fig.savefig(destination, dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("analysis_outputs") / "intent_lda")
    parser.add_argument(
        "--pair",
        nargs=2,
        metavar=("NEGATIVE_INTENT", "POSITIVE_INTENT"),
        default=("attempt_hand_open", "attempt_hand_close"),
    )
    parser.add_argument("--max-points-per-class", type=int, default=350)
    args = parser.parse_args()

    session = IntentCaptureSession.load(args.session)
    features, labels, _, _, _ = session.arrays()
    if features.size == 0:
        raise RuntimeError("Session contains no feature vectors")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    projected, explained, loadings = fit_lda(features, labels)
    all_centroids = plot_space(
        projected,
        labels,
        explained,
        args.output_dir / "lda_all_intents.png",
        "LDA Intent Space: Rest and All Recorded Gestures",
        args.max_points_per_class,
    )
    plot_loadings(loadings, args.output_dir / "lda_channel_loadings.png")

    pair_labels = np.asarray(["rest", args.pair[0], args.pair[1]], dtype=object)
    pair_mask = np.isin(labels, pair_labels)
    if not all(np.any(labels == label) for label in pair_labels):
        missing = [str(label) for label in pair_labels if not np.any(labels == label)]
        raise RuntimeError(f"Selected pair is missing labels: {missing}")
    pair_projected, pair_explained, pair_loadings = fit_lda(features[pair_mask], labels[pair_mask])
    pair_centroids = plot_space(
        pair_projected,
        labels[pair_mask],
        pair_explained,
        args.output_dir / "lda_selected_pair.png",
        f"LDA Control Space: {display_label(args.pair[0])} vs {display_label(args.pair[1])}",
        args.max_points_per_class,
    )

    summary = {
        "session": str(args.session),
        "classes": sorted(str(value) for value in np.unique(labels)),
        "all_intents_explained_discriminant_variance_percent": explained.tolist(),
        "all_intents_centroids": all_centroids,
        "all_intents_channel_loadings": loadings.tolist(),
        "selected_pair": list(args.pair),
        "selected_pair_explained_discriminant_variance_percent": pair_explained.tolist(),
        "selected_pair_centroids": pair_centroids,
        "selected_pair_channel_loadings": pair_loadings.tolist(),
        "note": "Descriptive fit on the complete session; use grouped cross-validation for accuracy claims.",
    }
    (args.output_dir / "lda_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
