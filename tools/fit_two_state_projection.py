"""Fit rest vs. one gesture and export a 2-D PCA/LDA decision visualization."""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np
from sklearn.decomposition import PCA
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.metrics import balanced_accuracy_score
from sklearn.preprocessing import StandardScaler


def orientation(path: Path) -> str:
    m = re.search(r"wrist-(neutral|pronated|supinated)", path.name)
    return m.group(1) if m else "unknown"


def labels_for_file(z):
    plan = json.loads(str(z["plan_json"].item()))
    t = np.asarray(z["timestamps"], dtype=float)
    t -= t[0]
    ends, labels, elapsed = [], [], 0.0
    for item in plan:
        labels.append(str(item["label"]))
        elapsed += float(item["duration"])
        ends.append(elapsed)
    idx = np.clip(np.searchsorted(ends, t, side="right"), 0, len(labels) - 1)
    return np.asarray(labels, dtype=object)[idx]


def features(emg):
    x = np.asarray(emg, dtype=float)
    if x.ndim == 4:
        x = x[..., 0]
    x -= x.mean(axis=1, keepdims=True)
    return np.log1p(np.sqrt(np.mean(x * x, axis=-1) + 1e-12))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_root", type=Path)
    parser.add_argument("--gesture", default="isolated_digits:index_extend")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--max-points", type=int, default=400)
    args = parser.parse_args()

    records = []
    for path in sorted(args.data_root.glob("raw/**/*.npz")):
        if "models" in path.parts:
            continue
        with np.load(path, allow_pickle=True) as z:
            phase = labels_for_file(z)
            keep = np.isin(phase, ["rest", args.gesture])
            if not np.any(keep):
                continue
            records.append({
                "X": features(z["emg"])[keep],
                "y": np.where(phase[keep] == args.gesture, "gesture", "rest"),
                "orientation": orientation(path),
                "session": path.parent.name,
            })
    if not records:
        raise RuntimeError("No matching recordings found")

    # Orientation-conditioned rest normalization, identical to the evaluation
    # protocol used for the pairwise results.
    rest_by_ori = {}
    for ori in sorted({r["orientation"] for r in records}):
        rest = np.concatenate([r["X"][r["y"] == "rest"] for r in records if r["orientation"] == ori])
        spread = rest.std(axis=0)
        rest_by_ori[ori] = (rest.mean(axis=0), spread + 0.25 * np.median(spread) + 1e-6)
    X = np.concatenate([(r["X"] - rest_by_ori[r["orientation"]][0]) / rest_by_ori[r["orientation"]][1] for r in records])
    y = np.concatenate([r["y"] for r in records])
    orientations = np.concatenate([np.full(len(r["y"]), r["orientation"], dtype=object) for r in records])
    sessions = np.concatenate([np.full(len(r["y"]), r["session"], dtype=object) for r in records])

    scaler = StandardScaler().fit(X)
    Xz = scaler.transform(X)
    pca = PCA(n_components=2, random_state=0).fit(Xz)
    coords = pca.transform(Xz)
    # The displayed boundary is the LDA boundary in the same two reprojected
    # axes. Full-feature held-out performance remains the authoritative metric.
    lda2 = LinearDiscriminantAnalysis(solver="lsqr", shrinkage="auto", priors=[0.5, 0.5]).fit(coords, y)
    pred = lda2.predict(coords)
    coef = lda2.coef_[0]
    intercept = float(lda2.intercept_[0])
    xlim = [float(coords[:, 0].min()), float(coords[:, 0].max())]
    if abs(coef[1]) > 1e-12:
        boundary = [{"x": x, "y": float(-(coef[0] * x + intercept) / coef[1])} for x in xlim]
    else:
        boundary = [{"x": float(-intercept / coef[0]), "y": float(coords[:, 1].min())}, {"x": float(-intercept / coef[0]), "y": float(coords[:, 1].max())}]

    selected = []
    for label in ("rest", "gesture"):
        for ori in sorted(set(orientations)):
            idx = np.flatnonzero((y == label) & (orientations == ori))
            selected.extend(idx[np.linspace(0, len(idx) - 1, min(args.max_points, len(idx)), dtype=int)].tolist())
    points = [{"x": float(coords[i, 0]), "y": float(coords[i, 1]), "label": str(y[i]), "orientation": str(orientations[i]), "session": str(sessions[i])} for i in selected]
    payload = {
        "gesture": args.gesture,
        "projection": "PCA of standardized, orientation-corrected log-RMS features",
        "axis_explained_variance": [float(v) for v in pca.explained_variance_ratio_],
        "training_balanced_accuracy_in_2d": float(balanced_accuracy_score(y, pred)),
        "points": points,
        "boundary": boundary,
        "xlim": xlim,
        "ylim": [float(coords[:, 1].min()), float(coords[:, 1].max())],
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload), encoding="utf-8")
    print(json.dumps({k: v for k, v in payload.items() if k not in {"points", "boundary"}}, indent=2))


if __name__ == "__main__":
    main()
