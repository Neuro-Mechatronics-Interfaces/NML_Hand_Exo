from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.metrics import balanced_accuracy_score, confusion_matrix, f1_score
from sklearn.preprocessing import StandardScaler


def orientation(path: Path) -> str:
    m = re.search(r"wrist-(neutral|pronated|supinated)", path.name)
    return m.group(1) if m else "unknown"


def labels_for_file(z):
    plan = json.loads(str(z["plan_json"].item()))
    t = np.asarray(z["timestamps"], dtype=float)
    t = t - t[0]
    ends = []
    labels = []
    cur = 0.0
    for item in plan:
        labels.append(str(item["label"]))
        cur += float(item["duration"])
        ends.append(cur)
    return np.asarray(labels, dtype=object)[np.clip(np.searchsorted(ends, t, side="right"), 0, len(labels) - 1)]


def features(emg):
    x = np.asarray(emg, dtype=float)
    if x.ndim == 4:
        x = x[..., 0]
    x = x - x.mean(axis=1, keepdims=True)
    # One feature vector per window/channel: remove spatial common mode,
    # then reduce the samples within each channel to log-RMS.
    return np.log1p(np.sqrt(np.mean(x * x, axis=-1) + 1e-12))


def make_samples(path: Path, trim: int):
    with np.load(path, allow_pickle=True) as z:
        y_phase = labels_for_file(z)
        x = features(z["emg"])
        keep = np.isin(y_phase, ["rest", "coordinated_grasp:open_close"])
        y = np.where(y_phase == "coordinated_grasp:open_close", "open_close", "rest")
        # Remove boundary windows where the plan transitions into/out of the gesture.
        gesture = np.flatnonzero(y_phase == "coordinated_grasp:open_close")
        if len(gesture) > 2 * trim:
            keep[gesture[:trim]] = False
            keep[gesture[-trim:]] = False
        return x[keep], y[keep], orientation(path), path.parent.name


def main():
    p = argparse.ArgumentParser()
    p.add_argument("data_root", type=Path)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--trim", type=int, default=2)
    args = p.parse_args()
    samples = [make_samples(f, args.trim) for f in sorted(args.data_root.glob("raw/**/*.npz")) if "models" not in f.parts]
    results = []
    for held in range(len(samples)):
        train = [s for i, s in enumerate(samples) if i != held]
        test = samples[held]
        Xtr = np.concatenate([s[0] for s in train])
        ytr = np.concatenate([s[1] for s in train])
        Xte, yte, ori_te, session_te = test
        for mode in ("raw", "orientation_corrected"):
            A, B = Xtr.copy(), Xte.copy()
            if mode == "orientation_corrected":
                rest_by_ori = {}
                for ori in sorted(set(s[2] for s in train)):
                    rest = np.concatenate([s[0][s[1] == "rest"] for s in train if s[2] == ori])
                    rest_by_ori[ori] = (rest.mean(axis=0), rest.std(axis=0) + 0.25 * np.median(rest.std(axis=0)))
                A = np.concatenate([(s[0] - rest_by_ori[s[2]][0]) / rest_by_ori[s[2]][1] for s in train])
                mean, scale = rest_by_ori.get(ori_te, (np.mean(Xtr[ytr == "rest"], axis=0), np.std(Xtr[ytr == "rest"], axis=0) + 1e-6))
                B = (B - mean) / scale
            scaler = StandardScaler().fit(A)
            # Equal priors prevent the larger rest class from making the
            # held-out classifier degenerate to an always-rest prediction.
            clf = LinearDiscriminantAnalysis(
                solver="lsqr", shrinkage="auto", priors=[0.5, 0.5]
            )
            clf.fit(scaler.transform(A), ytr)
            pred = clf.predict(scaler.transform(B))
            cm = confusion_matrix(yte, pred, labels=["rest", "open_close"]).tolist()
            results.append({
                "held_out_session": session_te,
                "held_out_orientation": ori_te,
                "mode": mode,
                "n_test": int(len(yte)),
                "balanced_accuracy": float(balanced_accuracy_score(yte, pred)),
                "f1_open_close": float(f1_score(yte, pred, pos_label="open_close")),
                "confusion_labels": ["rest", "open_close"],
                "confusion_matrix": cm,
            })
    summary = {"pair": ["rest", "open_close"], "trim_windows": args.trim, "folds": results}
    for mode in ("raw", "orientation_corrected"):
        vals = [r["balanced_accuracy"] for r in results if r["mode"] == mode]
        summary[mode] = {"mean_balanced_accuracy": float(np.mean(vals)), "std": float(np.std(vals)), "folds": len(vals)}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in summary.items() if k != "folds"}, indent=2))


if __name__ == "__main__":
    main()
