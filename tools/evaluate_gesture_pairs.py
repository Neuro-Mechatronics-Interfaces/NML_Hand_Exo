"""Session-held-out rest-vs-gesture evaluation for the Jonathan EMG recordings."""
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
    x = x - x.mean(axis=1, keepdims=True)
    return np.log1p(np.sqrt(np.mean(x * x, axis=-1) + 1e-12))


def samples(path: Path, gesture: str, trim: int):
    with np.load(path, allow_pickle=True) as z:
        phase = labels_for_file(z)
        x = features(z["emg"])
    keep = np.isin(phase, ["rest", gesture])
    y = np.where(phase == gesture, "gesture", "rest")
    gesture_idx = np.flatnonzero(phase == gesture)
    if len(gesture_idx) > 2 * trim:
        keep[gesture_idx[:trim]] = False
        keep[gesture_idx[-trim:]] = False
    return x[keep], y[keep], orientation(path), path.parent.name


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_root", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--trim", type=int, default=2)
    args = parser.parse_args()

    files = [f for f in sorted(args.data_root.glob("raw/**/*.npz")) if "models" not in f.parts]
    with np.load(files[0], allow_pickle=True) as z:
        all_labels = labels_for_file(z)
        plan = json.loads(str(z["plan_json"].item()))
        gestures = [str(item["label"]) for item in plan if str(item["label"]) != "rest"]
    gestures = list(dict.fromkeys(gestures))
    output = {"trim_windows": args.trim, "sessions": len(files), "gestures": gestures, "results": []}

    for gesture in gestures:
        data = [samples(f, gesture, args.trim) for f in files]
        folds = []
        for held, test in enumerate(data):
            train = [s for i, s in enumerate(data) if i != held]
            Xte, yte, ori_te, session_te = test
            for mode in ("raw", "orientation_corrected"):
                if mode == "raw":
                    Xtr = np.concatenate([s[0] for s in train])
                    Xtest = Xte
                else:
                    rest_by_ori = {}
                    for ori in sorted({s[2] for s in train}):
                        rest = np.concatenate([s[0][s[1] == "rest"] for s in train if s[2] == ori])
                        spread = rest.std(axis=0)
                        rest_by_ori[ori] = (rest.mean(axis=0), spread + 0.25 * np.median(spread) + 1e-6)
                    Xtr = np.concatenate([(s[0] - rest_by_ori[s[2]][0]) / rest_by_ori[s[2]][1] for s in train])
                    mean, scale = rest_by_ori.get(ori_te, (Xtr.mean(axis=0), Xtr.std(axis=0) + 1e-6))
                    Xtest = (Xte - mean) / scale
                ytr = np.concatenate([s[1] for s in train])
                scaler = StandardScaler().fit(Xtr)
                clf = LinearDiscriminantAnalysis(solver="lsqr", shrinkage="auto", priors=[0.5, 0.5])
                clf.fit(scaler.transform(Xtr), ytr)
                pred = clf.predict(scaler.transform(Xtest))
                folds.append({
                    "session": session_te,
                    "orientation": ori_te,
                    "mode": mode,
                    "balanced_accuracy": float(balanced_accuracy_score(yte, pred)),
                    "f1_gesture": float(f1_score(yte, pred, pos_label="gesture")),
                    "confusion_matrix": confusion_matrix(yte, pred, labels=["rest", "gesture"]).tolist(),
                })
        summary = {"gesture": gesture, "folds": folds}
        for mode in ("raw", "orientation_corrected"):
            vals = [r["balanced_accuracy"] for r in folds if r["mode"] == mode]
            summary[mode] = {"mean_balanced_accuracy": float(np.mean(vals)), "std": float(np.std(vals))}
        output["results"].append(summary)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, indent=2), encoding="utf-8")
    compact = [{"gesture": r["gesture"], "raw": r["raw"], "orientation_corrected": r["orientation_corrected"]} for r in output["results"]]
    print(json.dumps(compact, indent=2))


if __name__ == "__main__":
    main()
