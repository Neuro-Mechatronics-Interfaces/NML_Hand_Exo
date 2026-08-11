from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np
from sklearn.decomposition import PCA
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.metrics import silhouette_score
from sklearn.preprocessing import StandardScaler


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("data_root", type=Path)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--events-out", type=Path, required=True)
    return p.parse_args()


def orientation_from_name(path: Path) -> str:
    match = re.search(r"wrist-(neutral|pronated|supinated)", path.name)
    return match.group(1) if match else "unknown"


def plan_labels(plan_json: str) -> tuple[np.ndarray, np.ndarray]:
    plan = json.loads(plan_json)
    labels = []
    ends = []
    t = 0.0
    for item in plan:
        duration = float(item["duration"])
        label = str(item["label"])
        labels.append(label)
        t += duration
        ends.append(t)
    return np.asarray(labels, dtype=object), np.asarray(ends, dtype=float)


def label_windows(timestamps: np.ndarray, labels: np.ndarray, ends: np.ndarray) -> np.ndarray:
    # Timestamps in these files are session-relative seconds. If a future file
    # stores an absolute epoch, subtract its first sample before this call.
    t = np.asarray(timestamps, dtype=float)
    if t.size and t[0] > 1e3:
        t = t - t[0]
    idx = np.searchsorted(ends, t, side="right")
    idx = np.clip(idx, 0, len(labels) - 1)
    return labels[idx]


def extract_features(emg: np.ndarray) -> np.ndarray:
    x = np.asarray(emg, dtype=float)
    if x.ndim == 4:
        x = x[..., 0]
    # Match the decoder's first-pass feature intent: per-channel RMS after
    # common-mode removal, with log compression for amplitude stability.
    x = x - np.mean(x, axis=1, keepdims=True)
    rms = np.sqrt(np.mean(np.square(x), axis=1) + 1e-12)
    return np.log1p(rms)


def main():
    args = parse_args()
    records = []
    events = []
    for path in sorted(args.data_root.glob("raw/**/*.npz")):
        if "models" in path.parts:
            continue
        with np.load(path, allow_pickle=True) as z:
            labels, ends = plan_labels(str(z["plan_json"].item()))
            y = label_windows(z["timestamps"], labels, ends)
            features = extract_features(z["emg"])
            orientation = str(z["wrist_orientation"].item())
            session = str(z["session_id"].item())
            records.append({
                "path": str(path),
                "orientation": orientation,
                "session": session,
                "labels": y,
                "features": features,
                "window_count": int(features.shape[0]),
                "duration_s": float(z["timestamps"][-1] - z["timestamps"][0]),
            })
            relative_t = np.asarray(z["timestamps"], dtype=float) - float(z["timestamps"][0])
            starts = np.r_[0.0, ends[:-1]]
            for phase_index, (label, start_s, end_s) in enumerate(zip(labels, starts, ends), start=1):
                in_phase = (relative_t >= start_s) & (relative_t < end_s)
                idx = np.flatnonzero(in_phase)
                events.append({
                    "session": session,
                    "orientation": orientation,
                    "phase_index": phase_index,
                    "phase_label": str(label),
                    "class_label": str(label).split(":", 1)[-1],
                    "phase_type": "rest" if str(label) == "rest" else "gesture",
                    "start_time_s": float(start_s),
                    "end_time_s": float(end_s),
                    "duration_s": float(end_s - start_s),
                    "start_timestamp": float(z["timestamps"][idx[0]]) if len(idx) else "",
                    "end_timestamp": float(z["timestamps"][idx[-1]]) if len(idx) else "",
                    "start_window": int(idx[0]) if len(idx) else "",
                    "end_window": int(idx[-1]) if len(idx) else "",
                    "window_count": int(len(idx)),
                })

    if not records:
        raise RuntimeError(f"No recordings found below {args.data_root / 'raw'}")

    X = np.concatenate([r["features"] for r in records], axis=0)
    y = np.concatenate([r["labels"] for r in records], axis=0)
    orientations = np.concatenate([
        np.full(r["window_count"], r["orientation"], dtype=object) for r in records
    ])
    sessions = np.concatenate([
        np.full(r["window_count"], r["session"], dtype=object) for r in records
    ])
    # Keep every phase for the plot; rest is a useful reference class.
    keep = np.ones(len(y), dtype=bool)
    Xz = StandardScaler().fit_transform(X)
    coords = PCA(n_components=2, random_state=0).fit_transform(Xz)
    gesture_names = np.asarray([str(v).split(":", 1)[-1] for v in y], dtype=object)
    gesture_only = gesture_names != "rest"
    silhouette = float(silhouette_score(Xz[gesture_only], gesture_names[gesture_only]))
    lda = LinearDiscriminantAnalysis(solver="svd", n_components=2)
    lda_coords = lda.fit_transform(Xz, gesture_names)
    lda_gesture_silhouette = float(
        silhouette_score(lda_coords[gesture_only], gesture_names[gesture_only])
    )

    selected = []
    group_keys = [(str(gesture_names[i]), str(orientations[i])) for i in np.flatnonzero(keep)]
    for key in sorted(set(group_keys)):
        candidates = np.asarray([
            i for i in np.flatnonzero(keep)
            if (str(gesture_names[i]), str(orientations[i])) == key
        ], dtype=int)
        selected.extend(np.linspace(0, len(candidates) - 1, min(120, len(candidates)), dtype=int))
        selected[-min(120, len(candidates)):] = candidates[selected[-min(120, len(candidates)):]]
    points = []
    for i in selected:
        points.append({
            "pc1": float(coords[i, 0]),
            "pc2": float(coords[i, 1]),
            "lda1": float(lda_coords[i, 0]),
            "lda2": float(lda_coords[i, 1]),
            "gesture": str(gesture_names[i]),
            "phase": str(y[i]),
            "orientation": str(orientations[i]),
            "session": str(sessions[i]),
        })
    summary = {
        "recordings": [
            {"session": r["session"], "orientation": r["orientation"],
             "windows": r["window_count"], "duration_s": r["duration_s"]}
            for r in records
        ],
        "gesture_counts": {
            name: int(np.sum(gesture_names == name))
            for name in sorted(set(gesture_names[keep]))
        },
        "silhouette_gestures_only": silhouette,
        "lda_silhouette_gestures_only": lda_gesture_silhouette,
        "points": points,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary), encoding="utf-8")
    args.events_out.parent.mkdir(parents=True, exist_ok=True)
    fields = list(events[0].keys())
    with args.events_out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(events)
    print(json.dumps({k: v for k, v in summary.items() if k != "points"}, indent=2))


if __name__ == "__main__":
    main()
