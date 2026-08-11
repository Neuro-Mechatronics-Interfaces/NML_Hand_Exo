"""Summarize the labeled coordinated_grasp:open_close trajectory.

The source recordings label this as one trajectory, not separate open and close
classes. This tool preserves that fact and computes descriptive quantities that
can be used to choose (or reject) a data-driven split.
"""
from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np


def orientation(path: Path) -> str:
    m = re.search(r"wrist-(neutral|pronated|supinated)", path.name)
    return m.group(1) if m else "unknown"


def phase_labels(z):
    plan = json.loads(str(z["plan_json"].item()))
    t = np.asarray(z["timestamps"], dtype=float)
    t = t - t[0]
    ends, labels = [], []
    elapsed = 0.0
    for item in plan:
        labels.append(str(item["label"]))
        elapsed += float(item["duration"])
        ends.append(elapsed)
    idx = np.clip(np.searchsorted(ends, t, side="right"), 0, len(labels) - 1)
    return np.asarray(labels, dtype=object)[idx]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("data_root", type=Path)
    parser.add_argument("--out-csv", type=Path, required=True)
    parser.add_argument("--out-json", type=Path, required=True)
    args = parser.parse_args()

    rows = []
    summaries = []
    files = sorted(args.data_root.glob("raw/**/*.npz"))
    for path in files:
        if "models" in path.parts:
            continue
        with np.load(path, allow_pickle=True) as z:
            labels = phase_labels(z)
            idx = np.flatnonzero(labels == "coordinated_grasp:open_close")
            if len(idx) < 3:
                continue
            angles = np.asarray(z["angles"], dtype=float)
            t = np.asarray(z["timestamps"], dtype=float)
            t = t - t[0]
            a = angles[idx]
            # Use the phase-start posture as the reference. This is descriptive;
            # it does not assert that the trajectory is an opening or closing.
            displacement = np.linalg.norm(a - a[0], axis=1)
            speed = np.linalg.norm(np.gradient(a, axis=0), axis=1)
            peak_disp = int(np.argmax(displacement))
            peak_speed = int(np.argmax(speed))
            session = path.parent.name
            ori = orientation(path)
            for j, window in enumerate(idx):
                rows.append(
                    {
                        "session": session,
                        "orientation": ori,
                        "window_index": int(window),
                        "phase_elapsed_s": float(t[window] - t[idx[0]]),
                        "phase_fraction": float(j / max(len(idx) - 1, 1)),
                        "angle_displacement_norm_deg": float(displacement[j]),
                        "angle_speed_norm_deg_per_window": float(speed[j]),
                    }
                )
            summaries.append(
                {
                    "session": session,
                    "orientation": ori,
                    "window_count": int(len(idx)),
                    "phase_start_s": float(t[idx[0]]),
                    "phase_end_s": float(t[idx[-1]]),
                    "peak_displacement_fraction": float(peak_disp / max(len(idx) - 1, 1)),
                    "peak_speed_fraction": float(peak_speed / max(len(idx) - 1, 1)),
                    "peak_displacement_deg": float(displacement[peak_disp]),
                    "peak_speed_deg_per_window": float(speed[peak_speed]),
                }
            )

    args.out_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    payload = {
        "description": "Descriptive trajectory diagnostics; open_close is not relabeled into open and close.",
        "sessions": summaries,
        "n_sessions": len(summaries),
        "peak_displacement_fraction_median": float(np.median([s["peak_displacement_fraction"] for s in summaries])),
        "peak_displacement_fraction_iqr": [
            float(np.quantile([s["peak_displacement_fraction"] for s in summaries], 0.25)),
            float(np.quantile([s["peak_displacement_fraction"] for s in summaries], 0.75)),
        ],
    }
    args.out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
