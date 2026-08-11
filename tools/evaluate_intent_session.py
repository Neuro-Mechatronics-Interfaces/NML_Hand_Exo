"""Fit the guarded intent decoder and score a held-out imported XDF session."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from nml_hand_exo.decoding import IntentCaptureSession, IntentDecoderPipeline, OrientationSample


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("training_session", type=Path)
    parser.add_argument("holdout_session", type=Path)
    parser.add_argument("--open-label", default="attempt_hand_open")
    parser.add_argument("--close-label", default="attempt_hand_close")
    parser.add_argument("--without-orientation", action="store_true")
    args = parser.parse_args()

    training = IntentCaptureSession.load(args.training_session)
    holdout = IntentCaptureSession.load(args.holdout_session)
    train_x, train_y, _, train_roll, train_pitch = training.arrays()
    test_x, test_y, _, test_roll, test_pitch = holdout.arrays()
    train_keep = np.isin(train_y, ["rest", args.open_label, args.close_label])
    test_keep = np.isin(test_y, ["rest", args.open_label, args.close_label])
    require_orientation = not args.without_orientation
    pipeline = IntentDecoderPipeline(
        open_label=args.open_label,
        close_label=args.close_label,
        require_orientation=require_orientation,
    ).fit(
        train_x[train_keep], train_y[train_keep],
        train_roll[train_keep], train_pitch[train_keep],
    )

    predictions = []
    signed = []
    rejected = []
    for feature, roll, pitch in zip(test_x[test_keep], test_roll[test_keep], test_pitch[test_keep]):
        orientation = OrientationSample(
            None if not np.isfinite(roll) else float(roll),
            None if not np.isfinite(pitch) else float(pitch),
        )
        decision = pipeline.predict(feature, orientation)
        predictions.append("rejected" if decision.rejected else decision.state)
        signed.append(float(decision.signed_intent))
        rejected.append(bool(decision.rejected))

    truth = test_y[test_keep]
    predictions = np.asarray(predictions, dtype=object)
    signed = np.asarray(signed, dtype=np.float64)
    rejected = np.asarray(rejected, dtype=bool)
    per_class = {}
    for label in sorted(set(truth.tolist())):
        mask = truth == label
        values, counts = np.unique(predictions[mask], return_counts=True)
        per_class[label] = {
            "windows": int(np.sum(mask)),
            "accuracy": float(np.mean(predictions[mask] == label)),
            "rejection_rate": float(np.mean(rejected[mask])),
            "mean_signed_intent": float(np.mean(signed[mask])),
            "predictions": {str(value): int(count) for value, count in zip(values, counts)},
        }
    print(json.dumps({
        "training_session": str(args.training_session),
        "holdout_session": str(args.holdout_session),
        "require_orientation": require_orientation,
        "accuracy": float(np.mean(predictions == truth)),
        "rejection_rate": float(np.mean(rejected)),
        "per_class": per_class,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
