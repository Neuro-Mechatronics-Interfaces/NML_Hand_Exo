"""Evaluate EMG-only and state-conditioned intent baselines by whole trial."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .evaluation import evaluate_intent_grouped
from .xdf_import import PhysicsSession


def evaluate_session(
    session: PhysicsSession,
    *,
    state_contains: tuple[str, ...] = (
        "relative_angle_deg",
        "velocity_rpm",
        "present_current_mA",
    ),
    folds: int = 5,
) -> dict:
    emg_report = evaluate_intent_grouped(
        session.emg_rms,
        session.labels,
        session.trials,
        folds=folds,
    )
    report = {
        "schema": "nml.physics_baseline_comparison.v1",
        "source": session.metadata.get("source_xdf", ""),
        "emg_only": emg_report,
        "emg_plus_state": None,
        "state_channels": [],
        "excluded_state_rows": int(len(session.timestamps)),
    }
    if session.exo_state.shape[1] == 0:
        return report
    selected = [
        index
        for index, label in enumerate(session.exo_state_channels)
        if any(token in label for token in state_contains)
    ]
    if not selected:
        return report
    valid = session.exo_state_valid & np.all(
        np.isfinite(session.exo_state[:, selected]), axis=1
    )
    if np.count_nonzero(valid) < 4 or len(np.unique(session.trials[valid])) < 2:
        return report
    report["state_channels"] = [
        session.exo_state_channels[index] for index in selected
    ]
    report["excluded_state_rows"] = int(np.count_nonzero(~valid))
    report["emg_plus_state"] = evaluate_intent_grouped(
        session.emg_rms[valid],
        session.labels[valid],
        session.trials[valid],
        session.exo_state[valid][:, selected],
        folds=folds,
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--folds", type=int, default=5)
    parser.add_argument("--json", dest="json_path", type=Path)
    parser.add_argument(
        "--state-contains",
        action="append",
        default=[],
        help="Keep state channels containing this text; repeat as needed",
    )
    args = parser.parse_args()
    if args.folds < 2:
        parser.error("--folds must be at least 2")
    session = PhysicsSession.load(args.session)
    tokens = tuple(args.state_contains) or (
        "relative_angle_deg",
        "velocity_rpm",
        "present_current_mA",
    )
    report = evaluate_session(session, state_contains=tokens, folds=args.folds)
    rendered = json.dumps(report, indent=2)
    print(rendered)
    if args.json_path:
        args.json_path.parent.mkdir(parents=True, exist_ok=True)
        args.json_path.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
