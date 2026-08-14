"""Build one continuous open/close playback recording from marked XDF files.

The output is a generic recording-NPZ format for a compatible MindRove LSL
player; it is intentionally not an intent-session NPZ. Original signal samples are
copied without filtering or feature extraction.  Only their order and playback
timestamps change.
"""

from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Phase:
    phase: str
    gesture: str
    trial: str
    onset: float
    duration_s: float


def _marker_fields(value: str) -> dict[str, str]:
    parts = str(value).split("|")
    fields = {"event": parts[0]}
    for part in parts[1:]:
        if "=" in part:
            key, item = part.split("=", 1)
            fields[key] = item
    return fields


def _slug(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", value.strip().lower()).strip("_")


def _load_marked_xdf(path: Path) -> tuple[dict[str, np.ndarray], list[tuple[Phase, Phase]], float]:
    try:
        import pyxdf
    except Exception as exc:
        raise RuntimeError(f"XDF playback building requires pyxdf: {exc}") from exc

    streams, _ = pyxdf.load_xdf(str(path), verbose=False)
    marker_stream = None
    signal_stream = None
    for stream in streams:
        values = np.asarray(stream.get("time_series"))
        info = stream.get("info", {})
        name = str((info.get("name") or [""])[0]).lower()
        stream_type = str((info.get("type") or [""])[0]).lower()
        if values.ndim == 2 and values.shape[1] == 1 and (
            "marker" in name or "marker" in stream_type
        ):
            marker_stream = stream
        elif values.ndim == 2 and values.shape[1] >= 15:
            signal_stream = stream
    if marker_stream is None:
        raise RuntimeError(f"{path.name}: no marker stream")
    if signal_stream is None:
        raise RuntimeError(f"{path.name}: no 15+ channel MindRove stream")

    raw = np.asarray(signal_stream["time_series"], dtype=np.float32)
    signal_ts = np.asarray(signal_stream["time_stamps"], dtype=np.float64)
    if raw.shape[1] < 15 or signal_ts.size != raw.shape[0]:
        raise RuntimeError(f"{path.name}: invalid MindRove signal shape")

    diffs = np.diff(signal_ts)
    valid_diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
    nominal = float((signal_stream["info"].get("nominal_srate") or ["500"])[0])
    rate_hz = float(1.0 / np.median(valid_diffs)) if valid_diffs.size else nominal

    phases: list[Phase] = []
    for timestamp, row in zip(marker_stream["time_stamps"], marker_stream["time_series"]):
        fields = _marker_fields(row[0])
        if fields.get("event") != "prompt_onset":
            continue
        try:
            duration_s = float(fields["duration_s"])
        except (KeyError, TypeError, ValueError):
            continue
        phases.append(
            Phase(
                phase=fields.get("phase", ""),
                gesture=fields.get("gesture", ""),
                trial=fields.get("trial", ""),
                onset=float(timestamp),
                duration_s=duration_s,
            )
        )

    gestures = [phase for phase in phases if phase.phase == "gesture"]
    rests = [phase for phase in phases if phase.phase == "rest"]
    pairs: list[tuple[Phase, Phase]] = []
    for gesture in gestures:
        candidates = [rest for rest in rests if rest.onset < gesture.onset]
        if not candidates:
            raise RuntimeError(
                f"{path.name}: gesture trial {gesture.trial} has no preceding marked rest"
            )
        pairs.append((max(candidates, key=lambda item: item.onset), gesture))
    if not pairs:
        raise RuntimeError(f"{path.name}: no marked gesture trials")

    return {
        "timestamps": signal_ts,
        "emg": raw[:, 1:9],
        "accel": raw[:, 9:12],
        "gyro": raw[:, 12:15],
    }, pairs, rate_hz


def _extract_clip(data: dict[str, np.ndarray], phase: Phase) -> dict[str, np.ndarray]:
    timestamps = data["timestamps"]
    indices = np.flatnonzero(
        (timestamps >= phase.onset) & (timestamps < phase.onset + phase.duration_s)
    )
    if indices.size == 0:
        raise RuntimeError(
            f"No signal samples for {phase.phase} trial {phase.trial} at {phase.onset:.3f}"
        )
    return {key: values[indices] for key, values in data.items() if key != "timestamps"}


def build_playlist(source_paths: list[Path], output: Path) -> dict:
    if len(source_paths) < 2:
        raise ValueError("Provide at least two marked XDF recordings")

    sources = []
    for path in source_paths:
        data, pairs, rate_hz = _load_marked_xdf(path)
        sources.append((path, data, pairs, rate_hz))

    clips: dict[str, list[np.ndarray]] = {"emg": [], "accel": [], "gyro": []}
    sequence = []
    sample_cursor = 0
    max_trials = max(len(item[2]) for item in sources)
    for trial_index in range(max_trials):
        for path, data, pairs, _rate_hz in sources:
            if trial_index >= len(pairs):
                continue
            rest, gesture = pairs[trial_index]
            for phase in (rest, gesture):
                clip = _extract_clip(data, phase)
                sample_count = int(clip["emg"].shape[0])
                for key in clips:
                    clips[key].append(np.asarray(clip[key], dtype=np.float32))
                sequence.append(
                    {
                        "source": path.name,
                        "phase": phase.phase,
                        "gesture": phase.gesture,
                        "source_trial": phase.trial,
                        "start_sample": sample_cursor,
                        "sample_count": sample_count,
                    }
                )
                sample_cursor += sample_count

    combined = {key: np.concatenate(parts, axis=0) for key, parts in clips.items()}
    rate_hz = float(np.median([item[3] for item in sources]))
    timestamps = np.arange(sample_cursor, dtype=np.float64) / rate_hz
    metadata = {
        "format": "mindrove_recording_npz_v1",
        "source_format": "alternating_marked_xdf_playlist",
        "sampling_rate_hz": rate_hz,
        "sources": [str(item[0].resolve()) for item in sources],
        "sequence": sequence,
    }

    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        timestamps=timestamps,
        emg=combined["emg"],
        accel=combined["accel"],
        gyro=combined["gyro"],
        metadata_json=np.asarray(json.dumps(metadata)),
    )
    manifest_path = output.with_suffix(".json")
    manifest_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return {
        "output": str(output.resolve()),
        "manifest": str(manifest_path.resolve()),
        "samples": sample_cursor,
        "duration_s": float(sample_cursor / rate_hz),
        "rate_hz": rate_hz,
        "segments": len(sequence),
        "gesture_segments": sum(item["phase"] == "gesture" for item in sequence),
        "sequence_labels": [
            _slug(item["gesture"] or item["phase"])
            for item in sequence
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Output recording .npz")
    parser.add_argument("xdf", nargs="+", type=Path, help="Marked XDF files in desired alternating order")
    args = parser.parse_args()
    missing = [str(path) for path in args.xdf if not path.is_file()]
    if missing:
        parser.error(f"XDF file(s) not found: {', '.join(missing)}")
    summary = build_playlist(args.xdf, args.output)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
