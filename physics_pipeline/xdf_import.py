"""Build a synchronized physics-session dataset from a LabRecorder XDF file.

The importer never guesses an anatomical mapping.  It aligns versioned numeric
streams by LSL timestamp and carries their channel labels into the output.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import tempfile
from typing import Any

import numpy as np

from .contracts import PHYSICS_SESSION_SCHEMA
from .markers import parse_marker


@dataclass(frozen=True)
class XDFImportConfig:
    emg_stream_name: str = "MindRoveStream"
    marker_stream_name: str = "NML_TaskMarkers"
    exo_state_stream_name: str = "NMLHandExoStateV1"
    exo_command_stream_name: str = "NMLHandExoCommandV1"
    exo_event_stream_name: str = "NMLHandExoEventsV1"
    kinematics_stream_name: str = "NMLHandKinematicsV1"
    emg_channel_indices: tuple[int, ...] = tuple(range(1, 9))
    window_s: float = 0.250
    step_s: float = 0.050
    start_offset_s: float = 0.250
    end_offset_s: float = 0.150
    max_state_age_s: float = 0.100
    max_command_age_s: float = 0.250

    def validate(self) -> None:
        if not self.emg_channel_indices:
            raise ValueError("At least one EMG channel index is required")
        if min(self.emg_channel_indices) < 0:
            raise ValueError("EMG channel indices must be non-negative")
        if self.window_s <= 0 or self.step_s <= 0:
            raise ValueError("window_s and step_s must be greater than zero")
        if self.start_offset_s < 0 or self.end_offset_s < 0:
            raise ValueError("trial offsets cannot be negative")
        if self.max_state_age_s <= 0:
            raise ValueError("max_state_age_s must be greater than zero")
        if self.max_command_age_s <= 0:
            raise ValueError("max_command_age_s must be greater than zero")


@dataclass
class PhysicsSession:
    timestamps: np.ndarray
    emg_windows: np.ndarray
    emg_rms: np.ndarray
    labels: np.ndarray
    trials: np.ndarray
    marker_json: np.ndarray
    exo_state: np.ndarray
    exo_state_age_s: np.ndarray
    exo_state_valid: np.ndarray
    exo_state_channels: tuple[str, ...] = ()
    exo_command: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))
    exo_command_age_s: np.ndarray = field(default_factory=lambda: np.empty(0))
    exo_command_valid: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=bool))
    exo_command_channels: tuple[str, ...] = ()
    exo_event_timestamps: np.ndarray = field(default_factory=lambda: np.empty(0))
    exo_event_json: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=str))
    kinematics: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))
    kinematics_age_s: np.ndarray = field(default_factory=lambda: np.empty(0))
    kinematics_valid: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=bool))
    kinematics_channels: tuple[str, ...] = ()
    metadata: dict[str, Any] = field(default_factory=dict)

    def save(self, path: str | Path) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(
            prefix=f".{destination.name}.", suffix=".npz", dir=str(destination.parent)
        )
        os.close(fd)
        try:
            np.savez_compressed(
                temporary,
                metadata_json=np.asarray(json.dumps(self.metadata)),
                timestamps=np.asarray(self.timestamps, dtype=np.float64),
                emg_windows=np.asarray(self.emg_windows, dtype=np.float32),
                emg_rms=np.asarray(self.emg_rms, dtype=np.float64),
                labels=np.asarray(self.labels, dtype=str),
                trials=np.asarray(self.trials, dtype=str),
                marker_json=np.asarray(self.marker_json, dtype=str),
                exo_state=np.asarray(self.exo_state, dtype=np.float64),
                exo_state_age_s=np.asarray(self.exo_state_age_s, dtype=np.float64),
                exo_state_valid=np.asarray(self.exo_state_valid, dtype=bool),
                exo_state_channels=np.asarray(self.exo_state_channels, dtype=str),
                exo_command=np.asarray(self.exo_command, dtype=np.float64),
                exo_command_age_s=np.asarray(self.exo_command_age_s, dtype=np.float64),
                exo_command_valid=np.asarray(self.exo_command_valid, dtype=bool),
                exo_command_channels=np.asarray(self.exo_command_channels, dtype=str),
                exo_event_timestamps=np.asarray(self.exo_event_timestamps, dtype=np.float64),
                exo_event_json=np.asarray(self.exo_event_json, dtype=str),
                kinematics=np.asarray(self.kinematics, dtype=np.float64),
                kinematics_age_s=np.asarray(self.kinematics_age_s, dtype=np.float64),
                kinematics_valid=np.asarray(self.kinematics_valid, dtype=bool),
                kinematics_channels=np.asarray(self.kinematics_channels, dtype=str),
            )
            os.replace(temporary, destination)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    @classmethod
    def load(cls, path: str | Path) -> "PhysicsSession":
        with np.load(path, allow_pickle=False) as data:
            sample_count = len(data["timestamps"])
            return cls(
                timestamps=data["timestamps"],
                emg_windows=data["emg_windows"],
                emg_rms=data["emg_rms"],
                labels=data["labels"],
                trials=data["trials"],
                marker_json=data["marker_json"],
                exo_state=data["exo_state"],
                exo_state_age_s=data["exo_state_age_s"],
                exo_state_valid=data["exo_state_valid"],
                exo_state_channels=tuple(data["exo_state_channels"].astype(str)),
                exo_command=(
                    data["exo_command"]
                    if "exo_command" in data.files
                    else np.empty((sample_count, 0), dtype=np.float64)
                ),
                exo_command_age_s=(
                    data["exo_command_age_s"]
                    if "exo_command_age_s" in data.files
                    else np.full(sample_count, np.inf)
                ),
                exo_command_valid=(
                    data["exo_command_valid"]
                    if "exo_command_valid" in data.files
                    else np.zeros(sample_count, dtype=bool)
                ),
                exo_command_channels=(
                    tuple(data["exo_command_channels"].astype(str))
                    if "exo_command_channels" in data.files
                    else ()
                ),
                exo_event_timestamps=(
                    data["exo_event_timestamps"]
                    if "exo_event_timestamps" in data.files
                    else np.empty(0, dtype=np.float64)
                ),
                exo_event_json=(
                    data["exo_event_json"].astype(str)
                    if "exo_event_json" in data.files
                    else np.empty(0, dtype=str)
                ),
                kinematics=data["kinematics"],
                kinematics_age_s=data["kinematics_age_s"],
                kinematics_valid=data["kinematics_valid"],
                kinematics_channels=tuple(data["kinematics_channels"].astype(str)),
                metadata=json.loads(str(data["metadata_json"].item())),
            )


def _first(info: dict, key: str, default: Any = "") -> Any:
    value = info.get(key, [default])
    return value[0] if isinstance(value, list) and value else value


def _find_stream(streams: list[dict], name: str, *, required: bool) -> dict | None:
    matches = [stream for stream in streams if str(_first(stream.get("info", {}), "name", "")) == name]
    if len(matches) > 1:
        raise RuntimeError(f"XDF contains multiple streams named {name!r}")
    if not matches:
        if required:
            raise RuntimeError(f"XDF has no stream named {name!r}")
        return None
    return matches[0]


def _channel_labels(stream: dict) -> tuple[str, ...]:
    count = int(np.asarray(stream["time_series"]).shape[1])
    info = stream.get("info", {})
    try:
        desc = _first(info, "desc", {}) or {}
        channels = _first(desc, "channels", {}) or {}
        entries = channels.get("channel", []) or []
        labels = [str(_first(entry, "label", "")) for entry in entries]
    except (AttributeError, TypeError):
        labels = []
    return tuple(labels[index] if index < len(labels) and labels[index] else f"ch{index}" for index in range(count))


def _trial_intervals(marker_stream: dict, config: XDFImportConfig) -> list[dict[str, Any]]:
    intervals = []
    for timestamp, row in zip(marker_stream["time_stamps"], marker_stream["time_series"]):
        fields = parse_marker(str(row[0]))
        if fields.get("event") != "prompt_onset":
            continue
        try:
            duration = float(fields.get("duration_s", ""))
        except ValueError:
            continue
        start = float(timestamp) + config.start_offset_s
        stop = float(timestamp) + duration - config.end_offset_s
        if stop <= start:
            continue
        phase = fields.get("phase", "")
        label = "rest" if phase == "rest" else fields.get("gesture", "unknown")
        intervals.append(
            {
                "start": start,
                "stop": stop,
                "label": label,
                "trial": fields.get("trial", "000"),
                "fields": fields,
            }
        )
    if not intervals:
        raise RuntimeError("Marker stream contains no usable prompt_onset intervals")
    return intervals


def _nearest_age(source_ts: np.ndarray, targets: np.ndarray) -> np.ndarray:
    if source_ts.size == 0:
        return np.full(len(targets), np.inf)
    right = np.searchsorted(source_ts, targets, side="left")
    left = np.clip(right - 1, 0, len(source_ts) - 1)
    right = np.clip(right, 0, len(source_ts) - 1)
    return np.minimum(np.abs(targets - source_ts[left]), np.abs(source_ts[right] - targets))


def align_numeric_stream(
    stream: dict | None, targets: np.ndarray, max_age_s: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[str, ...]]:
    if stream is None:
        return (
            np.empty((len(targets), 0), dtype=np.float64),
            np.full(len(targets), np.inf),
            np.zeros(len(targets), dtype=bool),
            (),
        )
    timestamps = np.asarray(stream["time_stamps"], dtype=np.float64)
    values = np.asarray(stream["time_series"], dtype=np.float64)
    if values.ndim != 2 or len(values) != len(timestamps):
        raise RuntimeError("Numeric LSL stream has invalid sample/timestamp shape")
    if not len(timestamps):
        return (
            np.full((len(targets), values.shape[1]), np.nan, dtype=np.float64),
            np.full(len(targets), np.inf),
            np.zeros(len(targets), dtype=bool),
            _channel_labels(stream),
        )
    order = np.argsort(timestamps, kind="stable")
    timestamps = timestamps[order]
    values = values[order]
    unique, unique_index = np.unique(timestamps, return_index=True)
    values = values[unique_index]
    aligned = np.column_stack(
        [np.interp(targets, unique, values[:, index], left=np.nan, right=np.nan) for index in range(values.shape[1])]
    )
    age = _nearest_age(unique, targets)
    valid = (age <= max_age_s) & np.all(np.isfinite(aligned), axis=1)
    aligned[~valid] = np.nan
    return aligned, age, valid, _channel_labels(stream)


def align_step_stream(
    stream: dict | None, targets: np.ndarray, max_age_s: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[str, ...]]:
    """Align a held command snapshot using the latest sample at each target.

    NaN command fields are meaningful unknowns, so validity depends on sample
    presence and freshness rather than requiring every channel to be finite.
    """

    if stream is None:
        return (
            np.empty((len(targets), 0), dtype=np.float64),
            np.full(len(targets), np.inf),
            np.zeros(len(targets), dtype=bool),
            (),
        )
    timestamps = np.asarray(stream["time_stamps"], dtype=np.float64)
    values = np.asarray(stream["time_series"], dtype=np.float64)
    if values.ndim != 2 or len(values) != len(timestamps):
        raise RuntimeError("Command LSL stream has invalid sample/timestamp shape")
    if not len(timestamps):
        return (
            np.full((len(targets), values.shape[1]), np.nan, dtype=np.float64),
            np.full(len(targets), np.inf),
            np.zeros(len(targets), dtype=bool),
            _channel_labels(stream),
        )
    order = np.argsort(timestamps, kind="stable")
    timestamps = timestamps[order]
    values = values[order]
    # Preserve the final command snapshot when multiple updates share a stamp.
    reverse_unique, reverse_index = np.unique(timestamps[::-1], return_index=True)
    last_indices = len(timestamps) - 1 - reverse_index
    last_order = np.argsort(reverse_unique, kind="stable")
    unique = reverse_unique[last_order]
    values = values[last_indices[last_order]]
    indices = np.searchsorted(unique, targets, side="right") - 1
    present = indices >= 0
    safe_indices = np.clip(indices, 0, max(0, len(unique) - 1))
    aligned = values[safe_indices].copy()
    age = np.full(len(targets), np.inf, dtype=np.float64)
    age[present] = targets[present] - unique[safe_indices[present]]
    valid = present & (age >= 0) & (age <= max_age_s)
    aligned[~valid] = np.nan
    return aligned, age, valid, _channel_labels(stream)


def extract_event_stream(stream: dict | None) -> tuple[np.ndarray, np.ndarray]:
    if stream is None:
        return np.empty(0, dtype=np.float64), np.empty(0, dtype=str)
    timestamps = np.asarray(stream["time_stamps"], dtype=np.float64)
    values = np.asarray(stream["time_series"])
    if values.ndim != 2 or values.shape[1] < 1 or len(values) != len(timestamps):
        raise RuntimeError("Event LSL stream has invalid sample/timestamp shape")
    return timestamps, values[:, 0].astype(str)


def count_invalid_event_json(events: np.ndarray) -> int:
    invalid = 0
    for value in np.asarray(events, dtype=str):
        try:
            parsed = json.loads(str(value))
        except (TypeError, ValueError, json.JSONDecodeError):
            invalid += 1
            continue
        if not isinstance(parsed, dict):
            invalid += 1
    return invalid


def import_physics_xdf(
    path: str | Path, config: XDFImportConfig = XDFImportConfig()
) -> PhysicsSession:
    config.validate()
    try:
        import pyxdf
    except Exception as exc:
        raise RuntimeError(f"Physics-session import requires pyxdf: {exc}") from exc
    source = Path(path).resolve()
    streams, _header = pyxdf.load_xdf(str(source), verbose=False)
    emg_stream = _find_stream(streams, config.emg_stream_name, required=True)
    marker_stream = _find_stream(streams, config.marker_stream_name, required=True)
    exo_stream = _find_stream(streams, config.exo_state_stream_name, required=False)
    command_stream = _find_stream(
        streams, config.exo_command_stream_name, required=False
    )
    event_stream = _find_stream(streams, config.exo_event_stream_name, required=False)
    kinematics_stream = _find_stream(streams, config.kinematics_stream_name, required=False)

    raw = np.asarray(emg_stream["time_series"], dtype=np.float64)
    emg_ts = np.asarray(emg_stream["time_stamps"], dtype=np.float64)
    if raw.ndim != 2 or len(raw) != len(emg_ts):
        raise RuntimeError("EMG stream has invalid sample/timestamp shape")
    if max(config.emg_channel_indices) >= raw.shape[1]:
        raise RuntimeError(
            f"EMG index {max(config.emg_channel_indices)} is outside the {raw.shape[1]}-channel stream"
        )
    sample_rate = float(_first(emg_stream.get("info", {}), "nominal_srate", 0.0) or 0.0)
    if sample_rate <= 0:
        diffs = np.diff(emg_ts)
        valid_diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
        if not valid_diffs.size:
            raise RuntimeError("Cannot determine EMG sample rate")
        sample_rate = float(1.0 / np.median(valid_diffs))
    window_n = max(8, int(round(config.window_s * sample_rate)))
    step_n = max(1, int(round(config.step_s * sample_rate)))
    intervals = _trial_intervals(marker_stream, config)

    windows: list[np.ndarray] = []
    centers: list[float] = []
    labels: list[str] = []
    trials: list[str] = []
    marker_json: list[str] = []
    selected = raw[:, config.emg_channel_indices]
    for interval_index, interval in enumerate(intervals):
        start = int(np.searchsorted(emg_ts, interval["start"], side="left"))
        stop = int(np.searchsorted(emg_ts, interval["stop"], side="right"))
        for offset in range(start, stop - window_n + 1, step_n):
            end = offset + window_n
            window = selected[offset:end].T
            windows.append(window)
            centers.append(float((emg_ts[offset] + emg_ts[end - 1]) / 2.0))
            labels.append(str(interval["label"]))
            trials.append(f"{source.stem}:{interval_index:03d}:{interval['trial']}")
            marker_json.append(json.dumps(interval["fields"], sort_keys=True))
    if not windows:
        raise RuntimeError("No complete EMG windows fell inside marked intervals")

    timestamp_array = np.asarray(centers, dtype=np.float64)
    emg_windows = np.stack(windows).astype(np.float32)
    emg_rms = np.sqrt(np.mean(np.square(emg_windows.astype(np.float64)), axis=2))
    exo, exo_age, exo_valid, exo_channels = align_numeric_stream(
        exo_stream, timestamp_array, config.max_state_age_s
    )
    kine, kine_age, kine_valid, kine_channels = align_numeric_stream(
        kinematics_stream, timestamp_array, config.max_state_age_s
    )
    command, command_age, command_valid, command_channels = align_step_stream(
        command_stream, timestamp_array, config.max_command_age_s
    )
    event_timestamps, event_json = extract_event_stream(event_stream)
    invalid_event_count = count_invalid_event_json(event_json)
    metadata = {
        "schema": PHYSICS_SESSION_SCHEMA,
        "source_xdf": str(source),
        "emg_stream_name": config.emg_stream_name,
        "marker_stream_name": config.marker_stream_name,
        "exo_state_stream_name": config.exo_state_stream_name if exo_stream is not None else "",
        "exo_command_stream_name": (
            config.exo_command_stream_name if command_stream is not None else ""
        ),
        "exo_event_stream_name": (
            config.exo_event_stream_name if event_stream is not None else ""
        ),
        "kinematics_stream_name": config.kinematics_stream_name if kinematics_stream is not None else "",
        "emg_channel_indices": list(config.emg_channel_indices),
        "sample_rate_hz": sample_rate,
        "window_s": config.window_s,
        "step_s": config.step_s,
        "max_state_age_s": config.max_state_age_s,
        "max_command_age_s": config.max_command_age_s,
        "window_count": len(windows),
        "trial_count": len(set(trials)),
        "exo_valid_fraction": float(np.mean(exo_valid)),
        "command_valid_fraction": float(np.mean(command_valid)),
        "event_count": int(len(event_timestamps)),
        "invalid_event_json_count": int(invalid_event_count),
        "kinematics_valid_fraction": float(np.mean(kine_valid)),
    }
    return PhysicsSession(
        timestamps=timestamp_array,
        emg_windows=emg_windows,
        emg_rms=emg_rms,
        labels=np.asarray(labels, dtype=str),
        trials=np.asarray(trials, dtype=str),
        marker_json=np.asarray(marker_json, dtype=str),
        exo_state=exo,
        exo_state_age_s=exo_age,
        exo_state_valid=exo_valid,
        exo_state_channels=exo_channels,
        exo_command=command,
        exo_command_age_s=command_age,
        exo_command_valid=command_valid,
        exo_command_channels=command_channels,
        exo_event_timestamps=event_timestamps,
        exo_event_json=event_json,
        kinematics=kine,
        kinematics_age_s=kine_age,
        kinematics_valid=kine_valid,
        kinematics_channels=kine_channels,
        metadata=metadata,
    )


def _parse_indices(text: str) -> tuple[int, ...]:
    values = tuple(int(part.strip()) for part in text.split(",") if part.strip())
    if not values:
        raise argparse.ArgumentTypeError("Provide at least one comma-separated index")
    return values


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xdf", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--emg-stream", default="MindRoveStream")
    parser.add_argument("--marker-stream", default="NML_TaskMarkers")
    parser.add_argument("--exo-stream", default="NMLHandExoStateV1")
    parser.add_argument("--command-stream", default="NMLHandExoCommandV1")
    parser.add_argument("--event-stream", default="NMLHandExoEventsV1")
    parser.add_argument("--kinematics-stream", default="NMLHandKinematicsV1")
    parser.add_argument("--emg-indices", type=_parse_indices, default=tuple(range(1, 9)))
    parser.add_argument("--window-s", type=float, default=0.250)
    parser.add_argument("--step-s", type=float, default=0.050)
    parser.add_argument("--max-state-age-s", type=float, default=0.100)
    parser.add_argument("--max-command-age-s", type=float, default=0.250)
    args = parser.parse_args()
    config = XDFImportConfig(
        emg_stream_name=args.emg_stream,
        marker_stream_name=args.marker_stream,
        exo_state_stream_name=args.exo_stream,
        exo_command_stream_name=args.command_stream,
        exo_event_stream_name=args.event_stream,
        kinematics_stream_name=args.kinematics_stream,
        emg_channel_indices=args.emg_indices,
        window_s=args.window_s,
        step_s=args.step_s,
        max_state_age_s=args.max_state_age_s,
        max_command_age_s=args.max_command_age_s,
    )
    session = import_physics_xdf(args.xdf, config)
    session.save(args.output)
    print(json.dumps({"output": str(args.output.resolve()), **session.metadata}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
