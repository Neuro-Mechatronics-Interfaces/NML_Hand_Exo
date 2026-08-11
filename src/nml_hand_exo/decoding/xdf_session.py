"""Import event-marked MindRove XDF recordings into decoder sessions."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Callable, Iterable

import numpy as np

from .features import FeatureConfig, extract_emg_features
from .orientation import orientation_from_accel
from .preprocessing import PreprocessConfig, preprocess_emg
from .session import IntentCaptureSession


LABEL_ALIASES = {
    "handclose": "attempt_hand_close",
    "handopen": "attempt_hand_open",
    "indexextension": "attempt_index_extend",
    "indexflexion": "attempt_index_flex",
    "thumbindexpinchclose": "attempt_thumb_index_pinch_close",
    "thumbindexpinchopen": "attempt_thumb_index_pinch_open",
    "wristextension": "attempt_wrist_extend",
    "wristflexion": "attempt_wrist_flex",
    "rest": "rest",
}


def canonical_intent_label(value: str) -> str:
    text = str(value).strip().lower()
    compact = re.sub(r"[^a-z0-9]+", "", text)
    if compact in LABEL_ALIASES:
        return LABEL_ALIASES[compact]
    normalized = re.sub(r"[^a-z0-9]+", "_", text).strip("_")
    return normalized if normalized.startswith("attempt_") else f"attempt_{normalized}"


def parse_task_marker(value: str) -> dict[str, str]:
    parts = str(value).split("|")
    fields = {"event": parts[0]}
    for part in parts[1:]:
        if "=" in part:
            key, item = part.split("=", 1)
            fields[key] = item
    return fields


def _find_xdf_streams(streams):
    marker_stream = None
    signal_stream = None
    for stream in streams:
        info = stream["info"]
        name = (info.get("name") or [""])[0].lower()
        stream_type = (info.get("type") or [""])[0].lower()
        values = np.asarray(stream.get("time_series"))
        if "marker" in name or "marker" in stream_type:
            marker_stream = stream
        elif values.ndim == 2 and values.shape[1] >= 15:
            signal_stream = stream
    if marker_stream is None:
        raise RuntimeError("XDF has no task-marker stream")
    if signal_stream is None:
        raise RuntimeError("XDF has no 15+ channel MindRove signal stream")
    return marker_stream, signal_stream


def import_xdf_file(
    session: IntentCaptureSession,
    path: str | Path,
    *,
    window_s: float = 0.25,
    step_s: float = 0.12,
    start_offset_s: float = 0.5,
    end_offset_s: float = 0.25,
) -> tuple[int, int]:
    try:
        import pyxdf
    except Exception as exc:
        raise RuntimeError(f"XDF import requires pyxdf: {exc}") from exc

    source = Path(path)
    streams, _ = pyxdf.load_xdf(str(source))
    marker_stream, signal_stream = _find_xdf_streams(streams)
    values = np.asarray(signal_stream["time_series"], dtype=np.float64)
    timestamps = np.asarray(signal_stream["time_stamps"], dtype=np.float64)
    sample_rate = float((signal_stream["info"].get("nominal_srate") or ["500"])[0])
    window_n = max(8, int(round(window_s * sample_rate)))
    step_n = max(1, int(round(step_s * sample_rate)))
    trial_count = 0
    window_count = 0
    phase_counts: dict[str, int] = {}

    for marker_ts, row in zip(marker_stream["time_stamps"], marker_stream["time_series"]):
        marker = parse_task_marker(row[0])
        if marker.get("event") != "prompt_onset":
            continue
        phase = marker.get("phase", "")
        if phase not in {"rest", "gesture"}:
            continue
        raw_label = marker.get("gesture", "rest" if phase == "rest" else source.stem)
        label = "rest" if phase == "rest" else canonical_intent_label(raw_label)
        duration = float(marker.get("duration_s", "2.0" if phase == "rest" else "5.0"))
        start_ts = float(marker_ts) + start_offset_s
        stop_ts = float(marker_ts) + max(start_offset_s, duration - end_offset_s)
        start = int(np.searchsorted(timestamps, start_ts, side="left"))
        stop = int(np.searchsorted(timestamps, stop_ts, side="right"))
        if stop - start < window_n:
            continue

        phase_counts[label] = phase_counts.get(label, 0) + 1
        trial = marker.get("trial", f"{phase_counts[label]:03d}")
        group = f"{source.stem}:{label}:{trial}:{phase_counts[label]:02d}"
        trial_count += 1
        for offset in range(start, stop - window_n + 1, step_n):
            end = offset + window_n
            emg = values[offset:end, 1:9].T
            accel = np.mean(values[offset:end, 9:12], axis=0)
            gyro = np.mean(values[offset:end, 12:15], axis=0)
            processed = preprocess_emg(emg, PreprocessConfig(sample_rate_hz=sample_rate))
            feature = extract_emg_features(processed, FeatureConfig(common_mode="median"))
            orientation = orientation_from_accel(accel, gyro)
            session.add(
                feature,
                label,
                group,
                orientation.roll_deg,
                orientation.pitch_deg,
                emg_window=emg,
            )
            window_count += 1
    return trial_count, window_count


def import_xdf_session(
    paths: Iterable[str | Path],
    *,
    participant_id: str = "",
    progress: Callable[[int, int, Path], None] | None = None,
) -> tuple[IntentCaptureSession, dict[str, object]]:
    files = sorted({Path(path).resolve() for path in paths if Path(path).suffix.lower() == ".xdf"})
    if not files:
        raise RuntimeError("No XDF files were selected")
    session = IntentCaptureSession(
        participant_id=participant_id,
        device_name="MindRove 8 + IMU (XDF import)",
        channel_count=8,
    )
    trials = 0
    windows = 0
    errors = []
    for index, path in enumerate(files, start=1):
        if progress is not None:
            progress(index, len(files), path)
        try:
            file_trials, file_windows = import_xdf_file(session, path)
            trials += file_trials
            windows += file_windows
        except Exception as exc:
            errors.append({"file": str(path), "error": str(exc)})
    if not session.features:
        details = "; ".join(f"{Path(item['file']).name}: {item['error']}" for item in errors)
        raise RuntimeError(f"No decoder windows could be imported. {details}")
    return session, {
        "files": len(files),
        "trials": trials,
        "windows": windows,
        "class_counts": session.class_counts(),
        "errors": errors,
    }
