"""Inspect XDF stream structure, timing, labels, and recording quality.

Run from the repository root::

    python -m physics_pipeline.xdf_inspect recording.xdf
    python -m physics_pipeline.xdf_inspect recording.xdf --json report.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


def _first(info: dict, key: str, default: Any = "") -> Any:
    value = info.get(key, [default])
    if isinstance(value, list):
        return value[0] if value else default
    return value


def _channel_labels(info: dict, count: int) -> list[str]:
    try:
        desc = _first(info, "desc", {}) or {}
        channels = _first(desc, "channels", {}) or {}
        entries = channels.get("channel", []) or []
        labels = [str(_first(entry, "label", "")) for entry in entries]
    except (AttributeError, TypeError):
        labels = []
    return [labels[index] if index < len(labels) and labels[index] else f"ch{index}" for index in range(count)]


def _stream_schema(info: dict) -> str:
    try:
        desc = _first(info, "desc", {}) or {}
        return str(_first(desc, "schema", ""))
    except (AttributeError, TypeError):
        return ""


def summarize_stream(stream: dict) -> dict[str, Any]:
    info = stream.get("info", {})
    values = np.asarray(stream.get("time_series"))
    timestamps = np.asarray(stream.get("time_stamps", []), dtype=np.float64)
    count = int(values.shape[1]) if values.ndim == 2 else 0
    nominal = float(_first(info, "nominal_srate", 0.0) or 0.0)
    diffs = np.diff(timestamps)
    positive = diffs[np.isfinite(diffs) & (diffs > 0)]
    median_dt = float(np.median(positive)) if positive.size else None
    effective = float(1.0 / median_dt) if median_dt and median_dt > 0 else 0.0
    expected_dt = 1.0 / nominal if nominal > 0 else None
    gap_threshold = 3.0 * expected_dt if expected_dt else None
    gap_count = int(np.count_nonzero(diffs > gap_threshold)) if gap_threshold else 0
    largest_gap = float(np.max(positive)) if positive.size else None
    timestamp_reversals = int(np.count_nonzero(~np.isfinite(diffs) | (diffs < 0)))
    duplicate_intervals = int(np.count_nonzero(diffs == 0))
    duration = float(timestamps[-1] - timestamps[0]) if timestamps.size > 1 else 0.0
    return {
        "name": str(_first(info, "name", "")),
        "type": str(_first(info, "type", "")),
        "source_id": str(_first(info, "source_id", "")),
        "channel_format": str(_first(info, "channel_format", "")),
        "schema": _stream_schema(info),
        "channel_count": count,
        "channel_labels": _channel_labels(info, count),
        "nominal_srate_hz": nominal,
        "median_effective_srate_hz": effective,
        "samples": int(values.shape[0]) if values.ndim >= 1 else 0,
        "start_timestamp": float(timestamps[0]) if timestamps.size else None,
        "stop_timestamp": float(timestamps[-1]) if timestamps.size else None,
        "duration_s": duration,
        "nonmonotonic_intervals": timestamp_reversals,
        "duplicate_timestamp_intervals": duplicate_intervals,
        "gap_count_over_3x_nominal_period": gap_count,
        "largest_positive_gap_s": largest_gap,
        "metadata_has_channel_labels": any(
            not label.startswith("ch") for label in _channel_labels(info, count)
        ),
    }


def inspect_xdf(path: str | Path) -> dict[str, Any]:
    try:
        import pyxdf
    except Exception as exc:
        raise RuntimeError(f"XDF inspection requires pyxdf: {exc}") from exc
    source = Path(path).resolve()
    streams, _header = pyxdf.load_xdf(str(source), verbose=False)
    summaries = [summarize_stream(stream) for stream in streams]
    starts = [item["start_timestamp"] for item in summaries if item["start_timestamp"] is not None]
    stops = [item["stop_timestamp"] for item in summaries if item["stop_timestamp"] is not None]
    return {
        "schema": "nml.xdf_inspection.v1",
        "file": str(source),
        "file_size_bytes": source.stat().st_size,
        "stream_count": len(summaries),
        "recording_start_timestamp": min(starts) if starts else None,
        "recording_stop_timestamp": max(stops) if stops else None,
        "streams": summaries,
    }


def format_report(report: dict[str, Any]) -> str:
    lines = [
        f"XDF: {report['file']}",
        f"Streams: {report['stream_count']}",
    ]
    for stream in report["streams"]:
        lines.extend(
            [
                "",
                f"- {stream['name']} ({stream['type']})",
                f"  source_id: {stream['source_id'] or 'missing'}",
                f"  schema: {stream['schema'] or 'unspecified'}",
                f"  shape: {stream['samples']} samples x {stream['channel_count']} channels",
                f"  rate: nominal {stream['nominal_srate_hz']:.3f} Hz; median effective {stream['median_effective_srate_hz']:.3f} Hz",
                f"  duration: {stream['duration_s']:.3f} s",
                f"  nonmonotonic intervals: {stream['nonmonotonic_intervals']}",
                f"  duplicate-timestamp intervals: {stream['duplicate_timestamp_intervals']}",
                f"  >3x nominal gaps: {stream['gap_count_over_3x_nominal_period']}",
                f"  labels: {', '.join(stream['channel_labels'])}",
            ]
        )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xdf", type=Path)
    parser.add_argument("--json", dest="json_path", type=Path)
    args = parser.parse_args()
    if not args.xdf.is_file():
        parser.error(f"XDF file not found: {args.xdf}")
    report = inspect_xdf(args.xdf)
    print(format_report(report))
    if args.json_path:
        args.json_path.parent.mkdir(parents=True, exist_ok=True)
        args.json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
