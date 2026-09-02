"""Replay XDF streams through LSL with original relative timing.

This utility never commands the exoskeleton.  Use a stream-name prefix during
bench tests to prevent accidental connection to live-data consumers.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import heapq
from pathlib import Path
import time
from typing import Any

import numpy as np


FORMAT_MAP = {
    "float32": "float32",
    "double64": "double64",
    "int8": "int8",
    "int16": "int16",
    "int32": "int32",
    "int64": "int64",
    "string": "string",
}


def _first(info: dict, key: str, default: Any = "") -> Any:
    value = info.get(key, [default])
    return value[0] if isinstance(value, list) and value else value


@dataclass
class ReplayStream:
    name: str
    stream_type: str
    source_id: str
    nominal_srate: float
    channel_format: str
    values: np.ndarray
    timestamps: np.ndarray
    info: dict
    outlet: Any = None


def _channel_info(labels: list[tuple[str, str, str]]) -> dict:
    return {
        "desc": [
            {
                "channels": [
                    {
                        "channel": [
                            {
                                "label": [label],
                                "unit": [unit],
                                "type": [channel_type],
                            }
                            for label, unit, channel_type in labels
                        ]
                    }
                ]
            }
        ]
    }


def load_replay_streams(
    path: str | Path, *, mindrove_split: bool = False
) -> list[ReplayStream]:
    try:
        import pyxdf
    except Exception as exc:
        raise RuntimeError(f"XDF replay requires pyxdf: {exc}") from exc
    streams, _header = pyxdf.load_xdf(str(Path(path).resolve()), verbose=False)
    result = []
    for stream in streams:
        info = stream.get("info", {})
        values = np.asarray(stream.get("time_series"))
        timestamps = np.asarray(stream.get("time_stamps", []), dtype=np.float64)
        if values.ndim != 2 or len(values) != len(timestamps) or len(values) == 0:
            continue
        result.append(
            ReplayStream(
                name=str(_first(info, "name", "Unnamed")),
                stream_type=str(_first(info, "type", "Unknown")),
                source_id=str(_first(info, "source_id", "unknown")),
                nominal_srate=float(_first(info, "nominal_srate", 0.0) or 0.0),
                channel_format=str(_first(info, "channel_format", "float32")),
                values=values,
                timestamps=timestamps,
                info=info,
            )
        )
    if mindrove_split:
        combined = next(
            (
                stream
                for stream in result
                if stream.name == "MindRoveStream" and stream.values.shape[1] >= 15
            ),
            None,
        )
        if combined is None:
            raise RuntimeError(
                "--mindrove-split requires a 15+ channel MindRoveStream"
            )
        emg_labels = [(f"EMG{index + 1}", "uV", "emg") for index in range(8)]
        imu_labels = [
            ("acc_x", "g", "accelerometer"),
            ("acc_y", "g", "accelerometer"),
            ("acc_z", "g", "accelerometer"),
            ("gyro_x", "deg/s", "gyroscope"),
            ("gyro_y", "deg/s", "gyroscope"),
            ("gyro_z", "deg/s", "gyroscope"),
        ]
        result.extend(
            [
                ReplayStream(
                    name="MindRove_EMG",
                    stream_type="EMG",
                    source_id=f"{combined.source_id}_emg",
                    nominal_srate=combined.nominal_srate,
                    channel_format=combined.channel_format,
                    values=combined.values[:, 1:9],
                    timestamps=combined.timestamps,
                    info=_channel_info(emg_labels),
                ),
                ReplayStream(
                    name="MindRove_IMU",
                    stream_type="IMU",
                    source_id=f"{combined.source_id}_imu",
                    nominal_srate=combined.nominal_srate,
                    channel_format=combined.channel_format,
                    values=combined.values[:, 9:15],
                    timestamps=combined.timestamps,
                    info=_channel_info(imu_labels),
                ),
            ]
        )
    if not result:
        raise RuntimeError("XDF contains no replayable streams")
    return result


def build_event_schedule(streams: list[ReplayStream]) -> list[tuple[float, int, int]]:
    origin = min(float(stream.timestamps[0]) for stream in streams)
    schedule: list[tuple[float, int, int]] = []
    for stream_index, stream in enumerate(streams):
        schedule.extend(
            (float(timestamp) - origin, stream_index, sample_index)
            for sample_index, timestamp in enumerate(stream.timestamps)
        )
    heapq.heapify(schedule)
    return schedule


def _copy_metadata(source_info: dict, target_info: Any) -> None:
    desc_values = source_info.get("desc", [])
    if not desc_values or not desc_values[0]:
        return
    # XDF metadata is deeply nested and pylsl's XML API is append-only.  Copy
    # the channel fields needed for unambiguous replay rather than guessing at
    # arbitrary XML structure.
    try:
        channels = desc_values[0].get("channels", [{}])[0].get("channel", [])
    except (AttributeError, IndexError, TypeError):
        channels = []
    if not channels:
        return
    target_channels = target_info.desc().append_child("channels")
    for source_channel in channels:
        target_channel = target_channels.append_child("channel")
        for key in ("label", "unit", "type"):
            value = _first(source_channel, key, "")
            if value:
                target_channel.append_child_value(key, str(value))


def create_outlets(
    streams: list[ReplayStream], *, prefix: str, speed: float
) -> None:
    try:
        from pylsl import StreamInfo, StreamOutlet
    except Exception as exc:
        raise RuntimeError(f"XDF replay requires pylsl: {exc}") from exc
    for stream in streams:
        channel_count = int(stream.values.shape[1])
        name = f"{prefix}{stream.name}"
        source_id = f"{prefix}{stream.source_id}_replay"
        rate = stream.nominal_srate * speed if stream.nominal_srate > 0 else 0.0
        channel_format = FORMAT_MAP.get(stream.channel_format, "double64")
        info = StreamInfo(
            name,
            stream.stream_type,
            channel_count,
            rate,
            channel_format,
            source_id,
        )
        info.desc().append_child_value("schema", "nml.xdf_replay.v1")
        info.desc().append_child_value("original_name", stream.name)
        info.desc().append_child_value("original_source_id", stream.source_id)
        _copy_metadata(stream.info, info)
        stream.outlet = StreamOutlet(info)


def replay_once(streams: list[ReplayStream], *, speed: float) -> None:
    from pylsl import local_clock

    schedule = build_event_schedule(streams)
    replay_start = float(local_clock()) + 0.25
    while schedule:
        relative_s, stream_index, sample_index = heapq.heappop(schedule)
        target = replay_start + relative_s / speed
        while True:
            remaining = target - float(local_clock())
            if remaining <= 0:
                break
            time.sleep(min(remaining, 0.01))
        stream = streams[stream_index]
        sample = stream.values[sample_index]
        if stream.channel_format == "string":
            payload = [str(value) for value in sample.tolist()]
        else:
            payload = sample.tolist()
        stream.outlet.push_sample(payload, target)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xdf", type=Path)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--prefix", default="REPLAY_")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument(
        "--mindrove-split",
        action="store_true",
        help="Also publish MindRove_EMG (8 ch) and MindRove_IMU (6 ch) views",
    )
    args = parser.parse_args()
    if not args.xdf.is_file():
        parser.error(f"XDF file not found: {args.xdf}")
    if not np.isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be finite and greater than zero")
    streams = load_replay_streams(args.xdf, mindrove_split=args.mindrove_split)
    create_outlets(streams, prefix=args.prefix, speed=args.speed)
    print("Publishing replay streams:")
    for stream in streams:
        print(f"  {args.prefix}{stream.name} ({stream.stream_type}, {stream.values.shape[1]} ch)")
    try:
        while True:
            replay_once(streams, speed=args.speed)
            if not args.loop:
                break
    except KeyboardInterrupt:
        print("Replay stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
