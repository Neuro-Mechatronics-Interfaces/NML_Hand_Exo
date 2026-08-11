from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def parse_channel_spec(specification: str, channel_count: int) -> tuple[int, ...]:
    """Parse comma-separated indices and inclusive ranges such as ``1-8,12``."""
    text = specification.strip().lower()
    if not text or text == "none":
        return ()
    if text == "all":
        return tuple(range(channel_count))
    selected = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token:
            start_text, end_text = token.split("-", 1)
            start, end = int(start_text), int(end_text)
            step = 1 if end >= start else -1
            selected.extend(range(start, end + step, step))
        else:
            selected.append(int(token))
    unique = tuple(dict.fromkeys(selected))
    if any(index < 0 or index >= channel_count for index in unique):
        raise ValueError(
            f"Channel specification '{specification}' is outside 0-{channel_count - 1}"
        )
    return unique


@dataclass(frozen=True)
class StreamLayout:
    emg_channels: tuple[int, ...]
    accel_channels: tuple[int, ...] = ()
    gyro_channels: tuple[int, ...] = ()

    @classmethod
    def from_specs(
        cls,
        channel_count: int,
        emg: str,
        accel: str = "",
        gyro: str = "",
    ) -> "StreamLayout":
        layout = cls(
            parse_channel_spec(emg, channel_count),
            parse_channel_spec(accel, channel_count),
            parse_channel_spec(gyro, channel_count),
        )
        if not layout.emg_channels:
            raise ValueError("At least one EMG channel is required")
        if layout.accel_channels and len(layout.accel_channels) != 3:
            raise ValueError("Accelerometer layout must contain exactly three channels")
        if layout.gyro_channels and len(layout.gyro_channels) != 3:
            raise ValueError("Gyroscope layout must contain exactly three channels")
        return layout

    def select_emg(self, data: np.ndarray) -> np.ndarray:
        return np.asarray(data)[list(self.emg_channels), :]

    def mean_accel(self, data: np.ndarray) -> np.ndarray | None:
        if not self.accel_channels:
            return None
        return np.mean(np.asarray(data)[list(self.accel_channels), :], axis=1)

    def mean_gyro(self, data: np.ndarray) -> np.ndarray | None:
        if not self.gyro_channels:
            return None
        return np.mean(np.asarray(data)[list(self.gyro_channels), :], axis=1)
