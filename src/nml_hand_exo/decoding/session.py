from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class IntentCaptureSession:
    participant_id: str = ""
    device_name: str = ""
    channel_count: int = 0
    features: list[np.ndarray] = field(default_factory=list)
    labels: list[str] = field(default_factory=list)
    groups: list[str] = field(default_factory=list)
    roll_deg: list[float] = field(default_factory=list)
    pitch_deg: list[float] = field(default_factory=list)
    emg_windows: list[np.ndarray] = field(default_factory=list)
    comfort: dict[str, float] = field(default_factory=dict)

    def add(
        self,
        feature: np.ndarray,
        label: str,
        group: str,
        roll_deg: float | None = None,
        pitch_deg: float | None = None,
        emg_window: np.ndarray | None = None,
    ) -> None:
        value = np.asarray(feature, dtype=np.float64).reshape(-1)
        if self.features and value.shape != self.features[0].shape:
            raise ValueError("All captured feature vectors must have the same size")
        window = None
        if emg_window is not None:
            window = np.asarray(emg_window, dtype=np.float64)
            if window.ndim != 2:
                raise ValueError("Raw EMG windows must have shape (channels, samples)")
            if self.emg_windows and window.shape != self.emg_windows[0].shape:
                raise ValueError("All raw EMG windows must have the same shape")
            if self.features and not self.emg_windows:
                # Legacy feature-only sessions remain appendable without creating
                # a partially populated raw-window array.
                window = None
        elif self.emg_windows:
            raise ValueError("Raw EMG must be present for every sample once recording begins")
        self.features.append(value.copy())
        self.labels.append(str(label))
        self.groups.append(str(group))
        self.roll_deg.append(np.nan if roll_deg is None else float(roll_deg))
        self.pitch_deg.append(np.nan if pitch_deg is None else float(pitch_deg))
        if window is not None:
            self.emg_windows.append(window.copy())

    def arrays(self):
        if not self.features:
            return (
                np.empty((0, 0)), np.empty(0, dtype=object),
                np.empty(0, dtype=object), np.empty(0), np.empty(0),
            )
        return (
            np.vstack(self.features),
            np.asarray(self.labels, dtype=object),
            np.asarray(self.groups, dtype=object),
            np.asarray(self.roll_deg, dtype=np.float64),
            np.asarray(self.pitch_deg, dtype=np.float64),
        )

    def class_counts(self) -> dict[str, int]:
        return {label: self.labels.count(label) for label in sorted(set(self.labels))}

    def save(self, path: str | Path) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        X, y, groups, roll, pitch = self.arrays()
        metadata = {
            "schema": 1,
            "participant_id": self.participant_id,
            "device_name": self.device_name,
            "channel_count": self.channel_count,
            "comfort": self.comfort,
        }
        fd, temporary = tempfile.mkstemp(
            prefix=f".{destination.name}.", suffix=".npz", dir=str(destination.parent)
        )
        os.close(fd)
        try:
            np.savez_compressed(
                temporary,
                metadata_json=np.asarray(json.dumps(metadata)),
                features=X,
                labels=y.astype(str),
                groups=groups.astype(str),
                roll_deg=roll,
                pitch_deg=pitch,
                emg_windows=(
                    np.stack(self.emg_windows)
                    if self.emg_windows
                    else np.empty((0, 0, 0), dtype=np.float64)
                ),
            )
            os.replace(temporary, destination)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    @classmethod
    def load(cls, path: str | Path) -> "IntentCaptureSession":
        with np.load(path, allow_pickle=False) as data:
            metadata = json.loads(str(data["metadata_json"].item()))
            session = cls(
                participant_id=str(metadata.get("participant_id", "")),
                device_name=str(metadata.get("device_name", "")),
                channel_count=int(metadata.get("channel_count", 0)),
                comfort={str(k): float(v) for k, v in metadata.get("comfort", {}).items()},
            )
            raw_windows = data["emg_windows"] if "emg_windows" in data else None
            for index, (feature, label, group, roll, pitch) in enumerate(zip(
                data["features"], data["labels"], data["groups"],
                data["roll_deg"], data["pitch_deg"],
            )):
                raw = None
                if raw_windows is not None and len(raw_windows) == len(data["features"]):
                    raw = raw_windows[index]
                session.add(
                    feature, str(label), str(group),
                    None if np.isnan(roll) else float(roll),
                    None if np.isnan(pitch) else float(pitch),
                    emg_window=raw,
                )
        return session
