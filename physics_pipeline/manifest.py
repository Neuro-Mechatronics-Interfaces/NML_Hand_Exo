"""Sidecar session metadata with explicit unknowns and atomic persistence."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import os
from pathlib import Path
import tempfile
from typing import Any

from .contracts import PHYSICS_SESSION_SCHEMA


@dataclass
class SessionManifest:
    session_id: str
    participant_id: str = ""
    side: str = "unknown"
    condition: str = "unspecified"
    xdf_file: str = ""
    prompt_plan: str = ""
    calibration_profile: str = ""
    firmware_version: str = ""
    software_commit: str = ""
    electrode_layout: dict[str, Any] = field(default_factory=dict)
    hand_measurements: dict[str, Any] = field(default_factory=dict)
    exoskeleton_geometry: dict[str, Any] = field(default_factory=dict)
    motor_joint_mapping: dict[str, Any] = field(default_factory=dict)
    torque_estimation: dict[str, Any] = field(default_factory=dict)
    stream_inventory: dict[str, Any] = field(default_factory=dict)
    control_configuration: dict[str, Any] = field(default_factory=dict)
    kinematics_system: dict[str, Any] = field(default_factory=dict)
    interaction_force_system: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    extra: dict[str, Any] = field(default_factory=dict)
    schema: str = PHYSICS_SESSION_SCHEMA

    def validate(self) -> None:
        if self.schema != PHYSICS_SESSION_SCHEMA:
            raise ValueError(f"Unsupported manifest schema: {self.schema}")
        if not self.session_id.strip():
            raise ValueError("session_id is required")
        if self.side not in {"left", "right", "dual", "unknown"}:
            raise ValueError("side must be left, right, dual, or unknown")

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return asdict(self)

    def save(self, path: str | Path) -> None:
        self.validate()
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(
            prefix=f".{destination.name}.", suffix=".tmp", dir=str(destination.parent)
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(self.to_dict(), handle, indent=2, sort_keys=True)
                handle.write("\n")
            os.replace(temporary, destination)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    @classmethod
    def load(cls, path: str | Path) -> "SessionManifest":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        known = set(cls.__dataclass_fields__)
        values = {key: value for key, value in payload.items() if key in known}
        unknown = {key: value for key, value in payload.items() if key not in known}
        values.setdefault("extra", {}).update(unknown)
        manifest = cls(**values)
        manifest.validate()
        return manifest
