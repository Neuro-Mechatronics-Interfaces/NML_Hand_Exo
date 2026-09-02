"""Configurable reduced-coordinate hand/exoskeleton geometry.

The classes here deliberately do not encode an anatomical hand model.  They
make every motor-to-coordinate relationship explicit and serializable.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(frozen=True)
class GeneralizedCoordinate:
    name: str
    unit: str = "rad"
    lower_limit: float | None = None
    upper_limit: float | None = None
    description: str = ""

    def validate(self) -> None:
        if not self.name.strip():
            raise ValueError("Generalized coordinate name cannot be empty")
        if (
            self.lower_limit is not None
            and self.upper_limit is not None
            and self.lower_limit >= self.upper_limit
        ):
            raise ValueError(f"Invalid limits for coordinate {self.name}")


@dataclass(frozen=True)
class MotorCoordinateMapping:
    motor_id: int
    motor_name: str
    coordinate: str
    transmission_ratio: float
    sign: int
    offset: float = 0.0
    confidence: str = "unvalidated"

    def validate(self) -> None:
        if int(self.motor_id) <= 0:
            raise ValueError("motor_id must be positive")
        if not self.motor_name.strip() or not self.coordinate.strip():
            raise ValueError("motor_name and coordinate are required")
        if not np.isfinite(self.transmission_ratio) or self.transmission_ratio == 0:
            raise ValueError("transmission_ratio must be finite and nonzero")
        if self.sign not in {-1, 1}:
            raise ValueError("sign must be -1 or +1")


@dataclass
class ReducedGeometry:
    name: str
    side: str
    coordinates: list[GeneralizedCoordinate]
    mappings: list[MotorCoordinateMapping]
    hand_measurements: dict[str, Any] = field(default_factory=dict)
    actuator_attachments: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    schema: str = "nml.reduced_geometry.v1"

    def validate(self) -> None:
        if self.schema != "nml.reduced_geometry.v1":
            raise ValueError(f"Unsupported geometry schema: {self.schema}")
        if self.side not in {"left", "right", "dual", "unknown"}:
            raise ValueError("side must be left, right, dual, or unknown")
        for coordinate in self.coordinates:
            coordinate.validate()
        names = [coordinate.name for coordinate in self.coordinates]
        if len(names) != len(set(names)):
            raise ValueError("Coordinate names must be unique")
        ids = []
        for mapping in self.mappings:
            mapping.validate()
            if mapping.coordinate not in names:
                raise ValueError(f"Mapping references unknown coordinate {mapping.coordinate!r}")
            ids.append(mapping.motor_id)
        if len(ids) != len(set(ids)):
            raise ValueError("A motor ID may appear only once in a reduced geometry")

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return asdict(self)

    def save(self, path: str | Path) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "ReducedGeometry":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        payload["coordinates"] = [GeneralizedCoordinate(**item) for item in payload.get("coordinates", [])]
        payload["mappings"] = [MotorCoordinateMapping(**item) for item in payload.get("mappings", [])]
        geometry = cls(**payload)
        geometry.validate()
        return geometry

    def motor_to_coordinate_matrix(self) -> np.ndarray:
        """Map motor torque to generalized torque using explicit signs/ratios."""

        self.validate()
        coordinate_index = {item.name: index for index, item in enumerate(self.coordinates)}
        matrix = np.zeros((len(self.coordinates), len(self.mappings)), dtype=np.float64)
        for motor_index, mapping in enumerate(self.mappings):
            matrix[coordinate_index[mapping.coordinate], motor_index] = (
                float(mapping.sign) * float(mapping.transmission_ratio)
            )
        return matrix
