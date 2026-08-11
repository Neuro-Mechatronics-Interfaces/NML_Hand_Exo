from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .contracts import OrientationSample


def orientation_from_accel(
    accel_xyz: np.ndarray,
    gyro_xyz: np.ndarray | None = None,
) -> OrientationSample:
    accel = np.asarray(accel_xyz, dtype=np.float64).reshape(-1)
    if accel.size != 3 or not np.all(np.isfinite(accel)):
        return OrientationSample()
    ax, ay, az = (float(value) for value in accel)
    roll = float(np.degrees(np.arctan2(ay, az)))
    pitch = float(np.degrees(np.arctan2(-ax, np.hypot(ay, az))))
    gyro_norm = None
    if gyro_xyz is not None:
        gyro = np.asarray(gyro_xyz, dtype=np.float64).reshape(-1)
        if gyro.size == 3 and np.all(np.isfinite(gyro)):
            gyro_norm = float(np.linalg.norm(gyro))
    return OrientationSample(
        roll_deg=roll,
        pitch_deg=pitch,
        gyro_norm=gyro_norm,
        accel_norm=float(np.linalg.norm(accel)),
    )


def orientation_basis(roll_deg: np.ndarray, pitch_deg: np.ndarray) -> np.ndarray:
    roll = np.deg2rad(np.asarray(roll_deg, dtype=np.float64))
    pitch = np.deg2rad(np.asarray(pitch_deg, dtype=np.float64))
    if roll.shape != pitch.shape:
        raise ValueError("roll and pitch arrays must have matching shapes")
    return np.column_stack([
        np.ones(roll.size),
        np.sin(roll),
        np.cos(roll),
        np.sin(pitch),
        np.cos(pitch),
    ])


@dataclass
class ContinuousRestAdapter:
    coefficients: np.ndarray | None = None
    scale: np.ndarray | None = None
    global_baseline: np.ndarray | None = None
    ridge: float = 1e-3

    def fit(
        self,
        features: np.ndarray,
        labels: np.ndarray,
        roll_deg: np.ndarray,
        pitch_deg: np.ndarray,
        rest_label: str = "rest",
    ) -> "ContinuousRestAdapter":
        X = np.asarray(features, dtype=np.float64)
        y = np.asarray(labels, dtype=object)
        roll = np.asarray(roll_deg, dtype=np.float64)
        pitch = np.asarray(pitch_deg, dtype=np.float64)
        rest = y == rest_label
        if X.ndim != 2 or len(X) != len(y):
            raise ValueError("features and labels must contain matching rows")
        if np.sum(rest) < 3:
            raise ValueError("At least three rest samples are required")
        self.global_baseline = np.mean(X[rest], axis=0)
        available = rest & np.isfinite(roll) & np.isfinite(pitch)
        if np.sum(available) >= 6:
            design = orientation_basis(roll[available], pitch[available])
            regularizer = np.eye(design.shape[1]) * float(self.ridge)
            regularizer[0, 0] = 0.0
            self.coefficients = np.linalg.solve(
                design.T @ design + regularizer,
                design.T @ X[available],
            )
            fitted_rest = design @ self.coefficients
            residual = X[available] - fitted_rest
        else:
            self.coefficients = None
            residual = X[rest] - self.global_baseline
        spread = np.std(residual, axis=0)
        self.scale = spread + 0.25 * np.median(spread) + 1e-6
        return self

    def baseline(self, roll_deg: np.ndarray, pitch_deg: np.ndarray) -> np.ndarray:
        if self.global_baseline is None:
            raise RuntimeError("Rest adapter has not been fit")
        roll = np.asarray(roll_deg, dtype=np.float64)
        pitch = np.asarray(pitch_deg, dtype=np.float64)
        count = roll.size
        result = np.repeat(self.global_baseline.reshape(1, -1), count, axis=0)
        available = np.isfinite(roll) & np.isfinite(pitch)
        if self.coefficients is not None and np.any(available):
            result[available] = orientation_basis(
                roll[available], pitch[available]
            ) @ self.coefficients
        return result

    def transform(
        self,
        features: np.ndarray,
        roll_deg: np.ndarray,
        pitch_deg: np.ndarray,
    ) -> np.ndarray:
        if self.scale is None:
            raise RuntimeError("Rest adapter has not been fit")
        X = np.asarray(features, dtype=np.float64)
        return (X - self.baseline(roll_deg, pitch_deg)) / self.scale
