"""Baseline and reduced physics/state-space models.

These models are intentionally modest.  They provide falsifiable baselines and
an explicit dynamical scaffold without pretending that eight surface-EMG
channels uniquely identify every anatomical muscle force.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.linear_model import Ridge
from sklearn.preprocessing import StandardScaler


class StateConditionedIntentModel:
    """Shrinkage-LDA baseline using EMG features and optional measured state."""

    def __init__(self) -> None:
        self.scaler = StandardScaler()
        self.classifier = LinearDiscriminantAnalysis(solver="lsqr", shrinkage="auto")
        self.classes_: np.ndarray | None = None

    @staticmethod
    def _design(emg_features: np.ndarray, state: np.ndarray | None) -> np.ndarray:
        emg = np.asarray(emg_features, dtype=np.float64)
        if emg.ndim != 2:
            raise ValueError("emg_features must have shape (samples, features)")
        if state is None or np.asarray(state).size == 0:
            design = emg
        else:
            measured = np.asarray(state, dtype=np.float64)
            if measured.ndim != 2 or len(measured) != len(emg):
                raise ValueError("state must have shape (samples, state_features)")
            design = np.hstack([emg, measured])
        if not np.all(np.isfinite(design)):
            raise ValueError("Model inputs contain NaN or infinite values")
        return design

    def fit(
        self, emg_features: np.ndarray, labels: np.ndarray, state: np.ndarray | None = None
    ) -> "StateConditionedIntentModel":
        design = self._design(emg_features, state)
        y = np.asarray(labels).reshape(-1)
        if len(y) != len(design):
            raise ValueError("labels and features must contain matching samples")
        scaled = self.scaler.fit_transform(design)
        self.classifier.fit(scaled, y)
        self.classes_ = np.asarray(self.classifier.classes_)
        return self

    def predict(self, emg_features: np.ndarray, state: np.ndarray | None = None) -> np.ndarray:
        design = self._design(emg_features, state)
        return self.classifier.predict(self.scaler.transform(design))

    def predict_proba(self, emg_features: np.ndarray, state: np.ndarray | None = None) -> np.ndarray:
        design = self._design(emg_features, state)
        return self.classifier.predict_proba(self.scaler.transform(design))


@dataclass(frozen=True)
class ActivationDynamics:
    """First-order EMG-envelope to latent-activation dynamics."""

    activation_time_constant_s: float = 0.050
    relaxation_time_constant_s: float = 0.080

    def filter(self, excitation: np.ndarray, sample_period_s: float) -> np.ndarray:
        values = np.asarray(excitation, dtype=np.float64)
        if values.ndim == 1:
            values = values[:, None]
        if sample_period_s <= 0:
            raise ValueError("sample_period_s must be greater than zero")
        if self.activation_time_constant_s <= 0 or self.relaxation_time_constant_s <= 0:
            raise ValueError("Activation time constants must be greater than zero")
        output = np.zeros_like(values)
        output[0] = np.clip(values[0], 0.0, None)
        for index in range(1, len(values)):
            target = np.clip(values[index], 0.0, None)
            tau = np.where(
                target >= output[index - 1],
                self.activation_time_constant_s,
                self.relaxation_time_constant_s,
            )
            alpha = 1.0 - np.exp(-sample_period_s / tau)
            output[index] = output[index - 1] + alpha * (target - output[index - 1])
        return output


class LinearStateSpaceModel:
    """Regularized identification of x[t+1] = A*x[t] + B*u[t] + c."""

    def __init__(self, regularization: float = 1e-3) -> None:
        self.regularization = float(regularization)
        self.regressor: Ridge | None = None
        self.state_size = 0
        self.input_size = 0

    def fit(self, state: np.ndarray, inputs: np.ndarray) -> "LinearStateSpaceModel":
        x = np.asarray(state, dtype=np.float64)
        u = np.asarray(inputs, dtype=np.float64)
        if x.ndim != 2 or u.ndim != 2 or len(x) != len(u):
            raise ValueError("state and inputs must be matching two-dimensional arrays")
        if len(x) < 3:
            raise ValueError("At least three time samples are required")
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(u)):
            raise ValueError("State-space identification inputs must be finite")
        design = np.hstack([x[:-1], u[:-1]])
        self.regressor = Ridge(alpha=self.regularization, fit_intercept=True).fit(design, x[1:])
        self.state_size = x.shape[1]
        self.input_size = u.shape[1]
        return self

    @property
    def A(self) -> np.ndarray:
        if self.regressor is None:
            raise RuntimeError("State-space model has not been fit")
        return np.asarray(self.regressor.coef_[:, : self.state_size], dtype=np.float64)

    @property
    def B(self) -> np.ndarray:
        if self.regressor is None:
            raise RuntimeError("State-space model has not been fit")
        return np.asarray(self.regressor.coef_[:, self.state_size :], dtype=np.float64)

    @property
    def offset(self) -> np.ndarray:
        if self.regressor is None:
            raise RuntimeError("State-space model has not been fit")
        return np.asarray(self.regressor.intercept_, dtype=np.float64)

    def predict_step(self, state: np.ndarray, inputs: np.ndarray) -> np.ndarray:
        if self.regressor is None:
            raise RuntimeError("State-space model has not been fit")
        x = np.asarray(state, dtype=np.float64).reshape(-1)
        u = np.asarray(inputs, dtype=np.float64).reshape(-1)
        if len(x) != self.state_size or len(u) != self.input_size:
            raise ValueError("State or input vector has the wrong size")
        return self.A @ x + self.B @ u + self.offset

    def rollout(self, initial_state: np.ndarray, inputs: np.ndarray) -> np.ndarray:
        u = np.asarray(inputs, dtype=np.float64)
        if u.ndim != 2:
            raise ValueError("inputs must have shape (time, input_features)")
        states = np.empty((len(u) + 1, self.state_size), dtype=np.float64)
        states[0] = np.asarray(initial_state, dtype=np.float64).reshape(-1)
        for index, value in enumerate(u):
            states[index + 1] = self.predict_step(states[index], value)
        return states


@dataclass
class ReducedPhysicsParameters:
    inertia: np.ndarray
    stiffness: np.ndarray
    damping: np.ndarray
    rest_position: np.ndarray
    emg_torque_map: np.ndarray
    bias_torque: np.ndarray

    def validate(self) -> None:
        inertia = np.asarray(self.inertia, dtype=np.float64).reshape(-1)
        dof = len(inertia)
        for name in ("stiffness", "damping", "rest_position", "bias_torque"):
            if len(np.asarray(getattr(self, name)).reshape(-1)) != dof:
                raise ValueError(f"{name} must contain one value per coordinate")
        mapping = np.asarray(self.emg_torque_map, dtype=np.float64)
        if mapping.ndim != 2 or mapping.shape[0] != dof:
            raise ValueError("emg_torque_map must have shape (coordinates, activations)")
        if np.any(inertia <= 0) or not np.all(np.isfinite(inertia)):
            raise ValueError("Every inertia must be finite and positive")


class ReducedPhysicsModel:
    """Second-order reduced dynamics with explicit passive and exo torques."""

    def __init__(self, parameters: ReducedPhysicsParameters) -> None:
        parameters.validate()
        self.parameters = parameters

    @property
    def dof(self) -> int:
        return len(np.asarray(self.parameters.inertia).reshape(-1))

    def acceleration(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        activation: np.ndarray,
        exo_torque: np.ndarray,
    ) -> np.ndarray:
        p = self.parameters
        q = np.asarray(position, dtype=np.float64).reshape(-1)
        qd = np.asarray(velocity, dtype=np.float64).reshape(-1)
        a = np.asarray(activation, dtype=np.float64).reshape(-1)
        tau_exo = np.asarray(exo_torque, dtype=np.float64).reshape(-1)
        if len(q) != self.dof or len(qd) != self.dof or len(tau_exo) != self.dof:
            raise ValueError("Position, velocity and exo torque must match model DOFs")
        muscle = np.asarray(p.emg_torque_map) @ a + np.asarray(p.bias_torque)
        passive = np.asarray(p.stiffness) * (q - np.asarray(p.rest_position)) + np.asarray(p.damping) * qd
        return (muscle + tau_exo - passive) / np.asarray(p.inertia)

    def step(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        activation: np.ndarray,
        exo_torque: np.ndarray,
        dt_s: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        if dt_s <= 0:
            raise ValueError("dt_s must be greater than zero")
        q = np.asarray(position, dtype=np.float64).reshape(-1)
        qd = np.asarray(velocity, dtype=np.float64).reshape(-1)
        qdd = self.acceleration(q, qd, activation, exo_torque)
        next_velocity = qd + qdd * dt_s
        next_position = q + next_velocity * dt_s
        return next_position, next_velocity


def fit_quasistatic_emg_torque(
    activation: np.ndarray,
    position: np.ndarray,
    measured_balancing_torque: np.ndarray,
    regularization: float = 1e-3,
) -> tuple[np.ndarray, np.ndarray]:
    """Fit torque = W*activation + K*position + bias for isometric trials.

    This identifies a useful reduced mapping; it does not claim individual
    muscle-force identifiability.
    """

    a = np.asarray(activation, dtype=np.float64)
    q = np.asarray(position, dtype=np.float64)
    tau = np.asarray(measured_balancing_torque, dtype=np.float64)
    if a.ndim != 2 or q.ndim != 2 or tau.ndim != 2:
        raise ValueError("activation, position, and torque must be two-dimensional")
    if not (len(a) == len(q) == len(tau)):
        raise ValueError("activation, position, and torque must contain matching samples")
    design = np.hstack([a, q])
    model = Ridge(alpha=float(regularization), fit_intercept=True).fit(design, tau)
    return np.asarray(model.coef_, dtype=np.float64), np.asarray(model.intercept_, dtype=np.float64)
