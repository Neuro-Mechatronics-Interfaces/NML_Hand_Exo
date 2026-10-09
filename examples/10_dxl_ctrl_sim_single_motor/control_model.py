"""Single-motor Dynamixel-style controller and plant simulation.

This module deliberately models the control structure rather than attempting to
reproduce the XC330 firmware numerically.  The control-table gains are exposed
as raw values and converted through the documented Dynamixel scale factors, but
the plant gains are normalized so that the example remains easy to tune.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math
import random


TORQUE_CONSTANT_NM_PER_MA = 0.00115


class OperatingMode(str, Enum):
    """Dynamixel operating modes, with the raw XC330 control-table value."""

    CURRENT = "CURRENT"
    VELOCITY = "VELOCITY"
    POSITION = "POSITION"
    EXTENDED_POSITION = "EXTENDED_POSITION"
    CURRENT_POSITION = "CURRENT_POSITION"
    PWM = "PWM"

    @property
    def raw_value(self) -> int:
        return {
            OperatingMode.CURRENT: 0,
            OperatingMode.VELOCITY: 1,
            OperatingMode.POSITION: 3,
            OperatingMode.EXTENDED_POSITION: 4,
            OperatingMode.CURRENT_POSITION: 5,
            OperatingMode.PWM: 16,
        }[self]

    @property
    def library_constant(self) -> str:
        return {
            OperatingMode.CURRENT: "OP_CURRENT",
            OperatingMode.VELOCITY: "OP_VELOCITY",
            OperatingMode.POSITION: "OP_POSITION",
            OperatingMode.EXTENDED_POSITION: "OP_EXTENDED_POSITION",
            OperatingMode.CURRENT_POSITION: "OP_CURRENT_BASED_POSITION",
            OperatingMode.PWM: "OP_PWM",
        }[self]


class Objective(str, Enum):
    HOLD = "Hold at start"
    POSITION_TRAJECTORY = "Position trajectory"
    SECONDARY_GOAL_ASSIST = "Torque assist (secondary goal)"
    TORQUE_ASSIST = "Torque assist (dynamic current cap)"
    EXTERNAL_EFFORT_ASSIST = "External-effort assist"
    VELOCITY_COMMAND = "Velocity command"
    CURRENT_COMMAND = "Current command"
    PWM_COMMAND = "PWM command"


@dataclass
class SimulationConfig:
    mode: OperatingMode = OperatingMode.CURRENT_POSITION
    objective: Objective = Objective.SECONDARY_GOAL_ASSIST

    min_position_deg: float = 0.0
    max_position_deg: float = 360.0
    start_position_deg: float = 100.0
    end_position_deg: float = 260.0
    duration_s: float = 4.0

    # Signed values use the visual convention: positive is counter-clockwise.
    external_torque_nm: float = 0.0
    external_torque_profile_noise_sigma_nm: float = 0.0
    assist_torque_nm: float = 0.25
    # Keep the default plant deterministic; enable Gaussian effort noise
    # explicitly when studying observer robustness.
    external_torque_noise_std_nm: float = 0.05
    external_torque_profile_period_s: float = 45.0
    external_torque_schedule: tuple[tuple[float, float], ...] = ((0.0, 0.1), (3.0, 0.1), (15.0, 0.35), (20.0, 0.4), (25.0, 0.3), (30.0, -0.1), (32.0, -0.4), (42.0, -0.35), (43.0, -0.05), (45.0, 0.0))
    external_random_seed: int = 7
    collection_epoch_s: float = 1.0
    external_estimator_tau_s: float = 0.20
    assist_detect_threshold_nm: float = 0.08
    assist_detect_delay_s: float = 0.50
    external_assist_gain: float = 1.0
    external_assist_max_torque_nm: float = 0.50
    external_assist_ramp_s: float = 0.25
    direct_current_ma: float = 180.0
    direct_velocity_deg_s: float = 35.0
    direct_pwm_percent: float = 20.0

    # Soft endstop.  The selected side is the direction in which penetration
    # occurs.  A position below the lower physical limit is never allowed.
    endstop_enabled: bool = True
    endstop_position_deg: float = 180.0
    endstop_side: int = 1  # +1 = upper/CW visual direction, -1 = lower
    endstop_max_torque_nm: float = 0.8
    endstop_stiffness_nm_per_deg: float = 0.08
    endstop_damping_nm_s_per_deg: float = 0.02

    # Raw Dynamixel-style control-table gains.
    # The secondary-goal default is deliberately compliant: low P, no I, and
    # moderate D damping.  Other objectives can still tune these registers.
    position_p_raw: int = 350
    position_i_raw: int = 0
    position_d_raw: int = 160
    velocity_p_raw: int = 40
    velocity_i_raw: int = 1200
    goal_current_limit_ma: float = 300.0
    pwm_limit_percent: float = 100.0
    assist_target_bias_deg_per_nm: float = 0.0
    assist_velocity_deg_s_per_nm: float = 80.0
    secondary_goal_lead_deg: float = 8.0
    secondary_goal_lead_rate_deg_s: float = 24.0
    secondary_goal_stall_velocity_deg_s: float = 1.0
    secondary_goal_stall_delay_s: float = 0.50

    # Simplified plant parameters.
    inertia_kg_m2: float = 0.035
    viscous_damping_nm_s_per_deg: float = 0.018
    coulomb_friction_nm: float = 0.025
    pwm_torque_at_100_percent_nm: float = 1.0


@dataclass
class SimulationSnapshot:
    time_s: float
    position_deg: float
    velocity_deg_s: float
    reference_position_deg: float
    reference_velocity_deg_s: float
    position_error_deg: float
    actuator_torque_nm: float
    external_torque_nm: float
    endstop_torque_nm: float
    net_torque_nm: float
    current_ma: float
    goal_current_ma: float
    pwm_percent: float
    pid_torque_nm: float
    clamped_torque_nm: float
    integral_error_deg_s: float
    endstop_penetration_deg: float
    external_torque_mean_nm: float
    external_torque_estimate_nm: float
    external_torque_bias_nm: float
    debiased_external_torque_nm: float
    assist_active: bool
    assist_torque_command_nm: float
    collection_progress: float
    secondary_goal_position_deg: float
    secondary_goal_velocity_deg_s: float
    secondary_goal_lead_deg: float


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _sign(value: float) -> float:
    if value > 0:
        return 1.0
    if value < 0:
        return -1.0
    return 0.0


class SingleMotorSimulation:
    """A small, deterministic controller-plus-plant simulation."""

    def __init__(self, config: SimulationConfig | None = None) -> None:
        self.config = config or SimulationConfig()
        self.reset()

    def reset(self) -> None:
        c = self.config
        self.random = random.Random(c.external_random_seed)
        self.time_s = 0.0
        self.position_deg = _clamp(
            c.start_position_deg, c.min_position_deg, c.max_position_deg
        )
        self.velocity_deg_s = 0.0
        self.reference_position_deg = self.position_deg
        self.reference_velocity_deg_s = 0.0
        self.assist_position_offset_deg = 0.0
        self.assist_motion_started = False
        self.assist_target_direction = 0
        self.secondary_goal_position_deg = self.position_deg
        self.secondary_goal_velocity_deg_s = 0.0
        self.secondary_goal_lead_deg = 0.0
        self.secondary_goal_direction = 0
        self.secondary_assist_active = False
        self.secondary_rearm_blocked = False
        self.secondary_release_capture_position_deg = None
        self.secondary_intent_timer_s = 0.0
        self.secondary_release_timer_s = 0.0
        self.secondary_stall_timer_s = 0.0
        self.integral_error_deg_s = 0.0
        self.external_torque_estimate_nm = 0.0
        self.external_torque_bias_nm = 0.0
        self.debiased_external_torque_nm = 0.0
        self.assist_torque_command_nm = 0.0
        self.assist_active = False
        self.assist_detection_timer_s = 0.0
        self.assist_release_timer_s = 0.0
        self.collection_samples = 0
        external_mean = self._external_torque_mean(0.0)
        self.previous_snapshot = SimulationSnapshot(
            time_s=0.0,
            position_deg=self.position_deg,
            velocity_deg_s=0.0,
            reference_position_deg=self.position_deg,
            reference_velocity_deg_s=0.0,
            position_error_deg=0.0,
            actuator_torque_nm=0.0,
            external_torque_nm=external_mean,
            endstop_torque_nm=0.0,
            net_torque_nm=0.0,
            current_ma=0.0,
            goal_current_ma=self.effective_goal_current_ma(),
            pwm_percent=0.0,
            pid_torque_nm=0.0,
            clamped_torque_nm=0.0,
            integral_error_deg_s=0.0,
            endstop_penetration_deg=0.0,
            external_torque_mean_nm=external_mean,
            external_torque_estimate_nm=0.0,
            external_torque_bias_nm=0.0,
            debiased_external_torque_nm=0.0,
            assist_active=False,
            assist_torque_command_nm=0.0,
            collection_progress=1.0 if c.collection_epoch_s <= 0.0 else 0.0,
            secondary_goal_position_deg=self.position_deg,
            secondary_goal_velocity_deg_s=0.0,
            secondary_goal_lead_deg=0.0,
        )

    def set_config(self, config: SimulationConfig, *, reset: bool = False) -> None:
        previous_objective = self.config.objective
        self.config = config
        self.position_deg = _clamp(
            self.position_deg, config.min_position_deg, config.max_position_deg
        )
        if reset:
            self.reset()
            return

        # Refresh displayed command/reference values immediately when a UI
        # control changes, without advancing the plant or simulation clock.
        if config.objective in {
            Objective.TORQUE_ASSIST,
            Objective.SECONDARY_GOAL_ASSIST,
        }:
            self.assist_torque_command_nm = self._assist_command()
        elif config.objective != Objective.EXTERNAL_EFFORT_ASSIST:
            self.assist_torque_command_nm = 0.0

        if (
            config.objective == Objective.SECONDARY_GOAL_ASSIST
            and previous_objective != Objective.SECONDARY_GOAL_ASSIST
        ):
            self.secondary_goal_position_deg = self.position_deg
            self.secondary_goal_velocity_deg_s = 0.0
            self.secondary_goal_lead_deg = 0.0
            self.secondary_goal_direction = 0
            self.secondary_assist_active = False
            self.secondary_rearm_blocked = False
            self.secondary_release_capture_position_deg = None
            self.assist_active = False
            self.secondary_intent_timer_s = 0.0
            self.secondary_release_timer_s = 0.0
            self.secondary_stall_timer_s = 0.0
        elif config.objective != Objective.SECONDARY_GOAL_ASSIST:
            self.secondary_goal_position_deg = self.position_deg
            self.secondary_goal_velocity_deg_s = 0.0
            self.secondary_goal_lead_deg = 0.0
            self.secondary_goal_direction = 0
            self.secondary_assist_active = False
            self.secondary_rearm_blocked = False
            self.secondary_release_capture_position_deg = None
            if config.objective != Objective.EXTERNAL_EFFORT_ASSIST:
                self.assist_active = False
            self.secondary_intent_timer_s = 0.0
            self.secondary_release_timer_s = 0.0
            self.secondary_stall_timer_s = 0.0
        if config.mode not in {
            OperatingMode.POSITION,
            OperatingMode.EXTENDED_POSITION,
            OperatingMode.CURRENT_POSITION,
        } or config.objective not in {
            Objective.TORQUE_ASSIST,
            Objective.EXTERNAL_EFFORT_ASSIST,
        }:
            self.assist_position_offset_deg = 0.0
            self.assist_motion_started = False
            self.assist_target_direction = 0
        q_ref, v_ref = self._position_reference()
        self.reference_position_deg = q_ref
        self.reference_velocity_deg_s = v_ref
        self.previous_snapshot = replace(
            self.previous_snapshot,
            reference_position_deg=q_ref,
            reference_velocity_deg_s=v_ref,
            position_error_deg=q_ref - self.position_deg,
            external_torque_nm=self._external_torque_mean(),
            external_torque_mean_nm=self._external_torque_mean(),
            goal_current_ma=self.effective_goal_current_ma(),
            assist_active=self.assist_active,
            assist_torque_command_nm=self.assist_torque_command_nm,
            secondary_goal_position_deg=self.secondary_goal_position_deg,
            secondary_goal_velocity_deg_s=self.secondary_goal_velocity_deg_s,
            secondary_goal_lead_deg=self.secondary_goal_lead_deg,
        )

    @property
    def mode_label(self) -> str:
        c = self.config
        return f"{c.mode.value}  (raw Operating Mode {c.mode.raw_value})"

    @property
    def active_path(self) -> str:
        c = self.config
        if c.objective == Objective.EXTERNAL_EFFORT_ASSIST:
            if c.mode == OperatingMode.CURRENT:
                return "external profile → observer → zero-bias residual → sustained detector → current assist → plant"
            if c.mode == OperatingMode.CURRENT_POSITION:
                return "external profile → observer → zero-bias residual → target shift → position PID → Goal Current → plant"
            if c.mode == OperatingMode.VELOCITY:
                return "external profile → observer → zero-bias residual → velocity assist → PI → PWM clamp → plant"
        if c.objective == Objective.SECONDARY_GOAL_ASSIST and c.mode in {
            OperatingMode.POSITION,
            OperatingMode.EXTENDED_POSITION,
            OperatingMode.CURRENT_POSITION,
        }:
            suffix = "Goal Current ceiling" if c.mode == OperatingMode.CURRENT_POSITION else "PWM clamp"
            return f"external effort -> intent velocity -> q_goal + lead -> position PID -> {suffix} -> plant"
        if c.objective == Objective.TORQUE_ASSIST and c.mode in {
            OperatingMode.POSITION,
            OperatingMode.EXTENDED_POSITION,
            OperatingMode.CURRENT_POSITION,
        }:
            suffix = "Goal Current clamp" if c.mode == OperatingMode.CURRENT_POSITION else "PWM clamp"
            return f"external effort -> driven assist -> moving target -> position error -> PID -> {suffix} -> plant"
        if c.mode in {
            OperatingMode.POSITION,
            OperatingMode.EXTENDED_POSITION,
            OperatingMode.CURRENT_POSITION,
        }:
            suffix = " → Goal Current clamp" if c.mode == OperatingMode.CURRENT_POSITION else " → PWM clamp"
            return f"target/profile → position error → PID + feedforward{suffix} → plant"
        if c.mode == OperatingMode.VELOCITY:
            return "velocity reference → velocity error → PI → PWM clamp → plant"
        if c.mode == OperatingMode.CURRENT:
            return "signed current command → current/torque conversion → plant"
        return "signed PWM command → PWM clamp → plant"

    def _trajectory_reference(self) -> tuple[float, float]:
        c = self.config
        if c.objective == Objective.POSITION_TRAJECTORY:
            if c.duration_s <= 0:
                alpha = 1.0
            else:
                alpha = _clamp(self.time_s / c.duration_s, 0.0, 1.0)
            q = c.start_position_deg + alpha * (c.end_position_deg - c.start_position_deg)
            v = (c.end_position_deg - c.start_position_deg) / max(c.duration_s, 1e-6)
            return _clamp(q, c.min_position_deg, c.max_position_deg), v if alpha < 1.0 else 0.0
        return c.start_position_deg, 0.0

    def _external_torque_mean(self, time_s: float | None = None) -> float:
        """Evaluate the configured piecewise-linear external torque mean."""

        c = self.config
        points = sorted(
            ((float(t), float(value)) for t, value in c.external_torque_schedule),
            key=lambda item: item[0],
        )
        if not points:
            return c.external_torque_nm
        t = self.time_s if time_s is None else time_s
        period = max(0.0, c.external_torque_profile_period_s)
        if period > 0.0:
            t = t % period
        if t <= points[0][0]:
            profile_value = points[0][1]
        elif t >= points[-1][0]:
            profile_value = points[-1][1]
        else:
            profile_value = points[-1][1]
            for (t0, y0), (t1, y1) in zip(points, points[1:]):
                if t0 <= t <= t1:
                    alpha = (t - t0) / max(t1 - t0, 1e-9)
                    profile_value = y0 + alpha * (y1 - y0)
                    break
        return c.external_torque_nm + profile_value

    def _assist_objective_selected(self) -> bool:
        return self.config.objective in {
            Objective.TORQUE_ASSIST,
            Objective.EXTERNAL_EFFORT_ASSIST,
        }

    def _secondary_goal_assist_selected(self) -> bool:
        return self.config.objective == Objective.SECONDARY_GOAL_ASSIST

    def _assist_command(self) -> float:
        c = self.config
        if c.objective in {
            Objective.TORQUE_ASSIST,
            Objective.SECONDARY_GOAL_ASSIST,
        }:
            # The configured value is a ceiling.  The instantaneous command
            # is driven by the debiased motor-side external-effort estimate,
            # so it can turn on, vary, and return to zero with the sensed
            # effort instead of becoming a permanent fixed torque bias.
            requested = c.external_assist_gain * self.debiased_external_torque_nm
            return _clamp(
                requested,
                -abs(c.assist_torque_nm),
                abs(c.assist_torque_nm),
            )
        if c.objective == Objective.EXTERNAL_EFFORT_ASSIST:
            return self.assist_torque_command_nm
        return 0.0

    def _update_external_observer(
        self,
        dt: float,
        actuator_torque_nm: float,
        endstop_torque_nm: float,
        previous_velocity_deg_s: float,
    ) -> None:
        """Estimate external torque from motor-side state and actuator output.

        The simulator knows the actual external torque for the plant, but the
        observer deliberately does not use it. It reconstructs the residual
        from measured velocity change, modeled inertia/friction, actuator
        torque, and the modeled endstop reaction.
        """

        c = self.config
        measured_acceleration = (self.velocity_deg_s - previous_velocity_deg_s) / max(dt, 1e-9)
        friction = c.viscous_damping_nm_s_per_deg * self.velocity_deg_s
        if abs(self.velocity_deg_s) > 1e-4:
            friction += c.coulomb_friction_nm * _sign(self.velocity_deg_s)
        raw_estimate = (
            c.inertia_kg_m2 * measured_acceleration
            + friction
            - actuator_torque_nm
            - endstop_torque_nm
        )
        alpha = _clamp(dt / max(c.external_estimator_tau_s + dt, 1e-9), 0.0, 1.0)
        self.external_torque_estimate_nm += alpha * (
            raw_estimate - self.external_torque_estimate_nm
        )

        if c.collection_epoch_s > 0.0 and self.time_s <= c.collection_epoch_s:
            self.collection_samples += 1
            self.external_torque_bias_nm += (
                raw_estimate - self.external_torque_bias_nm
            ) / self.collection_samples
        elif c.collection_epoch_s <= 0.0:
            self.external_torque_bias_nm = 0.0

        self.debiased_external_torque_nm = (
            self.external_torque_estimate_nm - self.external_torque_bias_nm
        )
        self._update_external_assist_command(dt)

    def _update_external_assist_command(self, dt: float) -> None:
        c = self.config
        if c.objective != Objective.EXTERNAL_EFFORT_ASSIST:
            if not self._secondary_goal_assist_selected():
                self.assist_torque_command_nm = 0.0
                self.assist_active = False
                self.assist_detection_timer_s = 0.0
                self.assist_release_timer_s = 0.0
            return
        if c.collection_epoch_s > 0.0 and self.time_s <= c.collection_epoch_s:
            self.assist_torque_command_nm = 0.0
            self.assist_active = False
            self.assist_detection_timer_s = 0.0
            return

        magnitude = abs(self.debiased_external_torque_nm)
        threshold = max(0.0, c.assist_detect_threshold_nm)
        if magnitude >= threshold:
            self.assist_detection_timer_s += dt
            self.assist_release_timer_s = 0.0
        else:
            # A below-threshold residual is not evidence toward a future
            # activation.  Reset the activation dwell timer so a released
            # assist cannot immediately re-trigger from stale dwell time.
            self.assist_detection_timer_s = 0.0
            if self.assist_active and magnitude <= threshold * 0.5:
                self.assist_release_timer_s += dt
            else:
                self.assist_release_timer_s = 0.0

        if not self.assist_active and self.assist_detection_timer_s >= max(0.0, c.assist_detect_delay_s):
            self.assist_active = True
        if self.assist_active and self.assist_release_timer_s >= max(0.0, c.assist_detect_delay_s):
            self.assist_active = False
            self.assist_detection_timer_s = 0.0
            self.assist_release_timer_s = 0.0

        if self.assist_active:
            excess = _sign(self.debiased_external_torque_nm) * max(
                0.0, magnitude - threshold
            )
            desired = _clamp(
                c.external_assist_gain * excess,
                -abs(c.external_assist_max_torque_nm),
                abs(c.external_assist_max_torque_nm),
            )
        else:
            desired = 0.0
        ramp_alpha = _clamp(dt / max(c.external_assist_ramp_s, dt), 0.0, 1.0)
        self.assist_torque_command_nm += ramp_alpha * (
            desired - self.assist_torque_command_nm
        )

    def _update_secondary_goal_assist(self, dt: float) -> None:
        """Update the proposed compliant secondary-goal assist state.

        The sensed effort creates a sustained intent velocity.  That velocity
        integrates a persistent secondary goal while a bounded lead creates
        enough position error for the compliant position loop to move.  If
        the motor cannot make progress, the goal is captured at the achieved
        position and the lead decays instead of continuing to push.
        """

        c = self.config
        position_mode = c.mode in {
            OperatingMode.POSITION,
            OperatingMode.EXTENDED_POSITION,
            OperatingMode.CURRENT_POSITION,
        }
        if not self._secondary_goal_assist_selected() or not position_mode:
            self.assist_active = False
            return

        collection_complete = (
            c.collection_epoch_s <= 0.0 or self.time_s >= c.collection_epoch_s
        )
        intent_torque = self._assist_command() if collection_complete else 0.0
        self.assist_torque_command_nm = intent_torque
        magnitude = abs(intent_torque)
        threshold = max(0.0, c.assist_detect_threshold_nm)
        detect_delay = max(0.0, c.assist_detect_delay_s)

        if self.secondary_rearm_blocked:
            if magnitude <= threshold * 0.5:
                self.secondary_release_timer_s += dt
                if self.secondary_release_timer_s >= detect_delay:
                    self.secondary_rearm_blocked = False
                    self.secondary_release_capture_position_deg = None
                    self.secondary_release_timer_s = 0.0
            else:
                self.secondary_release_timer_s = 0.0
            self.secondary_assist_active = False
            self.secondary_goal_velocity_deg_s = 0.0
            self.secondary_stall_timer_s = 0.0
            self.secondary_goal_lead_deg = max(
                0.0,
                self.secondary_goal_lead_deg
                - abs(c.secondary_goal_lead_rate_deg_s) * dt,
            )
            if self.secondary_release_capture_position_deg is not None:
                self.secondary_goal_position_deg = _clamp(
                    self.secondary_release_capture_position_deg
                    - self.secondary_goal_direction * self.secondary_goal_lead_deg,
                    c.min_position_deg,
                    c.max_position_deg,
                )
            self.assist_active = False
            return

        if magnitude >= threshold:
            self.secondary_intent_timer_s += dt
            self.secondary_release_timer_s = 0.0
        else:
            self.secondary_intent_timer_s = 0.0
            if self.secondary_assist_active:
                self.secondary_release_timer_s += dt
            else:
                self.secondary_release_timer_s = 0.0

        if (
            not self.secondary_assist_active
            and self.secondary_intent_timer_s >= detect_delay
        ):
            self.secondary_assist_active = True
        if (
            self.secondary_assist_active
            and self.secondary_release_timer_s >= detect_delay
        ):
            self.secondary_assist_active = False

        if self.secondary_assist_active:
            self.secondary_goal_velocity_deg_s = _clamp(
                c.assist_velocity_deg_s_per_nm * intent_torque,
                -abs(c.assist_velocity_deg_s_per_nm * abs(c.assist_torque_nm)),
                abs(c.assist_velocity_deg_s_per_nm * abs(c.assist_torque_nm)),
            )
            direction = _sign(self.secondary_goal_velocity_deg_s)
            if direction != 0.0:
                self.secondary_goal_direction = int(direction)
                progress_velocity = direction * self.velocity_deg_s
                if progress_velocity < max(0.0, c.secondary_goal_stall_velocity_deg_s):
                    self.secondary_stall_timer_s += dt
                else:
                    self.secondary_stall_timer_s = 0.0
                self.secondary_goal_position_deg = _clamp(
                    self.secondary_goal_position_deg
                    + self.secondary_goal_velocity_deg_s * dt,
                    c.min_position_deg,
                    c.max_position_deg,
                )
                self.secondary_goal_lead_deg = min(
                    abs(c.secondary_goal_lead_deg),
                    self.secondary_goal_lead_deg
                    + abs(c.secondary_goal_lead_rate_deg_s) * dt,
                )
                if self.secondary_stall_timer_s >= max(0.0, c.secondary_goal_stall_delay_s):
                    self.secondary_assist_active = False
                    self.secondary_rearm_blocked = True
                    self.secondary_goal_position_deg = self.position_deg
                    self.secondary_goal_velocity_deg_s = 0.0
                    self.secondary_release_capture_position_deg = self.position_deg
                    self.secondary_stall_timer_s = 0.0
            else:
                self.secondary_goal_velocity_deg_s = 0.0
        else:
            self.secondary_goal_velocity_deg_s = 0.0
            self.secondary_stall_timer_s = 0.0
            self.secondary_goal_lead_deg = max(
                0.0,
                self.secondary_goal_lead_deg
                - abs(c.secondary_goal_lead_rate_deg_s) * dt,
            )

        self.assist_active = self.secondary_assist_active

    def effective_goal_current_ma(self) -> float:
        """Return the effective Goal Current driven by the selected objective.

        ``goal_current_limit_ma`` is the user-configured ceiling. In
        current-based position mode, a torque-assist objective supplies the
        effective current magnitude while the ceiling remains a safety limit.
        Goal Current is represented as a magnitude here; direction is visible
        in the target shift and applied torque signals.
        """

        c = self.config
        ceiling = max(0.0, c.goal_current_limit_ma)
        if c.mode == OperatingMode.CURRENT_POSITION:
            if c.objective == Objective.SECONDARY_GOAL_ASSIST:
                return ceiling
            if c.objective == Objective.TORQUE_ASSIST:
                requested = abs(self._assist_command()) / TORQUE_CONSTANT_NM_PER_MA
                return min(requested, ceiling)
            return ceiling
        if c.mode == OperatingMode.CURRENT:
            requested = c.direct_current_ma
            if self._assist_objective_selected():
                requested += self._assist_command() / TORQUE_CONSTANT_NM_PER_MA
            return min(abs(requested), ceiling)
        return 0.0

    def _position_reference(self) -> tuple[float, float]:
        c = self.config
        if self._secondary_goal_assist_selected() and c.mode in {
            OperatingMode.POSITION,
            OperatingMode.EXTENDED_POSITION,
            OperatingMode.CURRENT_POSITION,
        }:
            lead_direction = self.secondary_goal_direction
            q = self.secondary_goal_position_deg + lead_direction * self.secondary_goal_lead_deg
            return _clamp(q, c.min_position_deg, c.max_position_deg), self.secondary_goal_velocity_deg_s

        q, v = self._trajectory_reference()
        if self._assist_objective_selected() and c.mode in {
            OperatingMode.POSITION,
            OperatingMode.EXTENDED_POSITION,
            OperatingMode.CURRENT_POSITION,
        }:
            # A real XC330 current-position controller has no signed torque
            # bias register.  Assistance is therefore represented by a moving
            # position target whose rate is driven by the sensed assist torque.
            q += c.assist_target_bias_deg_per_nm * self._assist_command()
            q += self.assist_position_offset_deg
            v += c.assist_velocity_deg_s_per_nm * self._assist_command()
        return _clamp(q, c.min_position_deg, c.max_position_deg), v

    def _endstop_reaction(self) -> tuple[float, float]:
        c = self.config
        if not c.endstop_enabled:
            return 0.0, 0.0
        penetration = c.endstop_side * (self.position_deg - c.endstop_position_deg)
        if penetration <= 0.0:
            return 0.0, 0.0
        outward_speed = max(0.0, c.endstop_side * self.velocity_deg_s)
        raw = (
            c.endstop_stiffness_nm_per_deg * penetration
            + c.endstop_damping_nm_s_per_deg * outward_speed
        )
        reaction = -c.endstop_side * _clamp(raw, 0.0, c.endstop_max_torque_nm)
        return reaction, penetration

    def _position_controller(self, dt: float) -> tuple[float, float, float, float]:
        c = self.config
        q_ref, v_ref = self._position_reference()
        error = q_ref - self.position_deg
        self.integral_error_deg_s = _clamp(
            self.integral_error_deg_s + error * dt, -100.0, 100.0
        )

        # The scales preserve the documented raw register conversions while
        # mapping them into approachable simulation torque units.
        kp = (c.position_p_raw / 128.0) * 0.020
        ki = (c.position_i_raw / 65536.0) * 0.20
        kd = (c.position_d_raw / 16.0) * 0.010
        pid = kp * error + ki * self.integral_error_deg_s + kd * (v_ref - self.velocity_deg_s)

        if c.mode == OperatingMode.CURRENT_POSITION:
            cap = self.effective_goal_current_ma() * TORQUE_CONSTANT_NM_PER_MA
        else:
            cap = max(0.0, c.pwm_limit_percent / 100.0) * c.pwm_torque_at_100_percent_nm
        return q_ref, v_ref, pid, _clamp(pid, -cap, cap)

    def _velocity_controller(self, dt: float) -> tuple[float, float, float, float]:
        c = self.config
        velocity_ref = c.direct_velocity_deg_s
        if self._assist_objective_selected():
            velocity_ref += c.assist_velocity_deg_s_per_nm * self._assist_command()
        velocity_error = velocity_ref - self.velocity_deg_s
        self.integral_error_deg_s = _clamp(
            self.integral_error_deg_s + velocity_error * dt, -100.0, 100.0
        )
        kp = (c.velocity_p_raw / 128.0) * 0.040
        ki = (c.velocity_i_raw / 65536.0) * 0.015
        pid = kp * velocity_error + ki * self.integral_error_deg_s
        cap = max(0.0, c.pwm_limit_percent / 100.0) * c.pwm_torque_at_100_percent_nm
        return self.position_deg, velocity_ref, pid, _clamp(pid, -cap, cap)

    def _update_assist_position_target(self, dt: float) -> None:
        c = self.config
        if c.mode not in {
            OperatingMode.POSITION,
            OperatingMode.EXTENDED_POSITION,
            OperatingMode.CURRENT_POSITION,
        } or not self._assist_objective_selected():
            return
        target_rate_deg_s = c.assist_velocity_deg_s_per_nm * self._assist_command()
        direction = _sign(target_rate_deg_s)
        if direction == 0.0:
            self.assist_motion_started = False
            self.assist_target_direction = 0
            return
        if direction != self.assist_target_direction:
            self.assist_motion_started = False
            self.assist_target_direction = int(direction)

        velocity_along_target = direction * self.velocity_deg_s
        motion_deadband_deg_s = 1.0
        if velocity_along_target > motion_deadband_deg_s:
            self.assist_motion_started = True
        if self.assist_motion_started:
            # Once motion has started, let measured velocity determine how
            # quickly the position target advances.  A soft endstop brings
            # velocity back toward zero, which naturally pauses qref without
            # imposing the endstop position on the target itself.  The
            # deadband prevents low-speed numerical settling / compliance from
            # making the reference creep after the motor has effectively
            # stopped.
            target_rate_deg_s = direction * min(
                abs(target_rate_deg_s),
                max(0.0, velocity_along_target - motion_deadband_deg_s),
            )
        span = max(c.max_position_deg - c.min_position_deg, 1.0)
        self.assist_position_offset_deg = _clamp(
            self.assist_position_offset_deg + target_rate_deg_s * dt,
            -span,
            span,
        )

    def _controller_output(self, dt: float) -> tuple[float, float, float, float]:
        c = self.config
        if c.mode in {
            OperatingMode.POSITION,
            OperatingMode.EXTENDED_POSITION,
            OperatingMode.CURRENT_POSITION,
        }:
            return self._position_controller(dt)
        if c.mode == OperatingMode.VELOCITY:
            return self._velocity_controller(dt)
        if c.mode == OperatingMode.CURRENT:
            current_ma = c.direct_current_ma
            if self._assist_objective_selected():
                current_ma += self._assist_command() / TORQUE_CONSTANT_NM_PER_MA
            torque = current_ma * TORQUE_CONSTANT_NM_PER_MA
            cap = max(0.0, c.goal_current_limit_ma) * TORQUE_CONSTANT_NM_PER_MA
            return self.position_deg, 0.0, torque, _clamp(torque, -cap, cap)

        pwm = c.direct_pwm_percent
        if self._assist_objective_selected():
            pwm += (self._assist_command() / max(c.pwm_torque_at_100_percent_nm, 1e-6)) * 100.0
        pwm = _clamp(pwm, -abs(c.pwm_limit_percent), abs(c.pwm_limit_percent))
        torque = pwm / 100.0 * c.pwm_torque_at_100_percent_nm
        return self.position_deg, 0.0, torque, torque

    def step(self, dt: float = 0.02) -> SimulationSnapshot:
        """Advance the model by ``dt`` seconds and return the new state."""

        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be a positive finite number")
        c = self.config
        previous_velocity = self.velocity_deg_s
        external_torque_mean = self._external_torque_mean(self.time_s)
        external_torque = external_torque_mean + self.random.gauss(
            0.0, max(0.0, c.external_torque_noise_std_nm)
        )
        self.assist_torque_command_nm = self._assist_command()
        self._update_secondary_goal_assist(dt)
        self._update_assist_position_target(dt)
        q_ref, v_ref, pid_torque, actuator_torque = self._controller_output(dt)
        endstop_torque, penetration = self._endstop_reaction()

        friction = c.viscous_damping_nm_s_per_deg * self.velocity_deg_s
        if abs(self.velocity_deg_s) > 1e-4:
            friction += c.coulomb_friction_nm * _sign(self.velocity_deg_s)
        net_torque = actuator_torque + external_torque + endstop_torque - friction
        acceleration = net_torque / max(c.inertia_kg_m2, 1e-6)
        self.velocity_deg_s += acceleration * dt
        self.position_deg += self.velocity_deg_s * dt

        if self.position_deg < c.min_position_deg:
            self.position_deg = c.min_position_deg
            self.velocity_deg_s = max(0.0, self.velocity_deg_s)
        elif self.position_deg > c.max_position_deg:
            self.position_deg = c.max_position_deg
            self.velocity_deg_s = min(0.0, self.velocity_deg_s)

        self.time_s += dt
        self._update_external_observer(
            dt,
            actuator_torque,
            endstop_torque,
            previous_velocity,
        )
        if c.objective == Objective.TORQUE_ASSIST:
            self.assist_torque_command_nm = self._assist_command()
        error = q_ref - self.position_deg
        current_ma = actuator_torque / TORQUE_CONSTANT_NM_PER_MA
        if c.mode == OperatingMode.PWM:
            pwm = actuator_torque / max(c.pwm_torque_at_100_percent_nm, 1e-6) * 100.0
        else:
            pwm = _clamp(
                actuator_torque / max(c.pwm_torque_at_100_percent_nm, 1e-6) * 100.0,
                -100.0,
                100.0,
            )
        snapshot = SimulationSnapshot(
            time_s=self.time_s,
            position_deg=self.position_deg,
            velocity_deg_s=self.velocity_deg_s,
            reference_position_deg=q_ref,
            reference_velocity_deg_s=v_ref,
            position_error_deg=error,
            actuator_torque_nm=actuator_torque,
            external_torque_nm=external_torque,
            endstop_torque_nm=endstop_torque,
            net_torque_nm=net_torque,
            current_ma=current_ma,
            goal_current_ma=self.effective_goal_current_ma(),
            pwm_percent=pwm,
            pid_torque_nm=pid_torque,
            clamped_torque_nm=actuator_torque,
            integral_error_deg_s=self.integral_error_deg_s,
            endstop_penetration_deg=penetration,
            external_torque_mean_nm=external_torque_mean,
            external_torque_estimate_nm=self.external_torque_estimate_nm,
            external_torque_bias_nm=self.external_torque_bias_nm,
            debiased_external_torque_nm=self.debiased_external_torque_nm,
            assist_active=self.assist_active,
            assist_torque_command_nm=self.assist_torque_command_nm,
            collection_progress=(
                1.0
                if c.collection_epoch_s <= 0.0
                else _clamp(self.time_s / c.collection_epoch_s, 0.0, 1.0)
            ),
            secondary_goal_position_deg=self.secondary_goal_position_deg,
            secondary_goal_velocity_deg_s=self.secondary_goal_velocity_deg_s,
            secondary_goal_lead_deg=self.secondary_goal_lead_deg,
        )
        self.reference_position_deg = q_ref
        self.reference_velocity_deg_s = v_ref
        self.previous_snapshot = snapshot
        return snapshot
