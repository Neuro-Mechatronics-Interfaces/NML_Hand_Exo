from __future__ import annotations

from pathlib import Path
import sys


EXAMPLE_DIR = Path(__file__).resolve().parents[1] / "10_dxl_ctrl_sim_single_motor"
sys.path.insert(0, str(EXAMPLE_DIR))

from control_model import (  # noqa: E402
    Objective,
    OperatingMode,
    SimulationConfig,
    SingleMotorSimulation,
    TORQUE_CONSTANT_NM_PER_MA,
)


def test_operating_mode_values_match_dynamixel_control_table():
    assert OperatingMode.CURRENT.raw_value == 0
    assert OperatingMode.VELOCITY.raw_value == 1
    assert OperatingMode.POSITION.raw_value == 3
    assert OperatingMode.EXTENDED_POSITION.raw_value == 4
    assert OperatingMode.CURRENT_POSITION.raw_value == 5
    assert OperatingMode.PWM.raw_value == 16


def test_position_trajectory_reaches_endpoint():
    sim = SingleMotorSimulation(
        SimulationConfig(
            mode=OperatingMode.POSITION,
            objective=Objective.POSITION_TRAJECTORY,
            start_position_deg=20,
            end_position_deg=120,
            duration_s=1,
            position_p_raw=0,
            position_d_raw=0,
            external_torque_nm=0,
        )
    )
    # The first integration step uses the reference at t=0; one extra sample
    # observes the endpoint at t=duration.
    for _ in range(51):
        snap = sim.step(0.02)
    assert snap.reference_position_deg == 120


def test_current_position_effort_is_capped_by_goal_current():
    sim = SingleMotorSimulation(
        SimulationConfig(
            mode=OperatingMode.CURRENT_POSITION,
            objective=Objective.POSITION_TRAJECTORY,
            start_position_deg=0,
            end_position_deg=360,
            duration_s=1,
            position_p_raw=16383,
            goal_current_limit_ma=40,
        )
    )
    snap = sim.step(0.02)
    assert abs(snap.actuator_torque_nm) <= 40 * TORQUE_CONSTANT_NM_PER_MA + 1e-9


def test_secondary_goal_assist_is_the_default_and_keeps_current_ceiling_independent():
    config = SimulationConfig(
        external_torque_nm=0.2,
        external_torque_noise_std_nm=0.0,
        collection_epoch_s=0.0,
        assist_torque_nm=0.25,
        goal_current_limit_ma=300,
    )
    assert config.objective == Objective.SECONDARY_GOAL_ASSIST
    sim = SingleMotorSimulation(config)
    for _ in range(40):
        snapshot = sim.step(0.02)
    assert snapshot.assist_active
    assert snapshot.secondary_goal_position_deg > config.start_position_deg
    assert snapshot.goal_current_ma == 300


def test_secondary_goal_assist_releases_lead_after_stalled_endstop_progress():
    sim = SingleMotorSimulation(
        SimulationConfig(
            mode=OperatingMode.CURRENT_POSITION,
            objective=Objective.SECONDARY_GOAL_ASSIST,
            start_position_deg=100,
            external_torque_nm=0.2,
            external_torque_noise_std_nm=0.0,
            collection_epoch_s=0.0,
            assist_torque_nm=0.25,
            secondary_goal_lead_deg=8.0,
            secondary_goal_stall_delay_s=0.5,
            endstop_enabled=True,
            endstop_position_deg=110,
            endstop_side=1,
            endstop_max_torque_nm=0.8,
            endstop_stiffness_nm_per_deg=0.2,
        )
    )
    references = []
    for _ in range(500):
        snapshot = sim.step(0.02)
        references.append(snapshot.reference_position_deg)
    assert max(references) > 110.0
    assert not snapshot.assist_active
    assert snapshot.secondary_goal_lead_deg == 0.0
    assert snapshot.reference_position_deg == snapshot.secondary_goal_position_deg


def test_position_mode_ignores_hidden_direct_current_and_zero_disturbance_holds():
    sim = SingleMotorSimulation(
        SimulationConfig(
            mode=OperatingMode.CURRENT_POSITION,
            objective=Objective.HOLD,
            start_position_deg=100,
            direct_current_ma=910,
            external_torque_nm=0.0,
            external_torque_noise_std_nm=0.0,
            external_torque_schedule=((0.0, 0.0), (5.0, 0.0)),
            position_p_raw=900,
        )
    )
    for _ in range(250):
        snap = sim.step(0.02)
    assert snap.position_deg == 100
    assert snap.actuator_torque_nm == 0


def test_torque_assist_drives_effective_goal_current_below_ceiling():
    config = SimulationConfig(
        mode=OperatingMode.CURRENT_POSITION,
        objective=Objective.TORQUE_ASSIST,
        assist_torque_nm=0.2,
        external_torque_nm=-2.0,
        goal_current_limit_ma=300,
    )
    sim = SingleMotorSimulation(config)
    assert sim.previous_snapshot.goal_current_ma == 0
    sim.step(0.02)
    snap = sim.step(0.02)
    assert 0 < snap.goal_current_ma <= 0.2 / TORQUE_CONSTANT_NM_PER_MA

    prior_goal = snap.goal_current_ma
    config.assist_torque_nm = 0.5
    sim.set_config(config)
    assert sim.effective_goal_current_ma() == 300
    sim.step(0.02)
    snap = sim.step(0.02)
    assert snap.goal_current_ma > prior_goal
    assert snap.goal_current_ma <= 0.5 / TORQUE_CONSTANT_NM_PER_MA


def test_torque_assist_changes_current_position_torque_cap():
    sim = SingleMotorSimulation(
        SimulationConfig(
            mode=OperatingMode.CURRENT_POSITION,
            objective=Objective.TORQUE_ASSIST,
            assist_torque_nm=0.1,
            external_torque_nm=-2.0,
            position_p_raw=16383,
            goal_current_limit_ma=300,
        )
    )
    sim.step(0.02)
    snap = sim.step(0.02)
    assert abs(snap.actuator_torque_nm) <= 0.1 + 1e-9
    assert snap.goal_current_ma == 0.1 / TORQUE_CONSTANT_NM_PER_MA


def test_current_position_assist_advances_the_position_target():
    sim = SingleMotorSimulation(
        SimulationConfig(
            mode=OperatingMode.CURRENT_POSITION,
            objective=Objective.TORQUE_ASSIST,
            start_position_deg=100,
            external_torque_nm=0.2,
            external_torque_noise_std_nm=0.0,
            collection_epoch_s=0.0,
            assist_torque_nm=0.25,
            assist_velocity_deg_s_per_nm=80.0,
        )
    )
    sim.step(0.02)
    snapshot = sim.step(0.02)
    assert snapshot.assist_torque_command_nm > 0.0
    assert snapshot.reference_position_deg > 100.0
    assert snapshot.actuator_torque_nm > 0.0


def test_assist_target_stops_at_soft_endstop():
    sim = SingleMotorSimulation(
        SimulationConfig(
            mode=OperatingMode.CURRENT_POSITION,
            objective=Objective.TORQUE_ASSIST,
            start_position_deg=100,
            external_torque_nm=0.2,
            external_torque_noise_std_nm=0.0,
            collection_epoch_s=0.0,
            assist_torque_nm=0.25,
            assist_velocity_deg_s_per_nm=80.0,
            endstop_enabled=True,
            endstop_position_deg=110,
            endstop_side=1,
            endstop_max_torque_nm=0.8,
            endstop_stiffness_nm_per_deg=0.2,
        )
    )
    references = []
    for _ in range(500):
        snapshot = sim.step(0.02)
        references.append(snapshot.reference_position_deg)
    assert max(references) > 110.0
    assert references[-1] == references[-20]


def test_current_mode_applies_signed_assist_torque():
    sim = SingleMotorSimulation(
        SimulationConfig(
            mode=OperatingMode.CURRENT,
            objective=Objective.TORQUE_ASSIST,
            assist_torque_nm=0.2,
            direct_current_ma=0,
            goal_current_limit_ma=300,
            external_torque_nm=0.2,
            external_torque_noise_std_nm=0.0,
            collection_epoch_s=0.0,
        )
    )
    sim.step(0.02)
    snap = sim.step(0.02)
    assert snap.actuator_torque_nm > 0
    assert snap.current_ma > 0


def test_endstop_reaction_opposes_upper_penetration():
    sim = SingleMotorSimulation(
        SimulationConfig(
            mode=OperatingMode.CURRENT,
            objective=Objective.CURRENT_COMMAND,
            start_position_deg=100,
            direct_current_ma=0,
            endstop_enabled=True,
            endstop_position_deg=90,
            endstop_side=1,
            endstop_max_torque_nm=0.5,
        )
    )
    sim.position_deg = 100
    snap = sim.step(0.02)
    assert snap.endstop_penetration_deg > 0
    assert snap.endstop_torque_nm < 0


def test_external_torque_profile_interpolates_and_loops():
    sim = SingleMotorSimulation(
        SimulationConfig(
            external_torque_schedule=((0.0, 0.0), (1.0, 0.2), (2.0, -0.2)),
            external_torque_profile_period_s=2.0,
        )
    )
    assert sim._external_torque_mean(0.5) == 0.1
    assert sim._external_torque_mean(1.5) == 0.0
    assert sim._external_torque_mean(2.5) == 0.1


def test_external_effort_assist_zeroes_bias_and_requires_sustained_shift():
    sim = SingleMotorSimulation(
        SimulationConfig(
            mode=OperatingMode.CURRENT,
            objective=Objective.EXTERNAL_EFFORT_ASSIST,
            external_torque_noise_std_nm=0.0,
            external_torque_schedule=((0.0, 0.0), (1.0, 0.0), (1.5, 0.2), (3.0, 0.2)),
            collection_epoch_s=1.0,
            external_estimator_tau_s=0.05,
            assist_detect_threshold_nm=0.05,
            assist_detect_delay_s=0.2,
            external_assist_gain=1.0,
            external_assist_max_torque_nm=0.3,
        )
    )
    before_shift = None
    for _ in range(50):
        before_shift = sim.step(0.02)
    assert before_shift is not None
    assert not before_shift.assist_active
    assert abs(before_shift.debiased_external_torque_nm) < 0.03

    after_shift = before_shift
    for _ in range(50):
        after_shift = sim.step(0.02)
    assert after_shift.assist_active
    assert after_shift.assist_torque_command_nm > 0.0
    assert after_shift.assist_torque_command_nm <= 0.3
    assert abs(after_shift.external_torque_bias_nm) < 0.03


def test_external_effort_assist_releases_after_schedule_returns_to_zero():
    sim = SingleMotorSimulation(
        SimulationConfig(
            mode=OperatingMode.CURRENT,
            objective=Objective.EXTERNAL_EFFORT_ASSIST,
            direct_current_ma=0.0,
            external_torque_noise_std_nm=0.0,
            external_torque_schedule=(
                (0.0, 0.0),
                (1.0, 0.0),
                (1.5, 0.2),
                (3.0, 0.2),
                (3.5, 0.0),
                (6.0, 0.0),
            ),
            collection_epoch_s=1.0,
            external_estimator_tau_s=0.05,
            assist_detect_threshold_nm=0.05,
            assist_detect_delay_s=0.2,
            external_assist_gain=1.0,
            external_assist_max_torque_nm=0.3,
        )
    )
    active_seen = False
    for _ in range(300):
        snapshot = sim.step(0.02)
        active_seen = active_seen or snapshot.assist_active
    assert active_seen
    assert not snapshot.assist_active
    assert abs(snapshot.assist_torque_command_nm) < 1e-3
