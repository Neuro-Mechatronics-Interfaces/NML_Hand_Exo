from __future__ import annotations

from nml_hand_exo.decoding.shadow_contact import (
    ShadowContactEstimator,
    ShadowContactState,
)
from nml_hand_exo.interface import HandExo
from nml_hand_exo.testing.fake_openrb import FakeOpenRBComm


def test_shadow_api_is_read_only_and_parses_buffered_records():
    comm = FakeOpenRBComm(motor_ids=(15, 16))
    comm.connect()
    comm.control_mode = "velocity"
    comm.currents[15] = 87.0
    comm.angles[15] = 21.5
    comm.velocities[15] = 3.25
    exo = HandExo(comm, send_delay=0)

    exo.configure_shadow_telemetry([15, 16], interval_ms=2)
    exo.start_shadow_telemetry()
    snapshot = exo.get_shadow_telemetry()
    exo.stop_shadow_telemetry()

    assert snapshot["meta"]["enabled"] is True
    assert snapshot["meta"]["count"] == 2
    assert snapshot["records"][15]["current"] == 87.0
    assert snapshot["records"][15]["angle"] == 21.5
    assert snapshot["records"][15]["velocity_deg_s"] == 3.25
    assert {command.split(":", 1)[0] for command in comm.sent} == {
        "shadow_config", "shadow_start", "shadow_status", "shadow_stop"
    }
    assert comm.enabled == {15: False, 16: False}


def test_shadow_api_rejects_duplicate_or_non_explicit_ids():
    comm = FakeOpenRBComm(motor_ids=(15, 16))
    comm.connect()
    exo = HandExo(comm, send_delay=0)

    for ids in ([15, 15], [], [0], [-1], list(range(1, 11))):
        try:
            exo.configure_shadow_telemetry(ids)
        except ValueError:
            pass
        else:
            raise AssertionError(f"Expected ValueError for IDs {ids}")
    assert comm.sent == []


def test_shadow_estimator_requires_dwell_and_releases_with_hysteresis():
    estimator = ShadowContactEstimator()
    common = dict(
        sample_ms=1_000,
        intent=1.0,
        current_mA=100.0,
        velocity_deg_s=1.0,
        angle_deg=20.0,
        lower_limit_deg=-90.0,
        upper_limit_deg=90.0,
    )
    first = estimator.update(now_ms=1_000, **common)
    settled = estimator.update(now_ms=1_150, **common)
    released = estimator.update(
        now_ms=1_300,
        **{**common, "sample_ms": 1_300, "current_mA": 0.0},
    )

    assert first.state is ShadowContactState.CANDIDATE
    assert settled.state is ShadowContactState.CONTACT
    assert released.state is ShadowContactState.FREE


def test_shadow_estimator_distinguishes_limit_and_stale_samples():
    limit_estimator = ShadowContactEstimator()
    at_limit = limit_estimator.update(
        now_ms=500,
        sample_ms=500,
        intent=1.0,
        current_mA=150.0,
        velocity_deg_s=0.0,
        angle_deg=88.0,
        lower_limit_deg=-90.0,
        upper_limit_deg=90.0,
    )
    stale_estimator = ShadowContactEstimator()
    stale = stale_estimator.update(
        now_ms=1_000,
        sample_ms=500,
        intent=1.0,
        current_mA=150.0,
        velocity_deg_s=0.0,
        angle_deg=0.0,
        lower_limit_deg=-90.0,
        upper_limit_deg=90.0,
    )

    assert at_limit.state is ShadowContactState.LIMIT
    assert at_limit.near_limit is True
    assert stale.state is ShadowContactState.STALE
    assert stale.evidence is False
