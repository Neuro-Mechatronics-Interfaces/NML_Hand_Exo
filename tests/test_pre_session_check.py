from argparse import Namespace

import pytest

from nml_hand_exo.interface import HandExo
from nml_hand_exo.testing import FakeOpenRBComm
from tools.pre_session_check import (
    MOTION_CONFIRMATION,
    build_parser,
    collect_read_only_report,
    exercise_hold,
    exercise_motion,
    validate_args,
)


def _args(**overrides):
    values = dict(
        port="COM5",
        command_port=None,
        telemetry_port=None,
        baud=1_000_000,
        samples=3,
        sample_interval=0.1,
        exercise_motion=None,
        exercise_hold=None,
        motion_rpm=0.5,
        motion_duration=0.2,
        confirm_motion=None,
    )
    values.update(overrides)
    return Namespace(**values)


def test_default_diagnostic_is_read_only():
    args = build_parser().parse_args(["--port", "COM5"])
    validate_args(args)
    assert args.exercise_motion is None
    assert args.exercise_hold is None


def test_motion_requires_exact_confirmation():
    with pytest.raises(ValueError, match="confirm-motion"):
        validate_args(_args(exercise_motion=15))
    validate_args(
        _args(exercise_motion=15, confirm_motion=MOTION_CONFIRMATION)
    )


def test_motion_envelope_is_intentionally_small():
    with pytest.raises(ValueError, match="1 rpm"):
        validate_args(
            _args(
                exercise_motion=15,
                motion_rpm=1.1,
                confirm_motion=MOTION_CONFIRMATION,
            )
        )
    with pytest.raises(ValueError, match="0.05 and 0.5"):
        validate_args(
            _args(
                exercise_motion=15,
                motion_duration=1.0,
                confirm_motion=MOTION_CONFIRMATION,
            )
        )


def test_dual_cdc_requires_both_ports():
    with pytest.raises(ValueError, match="supplied together"):
        validate_args(_args(port=None, command_port="COM5"))


def _fake_exo():
    comm = FakeOpenRBComm()
    comm.connect()
    exo = HandExo(comm, send_delay=0)
    exo._firmware_version = (0, 6, 2)
    return exo, comm


def test_read_only_report_exercises_real_parsers_without_writes_that_move():
    exo, comm = _fake_exo()
    report = collect_read_only_report(exo, samples=3, interval_s=0)

    assert report["status"] == "read-only checks passed"
    assert report["motor_ids"] == list(range(11, 20))
    assert all(command.startswith(("info", "get_")) for command in comm.sent)


def test_fake_motion_and_hold_exercises_restore_disabled_position_state():
    exo, comm = _fake_exo()

    exercise_hold(exo, 14)
    assert comm.control_mode == "position"
    assert not comm.enabled[14]
    assert 14 not in comm.holds

    exercise_motion(exo, 15, rpm=0.5, duration_s=0.05)
    assert comm.control_mode == "position"
    assert not comm.enabled[15]


def test_active_exercise_refuses_when_another_motor_is_enabled():
    exo, comm = _fake_exo()
    comm.enabled[16] = True

    with pytest.raises(RuntimeError, match="other enabled motor IDs"):
        exercise_motion(exo, 15, rpm=0.5, duration_s=0.05)
