import math

import pytest

from nml_hand_exo.interface._hand_exo import HandExo, ProtocolResponseError


def _recording_exo(response=None):
    exo = HandExo.__new__(HandExo)
    exo.commands = []
    exo._firmware_version = (0, 6, 2)

    def send(command):
        exo.commands.append(command)

    exo.send_command = send
    exo._receive = lambda **_kwargs: (
        response
        if response is not None
        else (
            "OK: hold_position"
            if exo.commands[-1].startswith("hold_position:")
            else "OK: release_hold"
        )
    )
    return exo


def test_hold_position_uses_explicit_id_and_relative_angle():
    exo = _recording_exo()

    exo.hold_motor_position(14, 22.5)

    assert exo.commands == ["hold_position:14:22.5"]


def test_hold_position_can_request_per_hold_current():
    exo = _recording_exo("OK: hold_position id=14 angle=22.500 current_mA=80")

    response = exo.hold_motor_position(14, 22.5, 80)

    assert exo.commands == ["hold_position:14:22.5:80"]
    assert "current_mA=80" in response


@pytest.mark.parametrize("current", [0, -1, math.nan, math.inf])
def test_hold_position_rejects_invalid_per_hold_current(current):
    exo = _recording_exo()

    with pytest.raises(ValueError):
        exo.hold_motor_position(14, 22.5, current)

    assert exo.commands == []


def test_per_hold_current_releases_if_ack_lacks_applied_current():
    exo = _recording_exo("OK: hold_position id=14 angle=22.500")

    with pytest.raises(ProtocolResponseError, match="current_mA=<applied>"):
        exo.hold_motor_position(14, 22.5, 80)

    assert exo.commands == ["hold_position:14:22.5:80", "release_hold:14"]


def test_release_hold_uses_explicit_id():
    exo = _recording_exo()

    exo.release_motor_hold(14)

    assert exo.commands == ["release_hold:14"]


@pytest.mark.parametrize("angle", [math.nan, math.inf, -math.inf])
def test_hold_position_rejects_non_finite_angle(angle):
    exo = _recording_exo()

    with pytest.raises(ValueError):
        exo.hold_motor_position(14, angle)

    assert exo.commands == []


def test_hold_position_propagates_firmware_error():
    exo = _recording_exo("ERROR: position hold requires global velocity/current mode")

    with pytest.raises(RuntimeError):
        exo.hold_motor_position(14, 10)


def test_hold_position_rejects_pre_062_firmware():
    exo = _recording_exo()
    exo._firmware_version = (0, 6, 1)

    with pytest.raises(RuntimeError, match="firmware >= 0.6.2"):
        exo.hold_motor_position(14, 10)

    assert exo.commands == []
