from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from nml_hand_exo.applications.hand_exo_gui import HandExoGUI


@pytest.mark.parametrize("version,expected", [("0.8.0", 0), ("unknown", 0),
    ("0.9.0", 1), ("0.9.0-axon-0.3.0", 1), ("1.0.0", 1)])
def test_connection_clock_sync_is_version_gated(version, expected):
    exo = Mock()
    info = {"version": version}
    HandExoGUI._synchronize_device_clock(exo, info)
    assert exo.synchronize_utc.call_count == expected
    assert ("utc_sync" in info) == bool(expected)


def test_sync_failure_propagates_to_connection_error_handler():
    exo = Mock()
    exo.synchronize_utc.side_effect = TimeoutError("missing UTC ack")
    with pytest.raises(TimeoutError):
        HandExoGUI._synchronize_device_clock(exo, {"version": "0.9.0"})


def test_plot_separates_estimated_and_measured_samples_and_gaps():
    measured, estimated = Mock(), Mock()
    gui = SimpleNamespace(_torque_plot=Mock(), _torque_plot_t0=0,
                          _torque_curves={"wrist": measured},
                          _torque_estimated_curves={"wrist": estimated},
                          _torque_history={"wrist": (deque(), deque(), deque())})
    with patch("nml_hand_exo.applications.hand_exo_gui.time.monotonic", return_value=1):
        HandExoGUI._push_torque_plot(gui, {"wrist": 0.1}, {"wrist": "estimated"})
    assert estimated.setData.call_args.args[1] == [0.1]
    assert measured.setData.call_args.args[1] == []
    with patch("nml_hand_exo.applications.hand_exo_gui.time.monotonic", return_value=2):
        HandExoGUI._push_torque_plot(gui, {"wrist": 0.2}, {"wrist": "measured"})
    assert measured.setData.call_args.args[1][-1] == 0.2
    assert estimated.setData.call_args.args[1] == [0.1]
    assert not estimated.setData.call_args.kwargs["connect"].any()


def test_estimated_position_cannot_seed_position_hold():
    gui = SimpleNamespace(_last_telemetry_update_monotonic=1,
                          _buffered_telemetry_meta={"motor_field_sources": {11: {"position": "estimated"}}},
                          _averaged_telemetry_field=lambda field: {11: 20.0})
    with patch("nml_hand_exo.applications.hand_exo_gui.time.monotonic", return_value=1.01):
        assert HandExoGUI._fresh_cached_relative_angle(gui, 11) is None
