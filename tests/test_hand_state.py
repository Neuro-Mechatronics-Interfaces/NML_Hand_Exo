"""Hand visualization consumes the device model; it never predicts a second pose."""
import os
from types import SimpleNamespace
from unittest.mock import Mock, patch, call

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication

from nml_hand_exo.applications._hand_state import motor_flexion_fraction, motor_display_sample
from nml_hand_exo.applications.hand_exo_gui import HandExoGUI, HandSkeletonWidget, SerialWorker
from nml_hand_exo.interface._hand_exo import HandExo
from examples.tests.test_fast_telemetry import FakeFastTelemetryComm


CAL = dict(home=100, limit_min=80, limit_max=200, flip=False)
GUI_MODULE = "nml_hand_exo.applications.hand_exo_gui"


@pytest.mark.parametrize("relative,calibration,expected", [
    (0, CAL, 0), (50, CAL, 0.5), (100, CAL, 1),
    (-20, CAL, 0), (150, CAL, 1),
    # Reversed axis, and the firmware's opposite-span override in either direction.
    (40, dict(home=100, limit_min=20, limit_max=120, flip=True), 0.5),
    (-50, dict(home=100, limit_min=0, limit_max=120, flip=False), 0.5),
    (-50, dict(CAL, flip=True), 0.5),
    # Exactly 2:1 does not override the preferred span.
    (25, dict(home=100, limit_min=0, limit_max=150, flip=False), 0.5),
    # Multi-turn window and clamped origin.
    (1320, dict(home=200, limit_min=-189, limit_max=2840, flip=False), 0.5),
    (40, dict(home=60, limit_min=80, limit_max=120, flip=False), 0.5),
])
def test_normalization_matches_firmware_gesture_axis(relative, calibration, expected):
    assert motor_flexion_fraction(relative, calibration) == pytest.approx(expected)


@pytest.mark.parametrize("relative,calibration", [
    (None, CAL), (float("nan"), CAL), (float("inf"), CAL), (1, {}),
    (1, dict(CAL, home=float("nan"))), (1, dict(CAL, flip="false")),
    (1, dict(CAL, limit_min=200)), (1, dict(CAL, limit_max=80)),
    (1, dict(home=0, limit_min=0, limit_max=1.9, flip=False)),
])
def test_incomplete_or_invalid_calibration_is_unavailable(relative, calibration):
    assert motor_flexion_fraction(relative, calibration) is None


def test_calibration_api_reads_by_id_and_skips_invalid_motor_without_writes():
    exo = HandExo(FakeFastTelemetryComm(b""), auto_connect=False, send_delay=0)
    info = {"motors": {1: {"name": "wrist", "limits": [-189, 2840]},
                       11: {"name": "wrist", "limits": [80, 200]},
                       12: {"limits": [0, 200]}, 13: {"limits": [0, 200]}}}
    with patch.object(exo, "_receive", side_effect=[
        "Motor 1: {name: wrist, home: 200}\nMotor 11: {name: wrist, home: 100}\n"
        "Motor 12: {home: nan}\nMotor 13: {home: 100}",
        "Motor 1: {name: wrist, flip: false}\nMotor 11: {name: wrist, flip: true}\n"
        "Motor 12: {flip: false}\nMotor 13: {flip: invalid}",
    ]), patch.object(exo, "send_command") as send:
        result = exo.get_hand_visualization_calibration(info)
    assert send.call_args_list == [call("get_home:all"), call("get_flip:all")]
    assert set(result) == {1, 11}
    assert result[1]["limit_max"] == 2840
    assert result[11] == dict(CAL, flip=True)


def fake_gui(mode="Right Only"):
    dual = mode == "Dual"
    names = ["L:index", "R:index"] if dual else ["index"]
    return SimpleNamespace(
        exo_connected=True, motor_widgets=[dict(name=n, cmd_name="index", angle_lbl=Mock()) for n in names],
        _motor_dxl_id=[2, 12] if dual else [12],
        _active_cal_left=None, _active_cal_right=None, _active_cal_profile=None,
        _hand_calibration_by_id={2: CAL, 12: CAL},
        mode_combo=SimpleNamespace(currentText=lambda: mode),
        _hand_vis=Mock(), _udp_hand_vis=Mock(), _device_poll_interval_ms=lambda: 100,
        _publish_teleop_state=Mock(),
    )


def metadata(source="estimated", mid=12):
    return dict(motor_field_sources={mid: {"position": source}},
                motor_sample_timestamp_ms={mid: 1234},
                motor_utc_timestamp_ms={mid: 1789473600123},
                host_poll_completed_monotonic_s=100.0)


def test_both_views_share_device_calibrated_sample_and_clocks_without_profile():
    gui = fake_gui()
    meta = metadata()
    HandExoGUI._apply_motor_angles(gui, {12: 50}, meta)
    args = gui._hand_vis.update_joint_samples.call_args
    assert args == gui._udp_hand_vis.update_joint_samples.call_args
    sample = args.args[0]["index"]
    assert sample == dict(fraction=0.5, source="estimated", sample_timestamp_ms=1234,
                          utc_timestamp_ms=1789473600123, host_received_monotonic_s=100.0)
    gui._publish_teleop_state.assert_called_once_with({}, {}, {"index": 0.5}, meta)


def test_applied_profile_overrides_connection_calibration():
    gui = fake_gui()
    gui._active_cal_profile = {"motors": {"index": dict(CAL, limit_max=300)}}
    HandExoGUI._apply_motor_angles(gui, {12: 50}, metadata("measured"))
    assert gui._hand_vis.update_joint_samples.call_args.args[0]["index"]["fraction"] == 0.25


def test_dual_uses_left_when_right_is_unavailable_and_preserves_ids():
    gui = fake_gui("Dual")
    meta = metadata("measured", mid=2)
    meta["motor_field_sources"][12] = {"position": "unavailable"}
    HandExoGUI._apply_motor_angles(gui, {2: 50, 12: 0}, meta)
    args = gui._hand_vis.update_joint_samples.call_args
    assert args.kwargs["side"] == "left"
    assert args.args[0]["index"]["fraction"] == 0.5
    gui._publish_teleop_state.assert_called_once_with({"index": 0.5}, {"index": None}, {}, meta)


def test_failed_poll_invalidates_both_hand_views():
    gui = fake_gui()
    HandExoGUI._apply_motor_angles(gui, {})
    assert gui._hand_vis.update_joint_samples.call_args.args[0]["index"]["source"] == "unavailable"
    assert gui._hand_vis.update_joint_samples.call_args == gui._udp_hand_vis.update_joint_samples.call_args
    gui.motor_widgets[0]["angle_lbl"].setText.assert_called_with("unavailable")


def test_legacy_source_is_unknown_and_unavailable_never_becomes_open_hand():
    assert motor_display_sample(50, CAL, 12)["source"] == "unknown"
    sample = motor_display_sample(0, CAL, 12, metadata("unavailable"))
    assert sample["fraction"] is None and sample["source"] == "unavailable"


def test_render_pairs_latest_pose_with_latest_metadata_not_smoothed_history():
    gui = SimpleNamespace(exo_connected=True, _telemetry_buffer_dirty=True,
                          _averaged_telemetry_field=lambda field: {12: 25},
                          _latest_hand_angles={12: 50}, _latest_hand_meta=metadata(),
                          _buffered_telemetry_meta=metadata(),
                          _apply_motor_angles=Mock(), _apply_telemetry_result=Mock())
    HandExoGUI._render_buffered_telemetry(gui)
    gui._apply_motor_angles.assert_called_once_with({12: 50}, gui._latest_hand_meta)


@pytest.fixture(scope="module")
def qt_app():
    app = QApplication.instance() or QApplication([])
    yield app


def test_widget_holds_missing_shape_marks_stale_and_clears_on_side_change(qt_app):
    widget = HandSkeletonWidget()
    try:
        with patch(GUI_MODULE + ".time.monotonic", return_value=100):
            widget.update_joint_samples({"index": motor_display_sample(50, CAL, 12, metadata())})
            assert widget._joint_source("index") == "estimated"
            assert not widget.grab().isNull()  # Exercises QPainter styles and coordinates.
        with patch(GUI_MODULE + ".time.monotonic", return_value=102):
            assert widget._joint_source("index") == "unavailable"
            assert widget._t["index"] == 0.5
            widget.update_joint_samples({"index": {"fraction": None, "source": "unavailable"}})
            assert widget._t["index"] == 0.5
            widget.update_joint_samples({}, side="left")
            assert widget._t == {} and widget._side == "left"
            assert not widget.grab().isNull()
            widget.update_joint_samples({}, connected=False)
            assert not widget._age_timer.isActive() and widget._received_at is None
    finally:
        widget.close()


@pytest.mark.parametrize("include_telemetry,shadow", [(False, False), (True, True)])
def test_v09_worker_delivers_nx_sources_even_for_angle_only_and_shadow_polls(qt_app, include_telemetry, shadow):
    worker = SerialWorker()
    worker.set_exo(SimpleNamespace(_firmware_version=(0, 9, 0)))
    worker.set_motor_ids([12])
    worker.set_realtime_control(True, [12])
    worker.set_shadow_telemetry(shadow)
    record = dict(angle=50, absolute_angle=150, current=20, velocity_raw=1,
                  position_source="estimated", current_source="estimated", torque_source="estimated",
                  velocity_source="estimated", sources=0x2a, sample_timestamp_ms=1234,
                  utc_timestamp_ms=1789473600123, utc_synchronized=True, timestamp_ms=1240, flags=0x84)
    worker._get_fast_telemetry = Mock(return_value={12: record})
    worker._get_shadow_telemetry = Mock(return_value={"meta": {"enabled": True}, "records": {12: {"angle": 0}}})
    worker._get_motor_attribute = Mock(side_effect=AssertionError("No scalar fallback on v0.9"))
    results = []
    worker.completed.connect(results.append)
    worker._handle_poll(include_telemetry)
    result = results[-1]
    assert result["relative"] == {12: 50}
    assert result["telemetry_requested"] is include_telemetry
    assert result["telemetry_meta"]["motor_field_sources"][12]["position"] == "estimated"
    assert result["telemetry_meta"]["motor_sample_timestamp_ms"] == {12: 1234}
    assert result["telemetry_meta"]["motor_utc_timestamp_ms"] == {12: 1789473600123}
    assert bool(result["shadow"]) == shadow
    worker._get_motor_attribute.assert_not_called()


def test_worker_failure_is_empty_snapshot_without_scalar_fallback(qt_app):
    worker = SerialWorker()
    worker.set_exo(SimpleNamespace(_firmware_version=(0, 9, 0)))
    worker.set_motor_ids([12])
    worker._get_fast_telemetry = Mock(side_effect=TimeoutError("lost frame"))
    worker._get_motor_attribute = Mock()
    results = []
    worker.completed.connect(results.append)
    worker._handle_poll(False)
    assert results[-1]["relative"] is None
    worker._get_motor_attribute.assert_not_called()


def test_legacy_angle_poll_with_unknown_version_keeps_working(qt_app):
    worker = SerialWorker()
    worker.set_exo(SimpleNamespace(_firmware_version=None))
    worker.set_motor_ids([12])
    worker._get_motor_attribute = Mock(return_value={12: 50})
    worker._uses_dual_serial_transport = Mock(return_value=False)
    results = []
    worker.completed.connect(results.append)
    worker._handle_poll(False)
    assert results[-1]["relative"] == {12: 50}
