"""Setup limits, applied-versus-edited values, and firmware/GUI speed agreement."""
import os
import re
from pathlib import Path
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication

from nml_hand_exo.applications.hand_exo_gui import (
    HandExoGUI, SynchronizedHandExo, DIRECT_VELOCITY_LIMIT_RPM,
)
from nml_hand_exo.interface._hand_exo import HandExo, ProtocolResponseError
from nml_hand_exo.testing import FakeOpenRBComm, ReplyFault


@pytest.fixture
def setup_gui(monkeypatch):
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(QThread, "start", lambda *args: None)
    monkeypatch.setattr(HandExoGUI, "_load_stream_settings", lambda self: None)
    gui = HandExoGUI()
    gui._motor_dxl_id = [1, 11]
    gui.motor_names = ["L:wrist", "R:wrist"]
    gui.mode_combo.setCurrentText("Dual")
    gui._build_motor_rows()
    gui.exo_connected = True
    gui._run_bulk_serial_action = lambda action: action()
    gui._log = Mock()
    yield gui
    gui.exo_connected = False
    gui.exo = None
    gui.close()
    app.processEvents()


def device(gui, current=75, budget=800):
    raw = Mock()
    raw.get_motor_current_limit.return_value = current
    raw.get_total_current_limit.return_value = budget
    raw.set_total_current_limit.return_value = budget
    gui.exo = Mock()
    gui.exo.run_locked.side_effect = lambda callback: callback(raw)
    return raw


def test_setup_and_direct_widgets_allow_71_and_match_firmware(setup_gui):
    gui = setup_gui
    assert DIRECT_VELOCITY_LIMIT_RPM == 71
    for row in gui.motor_widgets:
        assert row["velocity_limit_spin"].maximum() == 71
        assert row["velocity_limit_rpm"] == 71
        assert row["current_limit_spin"].maximum() == 910
    assert gui._direct_command_spin.maximum() == 71
    assert gui._emg_max_command_spin.maximum() == 71
    gui._direct_mode = "velocity"
    assert gui._limit_direct_command_for_motor(11, 65) == 65
    assert gui._limit_direct_command_for_motor(11, -250) == -71
    config = (Path(__file__).resolve().parents[1] / "src/cpp/nml_hand_exo/config.h").read_text()
    rpm = float(re.search(r"DIRECT_VELOCITY_LIMIT_RPM = ([\d.]+)f", config)[1])
    raw = int(re.search(r"DIRECT_VELOCITY_LIMIT_RAW = (\d+)", config)[1])
    assert rpm == DIRECT_VELOCITY_LIMIT_RPM
    assert raw == round(rpm / 0.229)


def test_row_apply_targets_right_id_and_edits_do_not_change_applied_limits(setup_gui):
    gui = setup_gui
    raw = device(gui)
    row = gui.motor_widgets[1]
    row["current_limit_spin"].setValue(75)
    row["velocity_limit_spin"].setValue(65)
    gui._apply_motor_row_limits(row)
    raw.set_current_limit.assert_called_once_with(11, 75)
    raw.get_motor_current_limit.assert_called_once_with(11)
    row["current_limit_spin"].setValue(910)
    row["velocity_limit_spin"].setValue(71)
    gui._direct_mode = "current"
    assert gui._limit_direct_command_for_motor(11, 200) == 75
    gui._direct_mode = "velocity"
    assert gui._limit_direct_command_for_motor(11, -200) == -65
    assert gui.motor_widgets[0]["current_limit_mA"] == 0
    raw.set_motor_velocity_limit.assert_not_called()


@pytest.mark.parametrize("reply", [float("nan"), 30])
def test_failed_row_readback_never_raises_gui_ceiling(setup_gui, reply):
    gui = setup_gui
    raw = device(gui, current=reply)
    row = gui.motor_widgets[1]
    row["current_limit_spin"].setValue(75)
    row["velocity_limit_rpm"] = 3
    row["velocity_limit_spin"].setValue(71)
    gui._apply_motor_row_limits(row)
    assert row["velocity_limit_rpm"] == 3
    assert row["current_limit_mA"] == 0
    assert "failed" in gui._log.call_args.args[0]


def test_budget_uses_clamped_readback_and_ignores_unapplied_editor(setup_gui):
    gui = setup_gui
    raw = device(gui, budget=900)
    gui._total_current_spin.setValue(100)
    gui._set_total_current_limit()
    raw.set_total_current_limit.assert_called_once_with(100)
    raw.get_total_current_limit.assert_not_called()
    assert gui._total_current_spin.value() == 900
    gui._total_current_spin.setValue(9000)
    assert gui._available_emg_current_budget_ma([11]) == 900


def test_skipped_action_preserves_applied_limits_and_does_not_log_success(setup_gui):
    gui = setup_gui
    raw = device(gui)
    gui._applied_total_current_mA = 400
    gui._run_bulk_serial_action = lambda action: None
    gui._set_total_current_limit()
    gui._apply_motor_row_limits(gui.motor_widgets[1])
    assert gui._applied_total_current_mA == 400
    raw.set_current_limit.assert_not_called()
    raw.set_total_current_limit.assert_not_called()
    gui._log.assert_not_called()


def test_missing_budget_readback_blocks_emg_current_instead_of_using_editor(setup_gui):
    gui = setup_gui
    device(gui, budget=float("nan"))
    gui._set_total_current_limit()
    assert gui._available_emg_current_budget_ma([11]) == 0
    assert "error" in gui._log.call_args.args[0]


@pytest.mark.parametrize("flag,value", [
    ("_emg_live", True), ("_direct_command_active", True),
    ("_emg_hold_active", True), ("_home_sequence_active", True),
    ("_rom_running", True), ("_udp_binding_active_ids", {11}),
    ("_udp_stream_pending", {11: "command"}),
])
def test_active_control_blocks_both_limit_updates(setup_gui, flag, value):
    gui = setup_gui
    raw = device(gui)
    setattr(gui, flag, value)
    gui._set_total_current_limit()
    gui._apply_motor_row_limits(gui.motor_widgets[1])
    raw.set_current_limit.assert_not_called()
    raw.set_total_current_limit.assert_not_called()
    setattr(gui, flag, False if isinstance(value, bool) else type(value)())


def test_connect_reads_budget_and_preserves_zero_current_limit(setup_gui):
    gui = setup_gui
    raw = device(gui, current={1: 0, 11: 75}, budget=900)
    gui._sync_motor_limits_after_connect()
    assert gui.motor_widgets[0]["current_limit_spin"].value() == 0
    assert gui.motor_widgets[0]["current_limit_mA"] == 0
    assert gui.motor_widgets[1]["current_limit_mA"] == 75
    assert gui._applied_total_current_mA == 900
    raw.set_current_limit.assert_not_called()


def test_budget_query_does_not_parse_a_setter_ack_as_a_readback():
    exo = object.__new__(HandExo)
    exo._require_firmware = Mock()
    exo.send_command = Mock()
    exo._receive = Mock(return_value="OK: total_current_lim 900")
    with pytest.raises(ProtocolResponseError, match="get_total_current_lim"):
        exo.get_total_current_limit()


def test_setup_side_enable_clears_other_side_after_global_mode_change(setup_gui):
    gui = setup_gui
    raw = device(gui)
    gui._direct_mode = "velocity"
    gui._direct_armed_ids = {1, 11}
    for row in gui.motor_widgets:
        row["enabled"] = True
        row["status_lbl"].setText("DIRECT")
    gui._motor_side("enable", "right")
    gui.exo.set_control_mode.assert_called_once_with("current_position")
    raw.enable_motor.assert_called_once_with(11)
    assert not gui.motor_widgets[0]["enabled"]
    assert gui.motor_widgets[0]["status_lbl"].text() == "OFF"
    assert gui.motor_widgets[1]["enabled"]
    assert not gui._direct_armed_ids
    assert gui._direct_mode is None


def test_setup_disable_side_disarms_only_that_side(setup_gui):
    gui = setup_gui
    raw = device(gui)
    gui._direct_mode = "velocity"
    gui._direct_armed_ids = {1, 11}
    gui._motor_side("disable", "right")
    raw.disable_motor.assert_called_once_with(11)
    assert gui._direct_armed_ids == {1}
    gui.exo.set_control_mode.assert_not_called()


def test_setup_enable_does_not_show_on_when_bulk_action_is_skipped(setup_gui):
    gui = setup_gui
    raw = device(gui)
    gui._run_bulk_serial_action = lambda action: None
    gui._motor_all("enable")
    raw.enable_motor.assert_not_called()
    gui.exo.set_control_mode.assert_not_called()
    gui.exo.disable_motor.assert_not_called()
    assert all(not row["enabled"] for row in gui.motor_widgets)


@pytest.mark.parametrize("side", [None, "left", "right"])
def test_enable_keeps_polling_paused_from_mode_change_through_torque_enable(setup_gui, side):
    gui = setup_gui
    comm = FakeOpenRBComm(motor_ids=(1, 11))
    comm.connect()
    raw = HandExo(comm, auto_connect=False, send_delay=0)
    raw._firmware_version = (0, 9, 1)
    # Represent a global mode change on a dual bus with slow/absent motors.
    # This exceeds the former SDK deadline and must finish before enabling.
    comm.queue_fault(ReplyFault(command="set_control_mode:", delay_s=.9))
    gui.exo = SynchronizedHandExo(raw)
    # Exercise the production bulk-action guard, not the fixture's replacement.
    gui._run_bulk_serial_action = HandExoGUI._run_bulk_serial_action.__get__(gui)
    gui._wait_for_pending_poll = lambda timeout_ms: not gui._serial_worker.has_pending_poll()
    gui._angle_timer.start(100)
    poll_states = []
    original_send = comm.send
    def send(command):
        if command.startswith(("set_control_mode:", "enable:")):
            poll_states.append(gui._suspend_device_poll_requests)
        original_send(command)
    comm.send = send

    if side is None:
        gui._motor_all("enable")
        expected_ids = {1, 11}
    else:
        gui._motor_side("enable", side)
        expected_ids = {1} if side == "left" else {11}

    assert {mid for mid in (1, 11) if comm.enabled[mid]} == expected_ids
    assert {row["dxl_id"] for row in gui.motor_widgets if row["enabled"]} == expected_ids
    assert poll_states and all(poll_states)
    assert not gui._suspend_device_poll_requests


@pytest.mark.parametrize("side", [None, "left", "right"])
def test_failed_global_mode_change_never_proceeds_to_enable(setup_gui, side):
    gui = setup_gui
    raw = device(gui)
    gui.exo.set_control_mode.side_effect = ProtocolResponseError(
        command="set_control_mode:all:current_position",
        expected="Motor control mode: current_position",
        raw_response="ERROR: motor mode change failed safety verification",
    )
    if side is None:
        gui._motor_all("enable")
    else:
        gui._motor_side("enable", side)
    raw.enable_motor.assert_not_called()
    assert all(not row["enabled"] for row in gui.motor_widgets)
    assert "Could not restore position control" in gui._log.call_args.args[0]
