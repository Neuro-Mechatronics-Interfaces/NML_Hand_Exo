import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from unittest.mock import Mock

import pytest

from nml_hand_exo.interface import HandExo
from nml_hand_exo.interface._protobuf_serial import ProtobufSerialComm


def api(protocol="ascii-nx"):
    device = Mock(usb_protocol=protocol)
    exo = HandExo(device, auto_connect=False, send_delay=0)
    exo._command_transaction = Mock(return_value="OK")
    return exo


@pytest.mark.parametrize("protocol", ["ascii-nx", "protobuf-v1"])
def test_assist_routes_through_typed_interface(protocol):
    exo = api(protocol)
    exo.configure_assist(16, threshold_mA=12, gain_deg_per_mA=.02, current_cap_mA=40)
    exo.calibrate_assist([16, 17])
    exo.start_assist()
    exo.heartbeat_assist()
    exo.stop_assist()
    if protocol == "protobuf-v1":
        assert exo.device.assist_command.call_count == 5
        exo.device.assist_command.assert_any_call("assist_config", [16], (12., .02, 40.), timeout=1.)
        exo._command_transaction.assert_not_called()
    else:
        commands = [call.args[0] for call in exo._command_transaction.call_args_list]
        assert commands == ["assist_config:16:12:0.02:40", "assist_calibrate:16:17",
                            "assist_start", "assist_heartbeat", "assist_stop"]


@pytest.mark.parametrize("config", [dict(threshold_mA=float("nan")), dict(current_cap_mA=81),
                                     dict(gain_deg_per_mA=.1), dict(threshold_mA=40)])
def test_invalid_assist_parameters_never_reach_transport(config):
    exo = api("protobuf-v1")
    with pytest.raises(ValueError):
        exo.configure_assist(16, **config)
    exo.device.assist_command.assert_not_called()


def test_assist_targets_are_explicit_unique_integer_ids():
    exo = api()
    for ids in ([], ["index"], [True], [16, 16], list(range(1, 20))):
        with pytest.raises(ValueError):
            exo.calibrate_assist(ids)
    exo._command_transaction.assert_not_called()


def test_assist_status_preserves_fault_and_measured_diagnostics():
    exo = api()
    exo._command_transaction.return_value = (
        "ASSIST: state=4 reason=torque_off_unconfirmed\n"
        "ASSIST_JOINT: id=16 samples=50 bias_mA=5 noise_mA=1 deadzone_mA=10 "
        "effort_mA=-21 deflection_deg=-0.2 angle=100 goal=100.5 moving=0 steps=3 age_ms=25;"
    )
    status = exo.get_assist_status()
    assert status["state"] == 4 and status["reason"] == "torque_off_unconfirmed"
    assert status["joints"][16]["effort_mA"] == -21
    assert status["joints"][16]["deflection_deg"] == -.2


@pytest.mark.parametrize("protocol", ["ascii-nx", "protobuf-v1"])
def test_calibration_rejection_fetches_reason_and_motor_without_retry(protocol):
    from nml_hand_exo.interface._hand_exo import ProtocolResponseError
    exo = api(protocol)
    reject = ProtocolResponseError(command="assist_calibrate", expected="OK", raw_response="ERROR: rejected assist_calibrate") if protocol == "ascii-nx" else RuntimeError("Firmware rejected ASSIST_CALIBRATE: REJECTED")
    exo._assist_command = Mock(side_effect=reject)
    exo._command_transaction.return_value = (
        "ASSIST: state=0 reason=not_settled reject_id=17 "
        "reject_angle_deg=117.660 reject_velocity_deg_s=4.122;")
    with pytest.raises(ProtocolResponseError, match=r"not_settled.*ID 17.*velocity=4.122"):
        exo.calibrate_assist([17])
    exo._assist_command.assert_called_once()
    exo._command_transaction.assert_called_once_with("assist_status", expected="ASSIST:", timeout=.4)


def test_status_failure_preserves_original_calibration_rejection():
    exo = api("protobuf-v1")
    exo._assist_command = Mock(side_effect=RuntimeError("Firmware rejected ASSIST_CALIBRATE: REJECTED"))
    exo.get_assist_status = Mock(side_effect=TimeoutError("status unavailable"))
    with pytest.raises(RuntimeError, match="Firmware rejected ASSIST_CALIBRATE"):
        exo.calibrate_assist([17])


def test_protobuf_assist_commands_use_binary_opcodes():
    device = ProtobufSerialComm("fake", "fake2", 1000000)
    device._rpc = Mock()
    device.assist_command("assist_config", [16], [10, .01, 40])
    device._rpc.assert_called_once_with(device.pb.ASSIST_CONFIG, [16], values=[10, .01, 40], timeout=1.)


def test_worker_stop_cancels_unsent_assist_start():
    from nml_hand_exo.applications.hand_exo_gui import SerialWorker
    worker = SerialWorker()
    exo = Mock()
    exo.run_locked = lambda callback: callback(exo)
    exo.get_assist_status.return_value = dict(state=0, reason="stopped", joints={})
    worker.set_exo(exo)
    worker.request_assist("start")
    worker.request_assist("stop")
    worker._handle_assist(*worker._assist_q.get_nowait())
    worker._handle_assist(*worker._assist_q.get_nowait())
    exo.start_assist.assert_not_called()
    exo.stop_assist.assert_called_once()


def test_assist_panel_waits_for_calibration_and_reports_stop_fault(qapp):
    from PyQt5.QtCore import QObject, pyqtSignal
    from nml_hand_exo.applications._assist_panel import AssistPanel
    class Worker(QObject):
        assist_completed = pyqtSignal(str, object, str)
        request_assist = Mock()
    worker = Worker()
    panel = AssistPanel(worker)
    panel.set_connection(True, [(16, "index")], {"assist_protocol": "assist-v1"})
    assert not panel.toggle.isEnabled()
    panel.rows[16][1].setChecked(True)
    panel._calibrate()
    assert panel.busy and not panel.toggle.isEnabled()
    worker.assist_completed.emit("calibrate", dict(state=1, reason="calibrating", joints={}), "")
    assert panel.timer.isActive()
    worker.assist_completed.emit("heartbeat", dict(state=2, reason="ready", joints={}), "")
    assert panel.toggle.isEnabled() and not panel.toggle.isChecked()
    panel.toggle.click()
    worker.request_assist.assert_called_with("start")
    panel.stop_session()
    worker.assist_completed.emit("stop", dict(state=4, reason="torque_off_unconfirmed", joints={}), "")
    assert panel.busy and not panel.toggle.isEnabled()
    panel.set_connection(False)
    assert not panel.timer.isActive() and not panel.busy
    panel.set_connection(True, [(16, "index")], {"assist_protocol": "assist-v1"})
    error = "assist calibration rejected: motor_disabled (ID 16)"
    worker.assist_completed.emit("calibrate", {}, error)
    worker.assist_completed.emit("stop", dict(state=0, reason="motor_disabled", joints={}), "")
    assert panel.status.text() == error and not panel.busy
    panel.deleteLater()


@pytest.fixture
def qapp():
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    yield app


def test_stop_during_configuration_prevents_calibration_start():
    from nml_hand_exo.applications.hand_exo_gui import SerialWorker
    worker = SerialWorker()
    exo = Mock()
    exo.run_locked = lambda callback: callback(exo)
    exo.configure_assist.side_effect = lambda *args, **kwargs: worker.request_assist("stop")
    worker.set_exo(exo)
    worker.request_assist("calibrate", {16: {}, 17: {}})
    worker._handle_assist(*worker._assist_q.get_nowait())
    exo.configure_assist.assert_called_once()
    exo.calibrate_assist.assert_not_called()


def test_assist_automatically_selects_mode_before_calibrating():
    from nml_hand_exo.applications.hand_exo_gui import SerialWorker
    worker = SerialWorker()
    exo = Mock()
    exo.run_locked = lambda callback: callback(exo)
    worker.set_exo(exo)
    worker.request_assist("calibrate", {16: {}})
    worker._handle_assist(*worker._assist_q.get_nowait())
    names = [call[0] for call in exo.mock_calls]
    assert names.index('configure_assist') < names.index('set_control_mode') < names.index('calibrate_assist')
    exo.set_control_mode.assert_called_once_with('current_position')
    exo.calibrate_assist.assert_called_once_with([16])


def test_stop_during_mode_switch_prevents_late_assist_enable():
    from nml_hand_exo.applications.hand_exo_gui import SerialWorker
    worker = SerialWorker()
    exo = Mock()
    exo.run_locked = lambda callback: callback(exo)
    exo.set_control_mode.side_effect = lambda mode: worker.request_assist("stop")
    worker.set_exo(exo)
    worker.request_assist("calibrate", {16: {}})
    worker._handle_assist(*worker._assist_q.get_nowait())
    exo.set_control_mode.assert_called_once_with('current_position')
    exo.calibrate_assist.assert_not_called()


def test_assist_discards_old_motion_backlog():
    from nml_hand_exo.applications.hand_exo_gui import SerialWorker
    worker = SerialWorker()
    worker.enqueue_send("set_angle:16:50")
    worker.enqueue_send("stop:16")
    worker.request_direct_actions({16: ("current", 40)})
    worker.request_assist("calibrate", {16: {}})
    assert worker._urgent_q.get_nowait() == ("send", "stop:16")
    assert worker._urgent_q.empty()
    assert worker._direct_actions == {}
