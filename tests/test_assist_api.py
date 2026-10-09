import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from unittest.mock import Mock

import pytest

from nml_hand_exo.interface import HandExo
from nml_hand_exo.interface._protobuf_serial import ProtobufSerialComm


def api(protocol="ascii-nx"):
    device = Mock(usb_protocol=protocol)
    exo = HandExo(device, auto_connect=False, send_delay=0)
    exo.info = Mock(return_value={"assist_protocol": "assist-v2"})
    exo._command_transaction = Mock(return_value="OK")
    return exo


@pytest.mark.parametrize("protocol", ["ascii-nx", "protobuf-v1"])
def test_assist_routes_through_typed_interface(protocol):
    exo = api(protocol)
    exo.configure_assist(16, threshold_mA=12, gain_deg_s_per_mA=.02, current_cap_mA=40)
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


@pytest.mark.parametrize("config", [dict(threshold_mA=float("nan")), dict(current_cap_mA=911),
                                     dict(gain_deg_s_per_mA=.51), dict(threshold_mA=300)])
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
    if protocol == "protobuf-v1":
        exo.device.get_assist_status.return_value = dict(
            state=0, reason="not_settled", reject_id=17, reject_angle_deg=117.66,
            reject_velocity_deg_s=4.122, joints={})
    with pytest.raises(ProtocolResponseError, match=r"not_settled.*ID 17.*velocity=4.122"):
        exo.calibrate_assist([17])
    exo._assist_command.assert_called_once()
    if protocol == "protobuf-v1":
        exo.device.get_assist_status.assert_called_once_with(timeout=.4)
        exo._command_transaction.assert_not_called()
    else:
        exo._command_transaction.assert_called_once_with("assist_status", expected="ASSIST:", timeout=.4)


def test_status_failure_preserves_original_calibration_rejection():
    exo = api("protobuf-v1")
    exo._assist_command = Mock(side_effect=RuntimeError("Firmware rejected ASSIST_CALIBRATE: REJECTED"))
    exo.get_assist_status = Mock(side_effect=TimeoutError("status unavailable"))
    with pytest.raises(RuntimeError, match="Firmware rejected ASSIST_CALIBRATE"):
        exo.calibrate_assist([17])


def test_protobuf_assist_commands_use_binary_opcodes():
    device = ProtobufSerialComm("fake", "fake2", 1000000, assist_binary_supported=True)
    response = device.pb.Response(sequence=1, status=device.pb.OK,
                                  assist=device.pb.AssistStatus(state=0, reason=device.pb.ASSIST_REASON_OFF))
    device._rpc = Mock(return_value=response)
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


def test_assist_panel_one_click_baseline_and_reports_stop_fault(qapp):
    from PyQt5.QtCore import QObject, pyqtSignal
    from nml_hand_exo.applications._assist_panel import AssistPanel
    class Worker(QObject):
        assist_completed = pyqtSignal(str, object, str)
        request_assist = Mock()
    worker = Worker()
    panel = AssistPanel(worker)
    panel.set_connection(True, [(16, "index")], {"assist_protocol": "assist-v3"})
    assert panel.toggle.isEnabled()
    panel.toggle.click()
    assert panel.busy and panel.toggle.isChecked()
    assert worker.request_assist.call_args.args[0] == "enable"
    assert worker.request_assist.call_args.args[1]["ids"] == [16]
    worker.assist_completed.emit("enable", dict(state=1, reason="calibrating", joints={}), "")
    assert panel.timer.isActive()
    worker.assist_completed.emit("heartbeat", dict(state=3, reason="active", joints={}), "")
    assert panel.toggle.isEnabled() and panel.toggle.isChecked()
    assert panel.toggle.text() == "Disable Torque-Assist"
    panel.stop_session()
    worker.assist_completed.emit("stop", dict(state=4, reason="torque_off_unconfirmed", joints={}), "")
    assert panel.busy and not panel.toggle.isEnabled()
    panel.set_connection(False)
    assert not panel.timer.isActive() and not panel.busy
    panel.set_connection(True, [(16, "index")], {"assist_protocol": "assist-v3"})
    error = "assist calibration rejected: motor_disabled (ID 16)"
    worker.assist_completed.emit("enable", {}, error)
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


def test_secondary_goal_config_rejects_legacy_firmware():
    exo = api()
    exo.info.return_value = {"assist_protocol": "assist-v1"}
    with pytest.raises(RuntimeError, match="requires assist-v2"):
        exo.configure_assist(16)
    exo._command_transaction.assert_not_called()


@pytest.mark.parametrize("protocol", ["ascii-nx", "protobuf-v1"])
def test_assist_observer_uses_selected_transport(protocol):
    exo = api(protocol)
    exo.configure_assist_observer(16, inertia_Nm_s2_per_deg=.035,
                                  viscous_Nm_s_per_deg=.018, coulomb_Nm=.025)
    if protocol == "protobuf-v1":
        exo.device.assist_command.assert_called_once_with(
            "assist_observer", [16], (.035, .018, .025), timeout=1.)
        exo._command_transaction.assert_not_called()
    else:
        exo._command_transaction.assert_called_once_with(
            "assist_observer:16:0.035:0.018:0.025", expected="OK: assist_observer", timeout=1.)
        exo.device.assist_command.assert_not_called()
    exo._command_transaction.reset_mock()
    with pytest.raises(ValueError):
        exo.configure_assist_observer(16, coulomb_Nm=float("nan"))
    exo._command_transaction.assert_not_called()


def test_assist_panel_requires_v3_and_uses_simulator_defaults(qapp):
    from PyQt5.QtCore import QObject, pyqtSignal
    from nml_hand_exo.applications._assist_panel import AssistPanel
    class Worker(QObject):
        assist_completed = pyqtSignal(str, object, str)
        request_assist = Mock()
    worker = Worker()
    panel = AssistPanel(worker)
    panel.set_connection(True, [(16, "index")], {"assist_protocol": "assist-v1"})
    assert not panel.toggle.isEnabled()
    panel.set_connection(True, [(16, "index")], {"assist_protocol": "assist-v2"})
    assert not panel.toggle.isEnabled()
    panel.set_connection(True, [(16, "index")], {"assist_protocol": "assist-v3"})
    assert panel.threshold.value() == .08
    assert panel.gain.value() == 80
    assert panel.cap.value() == 300
    panel.toggle.click()
    parameters = worker.request_assist.call_args.args[1]["parameters"]
    assert parameters["velocity_gain_deg_s_per_Nm"] == 80
    assert parameters["inertia_Nm_s2_per_deg"] == 0
    assert not hasattr(panel, 'calibrate')
    panel.deleteLater()


@pytest.mark.parametrize('fault', [None, 0, 4])
def test_worker_uses_binary_heartbeat_snapshot_without_extra_status_query(fault):
    from nml_hand_exo.applications.hand_exo_gui import SerialWorker
    from nml_hand_exo.interface._protobuf_serial import AssistRejected
    worker = SerialWorker()
    exo = Mock()
    exo.run_locked = lambda callback: callback(exo)
    state = dict(state=3 if fault is None else fault,
                 reason='active' if fault is None else 'heartbeat_timeout', joints={})
    if fault is None:
        exo.heartbeat_assist.return_value = state
    else:
        exo.heartbeat_assist.side_effect = AssistRejected('Firmware rejected ASSIST_HEARTBEAT', state)
    observed = []
    worker.assist_completed.connect(lambda *args: observed.append(args))
    worker.set_exo(exo)
    worker.request_assist('heartbeat')
    worker._handle_assist(*worker._assist_q.get_nowait())
    assert observed == [('heartbeat', state, '')]
    exo.heartbeat_assist.assert_called_once()
    exo.get_assist_status.assert_not_called()

@pytest.mark.parametrize('protocol', ['ascii-nx', 'protobuf-v1'])
def test_enable_torque_assist_is_one_request_with_simulator_units(protocol):
    exo = api(protocol)
    exo.info.return_value = {'assist_protocol': 'assist-v3'}
    exo.enable_torque_assist([11, 16, 19])
    if protocol == 'protobuf-v1':
        exo.device.assist_command.assert_called_once_with(
            'assist_enable', [11, 16, 19], (.08/.00115, 80*.00115, 300., 0., 0., 0.), timeout=2.)
        exo.info.assert_not_called()
        exo._command_transaction.assert_not_called()
    else:
        exo._command_transaction.assert_called_once_with(
            'assist_enable:69.5652174:0.092:300:0:0:0:11:16:19', expected='OK: assist_enable', timeout=2.)


@pytest.mark.parametrize('params', [dict(threshold_Nm=float('nan')), dict(current_cap_mA=911),
    dict(threshold_Nm=.08, current_cap_mA=60), dict(velocity_gain_deg_s_per_Nm=1000),
    dict(coulomb_Nm=-1), dict(inertia_Nm_s2_per_deg=True)])
def test_invalid_enable_never_reaches_transport(params):
    exo = api('protobuf-v1')
    with pytest.raises(ValueError):
        exo.enable_torque_assist([16], **params)
    exo.device.assist_command.assert_not_called()


def test_worker_enable_avoids_legacy_mode_and_per_joint_config(monkeypatch):
    from nml_hand_exo.applications.hand_exo_gui import SerialWorker
    worker = SerialWorker()
    exo = Mock()
    exo.run_locked = lambda callback: callback(exo)
    exo.enable_torque_assist.return_value = dict(state=1, reason='calibrating', joints={})
    worker.set_exo(exo)
    stationary_feedback(monkeypatch, exo, [11, 16])
    worker.request_assist('enable', dict(ids=[11, 16], parameters=dict(threshold_Nm=.08)))
    worker._handle_assist(*worker._assist_q.get_nowait())
    exo.enable_torque_assist.assert_called_once_with([11, 16], threshold_Nm=.08)
    exo.configure_assist.assert_not_called()
    exo.set_control_mode.assert_not_called()
    exo.calibrate_assist.assert_not_called()
    exo.start_assist.assert_not_called()
    exo.get_assist_status.assert_not_called()


def test_stop_cancels_unsent_one_click_enable():
    from nml_hand_exo.applications.hand_exo_gui import SerialWorker
    worker = SerialWorker()
    exo = Mock()
    exo.run_locked = lambda callback: callback(exo)
    exo.stop_assist.return_value = dict(state=0, reason='stopped', joints={})
    worker.set_exo(exo)
    worker.request_assist('enable', dict(ids=[16], parameters={}))
    worker.request_assist('stop')
    worker._handle_assist(*worker._assist_q.get_nowait())
    worker._handle_assist(*worker._assist_q.get_nowait())
    exo.enable_torque_assist.assert_not_called()
    exo.stop_assist.assert_called_once()


def test_enable_requires_binary_capability_and_never_uses_ascii():
    device = ProtobufSerialComm('fake', 'fake2', 1000000, assist_binary_supported=True)
    device._cmd, device._telem = Mock(), Mock()
    exo = HandExo(device, auto_connect=False)
    with pytest.raises(RuntimeError, match='requires assist-v3'):
        exo.enable_torque_assist([16])
    device._cmd.write.assert_not_called()
    device._telem.write.assert_not_called()


def test_panel_targets_active_side_and_shows_effective_caps(qapp):
    from PyQt5.QtCore import QObject, pyqtSignal
    from nml_hand_exo.applications._assist_panel import AssistPanel
    class Worker(QObject):
        assist_completed = pyqtSignal(str, object, str)
        request_assist = Mock()
    worker = Worker()
    panel = AssistPanel(worker)
    info = dict(assist_protocol='assist-v3', usb_protocol='protobuf-v1',
                usb_features=['assist_binary', 'assist_enable'])
    panel.set_connection(True, [(11, 'wrist'), (16, 'index')], info)
    panel.toggle.click()
    assert worker.request_assist.call_args.args[1]['ids'] == [11, 16]
    worker.assist_completed.emit('enable', dict(state=1, reason='calibrating',
        total_current_budget_mA=800, joints={16: dict(samples=2, current_cap_mA=88)}), '')
    assert panel.table.item(1, 1).text() == '88'
    assert '800 mA' in panel.budget.text()
    panel.toggle.click()
    worker.request_assist.assert_called_with('stop')
    # A late startup reply cannot re-enable a stopped session.
    worker.assist_completed.emit('enable', dict(state=3, reason='active', joints={}), '')
    assert not panel.timer.isActive() and not panel.toggle.isChecked()
    worker.assist_completed.emit('stop', dict(state=0, reason='stopped', joints={}), '')
    assert panel.toggle.isEnabled() and not panel.busy
    panel.deleteLater()


def stationary_feedback(monkeypatch, exo, ids, adjust=None):
    from types import SimpleNamespace
    clock = SimpleNamespace(now=0., reads=0)
    monkeypatch.setattr('nml_hand_exo.applications.hand_exo_gui.time.monotonic', lambda: clock.now)
    def sleep(seconds):
        clock.now += seconds
    monkeypatch.setattr('nml_hand_exo.applications.hand_exo_gui.time.sleep', sleep)
    def read(**kwargs):
        assert kwargs['motor_ids'] == ids
        assert 0 < kwargs['timeout'] <= .25
        clock.now += .01
        clock.reads += 1
        records = {mid: dict(error=False, absolute_angle=100., velocity_raw=0,
                            position_source='measured', velocity_source='measured',
                            sample_timestamp_ms=round(clock.now*1000)) for mid in ids}
        if adjust:
            adjust(records, clock)
        return records
    exo.get_fast_telemetry.side_effect = read
    return clock


def run_worker_enable(exo):
    from nml_hand_exo.applications.hand_exo_gui import SerialWorker
    worker = SerialWorker()
    exo.run_locked = lambda callback: callback(exo)
    worker.set_exo(exo)
    worker.request_assist('enable', dict(ids=[11, 15], parameters={}))
    return worker, worker._assist_q.get_nowait()


def test_assist_waits_out_transient_velocity_count_before_one_enable(monkeypatch):
    exo = Mock()
    worker, request = run_worker_enable(exo)
    def transient(records, clock):
        if clock.reads <= 3:
            records[15]['velocity_raw'] = 1
    clock = stationary_feedback(monkeypatch, exo, [11, 15], transient)
    exo.enable_torque_assist.return_value = dict(state=1, reason='calibrating', joints={})
    worker._handle_assist(*request)
    assert clock.now >= .68
    exo.enable_torque_assist.assert_called_once_with([11, 15])


@pytest.mark.parametrize('problem', ['velocity', 'position', 'estimated', 'stale'])
def test_assist_startup_never_enables_on_moving_or_unmeasured_feedback(monkeypatch, problem):
    exo = Mock()
    worker, request = run_worker_enable(exo)
    def adjust(records, clock):
        if problem == 'velocity': records[15]['velocity_raw'] = 1
        elif problem == 'position': records[15]['absolute_angle'] += clock.reads * .1
        elif problem == 'estimated': records[15]['position_source'] = 'estimated'
        else: records[15]['sample_timestamp_ms'] = 1
    clock = stationary_feedback(monkeypatch, exo, [11, 15], adjust)
    result = []
    worker.assist_completed.connect(lambda *args: result.append(args))
    worker._handle_assist(*request)
    assert clock.now >= 5
    assert 'ID 15' in result[0][2]
    if problem == 'velocity': assert 'velocity=1.374 deg/s' in result[0][2]
    exo.enable_torque_assist.assert_not_called()


@pytest.mark.parametrize('problem', ['missing', 'error', 'nan'])
def test_assist_startup_rejects_unavailable_feedback(monkeypatch, problem):
    exo = Mock()
    worker, request = run_worker_enable(exo)
    def adjust(records, clock):
        if problem == 'missing': del records[15]
        elif problem == 'error': records[15]['error'] = True
        else: records[15]['velocity_raw'] = float('nan')
    stationary_feedback(monkeypatch, exo, [11, 15], adjust)
    result = []
    worker.assist_completed.connect(lambda *args: result.append(args))
    worker._handle_assist(*request)
    assert 'ID 15' in result[0][2]
    exo.enable_torque_assist.assert_not_called()


def test_stop_cancels_assist_while_waiting_for_stationary_feedback(monkeypatch):
    exo = Mock()
    worker, request = run_worker_enable(exo)
    def cancel(records, clock):
        worker.request_assist('stop')
    stationary_feedback(monkeypatch, exo, [11, 15], cancel)
    worker._handle_assist(*request)
    exo.enable_torque_assist.assert_not_called()
    worker._handle_assist(*worker._assist_q.get_nowait())
    exo.stop_assist.assert_called_once()


@pytest.mark.parametrize('protocol', ['ascii-nx', 'protobuf-v1'])
def test_one_click_rejection_keeps_pose_velocity_and_status_without_retry(protocol):
    from nml_hand_exo.interface._protobuf_serial import AssistRejected
    from nml_hand_exo.interface._hand_exo import ProtocolResponseError
    exo = api(protocol)
    exo.info.return_value = {'assist_protocol': 'assist-v3'}
    status = dict(state=0, reason='not_settled', reject_id=15, reject_angle_deg=117.66,
                  reject_velocity_deg_s=1.374, joints={})
    reject = (AssistRejected('Firmware rejected ASSIST_ENABLE', status) if protocol == 'protobuf-v1' else
              ProtocolResponseError(command='assist_enable', expected='OK', raw_response='ERROR: rejected assist_enable'))
    exo._assist_command = Mock(side_effect=reject)
    exo.get_assist_status = Mock(return_value=status)
    with pytest.raises(RuntimeError, match=r'ID 15.*angle=117.660 deg.*velocity=1.374 deg/s') as caught:
        exo.enable_torque_assist([15])
    assert caught.value.assist_status == status
    exo._assist_command.assert_called_once()
    if protocol == 'protobuf-v1': exo.get_assist_status.assert_not_called()
    else: exo.get_assist_status.assert_called_once()


def test_assist_stationary_wait_handles_device_uptime_rollover(monkeypatch):
    exo = Mock()
    worker, request = run_worker_enable(exo)
    def rollover(records, clock):
        for record in records.values():
            record['sample_timestamp_ms'] = (0xffffff00 + round(clock.now*1000)) & 0xffffffff
    clock = stationary_feedback(monkeypatch, exo, [11, 15], rollover)
    exo.enable_torque_assist.return_value = dict(state=1, reason='calibrating', joints={})
    worker._handle_assist(*request)
    assert clock.now >= .5
    exo.enable_torque_assist.assert_called_once()
