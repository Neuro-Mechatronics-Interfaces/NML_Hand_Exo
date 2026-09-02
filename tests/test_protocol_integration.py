from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import pytest

from nml_hand_exo.applications.hand_exo_gui import HandExoGUI, SerialWorker, SynchronizedHandExo
from nml_hand_exo.applications.emg_intent_decoder_gui import EmgIntentDecoderWindow
from nml_hand_exo.interface._hand_exo import HandExo, ProtocolResponseError
from nml_hand_exo.testing import FakeOpenRBComm, ReplyFault
from tools.check_protocol_contract import missing_host_commands


def _fake_exo():
    comm = FakeOpenRBComm()
    comm.connect()
    exo = HandExo(comm, send_delay=0)
    exo._firmware_version = (0, 6, 2)
    return exo, comm


def test_stateful_fake_runs_power_grasp_hold_workflow():
    exo, comm = _fake_exo()
    comm.angles[14] = 5.82

    assert exo.get_motor_angle(14) == pytest.approx(5.82)
    exo.hold_motor_position(14, 5.82)
    for motor_id in (15, 16, 17, 18, 19):
        exo.enable_motor(motor_id)
        exo.set_motor_velocity(motor_id, 0.5)

    assert comm.holds[14]["angle"] == pytest.approx(5.82)
    assert comm.holds[14]["current_mA"] == 25
    assert all(comm.enabled[mid] for mid in (14, 15, 16, 17, 18, 19))

    for motor_id in (15, 16, 17, 18, 19):
        exo.stop_direct_control(motor_id)
    exo.release_motor_hold(14)
    assert 14 not in comm.holds
    assert not comm.enabled[14]


def test_motor_limits_use_firmware_command_and_preserve_both_values():
    exo, comm = _fake_exo()
    comm.limits[14] = (-32.5, 47.25)

    assert exo.get_motor_limits(14) == pytest.approx([-32.5, 47.25])
    assert comm.sent[-1] == "get_motor_limits:14"


def test_per_hold_current_is_clamped_and_reported_by_fake_firmware():
    exo, comm = _fake_exo()
    comm.current_limits[14] = 70

    response = exo.hold_motor_position(14, 5.82, 100)

    assert comm.holds[14] == {"angle": pytest.approx(5.82), "current_mA": 70}
    assert "current_mA=70" in response


def test_indirect_attribute_queries_use_actual_firmware_command_names():
    exo, comm = _fake_exo()

    assert exo.get_baudrate(14) == 1_000_000
    assert exo.get_motor_acceleration(14) == 50
    assert exo.get_motor_current_limit(14) == pytest.approx(910.0)
    assert comm.sent[-3:] == [
        "get_baud:14",
        "get_goal_acceleration:14",
        "get_current_lim:14",
    ]


def test_velocity_limit_api_uses_rpm_and_explicit_motor_id():
    exo, comm = _fake_exo()

    exo.set_motor_velocity_limit(14, 12.0)
    measured = exo.get_motor_velocity_limit(14)

    assert comm.sent[-2:] == ["set_goal_velocity:14:52", "get_goal_velocity:14"]
    assert measured == pytest.approx(52 * 0.229)


def test_hand_exo_command_observer_is_noninvasive_and_reports_acknowledgement():
    exo, comm = _fake_exo()
    events = []
    exo.add_command_observer(events.append)
    exo.add_command_observer(lambda _event: (_ for _ in ()).throw(RuntimeError("ignored")))

    exo.set_control_mode("current")

    assert comm.sent[-1] == "set_control_mode:all:current"
    assert [event["status"] for event in events[-2:]] == ["sent", "acknowledged"]
    assert events[-1]["command"] == "set_control_mode:all:current"


def test_gui_direct_command_respects_each_motor_row_limit():
    class Spin:
        def __init__(self, value):
            self._value = value

        def value(self):
            return self._value

    gui = SimpleNamespace(
        _direct_mode="velocity",
        motor_widgets=[{"dxl_id": 14, "velocity_limit_rpm": 3.0}],
    )
    assert HandExoGUI._limit_direct_command_for_motor(gui, 14, 8.0) == 3.0
    assert HandExoGUI._limit_direct_command_for_motor(gui, 14, -8.0) == -3.0

    gui._direct_mode = "current"
    gui.motor_widgets[0]["current_limit_spin"] = Spin(75)
    assert HandExoGUI._limit_direct_command_for_motor(gui, 14, 100.0) == 75.0


def test_synthetic_intent_sine_is_bounded_and_starts_at_zero():
    sine = EmgIntentDecoderWindow._sine_test_value

    assert sine(0.0, 0.25, 10.0) == pytest.approx(0.0)
    assert sine(2.5, 0.25, 10.0) == pytest.approx(0.25)
    assert sine(5.0, 0.25, 10.0) == pytest.approx(0.0, abs=1e-12)
    assert sine(7.5, 0.25, 10.0) == pytest.approx(-0.25)
    assert abs(sine(1.0, 5.0, 4.0)) <= 1.0


def test_synthetic_intent_test_starts_publishing_automatically():
    class Widget:
        def __init__(self, value=None):
            self.value = value

        def setText(self, value):
            self.value = value

    outlet = object()
    messages = []
    gui = SimpleNamespace(
        _test_signal_active=False,
        _outlet=None,
        _pipeline=object(),
        _toggle_publish=lambda: setattr(gui, "_outlet", outlet),
        test_signal_btn=Widget(),
        workflow_status=Widget(),
        test_amplitude_spin=SimpleNamespace(value=lambda: 0.25),
        test_period_spin=SimpleNamespace(value=lambda: 10.0),
        _log=messages.append,
    )

    EmgIntentDecoderWindow._toggle_test_signal(gui)

    assert gui._outlet is outlet
    assert gui._test_signal_active
    assert gui.test_signal_btn.value == "STOP SINE TEST"
    assert gui.workflow_status.value == "SYNTHETIC TEST"
    assert messages == ["Synthetic sine test started: amplitude=0.25, period=10.0 s"]


def test_gui_hold_action_runs_through_real_host_api_and_fake_firmware():
    exo, comm = _fake_exo()
    comm.angles[14] = 5.82

    class Widget:
        def __init__(self):
            self.value = None

        def setValue(self, value):
            self.value = value

        def setText(self, value):
            self.value = value

        def setStyleSheet(self, _value):
            pass

        def setChecked(self, value):
            self.value = value

        def isChecked(self):
            return bool(self.value)

    status = Widget()
    toggle = Widget()
    motor_status = Widget()
    enabled = Widget()
    enabled.setChecked(True)
    gui = SimpleNamespace(
        exo_connected=True,
        exo=exo,
        _emg_live=False,
        _emg_hold_active=False,
        _emg_hold_angle=None,
        _emg_aux_hold_supported=lambda: True,
        _selected_emg_hold_motor_id=lambda: 14,
        _fresh_cached_relative_angle=lambda _motor_id: 5.82,
        _emg_hold_target_spin=toggle,
        _emg_hold_current_lbl=Widget(),
        _emg_hold_enable_cb=enabled,
        _emg_hold_status_lbl=status,
        _emg_hold_ready_reason=lambda: None,
        motor_widgets=[{
            "dxl_id": 14,
            "enabled": False,
            "user_disabled": True,
            "toggle_btn": Widget(),
            "status_lbl": motor_status,
        }],
        _update_emg_safety_status=lambda: None,
        _log=lambda _message: None,
    )

    assert HandExoGUI._capture_emg_hold_angle(gui)
    assert HandExoGUI._engage_emg_position_hold(gui)
    assert comm.holds[14]["angle"] == pytest.approx(5.82)
    HandExoGUI._release_emg_position_hold(gui)
    assert 14 not in comm.holds


@pytest.mark.parametrize(
    "fault_reply",
    ["", "OK: unrelated", "Motor: {name: thumbrot, id: 13, angle: 1.0}"],
)
def test_angle_parser_reports_command_expectation_and_raw_reply(fault_reply):
    exo, comm = _fake_exo()
    comm.queue_fault(ReplyFault(command="get_angle:14", response=fault_reply))

    with pytest.raises(ProtocolResponseError) as caught:
        exo.get_motor_angle(14)

    message = str(caught.value)
    assert "Command: get_angle:14" in message
    assert "ID 14" in message
    assert (fault_reply or "<empty response>") in message


def test_transport_write_errors_are_raised_not_swallowed():
    exo, comm = _fake_exo()

    def fail(_message):
        raise OSError("USB disconnected")

    comm.send = fail
    with pytest.raises(ConnectionError, match="USB disconnected"):
        exo.get_motor_angle(14)


def test_disconnected_and_delayed_transports_surface_actionable_errors():
    exo, comm = _fake_exo()
    comm.queue_fault(
        ReplyFault(command="get_angle:14", response="late", delay_s=0.02)
    )
    exo.device.response_timeout = 0.001
    with pytest.raises(ProtocolResponseError, match="empty response"):
        exo._get_motor_attribute("angle", 14, True, command="get_angle")

    comm.close()
    with pytest.raises(ConnectionError, match="disconnected"):
        exo.get_motor_angle(14)


def test_new_query_flushes_unconsumed_setter_acknowledgement():
    exo, comm = _fake_exo()
    comm.angles[14] = 5.82

    exo.set_motor_velocity(15, 0.5)  # firmware queues ``OK: set_velocity``
    assert exo.get_motor_angle(14) == pytest.approx(5.82)


def test_synchronized_exo_serializes_complete_concurrent_transactions():
    exo, comm = _fake_exo()
    for motor_id in comm.motor_ids:
        comm.angles[motor_id] = float(motor_id)
    synchronized = SynchronizedHandExo(exo)

    with ThreadPoolExecutor(max_workers=6) as pool:
        results = list(pool.map(synchronized.get_motor_angle, comm.motor_ids))

    assert results == [float(mid) for mid in comm.motor_ids]


def test_serial_worker_poll_and_gui_read_share_one_serialized_transport():
    exo, comm = _fake_exo()
    comm.angles[14] = 5.82
    synchronized = SynchronizedHandExo(exo)
    worker = SerialWorker()
    worker.set_exo(synchronized)
    worker.set_motor_ids([14, 15])

    with ThreadPoolExecutor(max_workers=2) as pool:
        poll_future = pool.submit(
            worker._get_motor_attribute, "get_angle:all", "angle", 0.5
        )
        read_future = pool.submit(synchronized.get_motor_angle, 14)
        polled = poll_future.result()
        direct_value = read_future.result()

    assert direct_value == pytest.approx(5.82)
    assert polled[14] == pytest.approx(5.82)


def test_serial_worker_reports_raw_malformed_poll_reply():
    exo, comm = _fake_exo()
    synchronized = SynchronizedHandExo(exo)
    worker = SerialWorker()
    worker.set_exo(synchronized)
    comm.queue_fault(ReplyFault(command="get_angle:all", response="OK: unrelated"))

    with pytest.raises(ProtocolResponseError) as caught:
        worker._get_motor_attribute("get_angle:all", "angle", 0.1)

    assert "Command: get_angle:all" in str(caught.value)
    assert "Received: OK: unrelated" in str(caught.value)


def test_literal_host_commands_exist_in_firmware_parser():
    root = Path(__file__).resolve().parents[1]
    missing = missing_host_commands(
        root / "src/nml_hand_exo/interface/_hand_exo.py",
        root / "src/cpp/nml_hand_exo/utils.cpp",
    )
    assert not missing, f"Host commands absent from firmware parser: {sorted(missing)}"


def test_velocity_mode_verifies_motor_hardware_limit_before_mode_change():
    root = Path(__file__).resolve().parents[1]
    config = (root / "src/cpp/nml_hand_exo/config.h").read_text()
    firmware = (root / "src/cpp/nml_hand_exo/nml_hand_exo.cpp").read_text()
    parser = (root / "src/cpp/nml_hand_exo/utils.cpp").read_text()

    assert "DIRECT_VELOCITY_LIMIT_RAW = 218" in config
    assert "readControlTableItem(VELOCITY_LIMIT, id)" in firmware
    assert "writeControlTableItem(\n            VELOCITY_LIMIT, id, DIRECT_VELOCITY_LIMIT_RAW)" in firmware
    assert firmware.index("dxl_.torqueOff(motorIds_[i]);") < firmware.index(
        "ensureDirectVelocityLimit(motorIds_[i])"
    )
    assert 'motorControlMode_ = "DISABLED"' in firmware
    assert "if (dxl_.ping(motorIds_[i]) == 0) continue;" in firmware
    assert "!directVelocityLimitVerified_[index]" in firmware
    assert "motor mode change failed safety verification" in parser
