from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from examples.tests.test_dual_serial_comm import FakeSerial
from nml_hand_exo.interface._interfaces import DualSerialComm
from nml_hand_exo.interface._hand_exo import HandExo
from nml_hand_exo.applications.hand_exo_gui import SerialWorker, HandExoGUI
from nml_hand_exo.calibration.rom import build_auto_rom_queue


RESULT = "ROM_CAL_RESULT: id=16 dir=flex home=100 endstop=150 current_mA=455 status=ok"


def transport():
    comm = DualSerialComm("FAKE_CMD", "FAKE_TELEM", 1000000, timeout=0.01)
    comm._cmd, comm._telem = FakeSerial(), FakeSerial()
    return comm


def test_reader_separates_rom_events_from_command_reply_frames():
    comm = transport()
    comm._start_reader()
    try:
        comm._telem.feed(RESULT + ";\r\nOK: calibrate_rom_gesture index dir=flex motors=1;\r\n")
        ack = comm.receive(wait_until_return=True, timeout=1)
        assert ack.startswith("OK: calibrate_rom_gesture")
        assert comm.drain_async_lines() == [RESULT]
        assert comm.drain_async_lines() == []
    finally:
        comm.close()


def test_pulse_observation_is_async_and_cannot_be_mistaken_for_final_result():
    comm = transport()
    exo = HandExo(comm, auto_connect=False, send_delay=0)
    pulse = 'ROM_CAL_PULSE: id=16 dir=flex predicted_deg=0.4 observed_deg=0.5'
    comm._publish_reply(pulse + '\nOK: get_joint_model')
    comm._publish_reply(RESULT)
    assert comm.receive() == 'OK: get_joint_model'
    assert exo.read_rom_result(timeout=.1)['endstop'] == 150
    assert comm.drain_async_lines() == []
    comm.close()


def test_telemetry_command_flush_preserves_completion_and_gui_advances_without_timeout():
    comm = transport()
    exo = HandExo(comm, auto_connect=False, send_delay=0)
    gui = SimpleNamespace(_rom_running=True, _rom_current=("index", "flex", 16),
                          _rom_expected=1, _rom_seen=0, _rom_results=[],
                          _mark_rom_endstop=Mock(), _rom_result_timer=Mock(), _rom_send_next=Mock())
    worker = SerialWorker()
    worker.set_exo(exo)
    worker.line_received.connect(lambda line: HandExoGUI._on_rom_serial_line(gui, line))
    comm._publish_reply(RESULT)
    comm._publish_reply("OK: old acknowledgement")
    # This is the flush performed before each NX telemetry poll.
    exo.send_command("get_telemetry_fast:16")
    assert comm.receive() == ""
    worker._drain_async_events()
    assert gui._rom_seen == 1 and gui._rom_results[0]["endstop"] == 150
    gui._rom_result_timer.stop.assert_called_once()
    gui._rom_send_next.assert_called_once()
    worker._drain_async_events()
    assert gui._rom_seen == 1  # every result is delivered once
    comm.close()


def test_early_completion_survives_arming_ack():
    gui = SimpleNamespace(_rom_running=True, _rom_current=("index", "flex", 16),
                          _rom_expected=1, _rom_seen=0, _rom_results=[],
                          _mark_rom_endstop=Mock(), _rom_result_timer=Mock(), _rom_send_next=Mock())
    HandExoGUI._on_rom_serial_line(gui, RESULT)
    HandExoGUI._on_rom_serial_line(gui, "OK: calibrate_rom id=16 dir=flex")
    assert gui._rom_seen == 1
    gui._rom_send_next.assert_called_once()


def test_blocking_sdk_consumes_one_event_at_a_time_and_times_out_when_empty():
    comm = transport()
    exo = HandExo(comm, auto_connect=False)
    comm._publish_reply(RESULT)
    comm._publish_reply(RESULT.replace("id=16", "id=17"))
    assert exo.read_rom_result(timeout=0.01)["id"] == 16
    assert exo.read_rom_result(timeout=0.01)["id"] == 17
    with pytest.raises(TimeoutError, match="No ROM_CAL_RESULT"):
        exo.read_rom_result(timeout=0.01)
    comm.close()


@pytest.mark.parametrize("mode,base,count", [("Right Only", 10, 18), ("Left Only", 0, 18), ("Dual", None, 36)])
def test_auto_rom_queues_only_selected_side_ids(mode, base, count):
    names = ["wrist", "wrist2", "thumbadd", "thumbrot", "thumbflex", "index", "middle", "ring", "pinky"]
    motors = [(offset + i, name) for offset in (0, 10) for i, name in enumerate(names, 1)]
    queue = build_auto_rom_queue(["thumb", "index", "middle", "ring", "pinky", "wrist"], ["flex", "extend"], motors, mode)
    assert len(queue) == count
    if base is not None:
        assert {entry[2] for entry in queue} == set(range(base + 1, base + 10))
    assert len([entry for entry in queue if entry[0] == "thumb"]) == (12 if mode == "Dual" else 6)


def test_rom_send_uses_single_id_command_and_exact_expected_result():
    gui = SimpleNamespace(_rom_queue=[("thumb", "flex", 13)], _rom_status_lbl=Mock(),
                          _serial_worker=Mock(), _rom_result_timer=Mock())
    HandExoGUI._rom_send_next(gui)
    gui._serial_worker.enqueue.assert_called_once_with("calibrate_rom:13:flex", timeout=3.0)
    assert gui._rom_expected == 1 and gui._rom_current == ("thumb", "flex", 13)


def test_wrong_hand_and_direction_results_do_not_advance_rom():
    gui = SimpleNamespace(_rom_running=True, _rom_current=("index", "flex", 16),
                          _rom_expected=1, _rom_seen=0, _rom_results=[],
                          _mark_rom_endstop=Mock(), _rom_result_timer=Mock(), _rom_send_next=Mock())
    HandExoGUI._on_rom_serial_line(gui, RESULT.replace("id=16", "id=6"))
    HandExoGUI._on_rom_serial_line(gui, RESULT.replace("dir=flex", "dir=extend"))
    assert not gui._rom_results
    gui._rom_send_next.assert_not_called()


def test_refused_motor_aborts_without_waiting_for_result_timer():
    gui = SimpleNamespace(_rom_running=True, _cancel_rom_calibration=Mock(),
                          _rom_status_lbl=Mock(), _log=Mock())
    HandExoGUI._on_rom_serial_line(gui, "ERROR: calibrate_rom needs CURRENT mode, a reachable idle joint")
    gui._cancel_rom_calibration.assert_called_once()


def test_timeout_cancels_instead_of_starting_another_motor():
    gui = SimpleNamespace(_rom_running=True, _rom_current=("index", "flex", 16),
                          _cancel_rom_calibration=Mock(), _rom_status_lbl=Mock(), _log=Mock(), _rom_send_next=Mock())
    HandExoGUI._rom_result_timeout(gui)
    gui._cancel_rom_calibration.assert_called_once()
    gui._rom_send_next.assert_not_called()


def test_pulsed_result_retains_model_fit_metadata_and_limit_has_no_endstop():
    result = HandExo._parse_rom_result(
        "ROM_CAL_RESULT: id=16 dir=flex home=100 endstop=NaN current_mA=150 "
        "status=limit pulses=15 fit_samples=8 model_gain=0.163;")
    assert result["status"] == "limit" and result["endstop"] is None
    assert result["pulses"] == 15 and result["fit_samples"] == 8
    assert result["model_gain"] == 0.163


@pytest.mark.parametrize("status", ["aborted", "timeout"])
def test_fault_result_stops_gui_campaign(status):
    gui = SimpleNamespace(_rom_running=True, _rom_current=("index", "flex", 16),
                          _rom_results=[], _cancel_rom_calibration=Mock(),
                          _rom_status_lbl=Mock(), _rom_send_next=Mock())
    HandExoGUI._on_rom_serial_line(gui, RESULT.replace("status=ok", f"status={status}"))
    gui._cancel_rom_calibration.assert_called_once()
    gui._rom_send_next.assert_not_called()


def test_rom_diagnostics_explain_the_abort_and_survive_parsing():
    result = HandExo._parse_rom_result(
        "ROM_CAL_RESULT: id=14 dir=flex home=247.28 endstop=nan current_mA=90 "
        "status=aborted pulses=2 fit_samples=0 model_gain=0.100000 "
        "reason=pulse_overrun pulse_on_ms=204 max_pulse_on_ms=204 recoveries=0 "
        "angle=246.92 limit_min=160.26 limit_max=260.86 "
        "fit_reason=velocity_limit pulse_stop=speed max_excursion_deg=1.25 "
        "predicted_deg=nan observed_deg=1.25 response_reason=pulse_limited;")
    assert result["reason"] == "pulse_overrun"
    assert result["pulse_on_ms"] == result["max_pulse_on_ms"] == 204
    assert result["recoveries"] == 0 and result["angle"] == 246.92
    assert result["limit_min"] == 160.26 and result["limit_max"] == 260.86
    assert result['fit_reason'] == 'velocity_limit' and result['pulse_stop'] == 'speed'
    assert result['max_excursion_deg'] == 1.25 and result['predicted_deg'] is None
    assert result['observed_deg'] == 1.25 and result['response_reason'] == 'pulse_limited'


def test_ceiling_summary_reports_encoder_travel_without_an_endstop():
    gui = SimpleNamespace(_rom_running=True, _rom_current=("thumb", "extend", 14),
                          _rom_expected=1, _rom_seen=0, _rom_results=[], _log=Mock(),
                          _mark_rom_endstop=Mock(), _rom_result_timer=Mock(), _rom_send_next=Mock())
    line = ("ROM_CAL_RESULT: id=14 dir=extend home=236.02 endstop=nan current_mA=80 "
            "status=ceiling pulses=33 fit_samples=0 reason=current_ceiling angle=236.95 "
            "net_travel_deg=0.930 response_samples=0 no_motion_samples=33 "
            "velocity_deg_s=1.374 velocity_raw=1 settle_check=rest "
            "settle_span_deg=0.088 settle_quiet_ms=200;")
    HandExoGUI._on_rom_serial_line(gui, line)
    result = gui._rom_results[0]
    assert result['endstop'] is None and result['response_samples'] == 0
    assert result['velocity_raw'] == 1 and result['velocity_deg_s'] == 1.374
    assert result['settle_check'] == 'rest' and result['settle_quiet_ms'] == 200
    messages = [call.args[0] for call in gui._log.call_args_list]
    assert any('+0.93 motor-encoder degrees' in message for message in messages)
    assert any('pulse ceiling' in message for message in messages)
    gui._rom_send_next.assert_called_once()


def test_saturation_result_retains_prior_motion_but_never_claims_an_endstop():
    gui = SimpleNamespace(_rom_running=True, _rom_current=("thumb", "flex", 15),
                          _rom_expected=1, _rom_seen=0, _rom_results=[], _log=Mock(),
                          _mark_rom_endstop=Mock(), _rom_result_timer=Mock(), _rom_send_next=Mock())
    HandExoGUI._on_rom_serial_line(gui,
        "ROM_CAL_RESULT: id=15 dir=flex home=122.85 endstop=nan current_mA=160 "
        "status=ceiling reason=response_saturated net_travel_deg=23.666 "
        "response_samples=36 ceiling_no_motion_pulses=6;")
    result = gui._rom_results[0]
    assert result['endstop'] is None and result['ceiling_no_motion_pulses'] == 6
    assert result['response_samples'] == 36
    assert any('repeated low-response pulses' in call.args[0] for call in gui._log.call_args_list)
    gui._rom_send_next.assert_called_once()
