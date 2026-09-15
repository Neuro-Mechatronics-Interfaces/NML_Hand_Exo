from datetime import datetime, timedelta, timezone
import struct
import json
from unittest.mock import patch

import pytest

from examples.tests.test_fast_telemetry import FakeFastTelemetryComm
from nml_hand_exo.interface._hand_exo import HandExo


def frame(sources=0x2a, error=0, utc=1789473600123, version=2):
    values = (11, error, -123, -4, 2048, 18025, -975)
    payload = struct.pack("<BBhiiiiBQI", *values, sources, utc, 1000) if version == 2 else struct.pack("<BBhiiii", *values)
    header = struct.pack("<2sBBBHIH", b"NX", version, 0x84, 1, len(payload), 1005, 0)
    checksum = (sum(header[:-2]) + sum(payload)) & 0xffff
    return header[:-2] + struct.pack("<H", checksum) + payload


def decode(data):
    exo = HandExo(FakeFastTelemetryComm(data), auto_connect=False, send_delay=0)
    return exo.get_fast_telemetry(timeout=0.1, motor_ids=[11])[11]


def api():
    exo = HandExo(FakeFastTelemetryComm(b""), auto_connect=False, send_delay=0)
    exo._firmware_version = (0, 9, 0)
    return exo


def test_v2_source_and_per_sample_utc():
    r = decode(frame())
    assert r["estimated"] and not r["error"]
    assert r["current"] == -123 and r["position_source"] == "estimated"
    assert r["torque_source"] == "estimated"
    assert r["utc_timestamp_ms"] == 1789473600123
    assert r["sample_timestamp_ms"] == 1000 and r["timestamp_ms"] == 1005
    assert r["utc_synchronized"]


def test_mixed_sources_unavailable_and_unsynchronized():
    r = decode(frame(sources=0x21, utc=0))
    assert r["position_source"] == "measured" and r["velocity_source"] == "estimated"
    assert r["current"] is None and r["current_source"] == "unavailable"
    assert not r["utc_synchronized"] and r["utc_timestamp_ms"] is None
    r = decode(frame(error=1))
    assert r["error"] and r["absolute_angle"] is None and r["velocity_raw"] is None
    assert not r["estimated"]


def test_v1_compatibility():
    r = decode(frame(version=1))
    assert r["current"] == -123 and r["current_source"] == "measured"
    assert r["sample_timestamp_ms"] == r["timestamp_ms"]
    assert r["utc_timestamp_ms"] is None


def test_bad_checksum_and_length_rejected():
    corrupt = bytearray(frame())
    corrupt[-1] ^= 1
    with pytest.raises(ValueError, match="checksum"):
        decode(corrupt)
    corrupt = bytearray(frame())
    corrupt[4] = 2  # count no longer matches payload length
    with pytest.raises(ValueError, match="length"):
        decode(corrupt)


def test_utc_format_timezone_conversion_and_get():
    exo = api()
    when = datetime(2026, 9, 15, 8, 0, 0, 123999, tzinfo=timezone(timedelta(hours=-4)))
    with patch.object(exo, "_command_transaction", return_value="OK: set_utc_time utc_ms=1789473600123 synchronized=1 uptime_ms=44;") as transact:
        result = exo.synchronize_utc(when)
    assert transact.call_args.args == ("set_utc_time:1789473600123",)
    assert result == dict(utc_ms=1789473600123, synchronized=True, uptime_ms=44)
    with patch.object(exo, "_command_transaction", return_value="UTC_TIME: utc_ms=0 synchronized=0 uptime_ms=80;"):
        assert exo.get_utc_time()["utc_ms"] is None


def test_utc_default_uses_host_clock_at_send_time():
    exo = api()
    with patch("nml_hand_exo.interface._hand_exo.time.time_ns", return_value=1789473600123456789), patch.object(
        exo, "_command_transaction", return_value="OK: set_utc_time utc_ms=1789473600123 synchronized=1 uptime_ms=44;"
    ) as transact:
        exo.synchronize_utc()
    assert transact.call_args.args == ("set_utc_time:1789473600123",)


@pytest.mark.parametrize("when", [datetime(2026, 1, 1), datetime(1960, 1, 1, tzinfo=timezone.utc), "2026-01-01"])
def test_utc_rejects_ambiguous_or_out_of_range_dates(when):
    exo = api()
    with patch.object(exo, "_command_transaction") as transact, pytest.raises(ValueError):
        exo.synchronize_utc(when)
    transact.assert_not_called()


@pytest.mark.parametrize("method,args", [("set_joint_model", (11,)), ("get_joint_model", (11,)),
    ("set_joint_trigger", (11, 0.2)), ("clear_joint_trigger", (11,)),
    ("set_estimate_holdoff", (250,)), ("synchronize_utc", ()), ("get_utc_time", ())])
def test_new_commands_gate_old_firmware(method, args):
    exo = api()
    exo._firmware_version = (0, 8, 0)
    with patch.object(exo, "_command_transaction") as transact, pytest.raises(RuntimeError, match="0.9.0"):
        getattr(exo, method)(*args)
    transact.assert_not_called()


def test_v091_joint_model_exposes_directional_response_learning():
    exo = api()
    reply = ('JOINT_MODEL: id=11 gain=0.1 time_constant=0.15 max_velocity=60 '
             'stiffness=0 moment=0 trigger=nan '
             'pulse_flex_slope=0.025 pulse_flex_samples=4 pulse_flex_no_motion=3 pulse_flex_gradients=2 '
             'pulse_extend_slope=nan pulse_extend_samples=0 pulse_extend_no_motion=1 pulse_extend_gradients=0')
    with patch.object(exo, '_command_transaction', return_value=reply):
        result = exo.get_joint_model(11)
    assert result['pulse_responses']['flex'] == dict(slope=.025,moving_samples=4,no_motion_samples=3,gradient_samples=2)
    assert result['pulse_responses']['extend']['slope'] is None


def test_model_roundtrip_and_threshold_commands():
    exo = api()
    with patch.object(exo, "_command_transaction", return_value="OK") as transact:
        exo.set_joint_model(11, moment=-0.25)
        assert transact.call_args.args == ("set_joint_model:11:0.1:0.15:60:0:-0.25",)
        exo.set_joint_trigger(11, 0.2)
        assert transact.call_args.args == ("set_joint_trigger:11:0.2",)
        exo.clear_joint_trigger(11)
        assert transact.call_args.args == ("clear_joint_trigger:11",)
    with patch.object(exo, "_command_transaction", return_value="JOINT_MODEL: id=11 gain=0.1 time_constant=0.15 max_velocity=60 stiffness=0 moment=-0.25 trigger=nan;"):
        result = exo.get_joint_model(11)
    assert result["trigger"] is None and result["moment"] == -0.25


@pytest.mark.parametrize("kwargs", [dict(gain=float("nan")), dict(moment=float("inf")),
    dict(time_constant=0), dict(max_velocity=301), dict(stiffness=-1)])
def test_model_invalid_inputs_do_not_send(kwargs):
    exo = api()
    with patch.object(exo, "_command_transaction") as transact, pytest.raises(ValueError):
        exo.set_joint_model(11, **kwargs)
    transact.assert_not_called()


@pytest.mark.parametrize("mid", ["wrist", "11", 11.5, True, 0, 254])
def test_model_requires_explicit_integer_ids(mid):
    with pytest.raises(ValueError):
        api().set_joint_model(mid)


def test_profile_tuning_is_applied_by_explicit_id(tmp_path):
    path = tmp_path / "profile.json"
    path.write_text(json.dumps({"motors": {"wrist": dict(home=180, limit_min=100,
        limit_max=270, flip=False, joint_model={"gain": 0.2}, joint_trigger=0.3)}}))
    exo = api()
    reply = "Motor 0: {name: wrist, id: 11, absolute_angle: 180, source: measured};"
    with patch.object(exo, "_receive", return_value=reply), patch.object(exo, "set_zero_offset"), \
         patch.object(exo, "set_motor_limits"), patch.object(exo, "set_flip"), \
         patch.object(exo, "set_joint_model") as model, patch.object(exo, "set_joint_trigger") as trigger:
        exo.apply_calibration(str(path), name_to_id={"wrist": 11})
    model.assert_called_once_with(11, gain=0.2, time_constant=0.15, max_velocity=60,
                                  stiffness=0, moment=0)
    trigger.assert_called_once_with(11, 0.3)


def test_invalid_profile_model_is_rejected_before_calibration_writes(tmp_path):
    path = tmp_path / "profile.json"
    path.write_text(json.dumps({"motors": {"wrist": dict(home=180, limit_min=100,
        limit_max=270, flip=False, joint_model={"gain": -1})}}))
    exo = api()
    with patch.object(exo, "send_command") as send, pytest.raises(ValueError):
        exo.apply_calibration(str(path), name_to_id={"wrist": 11})
    send.assert_not_called()


def test_calibration_waits_for_measured_epoch_before_mutations(tmp_path):
    path = tmp_path / "profile.json"
    path.write_text(json.dumps({"motors": {"wrist": dict(home=180, limit_min=100,
        limit_max=270, flip=False)}}))
    exo = api()
    replies = ["Motor 0: {name: wrist, id: 11, absolute_angle: 700, source: estimated};",
               "Motor 0: {name: wrist, id: 11, absolute_angle: 180, source: measured};"]
    with patch.object(exo, "_receive", side_effect=replies) as receive, \
         patch.object(exo, "set_zero_offset") as home, patch.object(exo, "set_motor_limits"), \
         patch.object(exo, "set_flip"), patch("nml_hand_exo.interface._hand_exo.time.sleep"):
        exo.apply_calibration(str(path), name_to_id={"wrist": 11})
    assert receive.call_count == 2
    home.assert_called_once_with(11, 180)
