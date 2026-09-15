"""Connect failures must release CDC handles and identify the failing phase."""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
import serial

from nml_hand_exo.applications.hand_exo_gui import HandExoGUI
from nml_hand_exo.interface._interfaces import SerialComm, DualSerialComm

GUI = "nml_hand_exo.applications.hand_exo_gui"
TRANSPORT = "nml_hand_exo.interface._interfaces"


def test_single_port_has_finite_write_timeout():
    comm = SerialComm("COM10", 1000000)
    with patch(TRANSPORT + ".serial.Serial") as open_port:
        comm.connect()
    assert open_port.call_args.kwargs["write_timeout"] == 1.0
    comm.close()


@pytest.mark.parametrize("failure", ["second_open", "write", "probe"])
def test_dual_failure_closes_open_ports_and_allows_retry(failure):
    first, second = Mock(is_open=True), Mock(is_open=True)
    comm = DualSerialComm("COM10", "COM11", 1000000)
    if failure == "write":
        first.write.side_effect = serial.SerialTimeoutException("Write timeout")
    opens = [first, serial.SerialException("Access denied")] if failure == "second_open" else [first, second]
    with patch(TRANSPORT + ".serial.Serial", side_effect=opens) as open_port, \
         patch(TRANSPORT + ".time.sleep"), patch.object(comm, "_probe", return_value=False):
        with pytest.raises((serial.SerialException, ConnectionError)):
            comm.connect()
    first.close.assert_called()
    if failure != "second_open":
        second.close.assert_called()
    assert all(c.kwargs["write_timeout"] == 1.0 for c in open_port.call_args_list)

    # A later attempt opens fresh handles after the previous pair was closed.
    retry_first, retry_second = Mock(is_open=True), Mock(is_open=True)
    with patch(TRANSPORT + ".serial.Serial", side_effect=[retry_first, retry_second]), \
         patch(TRANSPORT + ".time.sleep"), patch.object(comm, "_probe", return_value=True), \
         patch.object(comm, "_start_reader"):
        comm.connect()
    assert comm._cmd is retry_first and comm._telem is retry_second
    comm.close()


def device():
    exo = Mock()
    exo.info.return_value = {"version": "0.9.0", "motors": {11: {"limits": [0, 100]}}}
    exo.get_hand_visualization_calibration.return_value = {11: {"home": 0}}
    return exo


@pytest.mark.parametrize("failure,stage", [
    ("connect", "Opening serial ports"), ("set_debug", "Disabling debug"),
    ("info", "Reading firmware information"), ("synchronize_utc", "Synchronizing UTC"),
])
def test_handshake_errors_close_device_and_name_stage(failure, stage):
    exo = device()
    getattr(exo, failure).side_effect = TimeoutError("no response")
    progress = []
    with patch(GUI + ".HandExo", return_value=exo) as construct, patch(GUI + ".time.sleep"):
        with pytest.raises(ConnectionError, match=stage):
            HandExoGUI._open_device_handshake(Mock(), "right", dual_active=True, progress=progress.append)
    assert construct.call_args.kwargs["auto_connect"] is False
    exo.close.assert_called_once()
    assert stage in progress[-1]


def test_missing_info_does_not_attempt_clock_or_calibration():
    exo = device()
    exo.info.return_value = {}
    with patch(GUI + ".HandExo", return_value=exo), patch(GUI + ".time.sleep"):
        with pytest.raises(ConnectionError, match="No motor records"):
            HandExoGUI._open_device_handshake(Mock(), "right", dual_active=True, progress=Mock())
    exo.synchronize_utc.assert_not_called()
    exo.get_hand_visualization_calibration.assert_not_called()
    exo.close.assert_called_once()


def test_optional_calibration_timeout_is_logged_and_connection_completes():
    exo = device()
    exo.get_hand_visualization_calibration.side_effect = TimeoutError("missing get_home reply")
    progress = []
    with patch(GUI + ".HandExo", return_value=exo), patch(GUI + ".time.sleep"):
        _, info = HandExoGUI._open_device_handshake(Mock(), "right", dual_active=True, progress=progress.append)
    assert info["hand_calibration"] == {}
    assert exo.get_hand_visualization_calibration.call_args.kwargs["timeout"] == 0.5
    assert any("missing get_home reply" in message for message in progress)
    assert progress[-1] == "Handshake complete"
    exo.close.assert_not_called()


def test_second_connect_is_ignored_while_first_is_pending():
    HandExoGUI._connect(SimpleNamespace(exo_connected=False, _connecting=True))
    HandExoGUI._refresh_ports(SimpleNamespace(_connecting=True))


@pytest.mark.parametrize("version,expects_utc", [("0.9.0", True), ("0.8.0", False)])
def test_handshake_finishes_clock_sync_before_returning_device_for_polling(version, expects_utc):
    exo = device()
    exo.info.return_value["version"] = version
    with patch(GUI + ".HandExo", return_value=exo), patch(GUI + ".time.sleep"):
        HandExoGUI._open_device_handshake(Mock(), "right", dual_active=True, progress=Mock())
    names = [c[0] for c in exo.mock_calls]
    assert ("synchronize_utc" in names) is expects_utc
    if expects_utc:
        assert names.index("info") < names.index("synchronize_utc") < names.index("get_hand_visualization_calibration")
    assert "get_fast_telemetry" not in names


def test_ui_refresh_cannot_reenable_connect_during_pending_attempt():
    gui = Mock(exo_connected=False, _connecting=True, _direct_arm_checkboxes={})
    gui.port_combo.count.return_value = 1
    HandExoGUI._update_enabled_state(gui)
    for name in ("connect_btn", "refresh_btn", "probe_btn", "port_combo"):
        getattr(gui, name).setEnabled.assert_called_with(False)
