"""Terminal UI behavior and commands through the actual protocol adapters."""
import io
import os
from datetime import datetime
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtCore import QEvent, QThread, Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QGroupBox

from nml_hand_exo.applications._device_terminal import DeviceTerminal
from nml_hand_exo.applications.hand_exo_gui import HandExoGUI, SerialWorker
from nml_hand_exo.interface import AutoSerialComm, HandExo, SerialComm


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app


@pytest.fixture
def terminal(qapp):
    widget = DeviceTerminal()
    yield widget
    widget.close()


def test_timestamps_toggle_all_lines_and_copy_clear_preserve_clock_state(terminal):
    terminal.append_message("first\n<literal device output>", datetime(2026, 10, 9, 14, 3, 2, 123456))
    assert terminal.output.isReadOnly()
    assert terminal.output.toPlainText() == "[14:03:02.123] first\n[14:03:02.123] <literal device output>"
    terminal.output.clock_button.setChecked(False)
    assert terminal.output.toPlainText() == "first\n<literal device output>"
    terminal.output.copy_button.click()
    assert QApplication.clipboard().text() == terminal.output.toPlainText()
    terminal.append_message("new", datetime(2026, 10, 9, 14, 3, 3, 987000))
    terminal.output.clock_button.setChecked(True)
    assert terminal.output.toPlainText().endswith("[14:03:03.987] new")
    terminal.output.clear_button.click()
    terminal.output.clock_button.setChecked(False)
    terminal.output.clock_button.setChecked(True)
    assert terminal.output.toPlainText() == ""


def test_modal_terminal_fades_when_inactive_and_preserves_session(terminal, qapp, monkeypatch):
    terminal.append_message("before opening")
    terminal.open_terminal()
    qapp.processEvents()
    assert terminal.isModal()
    assert terminal.windowOpacity() == terminal.ACTIVE_OPACITY
    monkeypatch.setattr(terminal, "isActiveWindow", lambda: False)
    QApplication.sendEvent(terminal, QEvent(QEvent.ActivationChange))
    assert terminal.windowOpacity() == pytest.approx(terminal.INACTIVE_OPACITY, abs=0.01)
    monkeypatch.setattr(terminal, "isActiveWindow", lambda: True)
    QApplication.sendEvent(terminal, QEvent(QEvent.ActivationChange))
    assert terminal.windowOpacity() == terminal.ACTIVE_OPACITY
    QApplication.sendEvent(terminal.output, QEvent(QEvent.Enter))
    assert terminal.output.tools.isVisible()
    QApplication.sendEvent(terminal.output, QEvent(QEvent.Leave))
    assert not terminal.output.tools.isVisible()
    terminal.close()
    terminal.append_message("while closed")
    terminal.open_terminal()
    assert "before opening" in terminal.output.toPlainText()
    assert "while closed" in terminal.output.toPlainText()


def test_connection_gates_input_and_return_history(terminal):
    received = []
    terminal.command_requested.connect(received.append)
    terminal.command_edit.setText("info")
    terminal.send_button.click()
    assert received == []
    terminal.set_connection(True, "protobuf-v1")
    assert "Protobuf" in terminal.protocol_label.text()
    QTest.keyClick(terminal.command_edit, Qt.Key_Return)
    assert received == ["info"]
    terminal.command_edit.setText("version;")
    terminal.send_button.click()
    assert received == ["info", "version"]
    terminal.command_edit.setText("unfinished")
    QTest.keyClick(terminal.command_edit, Qt.Key_Up)
    assert terminal.command_edit.text() == "version"
    QTest.keyClick(terminal.command_edit, Qt.Key_Up)
    assert terminal.command_edit.text() == "info"
    QTest.keyClick(terminal.command_edit, Qt.Key_Down)
    QTest.keyClick(terminal.command_edit, Qt.Key_Down)
    assert terminal.command_edit.text() == "unfinished"
    terminal.set_connection(False)
    assert not terminal.command_edit.isEnabled()


def test_terminal_is_global_and_replaces_both_checkboxes(qapp, monkeypatch):
    monkeypatch.setattr(QThread, "start", lambda *args: None)
    monkeypatch.setattr(HandExoGUI, "_load_stream_settings", lambda self: None)
    gui = HandExoGUI()
    try:
        titles = [box.title() for box in gui.findChildren(QGroupBox)]
        assert "Session Log" not in titles
        assert "Raw Serial Command (advanced)" not in titles
        gui._log("logged before opening")
        gui.show()
        for index in range(gui.main_tabs.count()):
            gui.main_tabs.setCurrentIndex(index)
            qapp.processEvents()
            assert gui._terminal_button.isVisible()
        gui._terminal_button.click()
        assert gui._terminal.isVisible()
        assert "logged before opening" in gui._terminal.output.toPlainText()
        assert not gui._terminal.send_button.isEnabled()
        gui.exo_connected = True
        gui._serial_worker.enqueue_terminal = Mock()
        gui._send_terminal_command("info;stop:all")
        gui._serial_worker.enqueue_terminal.assert_not_called()
        gui._send_terminal_command("info;")
        gui._serial_worker.enqueue_terminal.assert_called_once_with("info", timeout=2.0)
    finally:
        gui.exo_connected = False
        gui._terminal.close()
        gui.close()


def test_ascii_terminal_uses_serial_format_and_displays_reply(qapp):
    comm = SerialComm("fake", 1000000)
    comm.device = Mock(is_open=True)
    comm.device.read_until.return_value = b"firmware: 0.9.0;"
    worker = SerialWorker()
    worker.set_exo(HandExo(comm, command_delimiter="\r\n", send_delay=0))
    lines = []
    worker.line_received.connect(lines.append)
    worker._handle_terminal_command("version", .1)
    comm.device.write.assert_called_once_with(b"version\r\n")
    assert lines == ["firmware: 0.9.0"]


def protobuf_worker():
    pb = pytest.importorskip("nml_hand_exo.interface.exo_usb_pb2")
    from nml_hand_exo.interface._protobuf_serial import ProtobufSerialComm, encode_frame, read_frame
    backend = ProtobufSerialComm("text", "binary", 1000000, motor_ids=[16])
    backend._cmd = Mock(is_open=True)
    backend._telem = Mock(is_open=True)
    replies = io.BytesIO()
    requests = []

    def write(frame):
        nonlocal replies
        request = pb.Request.FromString(read_frame(io.BytesIO(frame), .1))
        requests.append(request)
        response = pb.Response(sequence=request.sequence, status=pb.OK)
        replies = io.BytesIO(encode_frame(response.SerializeToString()))
        return len(frame)

    backend._telem.write.side_effect = write
    backend._telem.read.side_effect = lambda size: replies.read(size)
    # Exercise the wrapper used by actual auto-detected GUI connections.
    comm = AutoSerialComm("text")
    comm.backend = backend
    comm.usb_protocol = backend.usb_protocol
    worker = SerialWorker()
    worker.set_exo(HandExo(comm, command_delimiter="\r\n", send_delay=0))
    return worker, backend, requests, pb


def test_protobuf_terminal_uses_real_rpc_frames_and_decoded_ack(qapp):
    worker, backend, requests, pb = protobuf_worker()
    lines = []
    worker.line_received.connect(lines.append)
    worker._handle_terminal_command("set_current:16:50", .1)
    assert len(requests) == 1
    assert requests[0].command == pb.SET_CURRENT
    assert list(requests[0].motor_ids) == [16]
    assert requests[0].value == 50
    backend._cmd.write.assert_not_called()
    assert lines == ["OK: set_current"]


def test_protobuf_unsupported_motion_reports_error_without_ascii_fallback(qapp):
    worker, backend, requests, pb = protobuf_worker()
    lines = []
    worker.line_received.connect(lines.append)
    worker._handle_terminal_command("set_gesture:index:flex", .1)
    assert not requests
    backend._cmd.write.assert_not_called()
    assert "lacks batch_motion" in lines[0]


def test_terminal_binary_telemetry_uses_sdk_decoder_not_text_transaction(qapp):
    worker = SerialWorker()
    raw = Mock()
    raw.get_fast_telemetry.return_value = {16: {"absolute_angle": 123.45}}
    worker.set_exo(raw)
    # A plain HandExo has no run_locked; match that rather than Mock's catch-all.
    raw.run_locked = None
    lines = []
    worker.line_received.connect(lines.append)
    worker._handle_terminal_command("get_telemetry_fast:16", .1)
    raw.get_fast_telemetry.assert_called_once_with(timeout=.1, motor_ids=[16])
    raw.device.send.assert_not_called()
    assert '"absolute_angle": 123.45' in lines[0]


def test_queued_terminal_command_does_not_replay_after_reconnect(qapp):
    worker = SerialWorker()
    worker.set_exo(object())
    worker.enqueue_terminal("home:16")
    worker.set_exo(object())
    worker._handle_terminal_command = Mock()
    worker._drain_async_events = lambda: None
    worker._urgent_q.put(("stop", None, None))
    worker.run()
    worker._handle_terminal_command.assert_not_called()
