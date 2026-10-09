"""Staged home acknowledgement and final-settle handoff, without hardware."""
import os
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from nml_hand_exo.applications.hand_exo_gui import HandExoGUI, SerialWorker, HOME_GROUP_SETTLE_MS


@pytest.fixture(scope='module', autouse=True)
def qapp():
    # Keep the QApplication alive while constructing Qt workers in this module.
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    yield app


def test_worker_sends_coupled_group_before_waiting_and_ignores_older_enable_ack():
    worker = SerialWorker()
    exo = Mock()
    events = []
    exo.run_locked = lambda callback: callback(exo)
    exo.send_command.side_effect = lambda command: events.append(command)
    replies = iter(['OK: enable 15;', 'OK: home 13;', 'OK: home 14;', 'OK: home 15;'])
    def receive(**kwargs):
        events.append('read')
        return next(replies)
    exo._receive.side_effect = receive
    worker.set_exo(exo)
    worker.set_home_generation(1)
    result = []
    worker.home_stage_completed.connect(lambda *args: result.append(args))
    worker._handle_home_stage(1, [13,14,15])
    assert events == ['home:13\nhome:14\nhome:15', 'read', 'read', 'read', 'read']
    assert result == [(1, '')]


def test_cancelled_queued_home_stage_never_sends():
    worker = SerialWorker()
    exo = Mock()
    worker.set_exo(exo)
    worker.set_home_generation(2)
    worker._handle_home_stage(1, [19])
    exo.send_command.assert_not_called()


def test_cancel_during_ack_wait_returns_without_continuing_group():
    worker = SerialWorker()
    exo = Mock()
    exo.run_locked = lambda callback: callback(exo)
    def receive(**kwargs):
        worker.set_home_generation(2)
        return ''
    exo._receive.side_effect = receive
    worker.set_exo(exo)
    worker.set_home_generation(1)
    result = []
    worker.home_stage_completed.connect(lambda *args: result.append(args))
    worker._handle_home_stage(1, [19])
    exo._receive.assert_called_once()
    assert not result


def test_home_rejection_prevents_success_signal():
    worker = SerialWorker()
    exo = Mock()
    exo.run_locked = lambda callback: callback(exo)
    exo._receive.return_value = 'ERROR: assist owns bus;'
    worker.set_exo(exo)
    worker.set_home_generation(1)
    result = []
    worker.home_stage_completed.connect(lambda *args: result.append(args))
    worker._handle_home_stage(1, [19])
    assert result == [(1, 'ERROR: assist owns bus')]
    exo.send_command.assert_called_once_with('home:19')


def test_missing_home_ack_aborts_without_replaying_command(monkeypatch):
    worker = SerialWorker()
    exo = Mock()
    exo.run_locked = lambda callback: callback(exo)
    exo._receive.return_value = ''
    worker.set_exo(exo)
    worker.set_home_generation(1)
    result = []
    worker.home_stage_completed.connect(lambda *args: result.append(args))
    clock = iter([0, 1, 2])
    monkeypatch.setattr('nml_hand_exo.applications.hand_exo_gui.time.monotonic', lambda: next(clock))
    worker._handle_home_stage(1, [19])
    assert result == [(1, 'Home acknowledgement missing for IDs [19]')]
    exo.send_command.assert_called_once_with('home:19')


def gui():
    return SimpleNamespace(_home_generation=1, _home_sequence_active=True,
        _home_groups_pending=[[19]], _home_groups_total=6, exo_connected=True,
        exo=Mock(), _serial_worker=Mock(), _home_timer=Mock(), _log=Mock(),
        _finish_home_sequence=Mock())


def test_final_home_group_does_not_resume_polling_until_ack_and_settle():
    window = gui()
    HandExoGUI._home_next_group(window)
    window._serial_worker.enqueue_home_stage.assert_called_once_with(1, [19])
    window._home_timer.start.assert_not_called()
    window._finish_home_sequence.assert_not_called()
    HandExoGUI._on_home_stage_completed(window, 1, '')
    window._home_timer.start.assert_called_once_with(HOME_GROUP_SETTLE_MS)
    window._finish_home_sequence.assert_not_called()
    HandExoGUI._home_next_group(window) # settle timer expires
    window._finish_home_sequence.assert_called_once()


def test_stale_home_ack_cannot_restart_timer_after_cancellation():
    window = gui()
    window._home_generation = 2
    window._home_sequence_active = False
    HandExoGUI._on_home_stage_completed(window, 1, '')
    HandExoGUI._home_next_group(window)
    window._home_timer.start.assert_not_called()
    window._finish_home_sequence.assert_not_called()


def test_finishing_idle_home_does_not_log_spurious_sequence_complete():
    window = gui()
    window._home_sequence_active = False
    window.home_all_btn = Mock()
    window._resume_normal_polling = Mock()
    HandExoGUI._finish_home_sequence(window, resume_polling=False)
    window._log.assert_not_called()
    window._serial_worker.set_home_generation.assert_called_once_with(2)
    window._resume_normal_polling.assert_not_called()
