import time
from types import SimpleNamespace

import pytest

from nml_hand_exo.applications.hand_exo_gui import HandExoGUI


class _SpinValue:
    def __init__(self, value):
        self._value = value

    def value(self):
        return self._value


class _Label:
    def __init__(self):
        self.text = None

    def setText(self, text):
        self.text = text

    def setStyleSheet(self, _style):
        pass


class _Combo:
    def __init__(self, data, text=""):
        self._data = data
        self._text = text

    def currentData(self):
        return self._data

    def currentText(self):
        return self._text

    def currentIndex(self):
        return self._data if isinstance(self._data, int) else 0


class _CheckBox:
    def __init__(self, checked=False):
        self._checked = checked

    def isChecked(self):
        return self._checked


class _Exo:
    def __init__(self):
        self.commands = []

    def send_command(self, cmd):
        self.commands.append(cmd)


def _base_gui(**overrides):
    exo = overrides.pop("exo", _Exo())
    defaults = dict(
        exo_connected=True,
        exo=exo,
        _gate_live=True,
        _gate_last_decision=None,
        _gate_sample_lbl=_Label(),
        _gate_stale_ms_spin=_SpinValue(300),
        _gate_confidence_spin=_SpinValue(0.5),
        _gate_mode_combo=_Combo(0),
        _gate_posture_combo=_Combo("grasp"),
        _gate_motor_checks={},
        mode_combo=_Combo(None, "Right Only"),
        _log=lambda _text: None,
    )
    defaults.update(overrides)
    return SimpleNamespace(**defaults)


# -- Sample handling / edge detection ---------------------------------------

def test_invalid_sample_reports_and_does_not_dispatch():
    dispatched = []
    gui = _base_gui(_dispatch_gate_decision=lambda d: dispatched.append(d))

    HandExoGUI._on_gate_intent_sample(gui, {"values": [0.0, 0.0, 1.0]})

    assert "Invalid NMLIntentV1 sample" in gui._gate_sample_lbl.text
    assert dispatched == []


def test_stale_sample_is_ignored_while_updating_label():
    dispatched = []
    gui = _base_gui(_dispatch_gate_decision=lambda d: dispatched.append(d))
    sample = {
        "values": [1.0, 1.0, 0.9, 1.0],
        "received_monotonic": time.monotonic() - 1.0,
    }

    HandExoGUI._on_gate_intent_sample(gui, sample)

    assert "state=1" in gui._gate_sample_lbl.text
    assert dispatched == []


def test_low_confidence_sample_is_ignored():
    dispatched = []
    gui = _base_gui(_dispatch_gate_decision=lambda d: dispatched.append(d))
    sample = {
        "values": [1.0, 1.0, 0.1, 1.0],
        "received_monotonic": time.monotonic(),
    }
    gui._gate_confidence_spin = _SpinValue(0.5)

    HandExoGUI._on_gate_intent_sample(gui, sample)

    assert dispatched == []


def test_not_live_never_dispatches_even_with_a_good_sample():
    dispatched = []
    gui = _base_gui(
        _gate_live=False, _dispatch_gate_decision=lambda d: dispatched.append(d)
    )
    sample = {
        "values": [1.0, 1.0, 0.9, 1.0],
        "received_monotonic": time.monotonic(),
    }

    HandExoGUI._on_gate_intent_sample(gui, sample)

    assert dispatched == []


def test_transition_to_one_dispatches_exactly_once():
    dispatched = []
    gui = _base_gui(_dispatch_gate_decision=lambda d: dispatched.append(d))
    sample = {
        "values": [1.0, 1.0, 0.9, 1.0],
        "received_monotonic": time.monotonic(),
    }

    HandExoGUI._on_gate_intent_sample(gui, sample)
    HandExoGUI._on_gate_intent_sample(gui, sample)  # repeated state: no re-fire

    assert dispatched == [1]
    assert gui._gate_last_decision == 1


def test_transition_back_to_zero_dispatches_again():
    dispatched = []
    gui = _base_gui(_dispatch_gate_decision=lambda d: dispatched.append(d))
    on_sample = {"values": [1.0, 1.0, 0.9, 1.0], "received_monotonic": time.monotonic()}
    off_sample = {"values": [0.0, 0.0, 0.9, 0.0], "received_monotonic": time.monotonic()}

    HandExoGUI._on_gate_intent_sample(gui, on_sample)
    HandExoGUI._on_gate_intent_sample(gui, off_sample)

    assert dispatched == [1, 0]


def test_non_finite_state_is_ignored():
    dispatched = []
    gui = _base_gui(_dispatch_gate_decision=lambda d: dispatched.append(d))
    sample = {
        "values": [1.0, 1.0, 0.9, float("nan")],
        "received_monotonic": time.monotonic(),
    }

    HandExoGUI._on_gate_intent_sample(gui, sample)

    assert dispatched == []


# -- Dispatch: posture mode ---------------------------------------------------

def test_posture_mode_close_sends_set_gesture_close():
    exo = _Exo()
    gui = _base_gui(exo=exo, _gate_mode_combo=_Combo(0), _gate_posture_combo=_Combo("grasp"))

    HandExoGUI._dispatch_gate_decision(gui, 1)

    assert exo.commands == ["set_gesture:grasp:close"]


def test_posture_mode_open_sends_set_gesture_open():
    exo = _Exo()
    gui = _base_gui(exo=exo, _gate_mode_combo=_Combo(0), _gate_posture_combo=_Combo("keygrip"))

    HandExoGUI._dispatch_gate_decision(gui, 0)

    assert exo.commands == ["set_gesture:keygrip:open"]


def test_dispatch_does_nothing_when_not_connected():
    exo = _Exo()
    gui = _base_gui(exo=exo, exo_connected=False)

    HandExoGUI._dispatch_gate_decision(gui, 1)

    assert exo.commands == []


# -- Dispatch: custom motor mode, per-motor invert ---------------------------

def test_custom_motors_only_command_included_motors():
    exo = _Exo()
    gui = _base_gui(
        exo=exo,
        _gate_mode_combo=_Combo(1),
        _gate_motor_checks={
            "index": (_CheckBox(True), _CheckBox(False)),
            "wrist": (_CheckBox(False), _CheckBox(False)),
        },
    )

    HandExoGUI._dispatch_gate_decision(gui, 1)

    assert exo.commands == ["set_gesture:index:flex"]


def test_custom_motors_default_polarity_zero_is_extend_one_is_flex():
    exo = _Exo()
    gui = _base_gui(
        exo=exo,
        _gate_mode_combo=_Combo(1),
        _gate_motor_checks={"index": (_CheckBox(True), _CheckBox(False))},
    )

    HandExoGUI._dispatch_gate_decision(gui, 0)
    HandExoGUI._dispatch_gate_decision(gui, 1)

    assert exo.commands == ["set_gesture:index:extend", "set_gesture:index:flex"]


def test_custom_motors_invert_flips_polarity_independently():
    exo = _Exo()
    gui = _base_gui(
        exo=exo,
        _gate_mode_combo=_Combo(1),
        _gate_motor_checks={
            "index": (_CheckBox(True), _CheckBox(False)),  # normal
            "wrist": (_CheckBox(True), _CheckBox(True)),   # inverted
        },
    )

    HandExoGUI._dispatch_gate_decision(gui, 1)

    assert set(exo.commands) == {"set_gesture:index:flex", "set_gesture:wrist:extend"}


def test_custom_motors_can_control_just_index_and_wrist_together():
    exo = _Exo()
    gui = _base_gui(
        exo=exo,
        _gate_mode_combo=_Combo(1),
        _gate_motor_checks={
            "index": (_CheckBox(True), _CheckBox(False)),
            "wrist": (_CheckBox(True), _CheckBox(False)),
            "middle": (_CheckBox(False), _CheckBox(False)),
        },
    )

    HandExoGUI._dispatch_gate_decision(gui, 1)

    assert set(exo.commands) == {"set_gesture:index:flex", "set_gesture:wrist:flex"}


# -- Dual mode target application --------------------------------------------

def test_dual_mode_applies_target_motors_before_dispatch():
    exo = _Exo()
    applied = []
    gui = _base_gui(
        exo=exo,
        mode_combo=_Combo(None, "Dual"),
        _gate_target_combo=_Combo(None, "Left Only"),
        _apply_gesture_target_motors=lambda target: applied.append(target),
    )

    HandExoGUI._dispatch_gate_decision(gui, 1)

    assert applied == ["Left Only"]
    assert exo.commands == ["set_gesture:grasp:close"]


# -- START/STOP gating --------------------------------------------------------

def test_start_blocked_when_not_connected(monkeypatch):
    gui = _base_gui(exo_connected=False, _gate_live=False)
    warnings = []
    monkeypatch.setattr(
        "nml_hand_exo.applications.hand_exo_gui.QMessageBox.warning",
        lambda *a, **k: warnings.append(a),
    )

    HandExoGUI._on_gate_live_toggled(gui, True)

    assert gui._gate_live is False  # start bailed out before arming
    assert warnings


def test_start_blocked_in_custom_mode_with_no_motors_selected(monkeypatch):
    warnings = []
    gui = _base_gui(
        _gate_live=False,
        _gate_mode_combo=_Combo(1),
        _gate_motor_checks={"index": (_CheckBox(False), _CheckBox(False))},
    )
    monkeypatch.setattr(
        "nml_hand_exo.applications.hand_exo_gui.QMessageBox.warning",
        lambda *a, **k: warnings.append(a),
    )

    HandExoGUI._on_gate_live_toggled(gui, True)

    assert gui._gate_live is False
    assert warnings


def test_start_calls_ensure_gesture_ready_single_mode():
    calls = []
    gui = _base_gui(
        _gate_live=False,
        _ensure_gesture_ready=lambda **kwargs: calls.append(kwargs),
        _gate_start_btn=_Label(),
        _gate_stop_btn=_Label(),
        _gate_live_status_lbl=_Label(),
    )
    gui._gate_start_btn.setEnabled = lambda _v: None
    gui._gate_stop_btn.setEnabled = lambda _v: None

    HandExoGUI._on_gate_live_toggled(gui, True)

    assert calls == [{}]
    assert gui._gate_live is True


def test_stop_resets_live_state():
    gui = _base_gui(
        _gate_live=True,
        _gate_last_decision=1,
        _gate_start_btn=_Label(),
        _gate_stop_btn=_Label(),
        _gate_live_status_lbl=_Label(),
    )
    gui._gate_start_btn.setEnabled = lambda _v: None
    gui._gate_stop_btn.setEnabled = lambda _v: None

    HandExoGUI._on_gate_live_toggled(gui, False)

    assert gui._gate_live is False
    assert gui._gate_last_decision is None
