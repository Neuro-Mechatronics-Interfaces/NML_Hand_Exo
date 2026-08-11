import time
from types import SimpleNamespace

from nml_hand_exo.applications.hand_exo_gui import (
    EMG_FAST_TELEMETRY_TIMEOUT_S,
    HandExoGUI,
    SerialWorker,
)


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


class _Timer:
    def __init__(self):
        self.stopped = False
        self.started_with = None

    def stop(self):
        self.stopped = True

    def start(self, interval=None):
        self.started_with = interval


class _DirectWorker:
    def __init__(self):
        self.requests = []

    def request_direct_actions(self, actions):
        self.requests.append(dict(actions))


class _Combo:
    def __init__(self, data, text=""):
        self._data = data
        self._text = text

    def currentData(self):
        return self._data

    def currentText(self):
        return self._text

    def currentIndex(self):
        return 0


class _MutableCombo:
    def __init__(self, items):
        self.items = [[text, data] for text, data in items]
        self.index = 0

    def count(self):
        return len(self.items)

    def itemData(self, index):
        return self.items[index][1]

    def setItemText(self, index, text):
        self.items[index][0] = text

    def setCurrentIndex(self, index):
        self.index = index

    def currentData(self):
        return self.items[self.index][1]

    def currentText(self):
        return self.items[self.index][0]


def test_stale_intent_waits_without_disarming_teleop():
    stop_calls = []
    gui = SimpleNamespace(
        _emg_live=True,
        _emg_deadman_active=True,
        _emg_ready_reason=lambda: None,
        _emg_latest={"received_monotonic": time.monotonic() - 1.0},
        _emg_stale_ms_spin=_SpinValue(200),
        _stop_emg_control=lambda reason, **kwargs: stop_calls.append(
            (reason, kwargs)
        ),
    )

    HandExoGUI._emg_control_tick(gui)

    assert stop_calls == [("intent sample is stale", {"keep_live": True})]


def test_waiting_state_zeros_motor_but_keeps_control_loop_live():
    worker = _DirectWorker()
    timer = _Timer()
    status = _Label()
    command = _Label()
    gui = SimpleNamespace(
        _emg_live=True,
        _emg_deadman_active=True,
        _emg_control_timer=timer,
        _emg_commanded_ids={11},
        _emg_last_command_id=11,
        exo_connected=True,
        _serial_worker=worker,
        _emg_live_status_lbl=status,
        _emg_command_lbl=command,
        _direct_mode="velocity",
    )

    HandExoGUI._stop_emg_control(
        gui, "intent sample is stale", keep_live=True
    )

    assert gui._emg_live is True
    assert gui._emg_deadman_active is True
    assert timer.stopped is False
    assert worker.requests == [{11: ("stop", None)}]
    assert status.text == "Waiting: intent sample is stale"
    assert command.text == "Commanded output: 0.00 rpm"


def test_direct_mode_keeps_live_polling_at_capped_rate():
    timer = _Timer()
    poll_calls = []
    gui = SimpleNamespace(
        exo_connected=True,
        _teleop_streaming=False,
        _suspend_device_poll_requests=False,
        _direct_mode="velocity",
        _telemetry_rate_spin=_SpinValue(50),
        _angle_timer=timer,
        _request_device_poll=lambda **kwargs: poll_calls.append(kwargs),
    )

    HandExoGUI._start_device_polling(gui, force_refresh=True)

    assert timer.started_with == 100
    assert poll_calls == [{"force_telemetry": True}]


def test_position_mode_uses_configured_live_polling_rate():
    gui = SimpleNamespace(
        _direct_mode=None,
        _telemetry_rate_spin=_SpinValue(50),
    )

    assert HandExoGUI._device_poll_interval_ms(gui) == 20


def test_emg_control_reduces_telemetry_to_two_hz():
    gui = SimpleNamespace(
        _emg_live=True,
        _direct_mode="velocity",
        _telemetry_rate_spin=_SpinValue(50),
    )

    assert HandExoGUI._device_poll_interval_ms(gui) == 500


def test_connect_sync_does_not_replace_gui_velocity_ceiling_from_profile_register():
    class Spin:
        def __init__(self, value):
            self.current = value

        def setValue(self, value):
            self.current = value

    current_spin = Spin(150)
    velocity_spin = Spin(50.0)
    motor = {
        "dxl_id": 15,
        "current_limit_spin": current_spin,
        "velocity_limit_spin": velocity_spin,
        "velocity_limit_rpm": 50.0,
    }
    raw_exo = SimpleNamespace(
        get_motor_current_limit=lambda target: {15: 200}
    )
    gui = SimpleNamespace(
        exo_connected=True,
        motor_widgets=[motor],
        exo=SimpleNamespace(run_locked=lambda callback: callback(raw_exo)),
        _log=lambda _message: None,
    )

    HandExoGUI._sync_motor_limits_after_connect(gui)

    assert current_spin.current == 200
    assert velocity_spin.current == 50.0
    assert motor["velocity_limit_rpm"] == 50.0


def test_realtime_telemetry_failure_never_enters_text_fallback():
    fast_calls = []
    fallback_calls = []
    worker = SerialWorker()
    worker.set_exo(object())
    worker.set_realtime_control(True, [15, 16, 17, 18, 19])

    def fail_fast(timeout, motor_ids=None):
        fast_calls.append((timeout, motor_ids))
        raise TimeoutError("synthetic fast telemetry timeout")

    worker._get_fast_telemetry = fail_fast
    worker._get_motor_attribute = lambda *args: fallback_calls.append(args)

    worker._handle_poll(include_telemetry=True)

    assert fast_calls == [
        (EMG_FAST_TELEMETRY_TIMEOUT_S, [15, 16, 17, 18, 19])
    ]
    assert fallback_calls == []
    assert EMG_FAST_TELEMETRY_TIMEOUT_S < 0.250


def test_all_fingers_target_uses_only_thumb_and_digit_ids():
    motors = [
        {"name": "R:wrist", "cmd_name": "wrist", "dxl_id": 11},
        {"name": "R:wrist2", "cmd_name": "wrist2", "dxl_id": 12},
        {"name": "R:thumbadd", "cmd_name": "thumbadd", "dxl_id": 13},
        {"name": "R:thumbrot", "cmd_name": "thumbrot", "dxl_id": 14},
        {"name": "R:thumbflex", "cmd_name": "thumbflex", "dxl_id": 15},
        {"name": "R:index", "cmd_name": "index", "dxl_id": 16},
        {"name": "R:middle", "cmd_name": "middle", "dxl_id": 17},
        {"name": "R:ring", "cmd_name": "ring", "dxl_id": 18},
        {"name": "R:pinky", "cmd_name": "pinky", "dxl_id": 19},
    ]
    gui = SimpleNamespace(
        _emg_motor_combo=_Combo("right_fingers", "Right hand — all fingers"),
        _motor_dxl_id=list(range(1, 10)) + list(range(11, 20)),
        motor_widgets=motors,
    )
    gui._selected_emg_motor_id = lambda: HandExoGUI._selected_emg_motor_id(gui)

    assert HandExoGUI._emg_target_ids(gui) == [13, 14, 15, 16, 17, 18, 19]


def test_custom_finger_target_can_leave_thumb_positioning_motors_stationary():
    gui = SimpleNamespace(
        _emg_motor_combo=_Combo("custom_fingers", "Custom finger group"),
        _motor_dxl_id=list(range(11, 20)),
        motor_widgets=[],
        _emg_custom_motor_ids={"custom_fingers": {15, 16, 17, 18, 19}},
    )
    gui._selected_emg_motor_id = lambda: HandExoGUI._selected_emg_motor_id(gui)

    assert HandExoGUI._emg_target_ids(gui) == [15, 16, 17, 18, 19]


def test_active_intent_commands_every_selected_finger_id():
    worker = _DirectWorker()
    status = _Label()
    command_label = _Label()
    target_ids = [13, 14, 15, 16, 17, 18, 19]
    gui = SimpleNamespace(
        _emg_live=True,
        _emg_deadman_active=True,
        _emg_ready_reason=lambda: None,
        _emg_latest={
            "received_monotonic": time.monotonic(),
            "values": [1.0, 1.0, 0.95, 1.0],
        },
        _emg_stale_ms_spin=_SpinValue(200),
        _emg_confidence_spin=_SpinValue(0.7),
        _emg_deadband_spin=_SpinValue(0.15),
        _emg_target_ids=lambda: target_ids,
        _emg_safety_ids=lambda: target_ids,
        _emg_target_name=lambda: "Right hand — all fingers",
        _emg_direction_combo=_Combo(1.0),
        _emg_max_command_spin=_SpinValue(2.0),
        _direct_mode="velocity",
        _emg_commanded_ids=set(),
        _emg_last_command_id=None,
        _serial_worker=worker,
        _emg_live_status_lbl=status,
        _emg_command_lbl=command_label,
        _stop_emg_control=lambda *_args, **_kwargs: None,
    )

    HandExoGUI._emg_control_tick(gui)

    assert worker.requests == [
        {dxl_id: ("velocity", 2.0) for dxl_id in target_ids}
    ]
    assert gui._emg_commanded_ids == set(target_ids)
    assert "all fingers" in status.text


def test_serial_worker_coalesces_direct_actions_and_stop_wins():
    worker = SerialWorker()

    worker.request_direct_actions({15: ("velocity", 2.0), 16: ("velocity", 2.0)})
    worker.request_direct_actions({15: ("velocity", -1.0), 16: ("stop", None)})

    assert worker._urgent_q.qsize() == 1
    assert worker._direct_actions == {
        15: ("velocity", -1.0),
        16: ("stop", None),
    }


def test_serial_worker_writes_multi_motor_direct_set_as_one_payload():
    payloads = []
    worker = SerialWorker()
    worker.set_exo(
        SimpleNamespace(
            command_delimiter="\r\n",
            device=SimpleNamespace(send=payloads.append),
        )
    )
    worker.request_direct_actions(
        {
            15: ("velocity", 2.0),
            16: ("velocity", -1.0),
            17: ("stop", None),
        }
    )

    worker._handle_direct_actions()

    assert payloads == [
        "set_velocity:15:2.0\r\n"
        "set_velocity:16:-1.0\r\n"
        "stop:17\r\n"
    ]


def test_custom_finger_subset_is_ready_when_explicit_ids_are_safe_and_armed():
    target_ids = [15, 16, 17, 18, 19]
    gui = SimpleNamespace(
        _emg_target_ids=lambda: target_ids,
        _emg_safety_ids=lambda: target_ids,
        _update_emg_safety_status=lambda: None,
        exo_connected=True,
        _emg_intent_worker=SimpleNamespace(isRunning=lambda: True),
        _direct_mode="velocity",
        _emg_full_finger_group_selected=lambda: False,
        _emg_group_selected=lambda: True,
        _direct_armed_ids=set(target_ids),
        _emg_hold_ready_reason=lambda: None,
        _has_calibration_for_emg_motor=lambda _dxl_id: True,
    )

    assert HandExoGUI._emg_ready_reason(gui) is None


def test_global_stop_composes_existing_stop_paths_and_disables_active_ids():
    calls = []
    gui = SimpleNamespace(
        _finish_home_sequence=lambda **kwargs: calls.append(("home", kwargs)),
        _teleop_streaming=True,
        _on_teleop_stop=lambda: calls.append(("websocket", {})),
        _stop_emg_control=lambda reason, **kwargs: calls.append(
            ("emg", {"reason": reason, **kwargs})
        ),
        _stop_udp_binding_output=lambda **kwargs: calls.append(("udp", kwargs)),
        _stop_all_direct_control=lambda: calls.append(("direct", {})),
        exo_connected=True,
        _motor_all=lambda action: calls.append(("motors", {"action": action})),
        _log=lambda text: calls.append(("log", {"text": text})),
        _update_emg_preflight=lambda: calls.append(("preflight", {})),
    )

    HandExoGUI._global_stop_all_motion(gui)

    assert calls[:6] == [
        ("home", {"resume_polling": False}),
        ("websocket", {}),
        (
            "emg",
            {
                "reason": "global stop pressed",
                "stop_timer": True,
                "release_deadman": True,
            },
        ),
        ("udp", {"disable_motors": True}),
        ("direct", {}),
        ("motors", {"action": "disable"}),
    ]


def test_batch_arming_applies_checked_ids_with_no_per_motor_dialogs():
    calls = []
    gui = SimpleNamespace(
        exo_connected=True,
        _direct_mode="velocity",
        _motor_dxl_id=[13, 14, 15, 16],
        _direct_armed_ids={13},
        _checked_direct_arm_ids=lambda: {15, 16},
        _direct_arm_confirm_cb=SimpleNamespace(isChecked=lambda: False),
        exo=SimpleNamespace(
            stop_direct_control=lambda dxl_id: calls.append(("stop", dxl_id)),
            disable_motor=lambda dxl_id: calls.append(("disable", dxl_id)),
            enable_motor=lambda dxl_id: calls.append(("enable", dxl_id)),
        ),
        _set_direct_arm_checkboxes=lambda selected, **kwargs: calls.append(
            ("sync", sorted(selected), kwargs)
        ),
        _update_direct_motor_armed_widgets=lambda: None,
        _update_direct_arm_status=lambda: None,
        _update_emg_arm_status=lambda: None,
        _sync_armed_finger_motors_to_emg_target=lambda **kwargs: calls.append(
            ("emg_target", kwargs)
        ),
        _log=lambda text: calls.append(("log", text)),
        _direct_arm_selection_dirty=True,
    )

    assert HandExoGUI._apply_direct_arming_selection(gui) is True
    assert gui._direct_armed_ids == {15, 16}
    assert calls[:6] == [
        ("stop", 13),
        ("disable", 13),
        ("stop", 15),
        ("enable", 15),
        ("stop", 16),
        ("enable", 16),
    ]


def test_power_grasp_preset_excludes_thumb_positioning_ids():
    captured = []
    motors = [
        {"cmd_name": "thumbadd", "dxl_id": 13},
        {"cmd_name": "thumbrot", "dxl_id": 14},
        {"cmd_name": "thumbflex", "dxl_id": 15},
        {"cmd_name": "index", "dxl_id": 16},
        {"cmd_name": "middle", "dxl_id": 17},
        {"cmd_name": "ring", "dxl_id": 18},
        {"cmd_name": "pinky", "dxl_id": 19},
    ]
    gui = SimpleNamespace(
        motor_widgets=motors,
        _motor_dxl_id=list(range(11, 20)),
        _set_direct_arm_checkboxes=lambda ids, **kwargs: captured.append(
            (ids, kwargs)
        ),
    )
    gui._select_direct_motor_preset = lambda names: (
        HandExoGUI._select_direct_motor_preset(gui, names)
    )

    HandExoGUI._select_direct_power_grasp_motors(gui)

    assert captured == [({15, 16, 17, 18, 19}, {"dirty": True})]


def test_direct_preset_excludes_configured_hold_id():
    captured = []
    gui = SimpleNamespace(
        motor_widgets=[
            {"cmd_name": "thumbrot", "dxl_id": 14},
            {"cmd_name": "thumbflex", "dxl_id": 15},
            {"cmd_name": "index", "dxl_id": 16},
        ],
        _motor_dxl_id=[14, 15, 16],
        _emg_hold_enable_cb=SimpleNamespace(isChecked=lambda: True),
        _emg_hold_angle=22.5,
        _selected_emg_hold_motor_id=lambda: 14,
        _set_direct_arm_checkboxes=lambda ids, **kwargs: captured.append(
            (ids, kwargs)
        ),
    )

    HandExoGUI._select_direct_motor_preset(
        gui, {"thumbrot", "thumbflex", "index"}
    )

    assert captured == [({15, 16}, {"dirty": True})]


def test_armed_power_grasp_becomes_custom_emg_target():
    combo = _MutableCombo(
        [
            ("All fingers", "all_fingers"),
            ("Custom finger group (0 motors)", "custom_fingers"),
        ]
    )
    armed_ids = {15, 16, 17, 18, 19}
    names = ["thumbadd", "thumbrot", "thumbflex", "index", "middle", "ring", "pinky"]
    gui = SimpleNamespace(
        motor_widgets=[
            {"name": name, "cmd_name": name, "dxl_id": dxl_id}
            for name, dxl_id in zip(names, range(13, 20))
        ],
        _direct_armed_ids=armed_ids,
        mode_combo=_Combo(None, "Right Only"),
        _emg_motor_combo=combo,
        _emg_custom_motor_ids={},
        _motor_dxl_id=list(range(11, 20)),
        _emg_live=False,
        _update_emg_custom_status=lambda: None,
        _update_emg_safety_status=lambda: None,
        _update_emg_arm_status=lambda: None,
        _refresh_emg_readiness_message=lambda: None,
        _log=lambda _text: None,
    )
    gui._refresh_emg_custom_combo_text = lambda: (
        HandExoGUI._refresh_emg_custom_combo_text(gui)
    )

    assert HandExoGUI._sync_armed_finger_motors_to_emg_target(
        gui, show_warning=False
    )
    assert gui._emg_custom_motor_ids["custom_fingers"] == armed_ids
    assert combo.currentData() == "custom_fingers"
    assert combo.currentText() == "Custom finger group (5 motors)"


def test_auxiliary_position_hold_engages_and_releases_explicit_id():
    calls = []
    hold_status = _Label()
    motor_status = _Label()
    motor_toggle = _Label()
    gui = SimpleNamespace(
        _emg_hold_enable_cb=SimpleNamespace(isChecked=lambda: True),
        _emg_hold_ready_reason=lambda: None,
        _selected_emg_hold_motor_id=lambda: 14,
        _emg_hold_angle=22.5,
        _emg_hold_active=False,
        exo_connected=True,
        exo=SimpleNamespace(
            hold_motor_position=lambda dxl_id, angle: calls.append(
                ("hold", dxl_id, angle)
            ),
            is_enabled=lambda dxl_id: dxl_id == 14,
            release_motor_hold=lambda dxl_id: calls.append(("release", dxl_id)),
        ),
        motor_widgets=[
            {
                "dxl_id": 14,
                "enabled": False,
                "user_disabled": True,
                "toggle_btn": motor_toggle,
                "status_lbl": motor_status,
            }
        ],
        _emg_hold_status_lbl=hold_status,
        _log=lambda _text: None,
    )

    assert HandExoGUI._engage_emg_position_hold(gui)
    assert gui._emg_hold_active is True
    assert calls == [("hold", 14, 22.5)]
    assert motor_status.text == "HOLD +22.5°"

    HandExoGUI._release_emg_position_hold(gui)
    assert gui._emg_hold_active is False
    assert calls[-1] == ("release", 14)
    assert motor_status.text == "OFF"


def test_auxiliary_hold_requests_effort_and_verifies_torque_readback():
    calls = []
    status = _Label()
    gui = SimpleNamespace(
        _emg_hold_enable_cb=SimpleNamespace(isChecked=lambda: True),
        _emg_hold_ready_reason=lambda: None,
        _emg_aux_hold_current_supported=lambda: True,
        _emg_hold_effort_spin=SimpleNamespace(value=lambda: 80),
        _selected_emg_hold_motor_id=lambda: 14,
        _emg_hold_angle=5.82,
        _emg_hold_active=False,
        _emg_hold_applied_current_mA=None,
        exo_connected=True,
        exo=SimpleNamespace(
            hold_motor_position=lambda dxl_id, angle, current: (
                calls.append(("hold", dxl_id, angle, current))
                or "OK: hold_position id=14 angle=5.820 current_mA=70"
            ),
            is_enabled=lambda _dxl_id: True,
            release_motor_hold=lambda dxl_id: calls.append(("release", dxl_id)),
        ),
        motor_widgets=[{
            "dxl_id": 14,
            "enabled": False,
            "user_disabled": True,
            "toggle_btn": _Label(),
            "status_lbl": _Label(),
        }],
        _emg_hold_status_lbl=status,
        _log=lambda _text: None,
    )

    assert HandExoGUI._engage_emg_position_hold(gui)
    assert calls == [("hold", 14, 5.82, 80)]
    assert gui._emg_hold_applied_current_mA == 70
    assert "HOLD VERIFIED" in status.text
    assert "TORQUE ON" in status.text
    assert "APPLIED 70 mA" in status.text


def test_auxiliary_hold_releases_when_torque_verification_fails():
    calls = []
    status = _Label()
    gui = SimpleNamespace(
        _emg_hold_enable_cb=SimpleNamespace(isChecked=lambda: True),
        _emg_hold_ready_reason=lambda: None,
        _emg_aux_hold_current_supported=lambda: False,
        _selected_emg_hold_motor_id=lambda: 14,
        _emg_hold_angle=5.82,
        _emg_hold_active=False,
        _emg_hold_applied_current_mA=None,
        exo_connected=True,
        exo=SimpleNamespace(
            hold_motor_position=lambda dxl_id, angle: calls.append(
                ("hold", dxl_id, angle)
            ),
            is_enabled=lambda _dxl_id: False,
            release_motor_hold=lambda dxl_id: calls.append(("release", dxl_id)),
        ),
        motor_widgets=[],
        _emg_hold_status_lbl=status,
        _log=lambda _text: None,
    )

    assert not HandExoGUI._engage_emg_position_hold(gui)
    assert calls == [("hold", 14, 5.82), ("release", 14)]
    assert "HOLD FAILED" in status.text


def test_position_hold_removes_motor_from_direct_arming():
    calls = []
    gui = SimpleNamespace(
        _direct_armed_ids={14, 15},
        exo=SimpleNamespace(
            stop_direct_control=lambda dxl_id: calls.append(("stop", dxl_id)),
            disable_motor=lambda dxl_id: calls.append(("disable", dxl_id)),
        ),
        _set_direct_arm_checkboxes=lambda ids, **kwargs: calls.append(
            ("sync", set(ids), kwargs)
        ),
        _update_direct_motor_armed_widgets=lambda: None,
        _update_direct_arm_status=lambda: None,
        _update_emg_arm_status=lambda: None,
        _log=lambda text: calls.append(("log", text)),
    )

    HandExoGUI._exclude_emg_hold_from_direct_arming(gui, 14)

    assert gui._direct_armed_ids == {15}
    assert calls[:3] == [
        ("stop", 14),
        ("disable", 14),
        ("sync", {15}, {"dirty": False}),
    ]


def test_position_hold_target_uses_profile_relative_limits():
    gui = SimpleNamespace(
        motor_widgets=[
            {"name": "thumbrot", "cmd_name": "thumbrot", "dxl_id": 14}
        ],
        _active_cal_profile={
            "motors": {
                "thumbrot": {
                    "home": 150.0,
                    "limit_min": 120.0,
                    "limit_max": 175.0,
                    "flip": True,
                }
            }
        },
        _active_cal_left=None,
        _active_cal_right=None,
        _firmware_limits_by_id={14: (120.0, 175.0)},
    )

    assert HandExoGUI._relative_emg_hold_limits(gui, 14) == (-25.0, 30.0)


def test_position_hold_target_does_not_use_absolute_encoder_limits_as_relative():
    gui = SimpleNamespace(
        motor_widgets=[],
        _active_cal_profile=None,
        _active_cal_left=None,
        _active_cal_right=None,
        _firmware_limits_by_id={14: (120.0, 175.0)},
    )

    assert HandExoGUI._relative_emg_hold_limits(gui, 14) == (-55.0, 55.0)


def test_position_hold_capture_prefers_fresh_displayed_angle():
    class ValueWidget:
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

    def unexpected_serial_read(_motor_id):
        raise AssertionError("fresh displayed telemetry should avoid a serial read")

    target = ValueWidget()
    gui = SimpleNamespace(
        exo_connected=True,
        _emg_live=False,
        _emg_aux_hold_supported=lambda: True,
        _selected_emg_hold_motor_id=lambda: 14,
        _fresh_cached_relative_angle=lambda _motor_id: 5.82,
        exo=SimpleNamespace(get_motor_angle=unexpected_serial_read),
        _emg_hold_target_spin=target,
        _emg_hold_current_lbl=ValueWidget(),
        _emg_hold_enable_cb=ValueWidget(),
        _emg_hold_status_lbl=ValueWidget(),
        _update_emg_safety_status=lambda: None,
        _log=lambda _message: None,
    )

    assert HandExoGUI._capture_emg_hold_angle(gui)
    assert gui._emg_hold_angle == 5.82
    assert target.value == 5.82
