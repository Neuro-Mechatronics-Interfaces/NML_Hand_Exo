import json
import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
REPO_SRC = str(Path(__file__).resolve().parents[2] / "src")
if REPO_SRC in sys.path:
    sys.path.remove(REPO_SRC)
sys.path.insert(0, REPO_SRC)

from PyQt5.QtWidgets import QApplication, QPushButton

from nml_hand_exo.applications.task_cue_gui import (
    DisabledMarkerPublisher,
    ParticipantCueWindow,
    PromptStep,
    TaskCueOperatorWindow,
    TaskScheduler,
    TaskState,
    load_prompt_plan,
    validate_prompt_plan,
)


class FakeClock:
    def __init__(self, value=0.0):
        self.value = float(value)

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += float(seconds)


class FakePublisher:
    def __init__(self):
        self.events = []
        self.closed = False

    def publish(self, label, timestamp=None):
        self.events.append((label, timestamp))

    def close(self):
        self.closed = True


@pytest.fixture(scope="module")
def qapp():
    return QApplication.instance() or QApplication([])


def test_prompt_loading_and_validation(tmp_path):
    path = tmp_path / "plan.json"
    path.write_text(
        json.dumps([
            {"label": "rest", "duration": 2},
            {"label": "isolated_digits:thumb_flex", "duration": 5.5},
        ]),
        encoding="utf-8",
    )
    plan = load_prompt_plan(path)
    assert plan == (
        PromptStep("rest", 2.0),
        PromptStep("isolated_digits:thumb_flex", 5.5),
    )
    with pytest.raises(ValueError, match="non-empty"):
        validate_prompt_plan([])
    with pytest.raises(ValueError, match="greater than zero"):
        validate_prompt_plan([{"label": "rest", "duration": 0}])
    with pytest.raises(ValueError, match="reserved"):
        validate_prompt_plan([{"label": "bad|label", "duration": 1}])


def test_two_step_marker_sequence():
    clock = FakeClock(100.0)
    publisher = FakePublisher()
    scheduler = TaskScheduler(publisher, clock)
    scheduler.start((PromptStep("rest", 0.1), PromptStep("grasp:pinch", 0.2)), "short.json")
    clock.advance(0.1)
    scheduler.tick()
    clock.advance(0.21)
    scheduler.tick()
    assert [event for event, _ in publisher.events] == [
        "session_start",
        "prompt_sequence_start|plan=short.json",
        "rest_onset|duration_s=0.100",
        "prompt_onset|phase=rest|trial=000|gesture=rest|duration_s=0.100",
        "prompt_offset|phase=rest|trial=000|gesture=rest",
        "trial_start|trial=001|gesture=grasp:pinch|duration_s=0.200",
        "prompt_onset|phase=gesture|trial=001|gesture=grasp:pinch|duration_s=0.200",
        "prompt_offset|phase=gesture|trial=001|gesture=grasp:pinch",
        "trial_end|trial=001|gesture=grasp:pinch",
        "session_complete",
    ]
    assert publisher.events[4][1] == pytest.approx(100.1)
    assert publisher.events[-1][1] == pytest.approx(100.3)


def test_prompt_metadata_is_preserved_in_markers_and_saved_plan(tmp_path, qapp):
    plan = validate_prompt_plan(
        [
            {
                "label": "attempt_hand_close",
                "duration": 1.0,
                "condition": "exo_transparent",
                "posture_target": "mid",
                "assistance_level": 0,
            }
        ]
    )
    clock = FakeClock(5.0)
    publisher = FakePublisher()
    scheduler = TaskScheduler(publisher, clock)
    scheduler.start(plan, "physics.json")
    onset = [value for value, _timestamp in publisher.events if value.startswith("prompt_onset")][0]
    assert "condition=exo_transparent" in onset
    assert "posture_target=mid" in onset
    assert "assistance_level=0" in onset

    window = TaskCueOperatorWindow()
    window._set_plan(plan)
    destination = tmp_path / "saved.json"
    window.save_plan(destination)
    payload = json.loads(destination.read_text(encoding="utf-8"))
    assert payload[0]["condition"] == "exo_transparent"
    assert payload[0]["posture_target"] == "mid"


def test_pause_resume_freezes_remaining_time_and_extends_deadline():
    clock = FakeClock(10.0)
    publisher = FakePublisher()
    scheduler = TaskScheduler(publisher, clock)
    scheduler.start((PromptStep("isolated_digits:index_flex", 5.0),), "pause.json")
    clock.advance(2.0)
    scheduler.pause()
    assert scheduler.snapshot().remaining_s == pytest.approx(3.0)
    original_deadline = scheduler.deadline
    clock.advance(100.0)
    scheduler.tick()
    assert scheduler.state is TaskState.PAUSED
    assert scheduler.snapshot().remaining_s == pytest.approx(3.0)
    scheduler.resume()
    assert scheduler.deadline == pytest.approx(original_deadline + 100.0)
    clock.advance(2.9)
    scheduler.tick()
    assert scheduler.state is TaskState.ACTIVE
    clock.advance(0.11)
    scheduler.tick()
    assert scheduler.state is TaskState.COMPLETE
    markers = [event for event, _ in publisher.events]
    assert markers.count("session_pause") == 1
    assert markers.count("session_resume") == 1


def test_abort_and_completion_are_exactly_once():
    clock = FakeClock()
    aborted_publisher = FakePublisher()
    aborted = TaskScheduler(aborted_publisher, clock)
    aborted.start((PromptStep("rest", 1.0),), "abort.json")
    aborted.abort()
    aborted.abort()
    aborted.tick()
    abort_markers = [event for event, _ in aborted_publisher.events]
    assert abort_markers.count("session_abort") == 1
    assert "session_complete" not in abort_markers

    complete_publisher = FakePublisher()
    complete = TaskScheduler(complete_publisher, clock)
    complete.start((PromptStep("rest", 0.1),), "complete.json")
    clock.advance(1.0)
    complete.tick()
    complete.tick()
    complete_markers = [event for event, _ in complete_publisher.events]
    assert complete_markers.count("session_complete") == 1
    assert "session_abort" not in complete_markers


def test_headless_operator_and_participant_windows(qapp, monkeypatch):
    startup_publisher = FakePublisher()
    monkeypatch.setattr(
        TaskCueOperatorWindow,
        "_create_lsl",
        lambda self: (startup_publisher, FakeClock()),
    )
    cue = ParticipantCueWindow()
    operator = TaskCueOperatorWindow(cue)
    operator.show()
    qapp.processEvents()
    assert operator.timer.isActive()
    assert operator.start_button.text() == "Start Task"
    assert cue.findChildren(QPushButton) == []
    assert operator.publisher is startup_publisher
    assert operator._publisher_is_lsl
    assert "LIVE" in operator.marker_mode_label.text()
    operator._close_publisher()
    publisher = FakePublisher()
    operator.publisher = publisher
    operator.scheduler = TaskScheduler(publisher, FakeClock())
    operator.scheduler.start((PromptStep("rest", 10.0),), "smoke.json")
    operator.close()
    qapp.processEvents()
    markers = [event for event, _ in publisher.events]
    assert markers.count("session_abort") == 1
    assert publisher.closed


def test_operator_can_build_reorder_and_save_plan(qapp, tmp_path):
    operator = TaskCueOperatorWindow(ParticipantCueWindow())
    operator._quick_add("rest", 2.0)
    operator.custom_label_edit.setText("participant:chosen_gesture")
    operator.custom_duration_spin.setValue(3.5)
    operator._add_custom_step()

    assert operator.plan == (
        PromptStep("rest", 2.0),
        PromptStep("participant:chosen_gesture", 3.5),
    )
    assert operator.plan_path is None
    assert operator.plan_path_edit.text() == "Unsaved GUI plan"
    assert operator.start_button.isEnabled()

    operator.preview_table.selectRow(1)
    operator._move_selected_step(-1)
    assert [step.label for step in operator.plan] == [
        "participant:chosen_gesture",
        "rest",
    ]

    destination = tmp_path / "participant_plan"
    operator.save_plan(destination)
    saved = destination.with_suffix(".json")
    assert operator.plan_path == saved
    assert json.loads(saved.read_text(encoding="utf-8")) == [
        {"label": "participant:chosen_gesture", "duration": 3.5},
        {"label": "rest", "duration": 2.0},
    ]
    operator.close()
    qapp.processEvents()


def test_operator_can_run_visual_cues_with_lsl_disabled(qapp):
    operator = TaskCueOperatorWindow(ParticipantCueWindow())
    operator._quick_add("rest", 2.0)
    operator.publish_markers_check.setChecked(False)
    operator._create_lsl = lambda: (_ for _ in ()).throw(
        AssertionError("LSL outlet must not be created when markers are disabled")
    )

    operator._start_task()

    assert operator.scheduler is not None
    assert operator.scheduler.state is TaskState.ACTIVE
    assert isinstance(operator.publisher, DisabledMarkerPublisher)
    assert "visual cues only" in operator.status_label.text()
    assert not operator.stream_name_edit.isEnabled()
    assert not operator.source_id_edit.isEnabled()
    operator.close()
    qapp.processEvents()


def test_editable_quick_prompt_adds_updated_marker(qapp):
    operator = TaskCueOperatorWindow(ParticipantCueWindow())
    assert len(operator.quick_buttons) == 3
    operator._set_quick_preset(1, "custom:participant_pinch", 7.5)

    assert "Participant Pinch" in operator.quick_buttons[1].text()
    assert "7.5 s" in operator.quick_buttons[1].text()
    operator.quick_buttons[1].click()
    assert operator.plan == (PromptStep("custom:participant_pinch", 7.5),)
    operator.close()
    qapp.processEvents()
