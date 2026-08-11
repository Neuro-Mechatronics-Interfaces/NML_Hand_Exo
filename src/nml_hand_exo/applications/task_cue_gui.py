"""Participant task cues and timestamped LSL event markers.

This application deliberately has no exoskeleton control imports.  The task
scheduler is independent of Qt so marker ordering and deadline behavior can be
tested with a fake clock and publisher.
"""
from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Protocol, Sequence

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from nml_hand_exo.applications.styles import DARK_STYLE


class MarkerPublisher(Protocol):
    def publish(self, label: str, timestamp: float | None = None) -> None: ...

    def close(self) -> None: ...


class DisabledMarkerPublisher:
    """No-op publisher used when the operator chooses visual cues only."""

    def publish(self, label: str, timestamp: float | None = None) -> None:
        return None

    def close(self) -> None:
        return None


@dataclass(frozen=True)
class PromptStep:
    label: str
    duration_s: float

    @property
    def is_rest(self) -> bool:
        return self.label.casefold() == "rest"


class TaskState(str, Enum):
    IDLE = "IDLE"
    ACTIVE = "ACTIVE"
    PAUSED = "PAUSED"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


@dataclass(frozen=True)
class TaskSnapshot:
    state: TaskState
    label: str | None
    remaining_s: float
    step_index: int
    step_count: int
    trial_id: int


def validate_prompt_plan(payload: object) -> tuple[PromptStep, ...]:
    if not isinstance(payload, list) or not payload:
        raise ValueError("Prompt plan must be a non-empty JSON array")
    steps: list[PromptStep] = []
    for index, entry in enumerate(payload, start=1):
        if not isinstance(entry, dict):
            raise ValueError(f"Step {index} must be a JSON object")
        if "label" not in entry or "duration" not in entry:
            raise ValueError(f"Step {index} requires 'label' and 'duration'")
        label = entry["label"]
        if not isinstance(label, str) or not label.strip():
            raise ValueError(f"Step {index} has an empty or invalid label")
        label = label.strip()
        if "|" in label or "\n" in label or "\r" in label:
            raise ValueError(f"Step {index} label contains a reserved marker character")
        duration = entry["duration"]
        if isinstance(duration, bool) or not isinstance(duration, (int, float)):
            raise ValueError(f"Step {index} duration must be a number")
        duration_s = float(duration)
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError(f"Step {index} duration must be finite and greater than zero")
        steps.append(PromptStep(label=label, duration_s=duration_s))
    return tuple(steps)


def load_prompt_plan(path: str | Path) -> tuple[PromptStep, ...]:
    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"Could not read prompt plan: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"Invalid JSON at line {exc.lineno}, column {exc.colno}: {exc.msg}"
        ) from exc
    return validate_prompt_plan(payload)


class TaskScheduler:
    """Deterministic prompt state machine driven by one external timer."""

    def __init__(self, publisher: MarkerPublisher, clock: Callable[[], float]):
        self.publisher = publisher
        self.clock = clock
        self.state = TaskState.IDLE
        self.plan: tuple[PromptStep, ...] = ()
        self.plan_name = ""
        self.step_index = -1
        self.trial_id = 0
        self.current_trial_id = 0
        self.deadline: float | None = None
        self.pause_started: float | None = None
        self._complete_emitted = False
        self._abort_emitted = False

    @staticmethod
    def _trial_text(trial_id: int) -> str:
        return f"{trial_id:03d}"

    def _emit(self, marker: str, timestamp: float) -> None:
        self.publisher.publish(marker, timestamp=timestamp)

    def start(self, plan: Sequence[PromptStep], plan_name: str) -> None:
        if self.state is not TaskState.IDLE:
            raise RuntimeError("Reset the task before starting another session")
        if not plan:
            raise ValueError("Prompt plan is empty")
        now = float(self.clock())
        self.plan = tuple(plan)
        self.plan_name = Path(plan_name).name
        if not self.plan_name or any(token in self.plan_name for token in ("|", "\n", "\r")):
            raise ValueError("Prompt-plan filename contains a reserved marker character")
        self.step_index = -1
        self.trial_id = 0
        self.current_trial_id = 0
        self._complete_emitted = False
        self._abort_emitted = False
        self.state = TaskState.ACTIVE
        self._emit("session_start", now)
        self._emit(f"prompt_sequence_start|plan={self.plan_name}", now)
        self._begin_step(0, now)

    def _begin_step(self, index: int, timestamp: float) -> None:
        self.step_index = index
        step = self.plan[index]
        if step.is_rest:
            self.current_trial_id = 0
            self._emit(f"rest_onset|duration_s={step.duration_s:.3f}", timestamp)
            phase = "rest"
        else:
            self.trial_id += 1
            self.current_trial_id = self.trial_id
            self._emit(
                "trial_start|"
                f"trial={self._trial_text(self.current_trial_id)}|"
                f"gesture={step.label}|duration_s={step.duration_s:.3f}",
                timestamp,
            )
            phase = "gesture"
        self._emit(
            "prompt_onset|"
            f"phase={phase}|trial={self._trial_text(self.current_trial_id)}|"
            f"gesture={step.label}|duration_s={step.duration_s:.3f}",
            timestamp,
        )
        self.deadline = timestamp + step.duration_s

    def _finish_step(self, timestamp: float) -> None:
        step = self.plan[self.step_index]
        phase = "rest" if step.is_rest else "gesture"
        self._emit(
            "prompt_offset|"
            f"phase={phase}|trial={self._trial_text(self.current_trial_id)}|"
            f"gesture={step.label}",
            timestamp,
        )
        if not step.is_rest:
            self._emit(
                "trial_end|"
                f"trial={self._trial_text(self.current_trial_id)}|gesture={step.label}",
                timestamp,
            )

    def tick(self) -> None:
        if self.state is not TaskState.ACTIVE or self.deadline is None:
            return
        now = float(self.clock())
        while self.state is TaskState.ACTIVE and self.deadline is not None and now >= self.deadline:
            boundary = self.deadline
            self._finish_step(boundary)
            next_index = self.step_index + 1
            if next_index >= len(self.plan):
                self.deadline = None
                self.state = TaskState.COMPLETE
                if not self._complete_emitted:
                    self._emit("session_complete", boundary)
                    self._complete_emitted = True
                return
            self._begin_step(next_index, boundary)

    def pause(self) -> None:
        self.tick()
        if self.state is not TaskState.ACTIVE:
            raise RuntimeError("Task is not active")
        now = float(self.clock())
        self.pause_started = now
        self.state = TaskState.PAUSED
        self._emit("session_pause", now)

    def resume(self) -> None:
        if self.state is not TaskState.PAUSED or self.pause_started is None:
            raise RuntimeError("Task is not paused")
        now = float(self.clock())
        if self.deadline is not None:
            self.deadline += max(0.0, now - self.pause_started)
        self.pause_started = None
        self.state = TaskState.ACTIVE
        self._emit("session_resume", now)

    def abort(self) -> None:
        if self.state not in {TaskState.ACTIVE, TaskState.PAUSED}:
            return
        now = float(self.clock())
        self.state = TaskState.ABORTED
        self.deadline = None
        self.pause_started = None
        if not self._abort_emitted:
            self._emit("session_abort", now)
            self._abort_emitted = True

    def reset(self) -> None:
        if self.state in {TaskState.ACTIVE, TaskState.PAUSED}:
            raise RuntimeError("Abort the active task before resetting")
        self.state = TaskState.IDLE
        self.plan = ()
        self.plan_name = ""
        self.step_index = -1
        self.trial_id = 0
        self.current_trial_id = 0
        self.deadline = None
        self.pause_started = None
        self._complete_emitted = False
        self._abort_emitted = False

    def snapshot(self) -> TaskSnapshot:
        label = None
        if 0 <= self.step_index < len(self.plan):
            label = self.plan[self.step_index].label
        if self.state is TaskState.PAUSED and self.deadline is not None and self.pause_started is not None:
            remaining = max(0.0, self.deadline - self.pause_started)
        elif self.state is TaskState.ACTIVE and self.deadline is not None:
            remaining = max(0.0, self.deadline - float(self.clock()))
        else:
            remaining = 0.0
        return TaskSnapshot(
            state=self.state,
            label=label,
            remaining_s=remaining,
            step_index=self.step_index,
            step_count=len(self.plan),
            trial_id=self.current_trial_id,
        )


CUE_STYLE = """
QLabel#cue-state {
    color: #c0392b;
    font-size: 26px;
    font-weight: bold;
    letter-spacing: 4px;
}
QLabel#cue-gesture {
    color: #ffffff;
    font-size: 72px;
    font-weight: bold;
}
QLabel#cue-countdown {
    color: #ffffff;
    font-family: "Cascadia Code", "Consolas", monospace;
    font-size: 96px;
    font-weight: bold;
}
QLabel#cue-progress { color: #a0a0a0; font-size: 20px; }
QLabel#operator-title { color: #ffffff; font-size: 28px; font-weight: bold; }
QPushButton[quickPrompt="true"] {
    min-height: 28px;
    padding: 10px 16px;
    font-size: 15px;
}
QPushButton[quickEdit="true"] {
    min-height: 28px;
    padding: 10px 12px;
}
QPushButton#primary-action { min-height: 44px; font-size: 16px; font-weight: bold; }
QPushButton#abort-action { min-height: 44px; color: #ffffff; background: #8b1a1a; }
"""


def _display_label(label: str | None) -> str:
    if not label:
        return "READY"
    if label.casefold() == "rest":
        return "REST"
    leaf = label.split(":")[-1]
    return leaf.replace("_", " ").upper()


class ParticipantCueWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NML Participant Task Cue")
        self.resize(1100, 720)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(48, 42, 48, 42)
        self.state_label = QLabel("READY")
        self.state_label.setObjectName("cue-state")
        self.state_label.setAlignment(Qt.AlignCenter)
        self.gesture_label = QLabel("READY")
        self.gesture_label.setObjectName("cue-gesture")
        self.gesture_label.setAlignment(Qt.AlignCenter)
        self.gesture_label.setWordWrap(True)
        self.countdown_label = QLabel("--")
        self.countdown_label.setObjectName("cue-countdown")
        self.countdown_label.setAlignment(Qt.AlignCenter)
        self.progress_label = QLabel("Waiting for the operator")
        self.progress_label.setObjectName("cue-progress")
        self.progress_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.state_label)
        layout.addStretch(1)
        layout.addWidget(self.gesture_label)
        layout.addWidget(self.countdown_label)
        layout.addStretch(1)
        layout.addWidget(self.progress_label)

    def set_always_on_top(self, enabled: bool) -> None:
        was_visible = self.isVisible()
        self.setWindowFlag(Qt.WindowStaysOnTopHint, enabled)
        if was_visible:
            self.show()

    def render_snapshot(self, snapshot: TaskSnapshot | None) -> None:
        if snapshot is None or snapshot.state is TaskState.IDLE:
            state_text, gesture, countdown = "READY", "READY", "--"
            progress = "Waiting for the operator"
        elif snapshot.state is TaskState.COMPLETE:
            state_text, gesture, countdown = "COMPLETE", "TASK COMPLETE", "0.0"
            progress = f"{snapshot.step_count} of {snapshot.step_count} steps"
        elif snapshot.state is TaskState.ABORTED:
            state_text, gesture, countdown = "ABORTED", "TASK STOPPED", "--"
            progress = "Please wait for the operator"
        else:
            is_rest = bool(snapshot.label and snapshot.label.casefold() == "rest")
            state_text = "PAUSED" if snapshot.state is TaskState.PAUSED else ("REST" if is_rest else "ACTIVE")
            gesture = _display_label(snapshot.label)
            countdown = f"{snapshot.remaining_s:.1f}"
            trial = "REST" if snapshot.trial_id == 0 else f"TRIAL {snapshot.trial_id:03d}"
            progress = f"STEP {snapshot.step_index + 1} OF {snapshot.step_count}   |   {trial}"
        self.state_label.setText(state_text)
        self.gesture_label.setText(gesture)
        self.countdown_label.setText(countdown)
        self.progress_label.setText(progress)
        colors = {
            "ACTIVE": "#27ae60",
            "REST": "#4da3ff",
            "PAUSED": "#f1c40f",
            "COMPLETE": "#27ae60",
            "ABORTED": "#c0392b",
            "READY": "#aaaaaa",
        }
        self.state_label.setStyleSheet(f"color: {colors.get(state_text, '#c0392b')};")


class TaskCueOperatorWindow(QMainWindow):
    TIMER_INTERVAL_MS = 50

    def __init__(self, cue_window: ParticipantCueWindow | None = None):
        super().__init__()
        self.setWindowTitle("NML Task Cue - Operator")
        self.resize(1000, 900)
        self.cue_window = cue_window or ParticipantCueWindow()
        self.plan: tuple[PromptStep, ...] = ()
        self.plan_path: Path | None = None
        self.scheduler: TaskScheduler | None = None
        self.publisher: MarkerPublisher | None = None
        self.marker_clock: Callable[[], float] | None = None
        self._publisher_is_lsl = False
        self._active_stream_config: tuple[str, str] | None = None
        self._closing = False
        self._build_ui()
        self.timer = QTimer(self)
        self.timer.setInterval(self.TIMER_INTERVAL_MS)
        self.timer.timeout.connect(self._on_timer)
        self.timer.start()
        self._refresh_controls()
        self.cue_window.render_snapshot(None)
        QTimer.singleShot(0, self._initialize_marker_stream)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(12)
        title = QLabel("Participant Task Cues")
        title.setObjectName("operator-title")
        layout.addWidget(title)

        plan_box = QGroupBox("Prompt plan")
        plan_layout = QGridLayout(plan_box)
        plan_layout.setContentsMargins(12, 18, 12, 12)
        plan_layout.setHorizontalSpacing(10)
        plan_layout.setVerticalSpacing(12)
        self.plan_path_edit = QLineEdit()
        self.plan_path_edit.setReadOnly(True)
        self.plan_path_edit.setPlaceholderText("Select a JSON prompt plan")
        self.plan_button = QPushButton("Choose JSON...")
        self.plan_button.clicked.connect(self._choose_plan)
        self.save_plan_button = QPushButton("Save Plan As...")
        self.save_plan_button.clicked.connect(self._choose_save_plan)

        quick_row = QHBoxLayout()
        quick_row.setSpacing(10)
        quick_row.addWidget(QLabel("Quick add"))
        self.quick_presets = [
            PromptStep("rest", 2.0),
            PromptStep("coordinated_grasp:pinch", 5.0),
            PromptStep("hand_open", 5.0),
        ]
        self.quick_buttons: list[QPushButton] = []
        self.quick_edit_buttons: list[QPushButton] = []
        for index, preset in enumerate(self.quick_presets):
            preset_layout = QHBoxLayout()
            preset_layout.setSpacing(6)
            button = QPushButton()
            button.setProperty("quickPrompt", True)
            button.clicked.connect(
                lambda _checked=False, slot=index: self._add_quick_preset(slot)
            )
            edit_button = QPushButton("Edit")
            edit_button.setProperty("quickEdit", True)
            edit_button.setToolTip("Change this quick prompt's label and duration")
            edit_button.clicked.connect(
                lambda _checked=False, slot=index: self._edit_quick_preset(slot)
            )
            self.quick_buttons.append(button)
            self.quick_edit_buttons.append(edit_button)
            preset_layout.addWidget(button, 1)
            preset_layout.addWidget(edit_button)
            quick_row.addLayout(preset_layout, 1)
            self._refresh_quick_preset(index)
        quick_row.addStretch()

        builder_row = QHBoxLayout()
        builder_row.setSpacing(10)
        self.custom_label_edit = QLineEdit()
        self.custom_label_edit.setPlaceholderText(
            "Custom marker label, e.g. coordinated_grasp:power"
        )
        self.custom_duration_spin = QDoubleSpinBox()
        self.custom_duration_spin.setRange(0.1, 3600.0)
        self.custom_duration_spin.setDecimals(1)
        self.custom_duration_spin.setValue(5.0)
        self.custom_duration_spin.setSuffix(" s")
        self.add_step_button = QPushButton("Add Prompt")
        self.add_step_button.setProperty("accent", True)
        self.add_step_button.clicked.connect(self._add_custom_step)
        self.custom_label_edit.returnPressed.connect(self._add_custom_step)
        builder_row.addWidget(QLabel("Label"))
        builder_row.addWidget(self.custom_label_edit, 1)
        builder_row.addWidget(QLabel("Duration"))
        builder_row.addWidget(self.custom_duration_spin)
        builder_row.addWidget(self.add_step_button)

        self.preview_table = QTableWidget(0, 3)
        self.preview_table.setHorizontalHeaderLabels(["Step", "Prompt", "Duration"])
        self.preview_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.preview_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.preview_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.preview_table.verticalHeader().setVisible(False)
        self.preview_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.preview_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.preview_table.setSelectionMode(QTableWidget.SingleSelection)
        self.preview_table.setMinimumHeight(220)
        self.move_up_button = QPushButton("Move Up")
        self.move_down_button = QPushButton("Move Down")
        self.remove_step_button = QPushButton("Remove Selected")
        self.clear_plan_button = QPushButton("Clear Plan")
        self.move_up_button.clicked.connect(lambda: self._move_selected_step(-1))
        self.move_down_button.clicked.connect(lambda: self._move_selected_step(1))
        self.remove_step_button.clicked.connect(self._remove_selected_step)
        self.clear_plan_button.clicked.connect(self._clear_plan)
        edit_row = QHBoxLayout()
        edit_row.addWidget(self.move_up_button)
        edit_row.addWidget(self.move_down_button)
        edit_row.addWidget(self.remove_step_button)
        edit_row.addStretch()
        edit_row.addWidget(self.clear_plan_button)
        self.builder_controls = [
            self.custom_label_edit,
            self.custom_duration_spin,
            self.add_step_button,
            self.move_up_button,
            self.move_down_button,
            self.remove_step_button,
            self.clear_plan_button,
        ] + self.quick_buttons + self.quick_edit_buttons
        self.total_label = QLabel("No plan loaded")
        plan_layout.addWidget(self.plan_path_edit, 0, 0)
        file_actions = QHBoxLayout()
        file_actions.addWidget(self.plan_button)
        file_actions.addWidget(self.save_plan_button)
        plan_layout.addLayout(file_actions, 0, 1)
        plan_layout.addLayout(quick_row, 1, 0, 1, 2)
        plan_layout.addLayout(builder_row, 2, 0, 1, 2)
        plan_layout.addWidget(self.preview_table, 3, 0, 1, 2)
        plan_layout.addLayout(edit_row, 4, 0, 1, 2)
        plan_layout.addWidget(self.total_label, 5, 0, 1, 2)
        layout.addWidget(plan_box, 1)

        stream_box = QGroupBox("Marker stream")
        stream_layout = QGridLayout(stream_box)
        stream_layout.setContentsMargins(12, 18, 12, 12)
        stream_layout.setHorizontalSpacing(10)
        stream_layout.setVerticalSpacing(10)
        self.publish_markers_check = QCheckBox("Publish LSL markers")
        self.publish_markers_check.setChecked(True)
        self.publish_markers_check.setToolTip(
            "Disable to run participant cues without creating an LSL outlet."
        )
        self.publish_markers_check.toggled.connect(self._on_marker_toggle)
        self.stream_name_edit = QLineEdit("NML_TaskMarkers")
        self.source_id_edit = QLineEdit("nml_hand_exo_task_cue")
        self.stream_name_edit.editingFinished.connect(self._restart_marker_stream)
        self.source_id_edit.editingFinished.connect(self._restart_marker_stream)
        self.marker_mode_label = QLabel("Starting LSL marker stream...")
        stream_layout.addWidget(self.publish_markers_check, 0, 0)
        stream_layout.addWidget(self.marker_mode_label, 0, 1)
        stream_layout.addWidget(QLabel("Stream name"), 1, 0)
        stream_layout.addWidget(self.stream_name_edit, 1, 1)
        stream_layout.addWidget(QLabel("Source ID"), 2, 0)
        stream_layout.addWidget(self.source_id_edit, 2, 1)
        layout.addWidget(stream_box)

        display_row = QHBoxLayout()
        self.show_cue_button = QPushButton("Open / Show Cue Window")
        self.show_cue_button.clicked.connect(self._show_cue)
        self.always_on_top_check = QCheckBox("Keep cue window always on top")
        self.always_on_top_check.toggled.connect(self.cue_window.set_always_on_top)
        display_row.addWidget(self.show_cue_button)
        display_row.addWidget(self.always_on_top_check)
        display_row.addStretch()
        layout.addLayout(display_row)

        control_row = QHBoxLayout()
        self.start_button = QPushButton("Start Task")
        self.start_button.setObjectName("primary-action")
        self.start_button.setProperty("accent", True)
        self.start_button.clicked.connect(self._start_task)
        self.pause_button = QPushButton("Pause Task")
        self.pause_button.clicked.connect(self._pause_resume)
        self.abort_button = QPushButton("Stop / Abort")
        self.abort_button.setObjectName("abort-action")
        self.abort_button.clicked.connect(self._abort_task)
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self._reset_task)
        control_row.addWidget(self.start_button)
        control_row.addWidget(self.pause_button)
        control_row.addWidget(self.abort_button)
        control_row.addWidget(self.reset_button)
        layout.addLayout(control_row)
        self.status_label = QLabel("Build a prompt plan or load JSON to begin")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

    def _choose_plan(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open prompt plan", "", "JSON files (*.json)")
        if path:
            self.load_plan(path)

    def _choose_save_plan(self) -> None:
        if not self.plan:
            QMessageBox.warning(self, "Empty prompt plan", "Add at least one prompt before saving.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save prompt plan", "prompt_plan.json", "JSON files (*.json)"
        )
        if path:
            self.save_plan(path)

    def save_plan(self, path: str | Path) -> None:
        """Save the current GUI plan as reusable JSON."""
        if not self.plan:
            raise ValueError("Prompt plan is empty")
        destination = Path(path)
        if destination.suffix.casefold() != ".json":
            destination = destination.with_suffix(".json")
        payload = [
            {"label": step.label, "duration": step.duration_s} for step in self.plan
        ]
        try:
            destination.write_text(
                json.dumps(payload, indent=2) + "\n", encoding="utf-8"
            )
        except OSError as exc:
            QMessageBox.critical(self, "Could not save prompt plan", str(exc))
            return
        self.plan_path = destination
        self.plan_path_edit.setText(str(destination))
        self.status_label.setText("Plan saved and ready to start.")
        self._refresh_controls()

    def _set_plan(
        self,
        plan: Sequence[PromptStep],
        *,
        path: Path | None = None,
        selected_row: int | None = None,
    ) -> None:
        self.plan = tuple(plan)
        self.plan_path = path
        self.plan_path_edit.setText(str(path) if path is not None else "Unsaved GUI plan")
        self.preview_table.setRowCount(len(self.plan))
        for row, step in enumerate(self.plan):
            self.preview_table.setItem(row, 0, QTableWidgetItem(str(row + 1)))
            self.preview_table.setItem(row, 1, QTableWidgetItem(step.label))
            self.preview_table.setItem(row, 2, QTableWidgetItem(f"{step.duration_s:.3f} s"))
        total = sum(step.duration_s for step in self.plan)
        trials = sum(not step.is_rest for step in self.plan)
        if self.plan:
            self.total_label.setText(
                f"{len(self.plan)} steps  |  {trials} non-rest trials  |  total {total:.1f} s"
            )
            if selected_row is not None:
                selected_row = max(0, min(selected_row, len(self.plan) - 1))
                self.preview_table.selectRow(selected_row)
        else:
            self.plan_path_edit.clear()
            self.total_label.setText("No prompts yet")
        self._refresh_controls()

    def _quick_add(self, label: str, duration_s: float) -> None:
        step = validate_prompt_plan([{"label": label, "duration": duration_s}])[0]
        self._set_plan((*self.plan, step), selected_row=len(self.plan))
        self.status_label.setText("Prompt added. Continue building or start the task.")

    def _add_quick_preset(self, index: int) -> None:
        preset = self.quick_presets[index]
        self._quick_add(preset.label, preset.duration_s)

    def _refresh_quick_preset(self, index: int) -> None:
        preset = self.quick_presets[index]
        display_name = preset.label.rsplit(":", 1)[-1].replace("_", " ").strip().title()
        self.quick_buttons[index].setText(
            f"+ {display_name}   |   {preset.duration_s:g} s"
        )
        self.quick_buttons[index].setToolTip(
            f"Add {preset.label} for {preset.duration_s:g} seconds"
        )

    def _set_quick_preset(self, index: int, label: str, duration_s: float) -> None:
        preset = validate_prompt_plan([{"label": label, "duration": duration_s}])[0]
        self.quick_presets[index] = preset
        self._refresh_quick_preset(index)

    def _edit_quick_preset(self, index: int) -> None:
        preset = self.quick_presets[index]
        dialog = QDialog(self)
        dialog.setWindowTitle(f"Edit quick prompt {index + 1}")
        form = QFormLayout(dialog)
        label_edit = QLineEdit(preset.label)
        duration_spin = QDoubleSpinBox()
        duration_spin.setRange(0.1, 3600.0)
        duration_spin.setDecimals(1)
        duration_spin.setSuffix(" s")
        duration_spin.setValue(preset.duration_s)
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow("Marker label", label_edit)
        form.addRow("Duration", duration_spin)
        form.addRow(buttons)
        if dialog.exec_() != QDialog.Accepted:
            return
        try:
            self._set_quick_preset(index, label_edit.text(), duration_spin.value())
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid quick prompt", str(exc))
            return
        self.status_label.setText(f"Quick prompt {index + 1} updated.")

    def _add_custom_step(self) -> None:
        try:
            step = validate_prompt_plan(
                [{"label": self.custom_label_edit.text(), "duration": self.custom_duration_spin.value()}]
            )[0]
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid prompt", str(exc))
            return
        self._set_plan((*self.plan, step), selected_row=len(self.plan))
        self.custom_label_edit.clear()
        self.custom_label_edit.setFocus()
        self.status_label.setText("Prompt added. Continue building or start the task.")

    def _selected_plan_row(self) -> int | None:
        rows = self.preview_table.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def _move_selected_step(self, delta: int) -> None:
        row = self._selected_plan_row()
        if row is None:
            return
        destination = row + delta
        if destination < 0 or destination >= len(self.plan):
            return
        plan = list(self.plan)
        plan[row], plan[destination] = plan[destination], plan[row]
        self._set_plan(plan, selected_row=destination)
        self.status_label.setText("Plan order updated.")

    def _remove_selected_step(self) -> None:
        row = self._selected_plan_row()
        if row is None:
            return
        plan = list(self.plan)
        plan.pop(row)
        self._set_plan(plan, selected_row=min(row, len(plan) - 1) if plan else None)
        self.status_label.setText("Prompt removed." if plan else "Plan cleared. Add a prompt to begin.")

    def _clear_plan(self) -> None:
        self._set_plan(())
        self.status_label.setText("Plan cleared. Add a prompt to begin.")

    def load_plan(self, path: str | Path) -> None:
        try:
            plan = load_prompt_plan(path)
        except ValueError as exc:
            self.plan = ()
            self.plan_path = None
            self.plan_path_edit.clear()
            self.preview_table.setRowCount(0)
            self.total_label.setText("Invalid plan - select a corrected JSON file")
            self.status_label.setText(str(exc))
            self._refresh_controls()
            QMessageBox.warning(self, "Invalid prompt plan", str(exc))
            return
        self._set_plan(plan, path=Path(path))
        self.status_label.setText("Plan validated. Open the cue window, then start when ready.")

    def _create_lsl(self) -> tuple[MarkerPublisher, Callable[[], float]]:
        try:
            from pylsl import local_clock
            from nml_hand_exo.interface._lsl_publisher import LSLMessagePublisher
        except (ImportError, OSError, RuntimeError, SystemExit) as exc:
            raise RuntimeError(
                "pylsl/liblsl is unavailable. Install pylsl and verify the native liblsl runtime."
            ) from exc
        publisher = LSLMessagePublisher(
            name=self.stream_name_edit.text().strip() or "NML_TaskMarkers",
            stream_type="Markers",
            source_id=self.source_id_edit.text().strip() or "nml_hand_exo_task_cue",
            metadata={"application": "nml-task-cue", "protocol": "NML task cue markers v1"},
            only_on_change=False,
        )
        return publisher, local_clock

    def _initialize_marker_stream(self) -> None:
        if not self._closing and self.publish_markers_check.isChecked():
            self._ensure_lsl_outlet(show_error=True)

    def _ensure_lsl_outlet(self, *, show_error: bool) -> bool:
        if self.publisher is not None and self._publisher_is_lsl:
            return True
        self._close_publisher()
        stream_name = self.stream_name_edit.text().strip() or "NML_TaskMarkers"
        source_id = self.source_id_edit.text().strip() or "nml_hand_exo_task_cue"
        self.marker_mode_label.setText("Starting LSL marker stream...")
        try:
            self.publisher, self.marker_clock = self._create_lsl()
        except Exception as exc:
            self.publisher = None
            self.marker_clock = None
            self._publisher_is_lsl = False
            self.marker_mode_label.setText(f"LSL stream error: {exc}")
            self.marker_mode_label.setStyleSheet("color: #e74c3c;")
            print(f"[nml-task-cue] LSL marker stream failed: {exc}", file=sys.stderr, flush=True)
            if show_error:
                QMessageBox.critical(self, "Cannot start marker stream", str(exc))
            return False
        self._publisher_is_lsl = True
        self._active_stream_config = (stream_name, source_id)
        self.marker_mode_label.setText(
            f"LIVE - {stream_name} is available to LabRecorder"
        )
        self.marker_mode_label.setStyleSheet("color: #27ae60; font-weight: bold;")
        print(
            f"[nml-task-cue] LSL marker stream LIVE: "
            f"name={stream_name!r}, source_id={source_id!r}",
            flush=True,
        )
        return True

    def _restart_marker_stream(self) -> None:
        state = self.scheduler.state if self.scheduler is not None else TaskState.IDLE
        if not self.publish_markers_check.isChecked() or state is not TaskState.IDLE:
            return
        requested = (
            self.stream_name_edit.text().strip() or "NML_TaskMarkers",
            self.source_id_edit.text().strip() or "nml_hand_exo_task_cue",
        )
        if self._publisher_is_lsl and requested == self._active_stream_config:
            return
        self._close_publisher()
        self._ensure_lsl_outlet(show_error=True)

    def _on_marker_toggle(self, enabled: bool) -> None:
        if enabled:
            self.marker_mode_label.setText("Starting LSL marker stream...")
            self.marker_mode_label.setStyleSheet("")
            QTimer.singleShot(0, self._initialize_marker_stream)
        else:
            self._close_publisher()
            self.marker_mode_label.setText("Disabled - visual cues only; no LSL outlet")
            self.marker_mode_label.setStyleSheet("color: #aaaaaa;")
            print("[nml-task-cue] LSL marker stream disabled", flush=True)
        self._refresh_controls()

    def _show_cue(self) -> None:
        self.cue_window.show()
        self.cue_window.raise_()
        self.cue_window.activateWindow()

    def _start_task(self) -> None:
        if not self.plan:
            QMessageBox.warning(self, "No prompt plan", "Build or load a valid, non-empty prompt plan first.")
            return
        try:
            if self.publish_markers_check.isChecked():
                if not self._ensure_lsl_outlet(show_error=True):
                    self.status_label.setText("Start failed: LSL marker stream is unavailable")
                    return
                assert self.publisher is not None and self.marker_clock is not None
                clock = self.marker_clock
            else:
                self.publisher, clock = DisabledMarkerPublisher(), time.monotonic
                self._publisher_is_lsl = False
            self.scheduler = TaskScheduler(self.publisher, clock)
            plan_name = self.plan_path.name if self.plan_path is not None else "gui_prompt_plan.json"
            self.scheduler.start(self.plan, plan_name)
        except Exception as exc:
            self._close_publisher()
            self.scheduler = None
            QMessageBox.critical(self, "Cannot start marker stream", str(exc))
            self.status_label.setText(f"Start failed: {exc}")
            return
        self._show_cue()
        self.status_label.setText(
            "Task active - markers are publishing"
            if self.publish_markers_check.isChecked()
            else "Task active - visual cues only (LSL markers disabled)"
        )
        self._refresh_views()

    def _pause_resume(self) -> None:
        if self.scheduler is None:
            return
        try:
            if self.scheduler.state is TaskState.ACTIVE:
                self.scheduler.pause()
                self.status_label.setText("Task paused - current prompt is frozen")
            elif self.scheduler.state is TaskState.PAUSED:
                self.scheduler.resume()
                self.status_label.setText("Task resumed")
        except RuntimeError as exc:
            QMessageBox.warning(self, "Task state error", str(exc))
        self._refresh_views()

    def _abort_task(self) -> None:
        if self.scheduler is None:
            return
        self.scheduler.abort()
        self.status_label.setText(
            "Task aborted - session_abort emitted"
            if self.publish_markers_check.isChecked()
            else "Task aborted"
        )
        self._refresh_views()

    def _reset_task(self) -> None:
        if self.scheduler is not None:
            try:
                self.scheduler.reset()
            except RuntimeError as exc:
                QMessageBox.warning(self, "Task still active", str(exc))
                return
        self.scheduler = None
        if self.publish_markers_check.isChecked():
            self._ensure_lsl_outlet(show_error=False)
        else:
            self._close_publisher()
        self.cue_window.render_snapshot(None)
        self.status_label.setText("Ready to start the loaded plan")
        self._refresh_controls()

    def _on_timer(self) -> None:
        if self.scheduler is None:
            return
        previous = self.scheduler.state
        self.scheduler.tick()
        if previous is not self.scheduler.state and self.scheduler.state is TaskState.COMPLETE:
            self.status_label.setText(
                "Task complete - session_complete emitted"
                if self.publish_markers_check.isChecked()
                else "Task complete"
            )
        self._refresh_views()

    def _refresh_views(self) -> None:
        snapshot = self.scheduler.snapshot() if self.scheduler is not None else None
        self.cue_window.render_snapshot(snapshot)
        self._refresh_controls()

    def _refresh_controls(self) -> None:
        state = self.scheduler.state if self.scheduler is not None else TaskState.IDLE
        idle = state is TaskState.IDLE
        running = state in {TaskState.ACTIVE, TaskState.PAUSED}
        terminal = state in {TaskState.COMPLETE, TaskState.ABORTED}
        self.start_button.setEnabled(idle and bool(self.plan))
        self.pause_button.setEnabled(running)
        self.pause_button.setText("Resume Task" if state is TaskState.PAUSED else "Pause Task")
        self.abort_button.setEnabled(running)
        self.reset_button.setEnabled(terminal)
        markers_enabled = self.publish_markers_check.isChecked()
        self.publish_markers_check.setEnabled(idle)
        self.stream_name_edit.setEnabled(idle and markers_enabled)
        self.source_id_edit.setEnabled(idle and markers_enabled)
        self.plan_button.setEnabled(idle)
        self.save_plan_button.setEnabled(idle and bool(self.plan))
        self.preview_table.setEnabled(idle)
        for control in self.builder_controls:
            control.setEnabled(idle)

    def _close_publisher(self) -> None:
        if self.publisher is not None:
            self.publisher.close()
            self.publisher = None
        if self._publisher_is_lsl:
            print("[nml-task-cue] LSL marker stream closed", flush=True)
        self.marker_clock = None
        self._publisher_is_lsl = False
        self._active_stream_config = None

    def closeEvent(self, event) -> None:
        self._closing = True
        self.timer.stop()
        if self.scheduler is not None:
            self.scheduler.abort()
        self._close_publisher()
        self.cue_window.close()
        event.accept()


def main() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLE + CUE_STYLE)
    cue = ParticipantCueWindow()
    operator = TaskCueOperatorWindow(cue)
    operator.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
