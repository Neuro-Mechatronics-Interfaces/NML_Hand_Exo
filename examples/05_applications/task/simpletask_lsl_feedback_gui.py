from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass

from PyQt5.QtCore import QObject, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
    QHeaderView,
    QAbstractItemView,
    QComboBox,
)


DARK_STYLE = """
QWidget {
    background-color: #1a1a1a;
    color: #e0e0e0;
    font-family: "Segoe UI", "Helvetica Neue", Arial, sans-serif;
}
QGroupBox {
    background-color: #222222;
    border: 1px solid #333333;
    border-radius: 6px;
    margin-top: 1.2em;
    padding-top: 1.0em;
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
    color: #c0392b;
}
QLabel {
    color: #e0e0e0;
}
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
    background-color: #2a2a2a;
    color: #e0e0e0;
    border: 1px solid #444444;
    border-radius: 4px;
    padding: 4px 8px;
}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {
    border-color: #c0392b;
}
QComboBox::drop-down {
    border: none;
    background: #333333;
    width: 20px;
}
QComboBox QAbstractItemView {
    background-color: #2a2a2a;
    color: #e0e0e0;
    selection-background-color: #c0392b;
}
QCheckBox {
    color: #e0e0e0;
    spacing: 6px;
}
QCheckBox::indicator {
    width: 15px;
    height: 15px;
}
QCheckBox::indicator:unchecked {
    border: 1px solid #666666;
    background: #1f1f1f;
}
QCheckBox::indicator:checked {
    border: 1px solid #c0392b;
    background: #8b1a1a;
}
QPushButton {
    background-color: #2e2e2e;
    color: #e0e0e0;
    border: 1px solid #444444;
    border-radius: 4px;
    padding: 5px 14px;
    min-height: 1.4em;
}
QPushButton:hover {
    background-color: #3a3a3a;
    border-color: #c0392b;
}
QPushButton:pressed {
    background-color: #c0392b;
    color: #ffffff;
}
QPushButton:disabled {
    background-color: #252525;
    color: #555555;
    border-color: #333333;
}
QPushButton[accent="true"] {
    background-color: #8b1a1a;
    color: #ffffff;
    border-color: #c0392b;
}
QPushButton[accent="true"]:hover {
    background-color: #a52222;
}
QPushButton[accent="true"]:pressed {
    background-color: #c0392b;
}
QTabWidget::pane {
    border: 1px solid #333333;
    background-color: #1a1a1a;
}
QTabBar::tab {
    background-color: #2e2e2e;
    color: #e0e0e0;
    padding: 6px 18px;
    border: 1px solid #444444;
    border-bottom: none;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    min-width: 80px;
}
QTabBar::tab:selected {
    background-color: #1a1a1a;
    color: #ffffff;
    border-color: #c0392b;
    border-bottom: 2px solid #c0392b;
}
QTableWidget {
    background-color: #1a1a1a;
    alternate-background-color: #222222;
    color: #e0e0e0;
    gridline-color: #333333;
    border: 1px solid #333333;
}
QHeaderView::section {
    background-color: #2e2e2e;
    color: #e0e0e0;
    border: 1px solid #333333;
    padding: 4px 8px;
    font-weight: bold;
}
"""


@dataclass
class MotorState:
    visible: bool = True
    goal: float = 50.0
    value: float = 0.0


TORQUE_PHYSICAL_MAX_NM = 0.076
TORQUE_DEFAULT_VIEW_MAX_NM = 0.1
DEFAULT_GOAL_PERCENT = 50.0
DEFAULT_GOAL_BUFFER_PERCENT = 5.0


class VerticalFeedbackBar(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._value = 0.0
        self._goal = 0.5
        self._buffer = 0.05
        self._scale_min = 0.0
        self._scale_max = 1.0
        self.setMinimumSize(34, 220)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)

    def set_data(
        self,
        value: float,
        goal: float,
        goal_buffer: float,
        scale_min: float,
        scale_max: float,
    ) -> None:
        self._value = max(0.0, float(value))
        self._goal = max(0.0, float(goal))
        self._buffer = max(0.0, float(goal_buffer))
        lo = float(scale_min)
        hi = float(scale_max)
        if hi <= lo:
            hi = lo + 1e-6
        self._scale_min = lo
        self._scale_max = hi
        self.update()

    def _normalize(self, magnitude: float) -> float:
        span = self._scale_max - self._scale_min
        if span <= 0:
            return 0.0
        return max(0.0, min(1.0, (magnitude - self._scale_min) / span))

    def paintEvent(self, event):  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        outer = self.rect().adjusted(10, 6, -10, -6)
        if outer.height() <= 0 or outer.width() <= 0:
            return

        painter.setPen(QPen(QColor("#444444"), 1))
        painter.setBrush(QColor("#111111"))
        painter.drawRoundedRect(outer, 4, 4)

        goal_lo = max(self._scale_min, self._goal - self._buffer)
        goal_hi = min(self._scale_max, self._goal + self._buffer)
        goal_lo_n = self._normalize(goal_lo)
        goal_hi_n = self._normalize(goal_hi)
        if goal_hi_n >= goal_lo_n:
            y1 = outer.bottom() - int(goal_hi_n * outer.height())
            y2 = outer.bottom() - int(goal_lo_n * outer.height())
            goal_top = min(y1, y2)
            goal_height = max(2, abs(y2 - y1) + 1)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(39, 174, 96, 140))
            painter.drawRect(outer.left() + 2, goal_top, outer.width() - 3, goal_height)

            glow_rect = outer.adjusted(1, 0, -1, 0)
            glow_rect.setTop(goal_top - 1)
            glow_rect.setHeight(goal_height + 2)
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(QColor(42, 168, 255, 110), 4))
            painter.drawRoundedRect(glow_rect, 3, 3)
            painter.setPen(QPen(QColor(126, 200, 255, 220), 2))
            painter.drawRoundedRect(glow_rect.adjusted(1, 1, -1, -1), 2, 2)

        value_n = self._normalize(self._value)
        fill_top = outer.bottom() - int(value_n * outer.height())
        ratio = self._value / max(1e-6, self._goal) if self._goal > 0 else 0.0
        if ratio < 0.8:
            color = QColor("#2aa8ff")
        elif ratio <= 1.1:
            color = QColor("#27ae60")
        else:
            color = QColor("#c0392b")
        painter.setPen(Qt.NoPen)
        painter.setBrush(color)
        painter.drawRect(
            outer.left() + 2,
            fill_top,
            outer.width() - 3,
            max(1, outer.bottom() - fill_top),
        )

        marker_y = outer.bottom() - int(self._normalize(self._goal) * outer.height())
        painter.setPen(QPen(QColor("#7ec8ff"), 2))
        painter.drawLine(outer.left(), marker_y, outer.right(), marker_y)


class LSLStreamWorker(QObject):
    sample_received = pyqtSignal(dict)
    stream_changed = pyqtSignal(str, list)
    status_changed = pyqtSignal(str, bool)

    def __init__(self) -> None:
        super().__init__()
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._tick)
        self._inlet = None
        self._pylsl = None
        self._stream_kind = "torque"
        self._preferred_kind = "auto"
        self._last_resolve = 0.0
        self._resolve_interval_s = 3.0
        self._last_err = ""
        self._channel_names: list[str] = []

    def start(self) -> None:
        self._timer.start()

    def stop(self) -> None:
        self._timer.stop()
        if self._inlet is not None:
            try:
                self._inlet.close_stream()
            except Exception:
                pass
        self._inlet = None

    def set_preferred_kind(self, preferred_kind: str) -> None:
        self._preferred_kind = preferred_kind
        self._inlet = None
        self._channel_names = []
        self._last_resolve = 0.0

    def _emit_status(self, text: str, is_error: bool = False) -> None:
        marker = f"{int(is_error)}:{text}"
        if marker == self._last_err:
            return
        self._last_err = marker
        self.status_changed.emit(text, is_error)

    def _pick_targets(self) -> list[tuple[str, str]]:
        if self._preferred_kind == "torque":
            return [("torque", "NMLHandExoMotorTorque"), ("torque", "MotorTorque")]
        if self._preferred_kind == "position":
            return [("position", "NMLHandExoJointAngles"), ("position", "JointAngles")]
        return [
            ("torque", "NMLHandExoMotorTorque"),
            ("torque", "MotorTorque"),
            ("position", "NMLHandExoJointAngles"),
            ("position", "JointAngles"),
        ]

    def _resolve_stream(self) -> None:
        now = time.monotonic()
        if now - self._last_resolve < self._resolve_interval_s:
            return
        self._last_resolve = now
        try:
            if self._pylsl is None:
                from pylsl import StreamInlet, resolve_byprop  # type: ignore

                self._pylsl = (StreamInlet, resolve_byprop)
            stream_inlet_cls, resolve_byprop = self._pylsl
            for kind, query in self._pick_targets():
                by_name = resolve_byprop("name", query, timeout=0.03)
                stream = by_name[0] if by_name else None
                if stream is None:
                    by_type = resolve_byprop("type", query, timeout=0.03)
                    stream = by_type[0] if by_type else None
                if stream is None:
                    continue
                self._stream_kind = kind
                self._inlet = stream_inlet_cls(stream, recover=True)
                info = self._inlet.info()
                self._channel_names = self._extract_channels(info)
                self.stream_changed.emit(self._stream_kind, list(self._channel_names))
                self._emit_status(
                    f"Connected to {info.name()} ({self._stream_kind})", False
                )
                return
            self._emit_status("Waiting for LSL torque/position stream...", False)
        except Exception as exc:
            self._emit_status(f"LSL unavailable: {exc}", True)

    def _extract_channels(self, info) -> list[str]:
        labels: list[str] = []
        count = int(info.channel_count())
        try:
            node = info.desc().child("channels").child("channel")
            for i in range(count):
                label = node.child_value("label")
                labels.append(label or f"motor_{i + 1}")
                node = node.next_sibling()
        except Exception:
            labels = [f"motor_{i + 1}" for i in range(count)]
        return labels

    def _tick(self) -> None:
        if self._inlet is None:
            self._resolve_stream()
            return
        try:
            sample, _ = self._inlet.pull_sample(timeout=0.0)
        except Exception:
            self._inlet = None
            self._channel_names = []
            self._emit_status("LSL stream dropped, reconnecting...", True)
            return
        if sample is None:
            return
        if not self._channel_names:
            self._channel_names = [f"motor_{i + 1}" for i in range(len(sample))]
        values = {}
        for i, value in enumerate(sample):
            name = (
                self._channel_names[i]
                if i < len(self._channel_names)
                else f"motor_{i + 1}"
            )
            values[name] = float(value)
        self.sample_received.emit(values)


class MotorFeedbackCard(QFrame):
    def __init__(self, name: str) -> None:
        super().__init__()
        self.name = name
        self._value = 0.0
        self._goal = 0.5
        self._scale_min = 0.0
        self._scale_max = 1.0
        self._buffer = 0.05
        self.setObjectName("motor-card")
        self.setFrameShape(QFrame.StyledPanel)
        self.setMinimumWidth(120)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._build_ui()
        self.set_size_preset("Medium")

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        self.name_lbl = QLabel(self.name)
        self.name_lbl.setAlignment(Qt.AlignCenter)
        self.name_lbl.setStyleSheet("font-weight: bold;")
        self.value_lbl = QLabel("0.000")
        self.value_lbl.setAlignment(Qt.AlignCenter)
        self.value_lbl.setStyleSheet("font-size: 18px; color: #9cc8ff;")
        self.goal_lbl = QLabel("Goal: 50.0% (0.0380 Nm)")
        self.goal_lbl.setAlignment(Qt.AlignCenter)
        self.goal_lbl.setStyleSheet("color: #aaaaaa;")
        self.goal_range_lbl = QLabel("Range: 0.450 - 0.550")
        self.goal_range_lbl.setAlignment(Qt.AlignCenter)
        self.goal_range_lbl.setStyleSheet("color: #77cc99;")
        self.scale_max_lbl = QLabel("1.000")
        self.scale_max_lbl.setAlignment(Qt.AlignCenter)
        self.scale_max_lbl.setStyleSheet("color: #888888; font-size: 10px;")
        self.scale_min_lbl = QLabel("0.000")
        self.scale_min_lbl.setAlignment(Qt.AlignCenter)
        self.scale_min_lbl.setStyleSheet("color: #888888; font-size: 10px;")

        self.bar = VerticalFeedbackBar()
        self.bar.setMinimumHeight(220)

        layout.addWidget(self.name_lbl)
        layout.addWidget(self.value_lbl)
        layout.addWidget(self.scale_max_lbl)
        layout.addWidget(self.bar, 1, Qt.AlignHCenter)
        layout.addWidget(self.scale_min_lbl)
        layout.addWidget(self.goal_lbl)
        layout.addWidget(self.goal_range_lbl)

    def set_size_preset(self, preset: str) -> None:
        if preset == "Large":
            self.setMinimumWidth(168)
            self.bar.setMinimumHeight(320)
        else:
            self.setMinimumWidth(120)
            self.bar.setMinimumHeight(220)

    def update_values(
        self,
        value: float,
        goal: float,
        goal_pct: float,
        unit: str,
        scale_min: float,
        scale_max: float,
        goal_buffer: float,
    ) -> None:
        safe_value = float(value) if math.isfinite(float(value)) else 0.0
        safe_goal = float(goal) if math.isfinite(float(goal)) else 0.0
        self._value = safe_value
        self._goal = max(1e-6, abs(safe_goal))
        self._scale_min = max(0.0, float(scale_min))
        self._scale_max = max(self._scale_min + 1e-6, float(scale_max))
        self._buffer = max(0.0, float(goal_buffer))
        magnitude = abs(safe_value)
        self.value_lbl.setText(f"{safe_value:+.3f} {unit}")
        self.goal_lbl.setText(f"Goal: {goal_pct:.1f}% ({safe_goal:.4f} {unit})")
        goal_low = max(self._scale_min, self._goal - self._buffer)
        goal_high = min(self._scale_max, self._goal + self._buffer)
        self.goal_range_lbl.setText(f"Range: {goal_low:.3f} - {goal_high:.3f} {unit}")
        self.scale_max_lbl.setText(f"{self._scale_max:.3f} {unit}")
        self.scale_min_lbl.setText(f"{self._scale_min:.3f} {unit}")
        self.bar.set_data(
            value=magnitude,
            goal=self._goal,
            goal_buffer=self._buffer,
            scale_min=self._scale_min,
            scale_max=self._scale_max,
        )


class SimpleTaskLslFeedbackGui(QWidget):
    stream_preference_changed = pyqtSignal(str)
    stop_worker_requested = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("SimpleTask LSL Feedback")
        self.resize(1080, 700)
        self.setMinimumSize(760, 500)

        self._motor_states: dict[str, MotorState] = {}
        self._cards: dict[str, MotorFeedbackCard] = {}
        self._editing_goals = False
        self._active_kind = "torque"
        self._max_torque_nm = TORQUE_PHYSICAL_MAX_NM
        self._scale_size_preset = "Medium"
        self._display_scale: dict[str, tuple[float, float]] = {
            "torque": (0.0, TORQUE_DEFAULT_VIEW_MAX_NM),
            "position": (0.0, 180.0),
        }
        self._goal_buffer_by_kind: dict[str, float] = {
            "torque": DEFAULT_GOAL_BUFFER_PERCENT,
            "position": DEFAULT_GOAL_BUFFER_PERCENT,
        }

        self._worker_thread = QThread(self)
        self._worker = LSLStreamWorker()
        self._worker.moveToThread(self._worker_thread)
        self._worker_thread.started.connect(self._worker.start)
        self.stream_preference_changed.connect(self._worker.set_preferred_kind)
        self.stop_worker_requested.connect(self._worker.stop)
        self._worker.sample_received.connect(self._on_sample)
        self._worker.stream_changed.connect(self._on_stream_changed)
        self._worker.status_changed.connect(self._on_status_changed)

        self._build_ui()
        self._worker_thread.start()

    def closeEvent(self, event):  # type: ignore[override]
        self.stop_worker_requested.emit()
        self._worker_thread.quit()
        self._worker_thread.wait(1200)
        super().closeEvent(event)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        top_row = QHBoxLayout()
        self.status_lbl = QLabel("Waiting for LSL stream...")
        self.status_lbl.setStyleSheet("color: #888888;")
        self.stream_mode_combo = QComboBox()
        self.stream_mode_combo.addItems(
            ["Auto (prefer torque)", "Torque stream", "Position stream"]
        )
        self.stream_mode_combo.currentIndexChanged.connect(self._on_stream_preference_changed)
        top_row.addWidget(QLabel("Stream:"))
        top_row.addWidget(self.stream_mode_combo)
        top_row.addStretch()
        top_row.addWidget(self.status_lbl, 1)
        root.addLayout(top_row)

        self.tabs = QTabWidget()
        root.addWidget(self.tabs, 1)
        self.tabs.addTab(self._build_display_tab(), "Display")
        self.tabs.addTab(self._build_config_tab(), "Config")

    def _build_display_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        self.display_header = QLabel("No motors yet")
        self.display_header.setStyleSheet("font-weight: bold; color: #9cc8ff;")
        layout.addWidget(self.display_header)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        layout.addWidget(scroll, 1)

        self.display_content = QWidget()
        self.cards_layout = QGridLayout(self.display_content)
        self.cards_layout.setContentsMargins(2, 2, 2, 2)
        self.cards_layout.setHorizontalSpacing(10)
        self.cards_layout.setVerticalSpacing(10)
        scroll.setWidget(self.display_content)
        return tab

    def _build_config_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        goals_box = QGroupBox("Goal Controls")
        goals_layout = QHBoxLayout(goals_box)
        goals_layout.setContentsMargins(10, 8, 10, 8)
        self.lock_goals_cb = QCheckBox("Lock goals together (vertical align)")
        self.lock_goals_cb.setChecked(False)
        self.sync_current_btn = QPushButton("Set goals to current values")
        self.sync_current_btn.setProperty("accent", True)
        self.sync_current_btn.clicked.connect(self._sync_goals_to_current)
        self.align_goals_btn = QPushButton("Align all to first visible goal")
        self.align_goals_btn.clicked.connect(self._align_goals)
        goals_layout.addWidget(self.lock_goals_cb)
        goals_layout.addStretch()
        goals_layout.addWidget(self.sync_current_btn)
        goals_layout.addWidget(self.align_goals_btn)
        layout.addWidget(goals_box)

        scale_box = QGroupBox("Display Scale and Goal Buffer")
        scale_layout = QHBoxLayout(scale_box)
        scale_layout.setContentsMargins(10, 8, 10, 8)
        self.scale_min_spin = QDoubleSpinBox()
        self.scale_min_spin.setDecimals(4)
        self.scale_min_spin.setRange(0.0, 9999.0)
        self.scale_min_spin.setSingleStep(0.001)
        self.scale_max_spin = QDoubleSpinBox()
        self.scale_max_spin.setDecimals(4)
        self.scale_max_spin.setRange(0.0001, 9999.0)
        self.scale_max_spin.setSingleStep(0.001)
        self.goal_buffer_spin = QDoubleSpinBox()
        self.goal_buffer_spin.setDecimals(1)
        self.goal_buffer_spin.setRange(0.0, 100.0)
        self.goal_buffer_spin.setSingleStep(1.0)
        self.goal_buffer_spin.setSuffix(" %")
        self.scale_size_combo = QComboBox()
        self.scale_size_combo.addItems(["Medium", "Large"])
        self.scale_size_combo.setCurrentText(self._scale_size_preset)
        self.max_torque_spin = QDoubleSpinBox()
        self.max_torque_spin.setDecimals(4)
        self.max_torque_spin.setRange(0.0001, 9999.0)
        self.max_torque_spin.setSingleStep(0.001)
        self.max_torque_spin.setValue(self._max_torque_nm)
        self.scale_min_spin.valueChanged.connect(self._on_scale_controls_changed)
        self.scale_max_spin.valueChanged.connect(self._on_scale_controls_changed)
        self.goal_buffer_spin.valueChanged.connect(self._on_scale_controls_changed)
        self.max_torque_spin.valueChanged.connect(self._on_max_torque_changed)
        self.scale_size_combo.currentTextChanged.connect(self._on_scale_size_changed)
        scale_layout.addWidget(QLabel("Max torque (Nm)"))
        scale_layout.addWidget(self.max_torque_spin)
        scale_layout.addWidget(QLabel("Scale size"))
        scale_layout.addWidget(self.scale_size_combo)
        scale_layout.addWidget(QLabel("Min"))
        scale_layout.addWidget(self.scale_min_spin)
        scale_layout.addWidget(QLabel("Max"))
        scale_layout.addWidget(self.scale_max_spin)
        scale_layout.addWidget(QLabel("Goal Buffer (%)"))
        scale_layout.addWidget(self.goal_buffer_spin)
        scale_layout.addStretch()
        layout.addWidget(scale_box)

        table_box = QGroupBox("Motor Visibility and Goals")
        table_layout = QVBoxLayout(table_box)
        self.config_table = QTableWidget(0, 4)
        self.config_table.setHorizontalHeaderLabels(
            ["Show", "Motor", "Goal (%)", "Current"]
        )
        self.config_table.verticalHeader().setVisible(False)
        self.config_table.setAlternatingRowColors(True)
        self.config_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.config_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.config_table.itemChanged.connect(self._on_table_item_changed)
        hdr = self.config_table.horizontalHeader()
        hdr.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(1, QHeaderView.Stretch)
        hdr.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        table_layout.addWidget(self.config_table)
        layout.addWidget(table_box, 1)
        return tab

    def _on_stream_preference_changed(self, index: int) -> None:
        mapping = {0: "auto", 1: "torque", 2: "position"}
        self.stream_preference_changed.emit(mapping.get(index, "auto"))

    def _on_status_changed(self, text: str, is_error: bool) -> None:
        self.status_lbl.setText(text)
        self.status_lbl.setStyleSheet(
            "color: #c0392b;" if is_error else "color: #27ae60;"
        )

    def _on_stream_changed(self, stream_kind: str, channels: list[str]) -> None:
        self._active_kind = stream_kind
        for name in channels:
            if name not in self._motor_states:
                self._motor_states[name] = MotorState(
                    visible=True,
                    goal=self._default_goal_for_active_kind(),
                    value=0.0,
                )
        stale = [name for name in list(self._motor_states.keys()) if name not in channels]
        for name in stale:
            del self._motor_states[name]
            self._cards.pop(name, None)
        self._sync_scale_controls_from_kind()
        self._rebuild_config_table()
        self._rebuild_cards()

    def _on_sample(self, values: dict) -> None:
        for name, val in values.items():
            num = float(val)
            if not math.isfinite(num):
                continue
            state = self._motor_states.get(name)
            if state is None:
                state = MotorState(
                    visible=True,
                    goal=self._default_goal_for_active_kind(),
                    value=num,
                )
                self._motor_states[name] = state
            state.value = num
        self._refresh_current_cells()
        self._refresh_cards()

    def _unit(self) -> str:
        return "Nm" if self._active_kind == "torque" else "deg"

    def _default_goal_for_active_kind(self) -> float:
        return DEFAULT_GOAL_PERCENT

    def _max_reference_for_active_kind(self) -> float:
        if self._active_kind == "torque":
            return max(1e-6, float(self._max_torque_nm))
        _, scale_max = self._display_scale.get("position", (0.0, 180.0))
        return max(1e-6, float(scale_max))

    def _goal_value_from_percent(self, goal_pct: float) -> float:
        return self._max_reference_for_active_kind() * max(0.0, min(100.0, float(goal_pct))) / 100.0

    def _goal_buffer_value_from_percent(self, buffer_pct: float) -> float:
        return self._max_reference_for_active_kind() * max(0.0, min(100.0, float(buffer_pct))) / 100.0

    def _clamp_goal_pct(self, value: float) -> float:
        return max(0.0, min(100.0, float(value)))

    def _rebuild_config_table(self) -> None:
        self._editing_goals = True
        self.config_table.setRowCount(len(self._motor_states))
        for row, name in enumerate(sorted(self._motor_states.keys())):
            state = self._motor_states[name]

            show_item = QTableWidgetItem()
            show_item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsUserCheckable)
            show_item.setCheckState(Qt.Checked if state.visible else Qt.Unchecked)
            self.config_table.setItem(row, 0, show_item)

            name_item = QTableWidgetItem(name)
            name_item.setTextAlignment(Qt.AlignCenter)
            self.config_table.setItem(row, 1, name_item)

            goal_spin = QDoubleSpinBox()
            goal_spin.setRange(0.0, 100.0)
            goal_spin.setSingleStep(1.0)
            goal_spin.setDecimals(1)
            goal_spin.setSuffix(" %")
            goal_spin.setValue(self._clamp_goal_pct(state.goal))
            goal_spin.valueChanged.connect(
                lambda value, motor_name=name: self._on_goal_changed(motor_name, value)
            )
            self.config_table.setCellWidget(row, 2, goal_spin)

            current_item = QTableWidgetItem(f"{state.value:+.3f}")
            current_item.setTextAlignment(Qt.AlignCenter)
            self.config_table.setItem(row, 3, current_item)

        self._editing_goals = False
        self._sync_scale_controls_from_kind()

    def _refresh_current_cells(self) -> None:
        for row in range(self.config_table.rowCount()):
            name_item = self.config_table.item(row, 1)
            value_item = self.config_table.item(row, 3)
            if name_item is None or value_item is None:
                continue
            state = self._motor_states.get(name_item.text())
            if state is None:
                continue
            value_item.setText(f"{state.value:+.3f}")

    def _on_table_item_changed(self, item: QTableWidgetItem) -> None:
        if self._editing_goals or item.column() != 0:
            return
        name_item = self.config_table.item(item.row(), 1)
        if name_item is None:
            return
        state = self._motor_states.get(name_item.text())
        if state is None:
            return
        state.visible = item.checkState() == Qt.Checked
        self._rebuild_cards()

    def _on_goal_changed(self, motor_name: str, value: float) -> None:
        state = self._motor_states.get(motor_name)
        if state is None:
            return
        state.goal = self._clamp_goal_pct(value)
        if self.lock_goals_cb.isChecked():
            self._set_all_goals(state.goal)
        self._refresh_cards()

    def _set_all_goals(self, goal: float) -> None:
        clamped_goal = self._clamp_goal_pct(goal)
        self._editing_goals = True
        for row in range(self.config_table.rowCount()):
            spin = self.config_table.cellWidget(row, 2)
            if not isinstance(spin, QDoubleSpinBox):
                continue
            spin.blockSignals(True)
            spin.setValue(clamped_goal)
            spin.blockSignals(False)
        for state in self._motor_states.values():
            state.goal = clamped_goal
        self._editing_goals = False
        self._refresh_cards()

    def _sync_goals_to_current(self) -> None:
        if not self._motor_states:
            return
        self._editing_goals = True
        for row in range(self.config_table.rowCount()):
            name_item = self.config_table.item(row, 1)
            spin = self.config_table.cellWidget(row, 2)
            if name_item is None or not isinstance(spin, QDoubleSpinBox):
                continue
            state = self._motor_states.get(name_item.text())
            if state is None:
                continue
            max_ref = self._max_reference_for_active_kind()
            state.goal = self._clamp_goal_pct((abs(state.value) / max_ref) * 100.0)
            spin.blockSignals(True)
            spin.setValue(state.goal)
            spin.blockSignals(False)
        self._editing_goals = False
        self._refresh_cards()

    def _align_goals(self) -> None:
        target = None
        for name in sorted(self._motor_states.keys()):
            state = self._motor_states[name]
            if state.visible:
                target = state.goal
                break
        if target is None:
            return
        self._set_all_goals(target)

    def _rebuild_cards(self) -> None:
        while self.cards_layout.count():
            item = self.cards_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)

        visible_names = [
            name for name in sorted(self._motor_states.keys()) if self._motor_states[name].visible
        ]
        self.display_header.setText(
            f"Displaying {len(visible_names)} motor(s) from {self._active_kind} stream"
        )
        self._cards = {}
        for i, name in enumerate(visible_names):
            card = MotorFeedbackCard(name)
            card.set_size_preset(self._scale_size_preset)
            self._cards[name] = card
            row = i // 6
            col = i % 6
            self.cards_layout.addWidget(card, row, col)
        self._refresh_cards()

    def _sync_scale_controls_from_kind(self) -> None:
        scale_min, scale_max = self._display_scale.get(self._active_kind, (0.0, 1.0))
        goal_buffer = self._goal_buffer_by_kind.get(self._active_kind, 0.05)
        self.scale_min_spin.blockSignals(True)
        self.scale_max_spin.blockSignals(True)
        self.goal_buffer_spin.blockSignals(True)
        self.max_torque_spin.blockSignals(True)
        self.scale_size_combo.blockSignals(True)
        self.scale_min_spin.setValue(scale_min)
        self.scale_max_spin.setValue(scale_max)
        self.goal_buffer_spin.setValue(goal_buffer)
        self.max_torque_spin.setValue(self._max_torque_nm)
        self.scale_size_combo.setCurrentText(self._scale_size_preset)
        self.scale_min_spin.blockSignals(False)
        self.scale_max_spin.blockSignals(False)
        self.goal_buffer_spin.blockSignals(False)
        self.max_torque_spin.blockSignals(False)
        self.scale_size_combo.blockSignals(False)

    def _on_scale_controls_changed(self, _value: float) -> None:
        low = float(self.scale_min_spin.value())
        high = float(self.scale_max_spin.value())
        if high <= low:
            high = low + 0.0001
            self.scale_max_spin.blockSignals(True)
            self.scale_max_spin.setValue(high)
            self.scale_max_spin.blockSignals(False)
        self._display_scale[self._active_kind] = (low, high)
        self._goal_buffer_by_kind[self._active_kind] = max(
            0.0, min(100.0, float(self.goal_buffer_spin.value()))
        )
        self._refresh_cards()

    def _on_max_torque_changed(self, value: float) -> None:
        self._max_torque_nm = max(0.0001, float(value))
        self._refresh_cards()

    def _on_scale_size_changed(self, preset: str) -> None:
        self._scale_size_preset = "Large" if preset == "Large" else "Medium"
        for card in self._cards.values():
            card.set_size_preset(self._scale_size_preset)
        self._refresh_cards()

    def _refresh_cards(self) -> None:
        unit = self._unit()
        scale_min, scale_max = self._display_scale.get(self._active_kind, (0.0, 1.0))
        goal_buffer_pct = self._goal_buffer_by_kind.get(
            self._active_kind, DEFAULT_GOAL_BUFFER_PERCENT
        )
        goal_buffer = self._goal_buffer_value_from_percent(goal_buffer_pct)
        for name, card in self._cards.items():
            state = self._motor_states.get(name)
            if state is None:
                continue
            goal_pct = self._clamp_goal_pct(state.goal)
            goal_value = self._goal_value_from_percent(goal_pct)
            card.update_values(
                state.value,
                goal_value,
                goal_pct,
                unit,
                scale_min=scale_min,
                scale_max=scale_max,
                goal_buffer=goal_buffer,
            )


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLE)
    window = SimpleTaskLslFeedbackGui()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
