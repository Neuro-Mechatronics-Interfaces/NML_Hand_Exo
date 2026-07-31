"""
Joystick UDP Direct Control Test GUI

Quick test GUI that sends direct velocity/current commands to HandExo GUI
through UDP command input.

Requirements:
- HandExo GUI connected to device
- Settings -> UDP Command Input enabled
- Settings -> Advanced mode enabled (allows set_velocity/set_current)
- pygame installed for joystick access: pip install pygame
"""

from __future__ import annotations

import json
import socket
import sys

from PyQt5.QtCore import QPointF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QFormLayout,
    QGraphicsDropShadowEffect,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

try:
    import pygame
except ImportError:
    pygame = None


MAX_VELOCITY_RPM = 10.0
MAX_CURRENT_MA = 200.0


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
QGroupBox[enabledPanel="true"] {
    border: 2px solid #2aa8ff;
    background-color: #202630;
}
QGroupBox[enabledPanel="true"]::title {
    color: #7ec8ff;
}
QGroupBox[enabledPanel="false"] {
    border: 1px solid #303030;
    background-color: #1f1f1f;
}
QGroupBox[enabledPanel="false"]::title {
    color: #7a7a7a;
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
QPushButton[danger="true"] {
    background-color: #7a1515;
    color: #ffffff;
    border-color: #c0392b;
}
QPushButton[danger="true"]:hover {
    background-color: #9e1d1d;
}
QPushButton[danger="true"]:pressed {
    background-color: #c0392b;
}
QLabel#status-ok {
    color: #27ae60;
    font-weight: bold;
}
QLabel#status-error {
    color: #c0392b;
    font-weight: bold;
}
"""


class UdpCommandSender:
    def __init__(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_command(self, host: str, port: int, command: str) -> None:
        payload = json.dumps({"command": command}).encode("utf-8")
        self.sock.sendto(payload, (host, port))


class VirtualJoystick2D(QWidget):
    moved = pyqtSignal(float, float)

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(130, 130)
        self._x = 0.0
        self._y = 0.0
        self._dragging = False

    @property
    def x_value(self) -> float:
        return self._x

    @property
    def y_value(self) -> float:
        return self._y

    def reset(self) -> None:
        self._set_from_xy(0.0, 0.0)

    def _set_from_xy(self, x: float, y: float) -> None:
        self._x = max(-1.0, min(1.0, x))
        self._y = max(-1.0, min(1.0, y))
        self.moved.emit(self._x, self._y)
        self.update()

    def _set_from_pos(self, pos: QPointF) -> None:
        w = float(self.width())
        h = float(self.height())
        cx = w * 0.5
        cy = h * 0.5
        radius = min(w, h) * 0.42
        if radius <= 0.0:
            return

        dx = pos.x() - cx
        dy = pos.y() - cy
        mag = (dx * dx + dy * dy) ** 0.5
        if mag > radius:
            scale = radius / mag
            dx *= scale
            dy *= scale

        self._set_from_xy(dx / radius, -dy / radius)

    def mousePressEvent(self, event):  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._set_from_pos(event.localPos())
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):  # type: ignore[override]
        if self._dragging:
            self._set_from_pos(event.localPos())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):  # type: ignore[override]
        if event.button() == Qt.LeftButton and self._dragging:
            self._dragging = False
            self.reset()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def paintEvent(self, event):  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        w = float(self.width())
        h = float(self.height())
        cx = w * 0.5
        cy = h * 0.5
        radius = min(w, h) * 0.42
        knob_r = min(w, h) * 0.12

        painter.fillRect(self.rect(), QColor("#1a1a1a"))

        pen = QPen(QColor("#444444"))
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        cross = QPen(QColor("#333333"))
        cross.setWidth(1)
        painter.setPen(cross)
        painter.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))
        painter.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))

        # subtle center marker for at-a-glance neutral position
        painter.setPen(QPen(QColor("#555555"), 1))
        painter.setBrush(QColor("#252525"))
        painter.drawEllipse(QPointF(cx, cy), max(2.0, knob_r * 0.18), max(2.0, knob_r * 0.18))

        kx = cx + self._x * radius
        ky = cy - self._y * radius
        painter.setPen(QPen(QColor("#c0392b"), 2))
        painter.setBrush(QColor("#8b1a1a"))
        painter.drawEllipse(QPointF(kx, ky), knob_r, knob_r)

        # inner highlight to keep the knob readable on dense backgrounds
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#a52222"))
        painter.drawEllipse(QPointF(kx, ky), knob_r * 0.45, knob_r * 0.45)

        super().paintEvent(event)


class JoystickUdpDirectGui(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Joystick UDP Direct Control")
        self.setMinimumSize(520, 420)

        screen = QApplication.primaryScreen()
        if screen is not None:
            available = screen.availableGeometry()
            target_width = min(860, max(560, int(available.width() * 0.55)))
            target_height = min(860, max(500, int(available.height() * 0.78)))
            self.resize(target_width, target_height)
        else:
            self.resize(700, 560)

        self.sender = UdpCommandSender()
        self.joystick = None
        self.last_sent_targets = (0.0, 0.0)
        self._last_gate_message = ""

        self._build_ui()

        self.joystick_poll_timer = QTimer(self)
        self.joystick_poll_timer.timeout.connect(self._poll_joystick)
        self.joystick_poll_timer.start(20)

        self.command_timer = QTimer(self)
        self.command_timer.timeout.connect(self._send_streaming_command)
        self.command_timer.start(50)

        if pygame is not None:
            pygame.init()
            pygame.joystick.init()
            self._reconnect_joystick()
        else:
            self._set_status(
                "pygame not installed: physical joystick disabled; virtual 2D pad is available",
                error=True,
            )

    def closeEvent(self, event):  # type: ignore[override]
        try:
            self._send_zero()
        except Exception:
            pass
        if pygame is not None:
            pygame.joystick.quit()
            pygame.quit()
        super().closeEvent(event)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        root.addWidget(scroll)

        content = QWidget()
        scroll.setWidget(content)
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(2, 2, 2, 2)
        content_layout.setSpacing(8)

        conn_group = QGroupBox("UDP Target")
        self.conn_group = conn_group
        conn_layout = QFormLayout(conn_group)
        conn_layout.setSpacing(6)
        self.host_edit = QLineEdit("127.0.0.1")
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(10001)
        conn_layout.addRow("Host", self.host_edit)
        conn_layout.addRow("Port", self.port_spin)

        mode_group = QGroupBox("Direct Command")
        self.mode_group = mode_group
        mode_layout = QGridLayout(mode_group)
        mode_layout.setHorizontalSpacing(8)
        mode_layout.setVerticalSpacing(4)
        mode_layout.setColumnStretch(0, 0)
        mode_layout.setColumnStretch(1, 1)
        mode_layout.setColumnStretch(2, 0)
        mode_layout.setColumnStretch(3, 1)

        input_group = QGroupBox("Input Source")
        self.input_group = input_group
        input_layout = QGridLayout(input_group)
        input_layout.setHorizontalSpacing(8)
        input_layout.setVerticalSpacing(6)
        self.input_source_combo = QComboBox()
        self.input_source_combo.addItems(
            ["Physical joystick", "Virtual 2D joystick (mouse)"]
        )
        self.input_source_combo.currentIndexChanged.connect(
            self._on_input_source_changed
        )
        self.virtual_joystick = VirtualJoystick2D()
        self.virtual_joystick.moved.connect(self._on_virtual_joystick_moved)
        self.virtual_value_label = QLabel("x=+0.00, y=+0.00")
        self.virtual_value_label.setStyleSheet("color: #888888;")

        input_layout.addWidget(QLabel("Source"), 0, 0)
        input_layout.addWidget(self.input_source_combo, 0, 1)
        input_layout.addWidget(self.virtual_joystick, 1, 0, 1, 2)
        input_layout.addWidget(self.virtual_value_label, 2, 0, 1, 2)

        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["velocity", "current"])
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)

        self.timeout_spin = QSpinBox()
        self.timeout_spin.setRange(50, 5000)
        self.timeout_spin.setValue(250)
        self.timeout_spin.setSuffix(" ms")

        self.scale_spin = QDoubleSpinBox()
        self.scale_spin.setRange(0.1, 1.0)
        self.scale_spin.setSingleStep(0.05)
        self.scale_spin.setValue(0.5)

        self.deadband_spin = QDoubleSpinBox()
        self.deadband_spin.setRange(0.0, 0.5)
        self.deadband_spin.setSingleStep(0.01)
        self.deadband_spin.setValue(0.08)

        self.profile_combo = QComboBox()
        self.profile_combo.addItems(
            [
                "Custom axis",
                "Gamepad deadman (left stick Y + right trigger)",
            ]
        )
        self.profile_combo.currentIndexChanged.connect(self._on_profile_changed)

        self.axis_spin = QSpinBox()
        self.axis_spin.setRange(0, 7)
        self.axis_spin.setValue(1)

        self.x_axis_spin = QSpinBox()
        self.x_axis_spin.setRange(0, 7)
        self.x_axis_spin.setValue(0)

        self.trigger_axis_spin = QSpinBox()
        self.trigger_axis_spin.setRange(0, 7)
        self.trigger_axis_spin.setValue(5)

        self.trigger_threshold_spin = QDoubleSpinBox()
        self.trigger_threshold_spin.setRange(0.0, 1.0)
        self.trigger_threshold_spin.setSingleStep(0.05)
        self.trigger_threshold_spin.setValue(0.35)

        self.invert_checkbox = QCheckBox("Invert axis")
        self.invert_checkbox.setChecked(True)

        self.arm_checkbox = QCheckBox("Arm streaming")
        self.arm_checkbox.setChecked(False)

        toggles_group = QGroupBox("Control Toggles")
        self.toggles_group = toggles_group
        toggles_layout = QGridLayout(toggles_group)
        toggles_layout.setHorizontalSpacing(10)
        toggles_layout.setVerticalSpacing(6)
        toggles_layout.addWidget(self.arm_checkbox, 0, 0, 1, 2)
        toggles_layout.addWidget(self.invert_checkbox, 0, 2, 1, 2)

        self.apply_mode_btn = QPushButton("Apply mode + timeout")
        self.apply_mode_btn.setProperty("accent", True)
        self.apply_mode_btn.clicked.connect(self._apply_mode_timeout)

        self.stop_btn = QPushButton("Send zero now")
        self.stop_btn.setProperty("danger", True)
        self.stop_btn.clicked.connect(self._send_zero)

        mode_layout.addWidget(QLabel("Mode"), 0, 0)
        mode_layout.addWidget(self.mode_combo, 0, 1)

        mode_layout.addWidget(QLabel("Watchdog"), 1, 0)
        mode_layout.addWidget(self.timeout_spin, 1, 1)
        mode_layout.addWidget(QLabel("Scale"), 2, 0)
        mode_layout.addWidget(self.scale_spin, 2, 1)

        mode_layout.addWidget(QLabel("Deadband"), 3, 0)
        mode_layout.addWidget(self.deadband_spin, 3, 1)
        mode_layout.addWidget(QLabel("Y Axis"), 3, 2)
        mode_layout.addWidget(self.axis_spin, 3, 3)

        mode_layout.addWidget(QLabel("Profile"), 4, 0)
        mode_layout.addWidget(self.profile_combo, 4, 1)

        mode_layout.addWidget(QLabel("Trigger Axis"), 5, 0)
        mode_layout.addWidget(self.trigger_axis_spin, 5, 1)
        mode_layout.addWidget(QLabel("Trigger Thresh"), 5, 2)
        mode_layout.addWidget(self.trigger_threshold_spin, 5, 3)

        mode_layout.addWidget(QLabel("Motor"), 6, 0)
        mode_layout.addWidget(QLabel("ID"), 6, 1)
        mode_layout.addWidget(QLabel("RPM"), 6, 2)
        mode_layout.addWidget(QLabel("Link"), 6, 3)

        self._linked_motor_rows: list[dict] = []
        linked_defaults = [
            ("wrist", 11, False),
            ("wrist2", 12, False),
            ("thumbadd", 13, False),
            ("thumbrot", 14, False),
            ("thumbflex", 15, False),
            ("index", 16, True),
            ("middle", 17, False),
            ("ring", 18, False),
            ("pinky", 19, False),
        ]
        for i, (label, default_id, linked) in enumerate(linked_defaults, start=7):
            name_lbl = QLabel(label)
            id_spin = QSpinBox()
            id_spin.setRange(1, 253)
            id_spin.setValue(default_id)
            rpm_lbl = QLabel("0.00")
            rpm_lbl.setStyleSheet("color: #9cc8ff;")
            link_cb = QCheckBox()
            link_cb.setChecked(linked)
            link_cb.toggled.connect(lambda _checked, _l=label: self._on_link_toggled(_l))
            mode_layout.addWidget(name_lbl, i, 0)
            mode_layout.addWidget(id_spin, i, 1)
            mode_layout.addWidget(rpm_lbl, i, 2)
            mode_layout.addWidget(link_cb, i, 3)
            self._linked_motor_rows.append(
                {
                    "label": label,
                    "id_spin": id_spin,
                    "rpm_lbl": rpm_lbl,
                    "link_cb": link_cb,
                }
            )

        button_row = QHBoxLayout()
        button_row.addWidget(self.apply_mode_btn)
        button_row.addWidget(self.stop_btn)
        mode_layout.addLayout(button_row, 16, 0, 1, 4)

        quick_group = QGroupBox("Quick Motor IDs")
        self.quick_group = quick_group
        quick_layout = QHBoxLayout(quick_group)
        self.left_ids_btn = QPushButton("Left IDs (1-9)")
        self.left_ids_btn.clicked.connect(lambda: self._set_motor_id_range(1, 9, 1))
        self.right_ids_btn = QPushButton("Right IDs (11-19)")
        self.right_ids_btn.clicked.connect(lambda: self._set_motor_id_range(11, 19, 11))
        self.dual_ids_btn = QPushButton("Dual range (1-19)")
        self.dual_ids_btn.clicked.connect(lambda: self._set_motor_id_range(1, 19, 11))
        quick_layout.addWidget(self.left_ids_btn)
        quick_layout.addWidget(self.right_ids_btn)
        quick_layout.addWidget(self.dual_ids_btn)

        status_group = QGroupBox("Status")
        self.status_group = status_group
        status_layout = QFormLayout(status_group)
        status_layout.setSpacing(6)
        self.joy_label = QLabel("No joystick")
        self.axis_label = QLabel("0.000")
        self.target_label = QLabel("0.000")
        self.target_label.setWordWrap(True)
        self.deadman_label = QLabel("N/A")
        self.last_cmd_label = QLabel("None")
        self.last_cmd_label.setWordWrap(True)
        self.status_label = QLabel("Idle")
        self.status_label.setWordWrap(True)
        status_layout.addRow("Joystick", self.joy_label)
        status_layout.addRow("Axis", self.axis_label)
        status_layout.addRow("Target", self.target_label)
        status_layout.addRow("Deadman", self.deadman_label)
        status_layout.addRow("Last command", self.last_cmd_label)
        status_layout.addRow("Message", self.status_label)

        content_layout.addWidget(conn_group)
        content_layout.addWidget(input_group)
        content_layout.addWidget(toggles_group)
        content_layout.addWidget(mode_group)
        content_layout.addWidget(quick_group)
        content_layout.addWidget(status_group)
        content_layout.addStretch(1)
        self._on_input_source_changed(self.input_source_combo.currentIndex())
        self._on_profile_changed(self.profile_combo.currentIndex())
        self._update_target_label(0.0, 0.0)
        self._refresh_panel_states()

    def _set_panel_enabled_state(self, panel: QGroupBox, enabled: bool) -> None:
        enabled = bool(enabled)
        if panel.property("enabledPanel") == enabled and panel.isEnabled() == enabled:
            return
        panel.setEnabled(enabled)
        panel.setProperty("enabledPanel", bool(enabled))
        if enabled:
            glow = QGraphicsDropShadowEffect(panel)
            glow.setBlurRadius(20)
            glow.setOffset(0, 0)
            glow.setColor(QColor(42, 168, 255, 135))
            panel.setGraphicsEffect(glow)
        else:
            panel.setGraphicsEffect(None)
        panel.style().unpolish(panel)
        panel.style().polish(panel)
        panel.update()

    def _refresh_panel_states(self) -> None:
        self._set_panel_enabled_state(self.conn_group, True)
        self._set_panel_enabled_state(self.input_group, True)
        self._set_panel_enabled_state(self.toggles_group, True)
        self._set_panel_enabled_state(self.mode_group, True)
        self._set_panel_enabled_state(self.quick_group, True)
        self._set_panel_enabled_state(self.status_group, True)

    def _set_motor_id_range(self, min_id: int, max_id: int, default_id: int) -> None:
        span = max(1, max_id - min_id + 1)
        for i, row in enumerate(self._linked_motor_rows):
            motor_id = min_id + (i % span)
            row["id_spin"].setValue(motor_id)
        self._update_target_label(0.0, 0.0)

    def _on_link_toggled(self, label: str) -> None:
        self._set_status(f"Updated linked motor: {label}")
        self._update_target_label(0.0, 0.0)

    def _set_status(self, text: str, error: bool = False) -> None:
        self.status_label.setObjectName("status-error" if error else "status-ok")
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)
        self.status_label.setText(text)

    def _set_gate_status(self, text: str, error: bool = False) -> None:
        """Update stream gate status without repaint spam every timer tick."""
        marker = f"{int(error)}:{text}"
        if marker == self._last_gate_message:
            return
        self._last_gate_message = marker
        self._set_status(text, error=error)

    def _on_mode_changed(self, mode: str) -> None:
        self._update_target_label(0.0, 0.0)

    def _on_profile_changed(self, _: int) -> None:
        deadman_profile = self.profile_combo.currentIndex() == 1 and self._using_physical()
        self.trigger_axis_spin.setEnabled(deadman_profile)
        self.trigger_threshold_spin.setEnabled(deadman_profile)
        self.deadman_label.setText("Released" if deadman_profile else "N/A")

    def _on_input_source_changed(self, _: int) -> None:
        using_virtual = not self._using_physical()
        self.virtual_joystick.setEnabled(using_virtual)
        self.virtual_joystick.setVisible(True)
        self.virtual_value_label.setEnabled(using_virtual)
        self.axis_spin.setEnabled(not using_virtual)
        self._on_profile_changed(self.profile_combo.currentIndex())
        if using_virtual:
            self.joy_label.setText("Virtual 2D joystick")
            self._set_status("Using virtual 2D joystick")
        else:
            if pygame is None:
                self.joy_label.setText("pygame missing")
                self._set_status(
                    "pygame missing: switch to Virtual 2D joystick or install pygame",
                    error=True,
                )
            else:
                self._set_status("Using physical joystick")
        self._refresh_panel_states()

    def _using_physical(self) -> bool:
        return self.input_source_combo.currentIndex() == 0

    def _on_virtual_joystick_moved(self, x: float, y: float) -> None:
        self.virtual_value_label.setText(f"x={x:+0.2f}, y={y:+0.2f}")

    def _linked_motor_targets(self, y_target: float) -> list[tuple[str, int, float]]:
        return [
            (row["label"], row["id_spin"].value(), y_target)
            for row in self._linked_motor_rows
            if row["link_cb"].isChecked()
        ]

    def _update_linked_motor_rpm(self, y_target: float) -> None:
        for row in self._linked_motor_rows:
            row["rpm_lbl"].setText(f"{y_target:+0.2f}" if row["link_cb"].isChecked() else "0.00")

    def _update_target_label(self, y_target: float, x_target: float) -> None:
        unit = "rpm" if self.mode_combo.currentText() == "velocity" else "mA"
        linked = self._linked_motor_targets(y_target)
        self._update_linked_motor_rpm(y_target)
        if not linked:
            self.target_label.setText("No linked motors selected")
            return
        summary = ", ".join(f"{mid}:{val:+0.2f}" for _, mid, val in linked)
        self.target_label.setText(f"Linked Y -> {summary} {unit}")

    def _read_axis(self, axis_index: int) -> float:
        if self.joystick is None or not self.joystick.get_init():
            return 0.0
        if axis_index < 0 or axis_index >= self.joystick.get_numaxes():
            return 0.0
        return float(self.joystick.get_axis(axis_index))

    def _deadman_active(self) -> bool:
        if not self._using_physical():
            self.deadman_label.setText("N/A")
            return True
        if self.profile_combo.currentIndex() != 1:
            self.deadman_label.setText("N/A")
            return True
        raw_trigger = self._read_axis(self.trigger_axis_spin.value())
        trigger_norm = (raw_trigger + 1.0) * 0.5
        active = trigger_norm >= self.trigger_threshold_spin.value()
        self.deadman_label.setText(
            f"{'Pressed' if active else 'Released'} ({trigger_norm:.2f})"
        )
        return active

    def _reconnect_joystick(self) -> None:
        if pygame is None:
            self._refresh_panel_states()
            return
        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= 0:
            self.joystick = None
            self.joy_label.setText("Not connected")
            self._refresh_panel_states()
            return
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.joy_label.setText(self.joystick.get_name())
        self._set_status("Joystick connected")
        self._refresh_panel_states()

    def _poll_joystick(self) -> None:
        if not self._using_physical():
            x_axis = self.virtual_joystick.x_value
            y_axis = self.virtual_joystick.y_value
            if self.invert_checkbox.isChecked():
                y_axis = -y_axis
            self.axis_label.setText(f"x={x_axis:+0.3f}, y={y_axis:+0.3f}")
            self._deadman_active()
            return

        if pygame is None:
            self.axis_label.setText("+0.000")
            return
        pygame.event.pump()
        if self.joystick is None or not self.joystick.get_init():
            self._reconnect_joystick()
            return

        y_axis = self._read_axis(self.axis_spin.value())
        if self.invert_checkbox.isChecked():
            y_axis = -y_axis
        x_axis = self._read_axis(self.x_axis_spin.value())
        self.axis_label.setText(f"x={x_axis:+0.3f}, y={y_axis:+0.3f}")
        self._deadman_active()

    def _apply_axis_scale(self, axis_val: float) -> float:
        if abs(axis_val) < self.deadband_spin.value():
            axis_val = 0.0

        mode = self.mode_combo.currentText()
        max_value = MAX_VELOCITY_RPM if mode == "velocity" else MAX_CURRENT_MA
        scaled = axis_val * self.scale_spin.value() * max_value
        return max(-max_value, min(max_value, scaled))

    def _current_targets(self) -> tuple[float, float]:
        if self._using_physical():
            if self.joystick is None or not self.joystick.get_init():
                return 0.0, 0.0
            y_axis = self._read_axis(self.axis_spin.value())
            x_axis = self._read_axis(self.x_axis_spin.value())
        else:
            y_axis = self.virtual_joystick.y_value
            x_axis = self.virtual_joystick.x_value

        if self.invert_checkbox.isChecked():
            y_axis = -y_axis

        y_target = self._apply_axis_scale(y_axis)
        x_target = self._apply_axis_scale(x_axis)
        return y_target, x_target

    def _command_string_for_motor(self, motor_id: int, target: float) -> str:
        mode = self.mode_combo.currentText()
        if mode == "velocity":
            return f"set_velocity:{motor_id}:{target:.3f}"
        return f"set_current:{motor_id}:{target:.3f}"

    def _send_command(self, command: str) -> None:
        host = self.host_edit.text().strip() or "127.0.0.1"
        port = self.port_spin.value()
        self.sender.send_command(host, port, command)
        self.last_cmd_label.setText(command)

    def _apply_mode_timeout(self) -> None:
        try:
            self._send_command(f"set_control_mode:all:{self.mode_combo.currentText()}")
            self._send_command(f"set_command_timeout:{self.timeout_spin.value()}")
            self._set_status("Applied mode and timeout")
        except OSError as exc:
            self._set_status(f"UDP send failed: {exc}", error=True)

    def _send_zero(self) -> None:
        try:
            for _, motor_id, _ in self._linked_motor_targets(0.0):
                self._send_command(self._command_string_for_motor(motor_id, 0.0))
            self.last_sent_targets = (0.0, 0.0)
            self._update_target_label(0.0, 0.0)
            self._set_status("Zero command sent")
        except OSError as exc:
            self._set_status(f"UDP send failed: {exc}", error=True)

    def _send_streaming_command(self) -> None:
        if not self.arm_checkbox.isChecked():
            self._set_gate_status(
                "Streaming paused: enable Arm streaming in Control Toggles.",
                error=True,
            )
            return

        if not self._deadman_active():
            if abs(self.last_sent_targets[0]) > 1e-6 or abs(self.last_sent_targets[1]) > 1e-6:
                self._send_zero()
            self._set_gate_status(
                "Deadman not active: hold trigger or switch Profile to Custom axis.",
                error=True,
            )
            return

        y_target, x_target = self._current_targets()

        try:
            linked = self._linked_motor_targets(y_target)
            if not linked:
                self._set_gate_status("No linked motors selected.", error=True)
                return
            for _, motor_id, value in linked:
                self._send_command(self._command_string_for_motor(motor_id, value))
            self.last_sent_targets = (y_target, 0.0)
            self._update_target_label(y_target, x_target)
            self._set_gate_status("Streaming commands")
        except OSError as exc:
            self._set_status(f"UDP send failed: {exc}", error=True)


def _main() -> int:
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLE)
    window = JoystickUdpDirectGui()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(_main())
