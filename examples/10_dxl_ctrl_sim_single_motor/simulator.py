"""PyQt5 UI for the single-motor Dynamixel control simulator."""

from __future__ import annotations

import math
from pathlib import Path
import sys

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QBrush, QFont, QFontMetrics, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from control_model import (
    Objective,
    OperatingMode,
    SimulationConfig,
    SimulationSnapshot,
    SingleMotorSimulation,
)


ROOT = Path(__file__).resolve().parent


def _spin(
    minimum: float,
    maximum: float,
    value: float,
    step: float,
    decimals: int = 2,
) -> QDoubleSpinBox:
    box = QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setValue(value)
    box.setSingleStep(step)
    box.setDecimals(decimals)
    return box


def _int_spin(minimum: int, maximum: int, value: int, step: int = 1) -> QSpinBox:
    box = QSpinBox()
    box.setRange(minimum, maximum)
    box.setValue(value)
    box.setSingleStep(step)
    return box


def _objectives_for_mode(mode: OperatingMode) -> tuple[Objective, ...]:
    if mode in {
        OperatingMode.POSITION,
        OperatingMode.EXTENDED_POSITION,
        OperatingMode.CURRENT_POSITION,
    }:
        return (
            Objective.HOLD,
            Objective.POSITION_TRAJECTORY,
            Objective.SECONDARY_GOAL_ASSIST,
            Objective.TORQUE_ASSIST,
            Objective.EXTERNAL_EFFORT_ASSIST,
        )
    if mode == OperatingMode.VELOCITY:
        return (Objective.VELOCITY_COMMAND, Objective.TORQUE_ASSIST, Objective.EXTERNAL_EFFORT_ASSIST)
    if mode == OperatingMode.CURRENT:
        return (Objective.CURRENT_COMMAND, Objective.TORQUE_ASSIST, Objective.EXTERNAL_EFFORT_ASSIST)
    return (Objective.PWM_COMMAND, Objective.TORQUE_ASSIST)


class DynamicFlowWidget(QWidget):
    """Render the selected controller path and its live signal amplitudes."""

    COLORS = {
        "position": QColor("#38bdf8"),
        "velocity": QColor("#a78bfa"),
        "current": QColor("#f472b6"),
        "torque": QColor("#fb923c"),
        "plant": QColor("#94a3b8"),
        "feedback": QColor("#22d3ee"),
        "endstop": QColor("#f87171"),
        "assist": QColor("#4ade80"),
    }

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.config = SimulationConfig()
        self.snapshot = SingleMotorSimulation(self.config).previous_snapshot
        self.setMinimumHeight(300)
        self.setToolTip(
            "Signal colors: blue=position, violet=velocity, pink=current, "
            "orange=torque, red=endstop. Line width follows amplitude."
        )

    def update_state(self, config: SimulationConfig, snapshot: SimulationSnapshot) -> None:
        self.config = config
        self.snapshot = snapshot
        self.update()

    @staticmethod
    def _node_text(key: str, snapshot: SimulationSnapshot, config: SimulationConfig) -> str:
        values = {
            "input": f"scenario\nq={snapshot.position_deg:.1f}°",
            "assist": f"assist map\nτa={config.assist_torque_nm:+.2f}",
            "target": f"profile / target\nqref={snapshot.reference_position_deg:.1f}°",
            "error": f"position error\ne={snapshot.position_error_deg:+.1f}°",
            "pid": f"position PID\nτ={snapshot.pid_torque_nm:+.2f}",
            "goal_current": (
                f"Goal Current\nIgoal={snapshot.goal_current_ma:.0f} mA "
                f"/ limit={config.goal_current_limit_ma:.0f}"
            ),
            "pwm_clamp": f"PWM clamp\n{snapshot.pwm_percent:+.0f}%",
            "velocity_target": f"velocity target\nvref={snapshot.reference_velocity_deg_s:+.1f}°/s",
            "velocity_error": f"velocity error\nv={snapshot.velocity_deg_s:+.1f}°/s",
            "velocity_pi": f"velocity PI\nτ={snapshot.pid_torque_nm:+.2f}",
            "current": f"signed current\nI={snapshot.current_ma:+.0f} mA",
            "pwm": f"signed PWM\n{snapshot.pwm_percent:+.0f}%",
            "current_convert": f"current → torque\nτ={snapshot.actuator_torque_nm:+.2f}",
            "plant": f"motor + load\nq={snapshot.position_deg:.1f}°",
            "external": f"external effort\nτ={snapshot.external_torque_nm:+.2f}",
            "endstop": f"endstop reaction\nτ={snapshot.endstop_torque_nm:+.2f}",
            "state": f"feedback state\nv={snapshot.velocity_deg_s:+.1f}°/s",
        }
        return values.get(key, key)

    def _path(self) -> tuple[list[tuple[str, str, QColor]], list[tuple[str, str, QColor, float, str, float]]]:
        c = self.config
        s = self.snapshot
        nodes: list[tuple[str, str, QColor]] = []
        edges: list[tuple[str, str, QColor, float, str, float]] = []

        def add(key: str, color: str) -> None:
            nodes.append((key, self._node_text(key, s, c), self.COLORS[color]))

        def connect(a: str, b: str, color: str, value: float, label: str, scale: float) -> None:
            edges.append((a, b, self.COLORS[color], value, label, scale))

        add("input", "position")
        previous = "input"
        if c.objective in {
            Objective.SECONDARY_GOAL_ASSIST,
            Objective.TORQUE_ASSIST,
        }:
            add("assist", "assist")
            connect(previous, "assist", "assist", c.assist_torque_nm, "assist command", 1.0)
            previous = "assist"

        if c.mode in {OperatingMode.POSITION, OperatingMode.EXTENDED_POSITION, OperatingMode.CURRENT_POSITION}:
            add("target", "position")
            connect(previous, "target", "position", s.reference_position_deg, "qref", 360.0)
            add("error", "feedback")
            connect("target", "error", "position", s.position_error_deg, "qref − q", 180.0)
            add("pid", "torque")
            connect("error", "pid", "torque", s.position_error_deg, "PID", 180.0)
            previous = "pid"
            if c.mode == OperatingMode.CURRENT_POSITION:
                add("goal_current", "current")
                connect(previous, "goal_current", "current", s.goal_current_ma, "Goal Current", 910.0)
                previous = "goal_current"
            else:
                add("pwm_clamp", "torque")
                connect(previous, "pwm_clamp", "torque", s.actuator_torque_nm, "PWM cap", 1.0)
                previous = "pwm_clamp"
            add("plant", "plant")
            connect(previous, "plant", "torque", s.actuator_torque_nm, "physical torque", 1.0)
        elif c.mode == OperatingMode.VELOCITY:
            add("velocity_target", "velocity")
            connect(previous, "velocity_target", "velocity", s.reference_velocity_deg_s, "vref", 360.0)
            add("velocity_error", "feedback")
            connect("velocity_target", "velocity_error", "velocity", s.reference_velocity_deg_s, "vref − v", 360.0)
            add("velocity_pi", "torque")
            connect("velocity_error", "velocity_pi", "torque", s.velocity_deg_s, "PI", 360.0)
            add("pwm_clamp", "torque")
            connect("velocity_pi", "pwm_clamp", "torque", s.actuator_torque_nm, "PWM cap", 1.0)
            add("plant", "plant")
            connect("pwm_clamp", "plant", "torque", s.actuator_torque_nm, "physical torque", 1.0)
        elif c.mode == OperatingMode.CURRENT:
            add("current", "current")
            connect(previous, "current", "current", s.current_ma, "signed current", 910.0)
            add("current_convert", "torque")
            connect("current", "current_convert", "current", s.current_ma, "I → τ", 910.0)
            add("plant", "plant")
            connect("current_convert", "plant", "torque", s.actuator_torque_nm, "physical torque", 1.0)
        else:
            add("pwm", "current")
            connect(previous, "pwm", "current", s.pwm_percent, "signed PWM", 100.0)
            add("plant", "plant")
            connect("pwm", "plant", "torque", s.actuator_torque_nm, "physical torque", 1.0)

        add("external", "torque")
        connect("external", "plant", "torque", s.external_torque_nm, "external torque", 1.0)
        previous_plant = "plant"
        if c.endstop_enabled:
            add("endstop", "endstop")
            connect(previous_plant, "endstop", "endstop", s.endstop_torque_nm, "counter-torque", 1.0)
            previous_plant = "endstop"
        add("state", "feedback")
        connect(previous_plant, "state", "feedback", s.velocity_deg_s, "q, v, I", 360.0)

        # Feedback is intentionally drawn back to the error block for closed
        # loop modes; direct current/PWM paths remain visibly open-loop.
        if c.mode in {OperatingMode.POSITION, OperatingMode.EXTENDED_POSITION, OperatingMode.CURRENT_POSITION}:
            connect("state", "error", "feedback", s.position_error_deg, "feedback", 180.0)
        elif c.mode == OperatingMode.VELOCITY:
            connect("state", "velocity_error", "feedback", s.velocity_deg_s, "feedback", 360.0)
        return nodes, edges

    @staticmethod
    def _draw_arrow(painter: QPainter, start: QPointF, end: QPointF, color: QColor, width: float, label: str, phase: float, value: float) -> None:
        vector = end - start
        length = math.hypot(vector.x(), vector.y())
        if length < 1e-6:
            return
        unit = QPointF(vector.x() / length, vector.y() / length)
        pen = QPen(color, width, Qt.DashLine, Qt.RoundCap)
        pen.setDashPattern([8.0, 6.0])
        pen.setDashOffset(-phase * 28.0)
        painter.setPen(pen)
        painter.drawLine(start, end)

        tip = end
        side = QPointF(-unit.y(), unit.x())
        wing = 6.0 + width
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(color))
        painter.drawPolygon(QPolygonF([tip, tip - unit * (10.0 + width) + side * wing, tip - unit * (10.0 + width) - side * wing]))

        # Moving dots make the direction apparent even when the line is nearly
        # horizontal or the amplitude is small.
        for index in range(3):
            fraction = (phase + index / 3.0) % 1.0
            point = start + vector * fraction
            painter.drawEllipse(point, 2.0 + width * 0.35, 2.0 + width * 0.35)
        painter.setPen(QPen(color, 1))
        font = QFont("Arial", 8)
        painter.setFont(font)
        label_text = f"{label}  {value:+.2f}"
        midpoint = start + vector * 0.5
        metrics = QFontMetrics(font)
        label_width = float(metrics.horizontalAdvance(label_text))
        ascent = float(metrics.ascent())
        descent = float(metrics.descent())
        angle = math.radians(-30.0)
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        corners = (
            QPointF(0.0, -ascent),
            QPointF(label_width, -ascent),
            QPointF(0.0, descent),
            QPointF(label_width, descent),
        )
        rotated = tuple(
            QPointF(
                corner.x() * cos_angle - corner.y() * sin_angle,
                corner.x() * sin_angle + corner.y() * cos_angle,
            )
            for corner in corners
        )
        min_x = min(point.x() for point in rotated)
        max_x = max(point.x() for point in rotated)
        min_y = min(point.y() for point in rotated)
        max_y = max(point.y() for point in rotated)
        # Keep the rotated text visually centered around the connector
        # midpoint while retaining left alignment in its local frame.
        bbox_center = QPointF(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
        )
        normal_extent = max(
            abs(
                (point.x() - bbox_center.x()) * side.x()
                + (point.y() - bbox_center.y()) * side.y()
            )
            for point in rotated
        )
        label_center = midpoint - side * (normal_extent + 7.0) + unit * 8.0
        origin = QPointF(
            label_center.x() - (min_x + max_x) * 0.5,
            label_center.y() - (min_y + max_y) * 0.5,
        )
        painter.save()
        painter.translate(origin)
        painter.rotate(-30.0)
        painter.drawText(QPointF(0.0, 0.0), label_text)
        painter.restore()

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt API
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#0f172a"))
        nodes, edges = self._path()
        margin = 72.0
        y = self.height() * 0.48
        spacing = (self.width() - 2.0 * margin) / max(1, len(nodes) - 1)
        node_w = min(160.0, max(105.0, spacing * 0.86))
        node_h = 58.0
        positions: dict[str, QPointF] = {}
        for index, (key, _label, _color) in enumerate(nodes):
            positions[key] = QPointF(margin + index * spacing, y)

        # Branch signals sit below the main path to keep the direction clear.
        if "external" in positions:
            positions["external"] = QPointF(margin + spacing * 0.75, self.height() * 0.78)
        if "state" in positions:
            positions["state"] = QPointF(self.width() - margin, self.height() * 0.78)
        if "endstop" in positions:
            positions["endstop"] = QPointF(self.width() * 0.58, self.height() * 0.78)

        phase = (self.snapshot.time_s * 1.6) % 1.0
        for a, b, color, value, label, scale in edges:
            a_center = positions[a]
            b_center = positions[b]
            start = a_center + QPointF(node_w * 0.50, 0)
            end = b_center - QPointF(node_w * 0.50, 0)
            if abs(end.x() - start.x()) < 20.0:
                start = a_center + QPointF(0, node_h * 0.50)
                end = b_center - QPointF(0, node_h * 0.50)
            width = 1.5 + 7.0 * min(1.0, abs(value) / max(scale, 1e-6))
            self._draw_arrow(painter, start, end, color, width, label, phase, value)

        for key, label, color in nodes:
            center = positions[key]
            rect = QRectF(center.x() - node_w / 2, center.y() - node_h / 2, node_w, node_h)
            painter.setPen(QPen(color, 2))
            painter.setBrush(QBrush(QColor("#1e293b")))
            painter.drawRoundedRect(rect, 9, 9)
            painter.setPen(QPen(QColor("#f8fafc"), 1))
            painter.setFont(QFont("Arial", 8))
            painter.drawText(rect.adjusted(6, 5, -6, -5), Qt.AlignCenter, label)

        painter.setFont(QFont("Arial", 8))
        painter.setPen(QPen(QColor("#cbd5e1"), 1))
        painter.drawText(12, 18, f"Dynamic path: {self.config.mode.library_constant} | objective: {self.config.objective.value}")
        legend = [("position", "position"), ("velocity", "velocity"), ("current", "current"), ("torque", "torque"), ("endstop", "endstop")]
        x = 12.0
        for key, name in legend:
            color = self.COLORS[key]
            painter.setPen(QPen(color, 3))
            painter.drawLine(QPointF(x, self.height() - 15), QPointF(x + 18, self.height() - 15))
            painter.setPen(QPen(QColor("#cbd5e1"), 1))
            painter.drawText(QPointF(x + 23.0, float(self.height() - 11)), name)
            x += 82.0


class NestedFlowWidget(QWidget):
    """Compact overview with clickable controller sub-views."""

    COLORS = DynamicFlowWidget.COLORS
    VIEW_NAMES = {
        "overview": "Overview",
        "command": "Command / intent",
        "controller": "Controller",
        "actuator": "Output stage",
        "plant": "Motor + environment",
        "feedback": "Feedback",
    }

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.config = SimulationConfig()
        self.snapshot = SingleMotorSimulation(self.config).previous_snapshot
        self.view_stack = ["overview"]
        self._hit_regions: list[tuple[QRectF, str]] = []
        self._hover_action: str | None = None
        self.setMinimumHeight(300)
        self.setMouseTracking(True)
        self.setToolTip("Click a highlighted group to inspect its internal signal flow")

    def update_state(self, config: SimulationConfig, snapshot: SimulationSnapshot) -> None:
        old_key = (self.config.mode, self.config.objective)
        new_key = (config.mode, config.objective)
        if old_key != new_key:
            self.view_stack = ["overview"]
        self.config = config
        self.snapshot = snapshot
        self.update()

    def _node_label(self, key: str) -> str:
        c = self.config
        s = self.snapshot
        if c.mode == OperatingMode.CURRENT_POSITION:
            actuator_label = f"applied current\nI={s.current_ma:+.0f} mA\n|Goal|={s.goal_current_ma:.0f} mA"
        elif c.mode == OperatingMode.CURRENT:
            actuator_label = f"applied current\nI={s.current_ma:+.0f} mA\ntau={s.actuator_torque_nm:+.2f} N-m"
        elif c.mode == OperatingMode.PWM:
            actuator_label = f"PWM output\nPWM={s.pwm_percent:+.0f}%\ntau={s.actuator_torque_nm:+.2f} N-m"
        else:
            actuator_label = f"position output\ntau={s.actuator_torque_nm:+.2f} N-m\nPWM={s.pwm_percent:+.0f}%"
        labels = {
            "command": f"command / intent\n{c.objective.value}",
            "controller": "position loop\nqref/error/PID" if c.mode in {
                OperatingMode.POSITION,
                OperatingMode.EXTENDED_POSITION,
                OperatingMode.CURRENT_POSITION,
            } else (
                "velocity loop\nvref/error/PI" if c.mode == OperatingMode.VELOCITY
                else "direct command\nno position loop"
            ),
            "actuator": actuator_label,
            "plant": f"motor + load\nq={s.position_deg:.1f}°",
            "feedback": (
                f"observer\nr={s.debiased_external_torque_nm:+.2f} N-m"
                if c.objective == Objective.EXTERNAL_EFFORT_ASSIST
                else f"feedback state\nq={s.position_deg:.1f} deg\nv={s.velocity_deg_s:+.1f} deg/s"
            ),
        }
        return labels.get(key, key)

    def _graph(self, view: str) -> tuple[list[dict], list[tuple[str, str, QColor, float, str, float]]]:
        c = self.config
        s = self.snapshot
        nodes: list[dict] = []
        edges: list[tuple[str, str, QColor, float, str, float]] = []

        def add(key: str, label: str, color: str, drill: str | None = None) -> None:
            nodes.append({"key": key, "label": label, "color": self.COLORS[color], "drill": drill})

        def connect(a: str, b: str, color: str, value: float, label: str, scale: float) -> None:
            edges.append((a, b, self.COLORS[color], value, label, scale))

        position_mode = c.mode in {
            OperatingMode.POSITION,
            OperatingMode.EXTENDED_POSITION,
            OperatingMode.CURRENT_POSITION,
        }

        if view == "overview":
            add("command", self._node_label("command"), "position", "command")
            add("controller", self._node_label("controller"), "feedback", "controller")
            actuator_color = "current" if c.mode in {
                OperatingMode.CURRENT,
                OperatingMode.CURRENT_POSITION,
            } else ("torque" if c.mode == OperatingMode.PWM else "feedback")
            add("actuator", self._node_label("actuator"), actuator_color, "actuator")
            add("plant", self._node_label("plant"), "plant", "plant")
            add("feedback", self._node_label("feedback"), "feedback", "feedback")
            if position_mode:
                reference_value = s.reference_position_deg
                reference_label = "reference [deg]"
                reference_scale = 360.0
            elif c.mode == OperatingMode.VELOCITY:
                reference_value = s.reference_velocity_deg_s
                reference_label = "reference [deg/s]"
                reference_scale = 360.0
            elif c.mode == OperatingMode.CURRENT:
                reference_value = s.current_ma
                reference_label = "Icmd [mA]"
                reference_scale = 910.0
            else:
                reference_value = s.pwm_percent
                reference_label = "PWMcmd [%]"
                reference_scale = 100.0
            connect("command", "controller", "position", reference_value, reference_label, reference_scale)
            if c.mode in {OperatingMode.CURRENT, OperatingMode.CURRENT_POSITION}:
                control_value = s.current_ma
                control_label = "applied current [mA]"
                control_scale = 910.0
                control_color = "current"
            elif c.mode == OperatingMode.PWM:
                control_value = s.pwm_percent
                control_label = "PWM [%]"
                control_scale = 100.0
                control_color = "torque"
            else:
                control_value = s.actuator_torque_nm
                control_label = "effort [N-m]"
                control_scale = 1.0
                control_color = "torque"
            connect("controller", "actuator", control_color, control_value, control_label, control_scale)
            connect("actuator", "plant", "torque", s.actuator_torque_nm, "physical torque [N-m]", 1.0)
            connect("plant", "feedback", "feedback", s.velocity_deg_s, "q/v [deg, deg/s]", 360.0)
            return nodes, edges

        if view == "command" and c.objective == Objective.SECONDARY_GOAL_ASSIST:
            add("input", f"scenario / intent\n{c.objective.value}", "position")
            add(
                "assist",
                f"intent velocity\n"
                f"vgoal={s.secondary_goal_velocity_deg_s:+.1f} deg/s\n"
                f"{'ON' if s.assist_active else 'waiting'}",
                "assist",
            )
            connect("input", "assist", "assist", s.secondary_goal_velocity_deg_s, "r -> vgoal [deg/s]", 360.0)
            add(
                "target",
                f"secondary goal + lead\n"
                f"qgoal={s.secondary_goal_position_deg:.1f} deg\n"
                f"lead={s.secondary_goal_lead_deg:+.1f} deg",
                "position",
            )
            connect("assist", "target", "position", s.reference_position_deg, "qref [deg]", 360.0)
            return nodes, edges

        if view == "command":
            add("input", f"scenario\nq={s.position_deg:.1f}°", "position")
            previous = "input"
            if c.objective in {
                Objective.SECONDARY_GOAL_ASSIST,
                Objective.TORQUE_ASSIST,
                Objective.EXTERNAL_EFFORT_ASSIST,
            }:
                add(
                    "assist",
                    f"assist request\nτ={s.assist_torque_command_nm:+.2f} N-m\n"
                    f"ceiling={c.assist_torque_nm:+.2f} N-m",
                    "assist",
                )
                connect(previous, "assist", "assist", s.assist_torque_command_nm, "τassist request [N-m]", 1.0)
                previous = "assist"
            if position_mode:
                add("target", f"profile / target\nqref={s.reference_position_deg:.1f}°", "position")
                connect(previous, "target", "position", s.reference_position_deg, "qref [deg]", 360.0)
            elif c.mode == OperatingMode.VELOCITY:
                add("target", f"velocity target\nvref={s.reference_velocity_deg_s:+.1f}°/s", "velocity")
                connect(previous, "target", "velocity", s.reference_velocity_deg_s, "vref [deg/s]", 360.0)
            elif c.mode == OperatingMode.CURRENT:
                add("target", f"signed current\nI={s.current_ma:+.0f} mA", "current")
                connect(previous, "target", "current", s.current_ma, "Icmd [mA]", 910.0)
            else:
                add("target", f"signed PWM\n{s.pwm_percent:+.0f}%", "current")
                connect(previous, "target", "current", s.pwm_percent, "PWMcmd [%]", 100.0)
            return nodes, edges

        if view == "controller":
            if position_mode:
                add("target", f"reference\nqref={s.reference_position_deg:.1f}°", "position")
                add("state", f"measured state\nq={s.position_deg:.1f}°", "feedback")
                add("error", f"position error\ne={s.position_error_deg:+.1f}°", "feedback")
                add("pid", f"position PID\ntau={s.pid_torque_nm:+.2f} N-m", "torque")
                connect("target", "error", "position", s.position_error_deg, "qref - q [deg]", 180.0)
                connect("state", "error", "feedback", s.position_error_deg, "q [deg]", 180.0)
                connect("error", "pid", "torque", s.pid_torque_nm, "PID effort [N-m]", 1.0)
            elif c.mode == OperatingMode.VELOCITY:
                add("target", f"velocity target\nvref={s.reference_velocity_deg_s:+.1f}°/s", "velocity")
                add("state", f"measured state\nv={s.velocity_deg_s:+.1f}°/s", "feedback")
                add("error", f"velocity error\ne={s.reference_velocity_deg_s - s.velocity_deg_s:+.1f}", "feedback")
                add("pi", f"velocity PI\ntau={s.pid_torque_nm:+.2f} N-m", "torque")
                connect("target", "error", "velocity", s.reference_velocity_deg_s, "vref - v [deg/s]", 360.0)
                connect("state", "error", "feedback", s.velocity_deg_s, "v [deg/s]", 360.0)
                connect("error", "pi", "torque", s.pid_torque_nm, "PI effort [N-m]", 1.0)
            else:
                label = "signed current" if c.mode == OperatingMode.CURRENT else "signed PWM"
                value = f"{s.current_ma:+.0f} mA" if c.mode == OperatingMode.CURRENT else f"{s.pwm_percent:+.0f}%"
                add("direct", f"{label}\n{value}", "current")
                add("open_loop", "direct path\nno position feedback", "torque")
                connect(
                    "direct",
                    "open_loop",
                    "current" if c.mode == OperatingMode.CURRENT else "torque",
                    s.current_ma if c.mode == OperatingMode.CURRENT else s.pwm_percent,
                    "I [mA]" if c.mode == OperatingMode.CURRENT else "PWM [%]",
                    910.0 if c.mode == OperatingMode.CURRENT else 100.0,
                )
            return nodes, edges

        if view == "actuator":
            if position_mode:
                add("pid", f"desired effort\ntau={s.pid_torque_nm:+.2f} N-m", "torque")
                if c.mode == OperatingMode.CURRENT_POSITION:
                    add("limit", f"Goal Current limit\n|I|={s.goal_current_ma:.0f} / {c.goal_current_limit_ma:.0f} mA", "current")
                    connect("pid", "limit", "current", s.goal_current_ma, "current limit |I| [mA]", 910.0)
                else:
                    add("limit", f"PWM limit\n{s.pwm_percent:+.0f}%", "torque")
                    connect("pid", "limit", "torque", s.actuator_torque_nm, "PWM effort [%]", 1.0)
                add("applied", f"applied torque\ntau={s.actuator_torque_nm:+.2f} N-m", "torque")
                connect("limit", "applied", "torque", s.actuator_torque_nm, "physical torque [N-m]", 1.0)
            elif c.mode == OperatingMode.CURRENT:
                add("current", f"signed current\nI={s.current_ma:+.0f} mA", "current")
                add("applied", f"current → torque\nτ={s.actuator_torque_nm:+.2f}", "torque")
                connect("current", "applied", "current", s.current_ma, "I -> torque [mA]", 910.0)
            else:
                add("pwm", f"signed PWM\n{s.pwm_percent:+.0f}%", "current")
                add("applied", f"PWM → torque\nτ={s.actuator_torque_nm:+.2f}", "torque")
                connect("pwm", "applied", "torque", s.pwm_percent, "PWM -> torque [%]", 100.0)
            return nodes, edges

        if view == "plant":
            add("external", f"external effort\nτ={s.external_torque_nm:+.2f}", "torque")
            add("plant", f"motor + load\nq={s.position_deg:.1f}°", "plant")
            connect("external", "plant", "torque", s.external_torque_nm, "external torque [N-m]", 1.0)
            if c.endstop_enabled:
                add("endstop", f"endstop reaction\nτ={s.endstop_torque_nm:+.2f}", "endstop")
                connect("plant", "endstop", "endstop", s.endstop_torque_nm, "counter-torque [N-m]", 1.0)
                previous = "endstop"
            else:
                previous = "plant"
            add("state", f"feedback state\nq={s.position_deg:.1f} deg\nv={s.velocity_deg_s:+.1f} deg/s", "feedback")
            connect(previous, "state", "feedback", s.velocity_deg_s, "q/v [deg, deg/s]", 360.0)
            return nodes, edges

        add("state", f"feedback state\nq={s.position_deg:.1f} deg", "feedback")
        if c.objective == Objective.EXTERNAL_EFFORT_ASSIST:
            signals = (
                f"observer\nr={s.debiased_external_torque_nm:+.2f} N-m\n"
                f"{'ASSIST ON' if s.assist_active else 'waiting'}"
            )
        else:
            signals = f"telemetry\nv={s.velocity_deg_s:+.1f}°/s\nI={s.current_ma:+.0f} mA"
        add("signals", signals, "feedback")
        connect("state", "signals", "feedback", s.velocity_deg_s, "q/v [deg, deg/s]", 360.0)
        return nodes, edges

    def _draw_arrow(self, painter: QPainter, start: QPointF, end: QPointF, color: QColor, width: float, label: str, phase: float, value: float) -> None:
        vector = end - start
        length = math.hypot(vector.x(), vector.y())
        if length < 1e-6:
            return
        unit = QPointF(vector.x() / length, vector.y() / length)
        pen = QPen(color, width, Qt.DashLine, Qt.RoundCap)
        pen.setDashPattern([8.0, 6.0])
        pen.setDashOffset(-phase * 28.0)
        painter.setPen(pen)
        painter.drawLine(start, end)
        side = QPointF(-unit.y(), unit.x())
        tip = end
        wing = 6.0 + width
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(color))
        painter.drawPolygon(QPolygonF([tip, tip - unit * (10.0 + width) + side * wing, tip - unit * (10.0 + width) - side * wing]))
        for index in range(3):
            fraction = (phase + index / 3.0) % 1.0
            point = start + vector * fraction
            painter.drawEllipse(point, 2.0 + width * 0.35, 2.0 + width * 0.35)
        painter.setPen(QPen(color, 1))
        font = QFont("Arial", 8)
        painter.setFont(font)
        label_text = f"{label}  {value:+.2f}"
        midpoint = start + vector * 0.5
        metrics = QFontMetrics(font)
        label_width = float(metrics.horizontalAdvance(label_text))
        ascent = float(metrics.ascent())
        descent = float(metrics.descent())
        angle = math.radians(-30.0)
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        corners = (
            QPointF(0.0, -ascent),
            QPointF(label_width, -ascent),
            QPointF(0.0, descent),
            QPointF(label_width, descent),
        )
        rotated = tuple(
            QPointF(
                corner.x() * cos_angle - corner.y() * sin_angle,
                corner.x() * sin_angle + corner.y() * cos_angle,
            )
            for corner in corners
        )
        min_x = min(point.x() for point in rotated)
        max_x = max(point.x() for point in rotated)
        min_y = min(point.y() for point in rotated)
        max_y = max(point.y() for point in rotated)
        bbox_center = QPointF(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
        )
        normal_extent = max(
            abs(
                (point.x() - bbox_center.x()) * side.x()
                + (point.y() - bbox_center.y()) * side.y()
            )
            for point in rotated
        )
        label_center = midpoint - side * (normal_extent + 7.0) + unit * 8.0
        origin = QPointF(
            label_center.x() - (min_x + max_x) * 0.5,
            label_center.y() - (min_y + max_y) * 0.5,
        )
        painter.save()
        painter.translate(origin)
        painter.rotate(-30.0)
        painter.drawText(QPointF(0.0, 0.0), label_text)
        painter.restore()

    def _breadcrumb(self, painter: QPainter) -> None:
        self._hit_regions = []
        x = 14.0
        for index, view in enumerate(self.view_stack):
            text = self.VIEW_NAMES.get(view, view)
            width = max(72.0, len(text) * 7.0 + 18.0)
            rect = QRectF(x, 7.0, width, 22.0)
            active = index == len(self.view_stack) - 1
            hovered = self._hover_action == f"crumb:{index}"
            painter.setPen(QPen(QColor("#ffffff") if hovered else (QColor("#e2e8f0") if active else QColor("#7dd3fc")), 2 if hovered else 1))
            painter.setBrush(QBrush(QColor("#334155") if hovered else (QColor("#1e293b") if active else QColor("#172554"))))
            painter.drawRoundedRect(rect, 6, 6)
            painter.setPen(QPen(QColor("#f8fafc"), 1))
            painter.drawText(rect, Qt.AlignCenter, text)
            if not active:
                self._hit_regions.append((rect, f"crumb:{index}"))
            x += width + 8.0
            if not active:
                painter.setPen(QPen(QColor("#64748b"), 1))
                painter.drawText(QPointF(x - 7.0, 22.0), "/")

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt API
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#0f172a"))
        self._breadcrumb(painter)
        view = self.view_stack[-1]
        nodes, edges = self._graph(view)
        painter.setPen(QPen(QColor("#cbd5e1"), 1))
        painter.setFont(QFont("Arial", 9))
        painter.drawText(QPointF(14.0, 47.0), f"{self.VIEW_NAMES.get(view, view)} view — hover highlighted blocks, click to inspect")
        # Redraw the title with ASCII punctuation after older source encoding
        # artifacts have been covered.
        painter.fillRect(QRectF(0.0, 34.0, float(self.width()), 20.0), QColor("#0f172a"))
        painter.setPen(QPen(QColor("#cbd5e1"), 1))
        painter.setFont(QFont("Arial", 9))
        painter.drawText(QPointF(14.0, 47.0), f"{self.VIEW_NAMES.get(view, view)} view - hover highlighted blocks, click to inspect")
        margin = 12.0
        y = self.height() * 0.54
        node_count = max(1, len(nodes))
        node_w = 120.0
        spacing = 0.0
        for _ in range(3):
            spacing = (self.width() - 2.0 * margin) / max(1, node_count - 1)
            node_w = min(176.0, max(84.0, spacing * 0.54))
            margin = max(10.0, node_w * 0.50 + 8.0)
        node_h = 62.0
        positions: dict[str, QPointF] = {}
        for index, node in enumerate(nodes):
            positions[node["key"]] = QPointF(margin + index * spacing, y)

        phase = (self.snapshot.time_s * 1.6) % 1.0
        for a, b, color, value, label, scale in edges:
            start = positions[a] + QPointF(node_w * 0.50, 0)
            end = positions[b] - QPointF(node_w * 0.50, 0)
            width = 1.5 + 7.0 * min(1.0, abs(value) / max(scale, 1e-6))
            self._draw_arrow(painter, start, end, color, width, label, phase, value)

        for node in nodes:
            center = positions[node["key"]]
            rect = QRectF(center.x() - node_w / 2, center.y() - node_h / 2, node_w, node_h)
            action = node["drill"]
            if action:
                self._hit_regions.append((rect, f"view:{action}"))
            hovered = self._hover_action == f"view:{action}" if action else False
            fill = node["color"].lighter(145) if hovered else QColor("#1e293b")
            border = QColor("#ffffff") if hovered else node["color"]
            painter.setPen(QPen(border, 3 if hovered else 2))
            painter.setBrush(QBrush(fill))
            painter.drawRoundedRect(rect, 10, 10)
            painter.setPen(QPen(QColor("#f8fafc"), 1))
            painter.setFont(QFont("Arial", 8 if hovered else 7))
            painter.drawText(rect.adjusted(7, 6, -7, -6), Qt.AlignCenter, node["label"])

        painter.setFont(QFont("Arial", 8))
        painter.setPen(QPen(QColor("#cbd5e1"), 1))
        legend = [("position", "position"), ("velocity", "velocity"), ("current", "current"), ("torque", "torque"), ("endstop", "endstop")]
        x = 12.0
        for key, name in legend:
            color = self.COLORS[key]
            painter.setPen(QPen(color, 3))
            painter.drawLine(QPointF(x, self.height() - 15), QPointF(x + 18, self.height() - 15))
            painter.setPen(QPen(QColor("#cbd5e1"), 1))
            painter.drawText(QPointF(x + 23.0, float(self.height() - 11)), name)
            x += 82.0

    def _action_at(self, point: QPointF) -> str | None:
        for rect, action in reversed(self._hit_regions):
            if rect.contains(point):
                return action
        return None

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 - Qt API
        action = self._action_at(event.pos())
        if action != self._hover_action:
            self._hover_action = action
            self.setCursor(Qt.PointingHandCursor if action else Qt.ArrowCursor)
            self.update()
        super().mouseMoveEvent(event)

    def leaveEvent(self, event) -> None:  # noqa: N802 - Qt API
        self._hover_action = None
        self.setCursor(Qt.ArrowCursor)
        self.update()
        super().leaveEvent(event)

    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt API
        if event.button() == Qt.LeftButton:
            action = self._action_at(event.pos())
            if action and action.startswith("crumb:"):
                index = int(action.split(":", 1)[1])
                self.view_stack = self.view_stack[: index + 1]
                self._hover_action = None
                self.update()
                return
            if action and action.startswith("view:"):
                self.view_stack.append(action.split(":", 1)[1])
                self._hover_action = None
                self.update()
                return
        super().mousePressEvent(event)


class ServoView(QWidget):
    """Paint and directly manipulate the actuator and scenario handles."""

    interaction_changed = pyqtSignal(str, float)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.config = SimulationConfig()
        self.snapshot = SingleMotorSimulation(self.config).previous_snapshot
        self.setMinimumSize(520, 380)
        self.setMouseTracking(True)
        self._interactive_regions: list[tuple[QRectF, str]] = []
        self._drag_key: str | None = None
        self._hover_action: str | None = None
        self._editor: QDoubleSpinBox | None = None
        self._editor_key: str | None = None

    _EDIT_RANGES = {
        "start_position_deg": (0.0, 360.0, 1),
        "end_position_deg": (0.0, 360.0, 1),
        "endstop_position_deg": (0.0, 360.0, 1),
        "external_torque_nm": (-2.0, 2.0, 3),
        "assist_torque_nm": (-2.0, 2.0, 3),
    }

    def update_state(self, config: SimulationConfig, snapshot: SimulationSnapshot) -> None:
        self.config = config
        self.snapshot = snapshot
        self.update()

    @staticmethod
    def _point(center: QPointF, radius: float, angle_deg: float) -> QPointF:
        angle = math.radians(angle_deg)
        return QPointF(center.x() + radius * math.cos(angle), center.y() - radius * math.sin(angle))

    @staticmethod
    def _arrow(painter: QPainter, origin: QPointF, direction: QPointF, color: QColor, label: str) -> None:
        length = math.hypot(direction.x(), direction.y())
        if length < 1e-6:
            return
        unit = QPointF(direction.x() / length, direction.y() / length)
        tip = origin + direction
        side = QPointF(-unit.y(), unit.x())
        wing = 9.0
        painter.setPen(QPen(color, 3))
        painter.drawLine(origin, tip)
        painter.setBrush(QBrush(color))
        painter.drawPolygon(QPolygonF([tip, tip - unit * 13 + side * wing, tip - unit * 13 - side * wing]))
        painter.setPen(QPen(color, 1))
        painter.drawText(tip + QPointF(8, -6), label)

    def _value_for_key(self, key: str) -> float:
        return {
            "start_position_deg": self.config.start_position_deg,
            "end_position_deg": self.config.end_position_deg,
            "endstop_position_deg": self.config.endstop_position_deg,
            "external_torque_nm": self.config.external_torque_nm,
            "assist_torque_nm": self.config.assist_torque_nm,
        }[key]

    @staticmethod
    def _angle_at(center: QPointF, point: QPointF) -> float:
        return math.degrees(math.atan2(center.y() - point.y(), point.x() - center.x())) % 360.0

    def _position_handle(self, center: QPointF, radius: float, key: str) -> QPointF:
        return self._point(center, radius + 18.0, self._value_for_key(key))

    def _emit_edit(self, key: str, value: float) -> None:
        lower, upper, _decimals = self._EDIT_RANGES[key]
        bounded = max(lower, min(upper, value))
        self.interaction_changed.emit(key, bounded)

    def _open_editor(self, key: str, rect: QRectF) -> None:
        self._close_editor(commit=True)
        lower, upper, decimals = self._EDIT_RANGES[key]
        editor = QDoubleSpinBox(self)
        editor.setRange(lower, upper)
        editor.setDecimals(decimals)
        editor.setSingleStep(0.05 if "torque" in key else 1.0)
        editor.setValue(self._value_for_key(key))
        editor.setToolTip("Press Enter or click away to apply")
        editor.editingFinished.connect(self._commit_editor)
        editor.setGeometry(rect.toRect().adjusted(-4, -3, 4, 3))
        self._editor = editor
        self._editor_key = key
        editor.show()
        editor.setFocus()
        editor.selectAll()

    def _commit_editor(self) -> None:
        if self._editor is None or self._editor_key is None:
            return
        editor = self._editor
        key = self._editor_key
        self._editor = None
        self._editor_key = None
        self._emit_edit(key, editor.value())
        editor.deleteLater()

    def _close_editor(self, *, commit: bool) -> None:
        if self._editor is None:
            return
        if commit:
            self._commit_editor()
        else:
            editor = self._editor
            self._editor = None
            self._editor_key = None
            editor.deleteLater()

    def _action_at(self, point: QPointF) -> str | None:
        for rect, action in reversed(self._interactive_regions):
            if rect.contains(point):
                return action
        return None

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt API
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#0f172a"))

        width = self.width()
        height = self.height()
        footer_height = 70.0
        footer_top = max(0.0, float(height) - footer_height)
        diagram_top = 42.0
        diagram_bottom = max(diagram_top + 120.0, footer_top - 10.0)
        diagram_height = diagram_bottom - diagram_top
        center = QPointF(width * 0.50, diagram_top + diagram_height * 0.50)
        # Reserve space for the footer and the draggable markers.  The old
        # fixed vertical placement allowed the circle's lower arc to be
        # painted underneath the footer background on short panels.
        radius = min(width * 0.29, max(40.0, diagram_height * 0.50 - 34.0))
        c = self.config
        s = self.snapshot
        self._interactive_regions = []

        p.setPen(QPen(QColor("#334155"), 1))
        p.drawLine(QPointF(40, center.y()), QPointF(width - 40, center.y()))
        p.drawLine(QPointF(center.x(), diagram_top), QPointF(center.x(), diagram_bottom))

        # Physical range arc and soft endstop marker.
        p.setPen(QPen(QColor("#64748b"), 2))
        p.drawArc(QRectF(center.x() - radius, center.y() - radius, 2 * radius, 2 * radius), 0, 360 * 16)
        if c.endstop_enabled:
            stop = self._point(center, radius + 22, c.endstop_position_deg)
            p.setPen(QPen(QColor("#f87171"), 5))
            p.drawLine(self._point(center, radius - 8, c.endstop_position_deg), stop)
            p.drawText(stop + QPointF(8, 3), "soft endstop")
            self._interactive_regions.append((QRectF(stop.x() - 11, stop.y() - 11, 22, 22), "drag:endstop_position_deg"))

        # These handles edit the scenario geometry directly.  The handle
        # values are still emitted through the normal control widgets, so a
        # drag remains synchronized with the rest of the simulator.
        for key, color, label in (
            ("start_position_deg", QColor("#4ade80"), "start"),
            ("end_position_deg", QColor("#facc15"), "end"),
        ):
            handle = self._position_handle(center, radius, key)
            action = f"drag:{key}"
            hovered = self._hover_action == action
            self._interactive_regions.append((QRectF(handle.x() - 11, handle.y() - 11, 22, 22), action))
            p.setPen(QPen(QColor("#ffffff") if hovered else color, 2))
            p.setBrush(QBrush(color.lighter(145) if hovered else color))
            p.drawEllipse(handle, 8 if hovered else 6, 8 if hovered else 6)
            p.setPen(QPen(color, 1))
            p.drawText(handle + QPointF(8, 4), label)

        # Servo housing.
        body = QRectF(center.x() - 52, center.y() - 35, 104, 70)
        p.setPen(QPen(QColor("#94a3b8"), 2))
        p.setBrush(QBrush(QColor("#1e293b")))
        p.drawRoundedRect(body, 10, 10)
        p.setPen(QPen(QColor("#cbd5e1"), 1))
        p.drawText(body, Qt.AlignCenter, "XC330\nservo")

        # Target and actual arm.
        target = self._point(center, radius, s.reference_position_deg)
        actual = self._point(center, radius, s.position_deg)
        p.setPen(QPen(QColor("#facc15"), 2, Qt.DashLine))
        p.drawLine(center, target)
        p.setPen(QPen(QColor("#38bdf8"), 7, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(center, actual)
        p.setBrush(QBrush(QColor("#38bdf8")))
        p.drawEllipse(actual, 8, 8)
        p.setBrush(QBrush(QColor("#facc15")))
        p.drawEllipse(target, 6, 6)

        # Torque arrows are tangential to the arm.
        mid = self._point(center, radius * 0.65, s.position_deg)
        tangent = QPointF(-math.sin(math.radians(s.position_deg)), -math.cos(math.radians(s.position_deg)))
        def torque_vector(torque_nm: float) -> QPointF:
            normalized = max(-1.0, min(1.0, torque_nm))
            return tangent * (34.0 * normalized)

        self._arrow(p, mid, torque_vector(s.external_torque_nm), QColor("#fb923c"), "external")
        self._arrow(p, mid + QPointF(0, 26), torque_vector(s.actuator_torque_nm), QColor("#4ade80"), "actuator")
        if abs(s.endstop_torque_nm) > 1e-4:
            self._arrow(p, mid + QPointF(0, -28), torque_vector(s.endstop_torque_nm), QColor("#f87171"), "endstop")

        p.setPen(QPen(QColor("#e2e8f0"), 1))
        p.drawText(18, 24, f"q = {s.position_deg:7.2f}°    qref = {s.reference_position_deg:7.2f}°    v = {s.velocity_deg_s:7.2f}°/s")
        p.drawText(18, height - 14, f"actuator = {s.actuator_torque_nm:+.3f} N·m   external = {s.external_torque_nm:+.3f} N·m   endstop = {s.endstop_torque_nm:+.3f} N·m")
        # Replace the compact footer with separate, clickable value labels.
        p.fillRect(QRectF(0.0, footer_top, float(width), footer_height), QColor("#0f172a"))
        p.setPen(QPen(QColor("#e2e8f0"), 1))
        p.drawText(QPointF(18.0, 24.0), f"q = {s.position_deg:7.2f}°    qref = {s.reference_position_deg:7.2f}°    v = {s.velocity_deg_s:7.2f}°/s")
        external_rect = QRectF(18.0, height - 62.0, 225.0, 22.0)
        assist_rect = QRectF(250.0, height - 62.0, 225.0, 22.0)
        self._interactive_regions.append((external_rect, "edit:external_torque_nm"))
        if c.objective != Objective.EXTERNAL_EFFORT_ASSIST:
            self._interactive_regions.append((assist_rect, "edit:assist_torque_nm"))
        assist_value = s.assist_torque_command_nm
        p.setPen(QPen(QColor("#fb923c"), 1))
        p.drawText(external_rect, Qt.AlignLeft | Qt.AlignVCenter, f"external torque = {s.external_torque_nm:+.3f} N·m")
        p.setPen(QPen(QColor("#4ade80"), 1))
        assist_label = "detected assist" if c.objective == Objective.EXTERNAL_EFFORT_ASSIST else "assist torque"
        p.drawText(assist_rect, Qt.AlignLeft | Qt.AlignVCenter, f"{assist_label} = {assist_value:+.3f} N·m")
        p.setPen(QPen(QColor("#e2e8f0"), 1))
        p.drawText(QPointF(18.0, height - 14.0), f"actuator = {s.actuator_torque_nm:+.3f} N·m   endstop = {s.endstop_torque_nm:+.3f} N·m")
        p.setPen(QPen(QColor("#94a3b8"), 1))
        p.drawText(QPointF(width - 235.0, height - 14.0), "drag handles / click values")

    def _set_drag_value(self, point: QPointF) -> None:
        if self._drag_key is None:
            return
        width = self.width()
        height = self.height()
        footer_height = 70.0
        footer_top = max(0.0, float(height) - footer_height)
        diagram_top = 42.0
        diagram_bottom = max(diagram_top + 120.0, footer_top - 10.0)
        diagram_height = diagram_bottom - diagram_top
        center = QPointF(width * 0.50, diagram_top + diagram_height * 0.50)
        value = self._angle_at(center, point)
        self._emit_edit(self._drag_key, value)

    def _rect_for_action(self, action: str) -> QRectF | None:
        for rect, candidate in self._interactive_regions:
            if candidate == action:
                return rect
        return None

    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt API
        if event.button() != Qt.LeftButton:
            super().mousePressEvent(event)
            return
        if self._editor is not None and not self._editor.geometry().contains(event.pos()):
            self._close_editor(commit=True)
        action = self._action_at(event.pos())
        if action and action.startswith("drag:"):
            self._drag_key = action.split(":", 1)[1]
            self.setCursor(Qt.ClosedHandCursor)
            self._set_drag_value(event.pos())
            return
        if action and action.startswith("edit:"):
            key = action.split(":", 1)[1]
            rect = self._rect_for_action(action)
            if rect is not None:
                self._open_editor(key, rect)
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 - Qt API
        if self._drag_key is not None:
            self._set_drag_value(event.pos())
            return
        action = self._action_at(event.pos())
        if action != self._hover_action:
            self._hover_action = action
            self.setCursor(Qt.PointingHandCursor if action else Qt.ArrowCursor)
            self.update()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802 - Qt API
        if event.button() == Qt.LeftButton and self._drag_key is not None:
            self._drag_key = None
            self.setCursor(Qt.ArrowCursor)
            self.update()
            return
        super().mouseReleaseEvent(event)

    def leaveEvent(self, event) -> None:  # noqa: N802 - Qt API
        if self._drag_key is None:
            self._hover_action = None
            self.setCursor(Qt.ArrowCursor)
            self.update()
        super().leaveEvent(event)


class SimulatorWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Dynamixel Single-Motor Control Simulator")
        self.resize(1560, 900)
        self.sim = SingleMotorSimulation()
        self.running = False
        self._build_ui()
        self._apply_config(reset=True)

        self.timer = QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self._tick)

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)

        flow_group = QGroupBox("Control transformation")
        flow_layout = QVBoxLayout(flow_group)
        flow_layout.setContentsMargins(6, 10, 6, 6)
        self.flow = NestedFlowWidget()
        self.flow.setMinimumSize(0, 0)
        self.flow.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        flow_layout.addWidget(self.flow)
        self.active_path_label = QLabel()
        self.active_path_label.setWordWrap(True)
        flow_layout.addWidget(self.active_path_label)
        right = QSplitter(Qt.Vertical)
        right.addWidget(flow_group)
        self.view = ServoView()
        self.view.setMinimumSize(0, 0)
        self.view.interaction_changed.connect(self._view_interaction_changed)
        motor_group = QGroupBox("Simulated motor")
        motor_layout = QVBoxLayout(motor_group)
        motor_layout.setContentsMargins(6, 10, 6, 6)
        controls = self._build_controls()
        view_canvas = QWidget()
        view_canvas_layout = QGridLayout(view_canvas)
        view_canvas_layout.setContentsMargins(0, 0, 0, 0)
        view_canvas_layout.addWidget(self.view, 0, 0)
        view_canvas_layout.addWidget(
            self._build_motor_overlay(), 0, 0, Qt.AlignTop | Qt.AlignRight
        )
        motor_layout.addWidget(view_canvas)
        right.addWidget(motor_group)
        right.setSizes([390, 480])
        right.setStretchFactor(0, 1)
        right.setStretchFactor(1, 1)

        root.addWidget(controls, 0)
        root.addWidget(right, 1)

    def _build_motor_overlay(self) -> QWidget:
        overlay = QFrame()
        overlay.setObjectName("motorOverlay")
        overlay.setAttribute(Qt.WA_StyledBackground, True)
        overlay.setMaximumWidth(440)
        overlay_layout = QVBoxLayout(overlay)
        overlay_layout.setContentsMargins(8, 8, 8, 8)
        overlay_layout.setSpacing(6)

        button_row = QHBoxLayout()
        self.run_button = QPushButton("Run")
        self.run_button.clicked.connect(self._toggle_running)
        reset_button = QPushButton("Reset")
        reset_button.clicked.connect(self._reset)
        step_button = QPushButton("Step")
        step_button.clicked.connect(self._single_step)
        button_row.addWidget(self.run_button)
        button_row.addWidget(reset_button)
        button_row.addWidget(step_button)
        overlay_layout.addLayout(button_row)

        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        overlay_layout.addWidget(self.status_label)
        overlay.setStyleSheet(
            """
            QFrame#motorOverlay {
                background-color: rgba(15, 23, 42, 225);
                border: 1px solid #64748b;
                border-radius: 5px;
            }
            """
        )
        return overlay

    def _view_interaction_changed(self, key: str, value: float) -> None:
        fields = {
            "start_position_deg": self.start_box,
            "end_position_deg": self.end_box,
            "endstop_position_deg": self.endstop_position_box,
            "external_torque_nm": self.external_box,
            "assist_torque_nm": self.assist_box,
        }
        widget = fields.get(key)
        if widget is not None:
            widget.setValue(value)
        if key == "start_position_deg" and self.sim.time_s <= 1e-9:
            self._apply_config(reset=True)

    def _build_controls(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(450)
        self._field_rows: dict[str, list[QWidget]] = {}
        outer = QVBoxLayout(panel)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        content = QWidget()
        content.setMinimumWidth(420)
        layout = QVBoxLayout(content)

        mode_group = QGroupBox("Controller selection")
        mode_form = QFormLayout(mode_group)
        self.mode_combo = QComboBox()
        for mode in OperatingMode:
            self.mode_combo.addItem(f"{mode.library_constant}  (raw {mode.raw_value})", mode)
        self.mode_combo.setCurrentIndex(list(OperatingMode).index(OperatingMode.CURRENT_POSITION))
        mode_form.addRow("Operating mode", self.mode_combo)
        self.objective_combo = QComboBox()
        mode_form.addRow("Control objective", self.objective_combo)
        layout.addWidget(mode_group)

        motion_group = QGroupBox("Motion and user effort")
        motion = QFormLayout(motion_group)
        self.start_box = _spin(0, 360, SimulationConfig().start_position_deg, 5)
        self.end_box = _spin(0, 360, SimulationConfig().end_position_deg, 5)
        self.duration_box = _spin(0.1, 60, SimulationConfig().duration_s, 0.5)
        self.external_box = _spin(-2, 2, SimulationConfig().external_torque_nm, 0.05, 3)
        self.assist_box = _spin(-2, 2, SimulationConfig().assist_torque_nm, 0.05, 3)
        self.assist_target_box = _spin(-360, 360, SimulationConfig().assist_target_bias_deg_per_nm, 5)
        self.assist_velocity_box = _spin(-360, 360, SimulationConfig().assist_velocity_deg_s_per_nm, 5)
        self.secondary_lead_box = _spin(0, 90, SimulationConfig().secondary_goal_lead_deg, 1)
        self.secondary_lead_rate_box = _spin(0, 360, SimulationConfig().secondary_goal_lead_rate_deg_s, 1)
        self.secondary_stall_velocity_box = _spin(0, 30, SimulationConfig().secondary_goal_stall_velocity_deg_s, 0.5)
        self.secondary_stall_delay_box = _spin(0, 10, SimulationConfig().secondary_goal_stall_delay_s, 0.05)
        self.current_box = _spin(-910, 910, SimulationConfig().direct_current_ma, 10, 0)
        self.velocity_box = _spin(-360, 360, SimulationConfig().direct_velocity_deg_s, 5)
        self.pwm_box = _spin(-100, 100, SimulationConfig().direct_pwm_percent, 5)
        motion.addRow("Start position (deg)", self.start_box)
        motion.addRow("End position (deg)", self.end_box)
        motion.addRow("Duration (s)", self.duration_box)
        motion.addRow("External torque offset (N-m)", self.external_box)
        self.external_box.setToolTip(
            "Constant offset added to the piecewise-linear external torque mean."
        )
        motion.addRow("Assist / intent torque ceiling (N-m)", self.assist_box)
        motion.addRow("Direct current (mA)", self.current_box)
        motion.addRow("Direct velocity (deg/s)", self.velocity_box)
        motion.addRow("Direct PWM (%)", self.pwm_box)
        motion.addRow("Assist initial bias (deg per N-m)", self.assist_target_box)
        motion.addRow("Assist velocity gain (deg/s per N-m)", self.assist_velocity_box)
        motion.addRow("Secondary goal lead (deg)", self.secondary_lead_box)
        motion.addRow("Lead growth/release rate (deg/s)", self.secondary_lead_rate_box)
        motion.addRow("Stall velocity threshold (deg/s)", self.secondary_stall_velocity_box)
        motion.addRow("Stall release delay (s)", self.secondary_stall_delay_box)
        for key, widget in {
            "start": self.start_box,
            "end": self.end_box,
            "duration": self.duration_box,
            "external": self.external_box,
            "assist_torque": self.assist_box,
            "assist_target": self.assist_target_box,
            "assist_velocity": self.assist_velocity_box,
            "secondary_lead": self.secondary_lead_box,
            "secondary_lead_rate": self.secondary_lead_rate_box,
            "secondary_stall_velocity": self.secondary_stall_velocity_box,
            "secondary_stall_delay": self.secondary_stall_delay_box,
            "current": self.current_box,
            "velocity": self.velocity_box,
            "pwm": self.pwm_box,
        }.items():
            self._field_rows[key] = [motion.labelForField(widget), widget]
        layout.addWidget(motion_group)

        profile_group = QGroupBox("External torque distribution / observer")
        profile_layout = QVBoxLayout(profile_group)
        profile_form = QFormLayout()
        self.profile_period_box = _spin(0, 120, SimulationConfig().external_torque_profile_period_s, 0.5)
        self.profile_noise_box = _spin(0, 1, SimulationConfig().external_torque_noise_std_nm, 0.01, 3)
        self.collection_epoch_box = _spin(0, 30, SimulationConfig().collection_epoch_s, 0.1)
        self.estimator_tau_box = _spin(0.01, 10, SimulationConfig().external_estimator_tau_s, 0.05)
        self.detect_threshold_box = _spin(0, 2, SimulationConfig().assist_detect_threshold_nm, 0.01, 3)
        self.detect_delay_box = _spin(0, 10, SimulationConfig().assist_detect_delay_s, 0.05)
        self.external_gain_box = _spin(0, 10, SimulationConfig().external_assist_gain, 0.1)
        self.external_max_assist_box = _spin(0, 2, SimulationConfig().external_assist_max_torque_nm, 0.05, 3)
        self.external_ramp_box = _spin(0.01, 10, SimulationConfig().external_assist_ramp_s, 0.05)
        profile_form.addRow("Profile loop period (s; 0 = no loop)", self.profile_period_box)
        profile_form.addRow("Gaussian noise σ (N·m)", self.profile_noise_box)
        profile_form.addRow("Zero-collection epoch (s)", self.collection_epoch_box)
        profile_form.addRow("Estimator smoothing τ (s)", self.estimator_tau_box)
        profile_form.addRow("Assist detection threshold (N·m)", self.detect_threshold_box)
        profile_form.addRow("Sustained detection delay (s)", self.detect_delay_box)
        profile_form.addRow("External assist gain", self.external_gain_box)
        profile_form.addRow("Maximum assist torque (N·m)", self.external_max_assist_box)
        profile_form.addRow("Assist ramp time (s)", self.external_ramp_box)
        profile_layout.addLayout(profile_form)

        default_schedule = SimulationConfig().external_torque_schedule or ((0.0, 0.0),)
        self.profile_table = QTableWidget(len(default_schedule), 2)
        self.profile_table.setHorizontalHeaderLabels(["Time (s)", "Mean torque (N-m)"])
        self.profile_table.setFixedWidth(285)
        self.profile_table.setMinimumHeight(92)
        self.profile_table.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.profile_table.setColumnWidth(0, 82)
        self.profile_table.setColumnWidth(1, 150)
        self.profile_table.horizontalHeader().setFixedHeight(28)
        self.profile_table.verticalHeader().setMinimumWidth(20)
        self.profile_table.verticalHeader().setDefaultSectionSize(32)
        self.profile_table.setStyleSheet(
            """
            QTableWidget {
                background-color: #111827;
                color: #e5e7eb;
                gridline-color: #334155;
                selection-background-color: #2563eb;
                selection-color: #ffffff;
            }
            QTableWidget::item {
                padding: 3px;
            }
            QHeaderView::section {
                background-color: #1f2937;
                color: #e5e7eb;
                border: 1px solid #475569;
                padding: 4px;
            }
            QTableCornerButton::section {
                background-color: #1f2937;
                border: 1px solid #475569;
            }
            """
        )
        for row, (time_s, torque_nm) in enumerate(default_schedule):
            self.profile_table.setItem(row, 0, QTableWidgetItem(f"{time_s:g}"))
            self.profile_table.setItem(row, 1, QTableWidgetItem(f"{torque_nm:g}"))
        profile_layout.addWidget(QLabel("Piecewise-linear mean profile; noise is added around it."))
        profile_row = QHBoxLayout()
        profile_row.setContentsMargins(0, 0, 0, 0)
        profile_row.setSpacing(8)
        profile_row.addWidget(self.profile_table, 0)
        profile_buttons = QVBoxLayout()
        profile_buttons.setContentsMargins(0, 0, 0, 0)
        profile_buttons.setSpacing(6)
        add_profile_button = QPushButton("Add point")
        add_profile_button.clicked.connect(self._add_profile_point)
        remove_profile_button = QPushButton("Remove point")
        remove_profile_button.clicked.connect(self._remove_profile_point)
        profile_buttons.addWidget(add_profile_button)
        profile_buttons.addWidget(remove_profile_button)
        profile_buttons.addStretch(1)
        profile_row.addLayout(profile_buttons, 0)
        profile_row.addStretch(1)
        profile_layout.addLayout(profile_row)
        layout.addWidget(profile_group)

        gain_group = QGroupBox("Dynamixel-style registers")
        gains = QGridLayout(gain_group)
        defaults = SimulationConfig()
        self.p_box = _int_spin(0, 16383, defaults.position_p_raw, 25)
        self.i_box = _int_spin(0, 16383, defaults.position_i_raw, 25)
        self.d_box = _int_spin(0, 16383, defaults.position_d_raw, 25)
        self.vp_box = _int_spin(0, 16383, defaults.velocity_p_raw, 10)
        self.vi_box = _int_spin(0, 16383, defaults.velocity_i_raw, 25)
        self.goal_current_box = _spin(0, 910, defaults.goal_current_limit_ma, 10, 0)
        self.pwm_limit_box = _spin(0, 100, defaults.pwm_limit_percent, 5)
        rows = [
            ("Position P raw", self.p_box, "raw / 128"),
            ("Position I raw", self.i_box, "raw / 65536"),
            ("Position D raw", self.d_box, "raw / 16"),
            ("Velocity P raw", self.vp_box, "raw / 128"),
            ("Velocity I raw", self.vi_box, "raw / 65536"),
            ("Goal current limit (mA)", self.goal_current_box, "CURRENT_POSITION"),
            ("PWM limit (%)", self.pwm_limit_box, "POSITION / VELOCITY"),
        ]
        for row, (label, widget, hint) in enumerate(rows):
            label_widget = QLabel(label)
            hint_widget = QLabel(hint)
            gains.addWidget(label_widget, row, 0)
            gains.addWidget(widget, row, 1)
            gains.addWidget(hint_widget, row, 2)
            key = (
                "position_p",
                "position_i",
                "position_d",
                "velocity_p",
                "velocity_i",
                "goal_current_limit",
                "pwm_limit",
            )[row]
            self._field_rows[key] = [label_widget, widget, hint_widget]
        layout.addWidget(gain_group)

        endstop_group = QGroupBox("Soft endstop / counter-torque")
        endstop = QFormLayout(endstop_group)
        self.endstop_check = QCheckBox("Enable endstop reaction")
        self.endstop_check.setChecked(SimulationConfig().endstop_enabled)
        self.endstop_position_box = _spin(0, 360, SimulationConfig().endstop_position_deg, 5)
        self.endstop_side_combo = QComboBox()
        self.endstop_side_combo.addItem("Upper / positive direction", 1)
        self.endstop_side_combo.addItem("Lower / negative direction", -1)
        self.endstop_side_combo.setCurrentIndex(0 if SimulationConfig().endstop_side > 0 else 1)
        self.endstop_max_box = _spin(0, 2, SimulationConfig().endstop_max_torque_nm, 0.05, 3)
        self.endstop_stiffness_box = _spin(0, 2, SimulationConfig().endstop_stiffness_nm_per_deg, 0.01, 3)
        endstop.addRow(self.endstop_check)
        endstop.addRow("Endstop position (deg)", self.endstop_position_box)
        endstop.addRow("Direction", self.endstop_side_combo)
        endstop.addRow("Max counter-torque (N·m)", self.endstop_max_box)
        endstop.addRow("Stiffness (N·m/deg)", self.endstop_stiffness_box)
        layout.addWidget(endstop_group)

        playback_group = QGroupBox("Playback")
        playback = QFormLayout(playback_group)
        self.timescale_box = _spin(0.01, 10.0, 1.0, 0.05, 2)
        self.timescale_box.setToolTip("0.10 means 100 ms of simulation advances per second of wall time")
        playback.addRow("Simulation seconds / wall second", self.timescale_box)
        self.timescale_hint = QLabel("1.00× = real-time simulation")
        self.timescale_hint.setWordWrap(True)
        playback.addRow(self.timescale_hint)
        layout.addWidget(playback_group)

        derived_group = QGroupBox("Derived control outputs")
        derived_layout = QVBoxLayout(derived_group)
        self.derived_label = QLabel()
        self.derived_label.setWordWrap(True)
        derived_layout.addWidget(self.derived_label)
        layout.addWidget(derived_group)

        layout.addStretch(1)
        scroll.setWidget(content)
        outer.addWidget(scroll)

        for widget in (
            self.objective_combo,
            self.start_box,
            self.end_box,
            self.duration_box,
            self.external_box,
            self.assist_box,
            self.profile_period_box,
            self.profile_noise_box,
            self.collection_epoch_box,
            self.estimator_tau_box,
            self.detect_threshold_box,
            self.detect_delay_box,
            self.external_gain_box,
            self.external_max_assist_box,
            self.external_ramp_box,
            self.assist_target_box,
            self.assist_velocity_box,
            self.secondary_lead_box,
            self.secondary_lead_rate_box,
            self.secondary_stall_velocity_box,
            self.secondary_stall_delay_box,
            self.current_box,
            self.velocity_box,
            self.pwm_box,
            self.p_box,
            self.i_box,
            self.d_box,
            self.vp_box,
            self.vi_box,
            self.goal_current_box,
            self.pwm_limit_box,
            self.endstop_check,
            self.endstop_position_box,
            self.endstop_side_combo,
            self.endstop_max_box,
            self.endstop_stiffness_box,
            self.timescale_box,
        ):
            signal = getattr(widget, "currentIndexChanged", None) or getattr(widget, "valueChanged", None) or getattr(widget, "stateChanged", None)
            if signal is not None:
                signal.connect(self._controls_changed)
        self.profile_table.itemChanged.connect(self._profile_table_changed)
        self.mode_combo.currentIndexChanged.connect(self._mode_changed)
        self._refresh_objectives()
        return panel

    def _set_field_visible(self, key: str, visible: bool) -> None:
        for widget in self._field_rows.get(key, []):
            if widget is not None:
                widget.setVisible(visible)

    def _refresh_objectives(self) -> None:
        mode = self.mode_combo.currentData()
        valid = _objectives_for_mode(mode)
        current = self.objective_combo.currentData()
        if current not in valid:
            current = (
                Objective.SECONDARY_GOAL_ASSIST
                if mode == OperatingMode.CURRENT_POSITION
                else valid[0]
            )
        self.objective_combo.blockSignals(True)
        self.objective_combo.clear()
        for objective in valid:
            self.objective_combo.addItem(objective.value, objective)
        self.objective_combo.setCurrentIndex(valid.index(current))
        self.objective_combo.blockSignals(False)
        self._update_control_visibility()

    def _mode_changed(self, *_args) -> None:
        self._refresh_objectives()
        self._apply_config(reset=False)

    def _update_control_visibility(self) -> None:
        mode = self.mode_combo.currentData()
        objective = self.objective_combo.currentData()
        position_mode = mode in {
            OperatingMode.POSITION,
            OperatingMode.EXTENDED_POSITION,
            OperatingMode.CURRENT_POSITION,
        }
        self._set_field_visible("end", objective == Objective.POSITION_TRAJECTORY)
        self._set_field_visible("duration", objective == Objective.POSITION_TRAJECTORY)
        assist = objective in {
            Objective.SECONDARY_GOAL_ASSIST,
            Objective.TORQUE_ASSIST,
            Objective.EXTERNAL_EFFORT_ASSIST,
        }
        self._set_field_visible(
            "assist_torque",
            objective in {Objective.SECONDARY_GOAL_ASSIST, Objective.TORQUE_ASSIST},
        )
        self._set_field_visible(
            "assist_target",
            objective in {Objective.TORQUE_ASSIST, Objective.EXTERNAL_EFFORT_ASSIST}
            and position_mode,
        )
        self._set_field_visible(
            "assist_velocity",
            assist and (position_mode or mode == OperatingMode.VELOCITY),
        )
        secondary = objective == Objective.SECONDARY_GOAL_ASSIST
        for key in (
            "secondary_lead",
            "secondary_lead_rate",
            "secondary_stall_velocity",
            "secondary_stall_delay",
        ):
            self._set_field_visible(key, secondary and position_mode)
        self._set_field_visible("current", mode == OperatingMode.CURRENT)
        self._set_field_visible("velocity", mode == OperatingMode.VELOCITY)
        self._set_field_visible("pwm", mode == OperatingMode.PWM)
        self._set_field_visible("position_p", position_mode)
        self._set_field_visible("position_i", position_mode)
        self._set_field_visible("position_d", position_mode)
        self._set_field_visible("velocity_p", mode == OperatingMode.VELOCITY)
        self._set_field_visible("velocity_i", mode == OperatingMode.VELOCITY)
        self._set_field_visible(
            "goal_current_limit",
            mode in {OperatingMode.CURRENT_POSITION, OperatingMode.CURRENT},
        )
        self._set_field_visible(
            "pwm_limit",
            mode in {
                OperatingMode.POSITION,
                OperatingMode.EXTENDED_POSITION,
                OperatingMode.VELOCITY,
                OperatingMode.PWM,
            },
        )

    def _read_profile_schedule(self) -> tuple[tuple[float, float], ...]:
        points: list[tuple[float, float]] = []
        for row in range(self.profile_table.rowCount()):
            time_item = self.profile_table.item(row, 0)
            torque_item = self.profile_table.item(row, 1)
            try:
                if time_item is None or torque_item is None:
                    continue
                time_s = float(time_item.text())
                torque_nm = float(torque_item.text())
            except ValueError:
                continue
            if math.isfinite(time_s) and math.isfinite(torque_nm):
                points.append((max(0.0, time_s), torque_nm))
        return tuple(sorted(points)) or ((0.0, 0.0),)

    def _profile_table_changed(self, *_args) -> None:
        self._apply_config(reset=False)

    def _add_profile_point(self) -> None:
        row = self.profile_table.rowCount()
        self.profile_table.insertRow(row)
        previous_time = 0.0
        if row > 0 and self.profile_table.item(row - 1, 0) is not None:
            try:
                previous_time = float(self.profile_table.item(row - 1, 0).text())
            except ValueError:
                previous_time = float(row)
        self.profile_table.setItem(row, 0, QTableWidgetItem(f"{previous_time + 1.0:g}"))
        self.profile_table.setItem(row, 1, QTableWidgetItem("0"))

    def _remove_profile_point(self) -> None:
        row = self.profile_table.currentRow()
        if row >= 0 and self.profile_table.rowCount() > 1:
            self.profile_table.removeRow(row)

    def _read_config(self) -> SimulationConfig:
        return SimulationConfig(
            mode=self.mode_combo.currentData(),
            objective=self.objective_combo.currentData(),
            start_position_deg=self.start_box.value(),
            end_position_deg=self.end_box.value(),
            duration_s=self.duration_box.value(),
            external_torque_nm=self.external_box.value(),
            assist_torque_nm=self.assist_box.value(),
            external_torque_noise_std_nm=self.profile_noise_box.value(),
            external_torque_profile_period_s=self.profile_period_box.value(),
            external_torque_schedule=self._read_profile_schedule(),
            collection_epoch_s=self.collection_epoch_box.value(),
            external_estimator_tau_s=self.estimator_tau_box.value(),
            assist_detect_threshold_nm=self.detect_threshold_box.value(),
            assist_detect_delay_s=self.detect_delay_box.value(),
            external_assist_gain=self.external_gain_box.value(),
            external_assist_max_torque_nm=self.external_max_assist_box.value(),
            external_assist_ramp_s=self.external_ramp_box.value(),
            assist_target_bias_deg_per_nm=self.assist_target_box.value(),
            assist_velocity_deg_s_per_nm=self.assist_velocity_box.value(),
            secondary_goal_lead_deg=self.secondary_lead_box.value(),
            secondary_goal_lead_rate_deg_s=self.secondary_lead_rate_box.value(),
            secondary_goal_stall_velocity_deg_s=self.secondary_stall_velocity_box.value(),
            secondary_goal_stall_delay_s=self.secondary_stall_delay_box.value(),
            direct_current_ma=self.current_box.value(),
            direct_velocity_deg_s=self.velocity_box.value(),
            direct_pwm_percent=self.pwm_box.value(),
            endstop_enabled=self.endstop_check.isChecked(),
            endstop_position_deg=self.endstop_position_box.value(),
            endstop_side=self.endstop_side_combo.currentData(),
            endstop_max_torque_nm=self.endstop_max_box.value(),
            endstop_stiffness_nm_per_deg=self.endstop_stiffness_box.value(),
            position_p_raw=self.p_box.value(),
            position_i_raw=self.i_box.value(),
            position_d_raw=self.d_box.value(),
            velocity_p_raw=self.vp_box.value(),
            velocity_i_raw=self.vi_box.value(),
            goal_current_limit_ma=self.goal_current_box.value(),
            pwm_limit_percent=self.pwm_limit_box.value(),
        )

    def _apply_config(self, *, reset: bool = False) -> None:
        config = self._read_config()
        self.sim.set_config(config, reset=reset)
        self._render(self.sim.previous_snapshot)

    def _controls_changed(self, *_args) -> None:
        self._update_control_visibility()
        self._apply_config(reset=False)

    def _render(self, snapshot: SimulationSnapshot) -> None:
        timescale = self.timescale_box.value()
        self.timescale_hint.setText(
            f"{timescale:.2f}× = {timescale * 1000.0:.0f} ms of simulation per wall second"
        )
        self.active_path_label.setText(
            f"<b>Active:</b> {self.sim.mode_label} &nbsp; | &nbsp; <b>Path:</b> {self.sim.active_path}"
        )
        self.status_label.setText(
            f"t={snapshot.time_s:5.2f}s | q={snapshot.position_deg:7.2f}° | "
            f"qref={snapshot.reference_position_deg:7.2f}° | "
            f"v={snapshot.velocity_deg_s:7.2f}°/s\n"
            f"PID={snapshot.pid_torque_nm:+.3f} N·m | actuator={snapshot.actuator_torque_nm:+.3f} N·m | "
            f"current={snapshot.current_ma:+.1f} mA | Goal Current={snapshot.goal_current_ma:.1f} mA | "
            f"PWM={snapshot.pwm_percent:+.1f}%\n"
            f"endstop penetration={snapshot.endstop_penetration_deg:.2f}° | "
            f"integral state={snapshot.integral_error_deg_s:+.2f}"
        )
        if self.sim.config.objective == Objective.EXTERNAL_EFFORT_ASSIST:
            collection = "complete" if snapshot.collection_progress >= 1.0 else f"collecting {snapshot.collection_progress * 100.0:.0f}%"
            derived = (
                f"External observer: <b>{collection}</b>; "
                f"estimate {snapshot.external_torque_estimate_nm:+.3f} N-m; "
                f"zero bias {snapshot.external_torque_bias_nm:+.3f} N-m; "
                f"residual {snapshot.debiased_external_torque_nm:+.3f} N-m. "
                f"Assist: <b>{'ON' if snapshot.assist_active else 'waiting'}</b> "
                f"({snapshot.assist_torque_command_nm:+.3f} N-m)."
            )
        elif self.sim.config.mode == OperatingMode.CURRENT_POSITION:
            if self.sim.config.objective == Objective.SECONDARY_GOAL_ASSIST:
                derived = (
                    f"Intent velocity: <b>{snapshot.secondary_goal_velocity_deg_s:+.1f} deg/s</b>; "
                    f"assist {'ON' if snapshot.assist_active else 'waiting'}. "
                    f"q_goal={snapshot.secondary_goal_position_deg:.1f} deg; "
                    f"lead={snapshot.secondary_goal_lead_deg:+.1f} deg; "
                    f"Goal Current ceiling={self.sim.config.goal_current_limit_ma:.0f} mA."
                )
            elif self.sim.config.objective == Objective.TORQUE_ASSIST:
                base_reference, _ = self.sim._trajectory_reference()
                target_shift = snapshot.reference_position_deg - base_reference
                derived = (
                    f"Effective Goal Current: <b>{snapshot.goal_current_ma:.0f} mA</b> "
                    f"(assist-derived; ceiling {self.sim.config.goal_current_limit_ma:.0f} mA). "
                    f"Live assist: {snapshot.assist_torque_command_nm:+.3f} N-m; "
                    f"moving target offset: {target_shift:+.1f} deg."
                )
            else:
                derived = (
                    f"Effective Goal Current: <b>{snapshot.goal_current_ma:.0f} mA</b> "
                    f"(configured ceiling)."
                )
        elif self.sim.config.mode == OperatingMode.CURRENT:
            derived = (
                f"Signed current command: <b>{snapshot.current_ma:+.0f} mA</b>; "
                f"effective Goal Current magnitude: {snapshot.goal_current_ma:.0f} mA."
            )
        elif self.sim.config.mode == OperatingMode.VELOCITY:
            derived = f"Velocity reference: <b>{snapshot.reference_velocity_deg_s:+.1f} deg/s</b>."
        elif self.sim.config.mode == OperatingMode.PWM:
            derived = f"Signed PWM command: <b>{snapshot.pwm_percent:+.1f}%</b>."
        else:
            derived = f"Position reference: <b>{snapshot.reference_position_deg:.1f} deg</b>."
        self.derived_label.setText(derived)
        self.flow.update_state(self.sim.config, snapshot)
        self.view.update_state(self.sim.config, snapshot)

    def _tick(self) -> None:
        self._apply_config(reset=False)
        self._render(self.sim.step(0.02 * self.timescale_box.value()))

    def _toggle_running(self) -> None:
        self.running = not self.running
        if self.running:
            self.timer.start()
            self.run_button.setText("Pause")
        else:
            self.timer.stop()
            self.run_button.setText("Run")

    def _reset(self) -> None:
        self.timer.stop()
        self.running = False
        self.run_button.setText("Run")
        self._apply_config(reset=True)

    def _single_step(self) -> None:
        self.timer.stop()
        self.running = False
        self.run_button.setText("Run")
        self._apply_config(reset=False)
        self._render(self.sim.step(0.02 * self.timescale_box.value()))


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyleSheet(
        """
        QWidget { background: #111827; color: #e5e7eb; }
        QGroupBox { border: 1px solid #475569; border-radius: 5px; margin-top: 8px; padding-top: 10px; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
        QComboBox, QDoubleSpinBox, QSpinBox, QPushButton { background: #1f2937; border: 1px solid #64748b; padding: 4px; }
        QPushButton:hover { background: #334155; }
        QScrollArea { background: #111827; }
        """
    )
    window = SimulatorWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
