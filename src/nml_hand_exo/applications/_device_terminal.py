"""Persistent device terminal presentation; device I/O belongs to SerialWorker."""
from collections import deque
from datetime import datetime

from PyQt5.QtCore import QEvent, QSize, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFontDatabase, QIcon, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QDialog, QHBoxLayout, QLabel, QLineEdit, QPlainTextEdit,
    QPushButton, QToolButton, QVBoxLayout, QWidget,
)


def terminal_icon(kind):
    """Draw portable line icons without a platform icon theme or font glyphs."""
    pixmap = QPixmap(48, 48)
    pixmap.fill(Qt.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.Antialiasing)
    painter.scale(2, 2)
    painter.setPen(QPen(QColor("#e0e0e0"), 1.6, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
    if kind == "terminal":
        painter.drawRoundedRect(2, 3, 20, 18, 2, 2)
        painter.drawLine(6, 8, 10, 12)
        painter.drawLine(10, 12, 6, 16)
        painter.drawLine(13, 16, 18, 16)
    elif kind == "copy":
        painter.drawRoundedRect(8, 8, 12, 13, 1, 1)
        painter.drawLine(4, 16, 4, 3)
        painter.drawLine(4, 3, 16, 3)
    elif kind == "clear":
        painter.drawLine(4, 6, 20, 6)
        painter.drawLine(9, 3, 15, 3)
        painter.drawLine(6, 6, 7, 21)
        painter.drawLine(7, 21, 17, 21)
        painter.drawLine(17, 21, 18, 6)
        painter.drawLine(10, 10, 10, 17)
        painter.drawLine(14, 10, 14, 17)
    elif kind == "clock":
        painter.drawEllipse(3, 3, 18, 18)
        painter.drawLine(12, 6, 12, 12)
        painter.drawLine(12, 12, 16, 14)
    painter.end()
    return QIcon(pixmap)


class _TerminalOutput(QPlainTextEdit):
    """Read-only output with hover tools floating over its top-right corner."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        fixed_font = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.setFont(fixed_font)
        self.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.setStyleSheet(
            "QPlainTextEdit { background: #111111; color: #e0e0e0; border: 1px solid #444444; "
            f'font-family: "{fixed_font.family()}"; }}'
        )
        self.tools = QWidget(self)
        self.tools.setStyleSheet("""
            QToolButton { background: #2e2e2e; border: 1px solid #444444;
                          border-radius: 4px; padding: 4px; }
            QToolButton:hover { background: #3a3a3a; border-color: #c41230; }
            QToolButton:checked { background: #701020; border-color: #ef3a47; }
        """)
        layout = QHBoxLayout(self.tools)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.copy_button = self._tool(layout, "copy", "Copy all terminal output")
        self.clear_button = self._tool(layout, "clear", "Clear terminal output")
        self.clock_button = self._tool(layout, "clock", "Toggle timestamps (milliseconds)")
        self.clock_button.setCheckable(True)
        self.clock_button.setChecked(True)
        self.tools.adjustSize()
        self.tools.hide()

    def _tool(self, layout, kind, tooltip):
        button = QToolButton(self.tools)
        button.setIcon(terminal_icon(kind))
        button.setIconSize(QSize(18, 18))
        button.setFixedSize(30, 30)
        button.setToolTip(tooltip)
        button.setAccessibleName(tooltip)
        layout.addWidget(button)
        return button

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.tools.move(max(0, self.viewport().width() - self.tools.width() - 8), 8)
        self.tools.raise_()

    def enterEvent(self, event):
        self.tools.show()
        self.tools.raise_()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.tools.hide()
        super().leaveEvent(event)


class DeviceTerminal(QDialog):
    command_requested = pyqtSignal(str)
    MAX_LINES = 10000
    ACTIVE_OPACITY = 1.0
    INACTIVE_OPACITY = 0.28

    def __init__(self, parent=None):
        super().__init__(parent, Qt.Window)
        self.setModal(True)
        self.setWindowTitle("Device Terminal — NML EXO")
        self.setWindowIcon(terminal_icon("terminal"))
        self.setWindowOpacity(self.ACTIVE_OPACITY)
        self.resize(780, 460)
        self._lines = deque(maxlen=self.MAX_LINES)
        self._history = []
        self._history_index = 0
        self._draft = ""
        layout = QVBoxLayout(self)
        input_row = QHBoxLayout()
        self.command_edit = QLineEdit()
        fixed_font = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.command_edit.setFont(fixed_font)
        self.command_edit.setStyleSheet(f'QLineEdit {{ font-family: "{fixed_font.family()}"; }}')
        self.command_edit.setPlaceholderText("Firmware command, e.g. info or current_status")
        self.command_edit.setToolTip(
            "Send one command using the detected firmware protocol. Use integer motor IDs. "
            "Commands bypass GUI targeting and safety-state bookkeeping. Up/Down recall commands."
        )
        self.command_edit.installEventFilter(self)
        self.command_edit.returnPressed.connect(self._submit)
        self.send_button = QPushButton("Send")
        self.send_button.setAutoDefault(False)
        self.send_button.clicked.connect(self._submit)
        input_row.addWidget(self.command_edit, 1)
        input_row.addWidget(self.send_button)
        layout.addLayout(input_row)
        self.protocol_label = QLabel()
        self.protocol_label.setWordWrap(True)
        layout.addWidget(self.protocol_label)
        self.output = _TerminalOutput(self)
        self.output.document().setMaximumBlockCount(self.MAX_LINES)
        self.output.copy_button.clicked.connect(self.copy_output)
        self.output.clear_button.clicked.connect(self.clear_output)
        self.output.clock_button.toggled.connect(self._render)
        layout.addWidget(self.output, 1)
        self.set_connection(False)

    def set_connection(self, connected, protocol=""):
        self.command_edit.setEnabled(connected)
        self.send_button.setEnabled(connected)
        if not connected:
            text = "Disconnected — connect a device to send commands"
        elif protocol == "protobuf-v1":
            text = "Protobuf USB — binary RPC commands and replies; text endpoint for maintenance commands"
        elif protocol == "protobuf-v2":
            text = "Protobuf USB — framed command and response CDCs"
        else:
            text = f"{protocol or 'ASCII serial'} — firmware text commands; decoded binary telemetry"
        self.protocol_label.setText(text)

    def _submit(self):
        command = self.command_edit.text().strip().rstrip(";").strip()
        if command and self.send_button.isEnabled():
            self._history.append(command)
            self._history = self._history[-200:]
            self._history_index = len(self._history)
            self._draft = ""
            self.command_edit.clear()
            self.command_requested.emit(command)
            self.command_edit.setFocus()

    def eventFilter(self, obj, event):
        if obj is self.command_edit and event.type() == QEvent.KeyPress:
            if event.key() in (Qt.Key_Up, Qt.Key_Down) and self._history:
                if self._history_index == len(self._history):
                    self._draft = self.command_edit.text()
                step = -1 if event.key() == Qt.Key_Up else 1
                self._history_index = max(0, min(len(self._history), self._history_index + step))
                text = (self._draft if self._history_index == len(self._history)
                        else self._history[self._history_index])
                self.command_edit.setText(text)
                return True
        return super().eventFilter(obj, event)

    def append_message(self, message, timestamp=None):
        timestamp = timestamp or datetime.now()
        stamp = timestamp.strftime("%H:%M:%S.%f")[:-3]
        lines = str(message).splitlines() or [""]
        at_bottom = self.output.verticalScrollBar().value() == self.output.verticalScrollBar().maximum()
        for line in lines:
            self._lines.append((stamp, line))
            self.output.appendPlainText(self._format_line(stamp, line))
        if at_bottom:
            self.output.verticalScrollBar().setValue(self.output.verticalScrollBar().maximum())

    def _format_line(self, stamp, line):
        return f"[{stamp}] {line}" if self.output.clock_button.isChecked() else line

    def _render(self):
        scrollbar = self.output.verticalScrollBar()
        old_position = scrollbar.value()
        at_bottom = old_position == scrollbar.maximum()
        self.output.setPlainText("\n".join(self._format_line(*entry) for entry in self._lines))
        scrollbar.setValue(scrollbar.maximum() if at_bottom else old_position)

    def copy_output(self):
        QApplication.clipboard().setText(self.output.toPlainText())

    def clear_output(self):
        self._lines.clear()
        self.output.clear()

    def changeEvent(self, event):
        if event.type() == QEvent.ActivationChange:
            self.setWindowOpacity(
                self.ACTIVE_OPACITY if self.isActiveWindow() else self.INACTIVE_OPACITY
            )
        super().changeEvent(event)

    def open_terminal(self):
        self.setWindowOpacity(self.ACTIVE_OPACITY)
        self.show()
        self.raise_()
        self.activateWindow()
        if self.command_edit.isEnabled():
            self.command_edit.setFocus()

    def reject(self):
        # Escape closes the popout without losing the session's output/history.
        self.hide()
