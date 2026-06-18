import sys
import json
import serial
import threading
import time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QProgressBar, QLineEdit, QTextEdit, QSpacerItem, QSizePolicy,
    QFileDialog,
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt5.QtGui import QPalette, QColor


class SerialSignal(QObject):
    new_line = pyqtSignal(str)

class TaskPrompt(QWidget):
    def __init__(self, task_config):
        super().__init__()
        self.setWindowTitle("Exoskeleton Task Trainer")
        self.resize(900, 500)
        self.task_config = task_config
        self.task_index = 0
        self.effort_index = 0
        self.current_task = None

        # Serial connection vars
        self.serial_connection = None
        self.serial_thread = None
        self.running_serial = False
        self.port = "COM6"
        self.baud = 57600

        self.task_log = []  # For storing effort trial summaries
        self.countdown_timer = QTimer()
        self.countdown_timer.timeout.connect(self.update_countdown)
        self.countdown_value = 3  # Countdown seconds

        self.build_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_progress)

        self.serial_signals = SerialSignal()
        self.serial_signals.new_line.connect(self.feedback_box.append)

    def build_ui(self):
        main_layout = QHBoxLayout()
        task_panel = QVBoxLayout()
        control_panel = QVBoxLayout()

        # Task & Feedback Section
        self.label = QLabel("Welcome to the Exoskeleton Task Trainer")
        self.instruction = QLabel("")
        self.progress = QProgressBar()
        self.feedback_box = QTextEdit()
        self.feedback_box.setReadOnly(True)
        self.feedback_box.setMinimumHeight(200)
        self.feedback_box.setStyleSheet("font-family: Consolas; font-size: 7pt;")

        # Task Control Buttons
        self.start_button = QPushButton("Start Task")
        self.start_button.clicked.connect(self.start_task)

        self.stop_button = QPushButton("Stop Task")
        self.stop_button.clicked.connect(self.stop_task)

        self.load_button = QPushButton("Load Task")
        self.load_button.clicked.connect(self.load_task_config)

        task_button_row = QHBoxLayout()
        task_button_row.addWidget(self.start_button)
        task_button_row.addWidget(self.stop_button)
        task_button_row.addWidget(self.load_button)

        task_panel.addWidget(self.label)
        task_panel.addWidget(self.instruction)
        task_panel.addWidget(self.progress)
        #task_panel.addWidget(self.next_button)
        task_panel.addLayout(task_button_row)
        task_panel.addWidget(self.feedback_box)

        # --- Serial Control Section (Top Right) ---
        self.com_label = QLabel("COM Port:")
        self.com_input = QLineEdit(self.port)
        self.baud_label = QLabel("Baud Rate:")
        self.baud_input = QLineEdit(str(self.baud))

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_serial_connection)

        com_row = QHBoxLayout()
        com_row.addWidget(self.com_label)
        com_row.addWidget(self.com_input)
        com_row.addWidget(self.baud_label)
        com_row.addWidget(self.baud_input)
        com_row.addWidget(self.connect_button)

        # --- User Command Entry Section ---
        self.user_msg_input = QLineEdit()
        self.user_msg_input.setPlaceholderText("Enter custom command to send...")
        self.user_msg_input.returnPressed.connect(self.send_custom_command)
        self.send_button = QPushButton("Send")
        self.send_button.clicked.connect(self.send_custom_command)

        send_row = QHBoxLayout()
        send_row.addWidget(self.user_msg_input)
        send_row.addWidget(self.send_button)

        control_panel.addLayout(com_row)
        control_panel.addLayout(send_row)
        control_panel.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        # Add to main layout
        main_layout.addLayout(task_panel, 3)
        main_layout.addLayout(control_panel, 1)
        self.setLayout(main_layout)

    def set_input_color(self, widget, color: str):
        palette = widget.palette()
        palette.setColor(QPalette.Base, QColor(color))
        widget.setPalette(palette)

    def toggle_serial_connection(self):
        if self.serial_connection is None:
            try:
                port = self.com_input.text()
                baud = int(self.baud_input.text())
                self.serial_connection = serial.Serial(port, baud, timeout=0.1)
                self.feedback_box.append(f"[Connected to {port} @ {baud} baud]")
                self.connect_button.setText("Disconnect")
                self.set_input_color(self.com_input, "#d4ffd4")
                self.set_input_color(self.baud_input, "#d4ffd4")

                self.running_serial = True
                self.serial_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
                self.serial_thread.start()
            except Exception as e:
                self.feedback_box.append(f"[Connection Failed] {e}")
        else:
            self.running_serial = False
            if self.serial_thread:
                self.serial_thread.join(timeout=1)
            try:
                self.serial_connection.close()
            except:
                pass
            self.serial_connection = None
            self.connect_button.setText("Connect")
            self.set_input_color(self.com_input, "white")
            self.set_input_color(self.baud_input, "white")
            self.feedback_box.append("[Disconnected]")

    def read_serial_loop(self):
        while self.running_serial and self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting:
                    lines = self.serial_connection.read(self.serial_connection.in_waiting).decode(
                        errors="ignore").splitlines()
                    for line in lines:
                        if line.strip():
                            self.serial_signals.new_line.emit(line.strip())
            except Exception as e:
                self.serial_signals.new_line.emit(f"[Read Error] {e}")
            time.sleep(0.05)

    def send_custom_command(self):
        if self.serial_connection and self.serial_connection.is_open:
            cmd = self.user_msg_input.text().strip()
            if cmd:
                try:
                    self.serial_connection.write((cmd + '\n').encode())
                    self.feedback_box.append(f"> {cmd}")
                    self.user_msg_input.clear()
                except Exception as e:
                    self.feedback_box.append(f"[Send Error] {e}")
        else:
            self.feedback_box.append("[Not Connected]")

    def start_task(self):
        if self.task_index >= len(self.task_config.get('tasks', [])):
            self.label.setText("All tasks completed.")
            self.instruction.setText("Training complete.")
            self.show_task_summary()
            return
        try:
            self.current_task = self.task_config['tasks'][self.task_index]
            self.effort_levels = self.current_task['effort_levels']
            self.effort_index = 0
            self.run_effort()
        except Exception as e:
            self.feedback_box.append(f"[Error in task config] {e}")

    def stop_task(self):
        self.timer.stop()
        self.instruction.setText("Task stopped.")
        self.progress.setValue(0)
        self.task_index = 0
        self.effort_index = 0

    def show_task_summary(self):
        self.feedback_box.append("\n=== Task Completion Summary ===")
        for entry in self.task_log:
            self.feedback_box.append(f"• {entry}")
        self.feedback_box.append("=== End of Summary ===\n")

    def load_task_config(self):
        file_name, _ = QFileDialog.getOpenFileName(self, "Open Task Config", "", "JSON Files (*.json)")
        if file_name:
            try:
                with open(file_name, "r") as f:
                    self.task_config = json.load(f)
                self.feedback_box.append(f"[Task config loaded from {file_name}]")
                self.task_index = 0
                self.effort_index = 0
                self.instruction.setText("Ready to start new task.")
            except Exception as e:
                self.feedback_box.append(f"[Error loading task config] {e}")

    def run_effort(self):
        if self.effort_index >= len(self.effort_levels):
            self.task_index += 1
            QTimer.singleShot(100, self.start_task)
            return

        self.countdown_value = 3
        effort = self.effort_levels[self.effort_index]
        self.instruction.setText(
            f"Get ready for {self.current_task['name']} at {effort}% effort..."
        )
        self.progress.setValue(0)
        self.countdown_timer.start(1000)

    def update_countdown(self):
        if self.countdown_value > 0:
            self.instruction.setText(f"Starting in {self.countdown_value}...")
            self.countdown_value -= 1
        else:
            self.countdown_timer.stop()
            self.begin_effort()

    def begin_effort(self):
        effort = self.effort_levels[self.effort_index]
        task_name = self.current_task['name']

        self.instruction.setText(
            f"Task: {task_name} – {effort}% effort for {self.current_task['hold_time_sec']} sec"
        )
        self.feedback_box.append(f"[Task] {task_name} @ {effort}% effort")

        self.progress.setValue(0)
        self.timer_count = 0
        self.hold_time_ms = self.current_task['hold_time_sec'] * 1000
        self.timer.start(100)

        self.task_log.append(f"{task_name}: {effort}% effort for {self.current_task['hold_time_sec']}s")

    def update_progress(self):
        self.timer_count += 100
        percent = int(100 * self.timer_count / self.hold_time_ms)
        self.progress.setValue(min(percent, 100))
        if self.timer_count >= self.hold_time_ms:
            self.timer.stop()
            self.instruction.setText("Rest...")
            QTimer.singleShot(self.current_task['rest_time_sec'] * 1000, self.advance_effort)

    def advance_effort(self):
        self.effort_index += 1
        self.run_effort()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    with open("task_config.json", "r") as f:
        config = json.load(f)
    gui = TaskPrompt(config)
    gui.show()
    sys.exit(app.exec_())
