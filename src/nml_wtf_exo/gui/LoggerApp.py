import sys
import os
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton, QLabel,
    QComboBox, QLineEdit, QFileDialog, QMessageBox
)
from PyQt5.QtCore import Qt
from pylsl import resolve_streams, StreamInlet
from nml_wtf_exo.lsl.ParameterLogger import ParameterLogger
from nml_wtf_exo.lsl.StreamLogger import StreamLogger


class LoggerApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Parameter Logger")
        self.setGeometry(100, 100, 400, 200)

        self.logger = None

        # UI Elements
        self.stream_select = QComboBox()
        self.stream_refresh_btn = QPushButton("Refresh Streams")
        self.filename_input = QLineEdit("logsession")
        self.folder_btn = QPushButton("Select Log Folder")
        self.start_btn = QPushButton("Start Logging")
        self.stop_btn = QPushButton("Stop Logging")
        self.status = QLabel("Idle")
        self.status.setAlignment(Qt.AlignCenter)

        self.log_dir = "landmarks"
        self.selected_stream_name = None

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Stream to log:"))
        layout.addWidget(self.stream_select)
        layout.addWidget(self.stream_refresh_btn)

        layout.addWidget(QLabel("Base filename suffix:"))
        layout.addWidget(self.filename_input)

        layout.addWidget(self.folder_btn)
        layout.addWidget(self.start_btn)
        layout.addWidget(self.stop_btn)
        layout.addWidget(self.status)

        self.setLayout(layout)

        # Bindings
        self.stream_refresh_btn.clicked.connect(self.refresh_streams)
        self.folder_btn.clicked.connect(self.select_folder)
        self.start_btn.clicked.connect(self._on_start_clicked)
        self.stop_btn.clicked.connect(self._on_stop_clicked)
        self.stop_btn.setEnabled(False)

        self.refresh_streams()

    def _on_start_clicked(self):
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.folder_btn.setEnabled(False)
        self.start_logging()

    def _on_stop_clicked(self):
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.folder_btn.setEnabled(True)
        self.stop_logging()

    def refresh_streams(self):
        self.stream_select.clear()
        try:
            streams = resolve_streams()
            self.available_streams = streams
            for s in self.available_streams:
                self.stream_select.addItem(f"{s.name()} [{s.source_id()}]")
        except Exception as e:
            QMessageBox.critical(self, "Stream Error", f"Failed to resolve streams:\n{e}")

    def select_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Log Folder", self.log_dir)
        if folder:
            self.log_dir = folder

    def stop_logging(self):
        if self.logger:
            self.logger.stop()
            self.status.setText("Logging stopped.")
            self.logger = None
        else:
            QMessageBox.information(self, "Not Running", "No logger is currently running.")

    def start_logging(self):
        if self.logger:
            QMessageBox.warning(self, "Already Running", "Logger is already running.")
            return

        if self.stream_select.currentIndex() == -1:
            QMessageBox.warning(self, "No Stream", "Please select an LSL stream.")
            return

        try:
            stream_info = self.available_streams[self.stream_select.currentIndex()]
            suffix = self.filename_input.text().strip()
            if not suffix:
                suffix = "log"

            os.makedirs(self.log_dir, exist_ok=True)

            # Inject custom name suffix into filename
            if stream_info.type() == "Markers":
                self.logger = ParameterLogger(log_dir=self.log_dir, suffix=suffix)
            else:
                self.logger = StreamLogger(stream_info.name(), log_dir=self.log_dir, suffix=suffix) 
            self.logger.inlet = StreamInlet(stream_info)
            self.logger.start()
            self.status.setText(f"Logging to: {self.logger.base_filename}")

        except Exception as e:
            QMessageBox.critical(self, "Logging Error", f"Failed to start logging:\n{e}")
            return