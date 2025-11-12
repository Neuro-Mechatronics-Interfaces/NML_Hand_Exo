# nml/gui/KeyboardApp.py
# A simple GUI application to connect to an LSL stream and visualize keyboard events.
import sys
from typing import Optional
from nml_wtf_exo.utils.paths import PATHS

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton, QLabel,
    QComboBox, QLineEdit, QFileDialog, QMessageBox, QHBoxLayout, QCheckBox
)

from pylsl import resolve_streams
from nml_wtf_exo.controller.KeyboardEventWorker import KeyboardEventWorker
from nml_wtf_exo.controller.FixedRateKeyParser import FixedRateKeyParser
from nml_wtf_exo.lsl.LSLKeyboardHandler import LSLKeyboardHandler
from nml_wtf_exo.gui.ui.KeyboardOverlay import KeyboardOverlay

class KeyboardApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Keyboard Controller")
        self.setGeometry(100, 100, 900, 320)

        self.worker = KeyboardEventWorker(poll_interval=0.01)
        self.worker.start()

        self.handler: Optional[LSLKeyboardHandler] = None
        self.available_streams = []
        self.config_path: Optional[str] = None

        # --- UI ---
        self.stream_select = QComboBox()
        self.refresh_btn = QPushButton("Refresh Streams")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.status = QLabel("Idle"); self.status.setAlignment(Qt.AlignCenter)

        cfg_row = QHBoxLayout()
        self.cfg_edit = QLineEdit("")
        self.cfg_btn = QPushButton("Load Parser Config...")
        self.enable_checkbox = QCheckBox("Enable OS Keys")
        self.enable_checkbox.setChecked(True)
        cfg_row.addWidget(self.cfg_edit)
        cfg_row.addWidget(self.cfg_btn)
        cfg_row.addWidget(self.enable_checkbox)
        self.txt_edit = QLineEdit("")
        self.txt_edit.setPlaceholderText("Parsed text will appear here...")
        self.txt_edit.setEnabled(False)
        self.enable_checkbox.toggled.connect(self._on_checkbox_toggled)

        # Overlay
        self.overlay = KeyboardOverlay(PATHS["keyboard_layout_json"])
        self.overlay.setFixedSize(self.overlay._pixmap.size())

        # Layout
        root = QVBoxLayout()
        root.addWidget(QLabel("LSL Stream:"))
        root.addWidget(self.stream_select)
        root.addWidget(self.refresh_btn)
        root.addLayout(cfg_row)
        root.addWidget(self.txt_edit)
        root.addWidget(self.connect_btn)
        root.addWidget(self.disconnect_btn)
        root.addWidget(self.overlay, stretch=1)
        root.addWidget(self.status)
        self.setLayout(root)

        # Bind
        self.refresh_btn.clicked.connect(self.refresh_streams)
        self.cfg_btn.clicked.connect(self.select_config)
        self.connect_btn.clicked.connect(self.connect_stream)
        self.disconnect_btn.clicked.connect(self.disconnect_stream)

        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self._connected = False

        self.refresh_streams()

    def _on_checkbox_toggled(self, checked: bool):
        """Enable or disable OS key input based on checkbox state."""
        if self.handler:
            self.handler.setEnable(checked)
        if checked:
            self.txt_edit.setFocus()

    def _on_newline(self):
        """Handle newline signal from the handler."""
        self.txt_edit.setText("")

    def _on_connected(self):
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.stream_select.setEnabled(False)
        self.refresh_btn.setEnabled(False)
        self.cfg_btn.setEnabled(False)
        self.cfg_edit.setEnabled(False)
        self.txt_edit.setEnabled(True)
        self.status.setText("Connected")
        if self.enable_checkbox.isChecked():
            self.txt_edit.setFocus()
        self._connected = True

    def _on_disconnected(self):
        self.overlay.release_all()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.stream_select.setEnabled(True)
        self.refresh_btn.setEnabled(True)
        self.cfg_btn.setEnabled(True)
        self.cfg_edit.setEnabled(True)
        self.txt_edit.setEnabled(False)
        self.status.setText("Disconnected")
        self._connected = False

    def refresh_streams(self):
        self.stream_select.clear()
        self.status.setText("Refreshing streams...")
        try:
            streams = resolve_streams()
            self.available_streams = streams
            for s in streams:
                sr = s.nominal_srate()
                rate = f"{sr:.1f} Hz" if sr and sr > 0 else "event"
                self.stream_select.addItem(f"{s.name()} [{s.type()} @ {rate}]  ({s.source_id()})")
            self.status.setText("Select a stream." if streams else "No LSL streams found.")
        except Exception as e:
            QMessageBox.critical(self, "Stream Error", f"Failed to resolve streams:\n{e}")

    def select_config(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select Parser Config (JSON)", "", "JSON Files (*.json)")
        if path:
            self.config_path = path
            self.cfg_edit.setText(path)

    def connect_stream(self):
        if self._connected:
            self.status.setText("Connected")
            return
        if self.handler and self.handler.isRunning():
            QMessageBox.warning(self, "Already Connected", "A stream is already connected.")
            return
        if self.stream_select.currentIndex() == -1 or not self.available_streams:
            QMessageBox.warning(self, "No Stream", "Please select an LSL stream.")
            return

        try:
            s = self.available_streams[self.stream_select.currentIndex()]
            srate = float(s.nominal_srate())
            json_mode = not (srate > 0.0 and (srate != float("inf")))
            parser = None
            if not json_mode:
                parser = FixedRateKeyParser()
                parser.configure(self.config_path)
            self.handler = LSLKeyboardHandler(s, self.worker, parser, self.overlay)
            self.handler.status.connect(self.status.setText)
            self.handler.error.connect(lambda msg: self._show_error("Reader Error", msg))
            self.handler.connected.connect(self._on_connected)
            self.handler.disconnected.connect(self._on_disconnected)
            self.handler.newline.connect(self._on_newline)
            self.handler.start()
            self.status.setText("Connecting...")

        except Exception as e:
            self._show_error("Connect Error", str(e))

    def disconnect_stream(self):
        if not self._connected:
            self.status.setText("Disconnected")
            return
        if self.handler and self.handler.isRunning():
            self.handler.stop()
            self.handler.wait(1500)
            self.handler = None
            self.overlay.release_all()
            self.status.setText("Disconnected")
        else:
            QMessageBox.information(self, "Not Connected", "No active stream to disconnect.")

    def closeEvent(self, event):
        try:
            if self._connected:
                self.disconnect_stream()
        finally:
            try: self.worker.stop()
            except Exception: pass
        super().closeEvent(event)

    def _show_error(self, title: str, msg: str):
        self.status.setText("Error")
        QMessageBox.critical(self, title, msg)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = KeyboardApp()
    w.show()
    sys.exit(app.exec_())