from __future__ import annotations

import sys
from pathlib import Path

try:
    from serial.tools import list_ports
except Exception:
    list_ports = None


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
EXAMPLE_ROOT = Path(__file__).resolve().parent
SRC_ROOT_STR = str(SRC_ROOT)
EXAMPLE_ROOT_STR = str(EXAMPLE_ROOT)
if SRC_ROOT_STR not in sys.path:
    sys.path.insert(0, SRC_ROOT_STR)
if EXAMPLE_ROOT_STR not in sys.path:
    sys.path.insert(0, EXAMPLE_ROOT_STR)

from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from mindrove.board_shim import BoardShim, MindRoveInputParams

from MindRoveExoDemo_4_10 import (
    BOARD_ID,
    IMU_FREEZE_THRESHOLD,
    STEP_SEC,
    USE_IMU_FREEZE,
    GrabRestClassifier,
    extract_activity_score,
    extract_movement_score,
    get_accel_channels_safe,
    get_gyro_channels_safe,
    get_latest_window,
)
from nml_hand_exo.interface import HandExo, SerialComm


class CalibrationWorker(QThread):
    finished = pyqtSignal(bool, str)

    def __init__(self, classifier: GrabRestClassifier, board_shim, exg_channels: list[int], fs: int):
        super().__init__()
        self.classifier = classifier
        self.board_shim = board_shim
        self.exg_channels = exg_channels
        self.fs = fs

    def run(self) -> None:
        try:
            ok = self.classifier.calibrate(self.board_shim, self.exg_channels, self.fs)
            if ok:
                self.finished.emit(True, "Calibration complete.")
            else:
                self.finished.emit(False, "Calibration failed. Check the stream and try again.")
        except Exception as exc:
            self.finished.emit(False, f"Calibration failed: {exc}")


class MindRoveExoControlPanel(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("MindRove + Exo Control Panel")
        self.resize(1180, 620)

        self.board_shim = None
        self.board_id = None
        self.exg_channels: list[int] = []
        self.accel_channels: list[int] = []
        self.gyro_channels: list[int] = []
        self.fs = 0
        self.window_points = 0
        self.classifier = GrabRestClassifier()
        self.exo: HandExo | None = None
        self.calibration_worker: CalibrationWorker | None = None

        self._build_ui()
        self._apply_console_theme()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll_mindrove)
        self.poll_timer.setInterval(int(STEP_SEC * 1000))

        self._refresh_serial_ports()
        self._update_mindrove_labels()
        self._update_exo_labels()

    def _apply_console_theme(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow {
                background-color: #020617;
            }

            QWidget#Root {
                background: qlineargradient(
                    x1: 0, y1: 0, x2: 1, y2: 1,
                    stop: 0 #020617,
                    stop: 0.45 #07111f,
                    stop: 1 #111827
                );
                color: #E2E8F0;
            }

            QFrame#CompactBar {
                background-color: rgba(10, 18, 32, 238);
                border: 1px solid rgba(96, 165, 250, 0.16);
                border-radius: 22px;
            }

            QFrame#GlassCard,
            QFrame#LinkCard,
            QFrame#MetricCard {
                background-color: rgba(9, 14, 25, 224);
                border: 1px solid rgba(96, 165, 250, 0.18);
                border-radius: 24px;
            }

            QFrame#LinkCard[connected="true"],
            QFrame#StatusChip[connected="true"] {
                background-color: rgba(11, 41, 28, 236);
                border: 1px solid rgba(74, 222, 128, 0.72);
            }

            QFrame#StatusChip {
                background-color: rgba(15, 23, 42, 220);
                border: 1px solid rgba(71, 85, 105, 0.5);
                border-radius: 16px;
            }

            QLabel#Eyebrow {
                color: #60A5FA;
                font-size: 12px;
                font-weight: 700;
                letter-spacing: 0.08em;
                text-transform: uppercase;
            }

            QLabel#HeroTitle {
                color: #F8FAFC;
                font-size: 30px;
                font-weight: 700;
            }

            QLabel#SectionTitle {
                color: #F8FAFC;
                font-size: 22px;
                font-weight: 600;
            }

            QLabel#Muted {
                color: #94A3B8;
                font-size: 12px;
            }

            QLabel#TinyMuted,
            QLabel#MetricCaption,
            QLabel#FieldLabel,
            QLabel#ChipCaption {
                color: #64748B;
                font-size: 11px;
                font-weight: 600;
                letter-spacing: 0.04em;
            }

            QLabel#ChipValue {
                color: #F8FAFC;
                font-size: 14px;
                font-weight: 700;
            }

            QLabel#ChipValue[connected="true"] {
                color: #DCFCE7;
            }

            QLabel#MetricValue {
                color: #F8FAFC;
                font-size: 24px;
                font-weight: 700;
            }

            QLabel#FieldValue {
                color: #E2E8F0;
                font-size: 13px;
                font-weight: 500;
            }

            QPushButton {
                border-radius: 16px;
                padding: 10px 14px;
                background-color: #1E293B;
                border: 1px solid rgba(96, 165, 250, 0.18);
                color: #F8FAFC;
            }

            QPushButton:hover {
                background-color: #273449;
            }

            QPushButton:pressed {
                background-color: #1B2638;
            }

            QPushButton#PrimaryButton {
                background-color: #2563EB;
                border: 1px solid #3B82F6;
            }

            QPushButton#PrimaryButton:hover {
                background-color: #1D4ED8;
            }

            QLineEdit,
            QComboBox,
            QTextEdit {
                background-color: rgba(15, 23, 42, 220);
                border: 1px solid rgba(71, 85, 105, 0.8);
                border-radius: 14px;
                padding: 10px 12px;
                color: #F8FAFC;
                selection-background-color: #2563EB;
            }

            QLineEdit:focus,
            QComboBox:focus,
            QTextEdit:focus {
                border: 2px solid #60A5FA;
            }

            QComboBox::drop-down {
                border: none;
            }

            QTextEdit#LogView {
                font-family: "Cascadia Mono", "Consolas", monospace;
                font-size: 12px;
                line-height: 1.3;
            }
            """
        )

    def _create_card(self, title: str, subtitle: str, *, object_name: str = "GlassCard") -> tuple[QFrame, QVBoxLayout]:
        card = QFrame()
        card.setObjectName(object_name)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)

        title_label = QLabel(title)
        title_label.setObjectName("SectionTitle")
        subtitle_label = QLabel(subtitle)
        subtitle_label.setObjectName("Muted")
        subtitle_label.setWordWrap(True)

        layout.addWidget(title_label)
        layout.addWidget(subtitle_label)
        return card, layout

    def _create_status_chip(self, caption: str, value: str, attr_prefix: str) -> QFrame:
        chip = QFrame()
        chip.setObjectName("StatusChip")
        layout = QVBoxLayout(chip)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(2)

        caption_label = QLabel(caption)
        caption_label.setObjectName("ChipCaption")
        value_label = QLabel(value)
        value_label.setObjectName("ChipValue")

        layout.addWidget(caption_label)
        layout.addWidget(value_label)

        setattr(self, f"{attr_prefix}_chip_frame", chip)
        setattr(self, f"{attr_prefix}_chip_value", value_label)
        return chip

    def _create_metric_card(self, title: str, value_label: QLabel) -> QFrame:
        card = QFrame()
        card.setObjectName("MetricCard")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(4)

        caption = QLabel(title)
        caption.setObjectName("MetricCaption")
        value_label.setObjectName("MetricValue")

        layout.addWidget(caption)
        layout.addWidget(value_label)
        return card

    def _create_field_grid(self, fields: list[tuple[str, QLabel]]) -> QGridLayout:
        grid = QGridLayout()
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(8)
        for row, (label_text, value_widget) in enumerate(fields):
            label = QLabel(label_text)
            label.setObjectName("FieldLabel")
            value_widget.setObjectName("FieldValue")
            grid.addWidget(label, row, 0)
            grid.addWidget(value_widget, row, 1)
        grid.setColumnStretch(1, 1)
        return grid

    def _set_connected_style(self, widget: QWidget, connected: bool) -> None:
        widget.setProperty("connected", connected)
        widget.style().unpolish(widget)
        widget.style().polish(widget)
        widget.update()

    def _build_ui(self) -> None:
        root = QWidget()
        root.setObjectName("Root")
        self.setCentralWidget(root)

        outer = QVBoxLayout(root)
        outer.setContentsMargins(14, 14, 14, 14)
        outer.setSpacing(12)

        header = QFrame()
        header.setObjectName("CompactBar")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(14, 12, 14, 12)
        header_layout.setSpacing(12)

        title_stack = QVBoxLayout()
        title_stack.setSpacing(2)
        eyebrow = QLabel("Operator Surface")
        eyebrow.setObjectName("Eyebrow")
        title = QLabel("MindRove Stream + Exo Link")
        title.setObjectName("HeroTitle")
        subtitle = QLabel("Live EMG stream status on the left, exoskeleton USB serial transport on the right.")
        subtitle.setObjectName("Muted")
        subtitle.setWordWrap(True)
        title_stack.addWidget(eyebrow)
        title_stack.addWidget(title)
        title_stack.addWidget(subtitle)
        header_layout.addLayout(title_stack, stretch=3)
        header_layout.addStretch(1)

        chip_row = QHBoxLayout()
        chip_row.setSpacing(10)
        chip_row.addWidget(self._create_status_chip("STREAM", "Offline", "stream"))
        chip_row.addWidget(self._create_status_chip("CLASSIFIER", "Uncal", "classifier"))
        chip_row.addWidget(self._create_status_chip("LINK", "Offline", "link"))
        header_layout.addLayout(chip_row)
        outer.addWidget(header)

        body = QHBoxLayout()
        body.setSpacing(12)
        outer.addLayout(body, stretch=1)

        body.addWidget(self._build_mindrove_panel(), stretch=3)
        body.addWidget(self._build_exo_panel(), stretch=2)

        outer.addWidget(self._build_log_panel())

    def _build_mindrove_panel(self) -> QWidget:
        self.stream_card, layout = self._create_card(
            "MindRove Streaming",
            "MindRove acquisition, calibration, and classifier state use the same dark operator-console treatment as the other control surfaces.",
        )

        button_row = QHBoxLayout()
        self.start_stream_button = QPushButton("Start Stream")
        self.start_stream_button.setObjectName("PrimaryButton")
        self.start_stream_button.clicked.connect(self._start_mindrove_stream)
        self.stop_stream_button = QPushButton("Stop Stream")
        self.stop_stream_button.clicked.connect(self._stop_mindrove_stream)
        self.calibrate_button = QPushButton("Calibrate")
        self.calibrate_button.clicked.connect(self._start_calibration)
        button_row.addWidget(self.start_stream_button)
        button_row.addWidget(self.stop_stream_button)
        button_row.addWidget(self.calibrate_button)
        button_row.addStretch(1)
        layout.addLayout(button_row)

        self.stream_status_value = QLabel("Offline")
        self.fs_value = QLabel("n/a")
        self.state_value = QLabel("REST")
        self.freeze_value = QLabel("n/a")

        metrics_grid = QGridLayout()
        metrics_grid.setHorizontalSpacing(10)
        metrics_grid.setVerticalSpacing(10)
        metrics_grid.addWidget(self._create_metric_card("Stream", self.stream_status_value), 0, 0)
        metrics_grid.addWidget(self._create_metric_card("Sample Rate", self.fs_value), 0, 1)
        metrics_grid.addWidget(self._create_metric_card("State", self.state_value), 1, 0)
        metrics_grid.addWidget(self._create_metric_card("Freeze Gate", self.freeze_value), 1, 1)
        layout.addLayout(metrics_grid)

        self.board_value = QLabel("n/a")
        self.channels_value = QLabel("n/a")
        self.calibration_value = QLabel("Not calibrated")
        self.emg_value = QLabel("n/a")
        self.imu_value = QLabel("n/a")
        self.band_value = QLabel("n/a")
        self.rms_value = QLabel("n/a")

        detail_fields = [
            ("Board", self.board_value),
            ("Channels", self.channels_value),
            ("Calibration", self.calibration_value),
            ("EMG score", self.emg_value),
            ("IMU score", self.imu_value),
            ("Band mean", self.band_value),
            ("RMS mean", self.rms_value),
        ]

        layout.addLayout(self._create_field_grid(detail_fields))

        hint = QLabel(
            "This panel only monitors the MindRove stream and classifier state. "
            "It does not send exo motion commands automatically."
        )
        hint.setWordWrap(True)
        hint.setObjectName("TinyMuted")
        layout.addWidget(hint)

        layout.addStretch(1)
        return self.stream_card

    def _build_exo_panel(self) -> QWidget:
        self.link_card, layout = self._create_card(
            "Exo USB Serial",
            "Live transport controls mirror the same link-card treatment used in the other GUIs, with status surfaced as cards and chips instead of a plain form.",
            object_name="LinkCard",
        )

        port_row = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setInsertPolicy(QComboBox.NoInsert)
        self.port_combo.currentTextChanged.connect(lambda value: self.selected_port_value.setText(value.strip() or "n/a"))
        if self.port_combo.lineEdit() is not None:
            self.port_combo.lineEdit().setPlaceholderText("COM4")
        self.refresh_ports_button = QPushButton("Refresh Ports")
        self.refresh_ports_button.clicked.connect(self._refresh_serial_ports)
        port_row.addWidget(self.port_combo, stretch=1)
        port_row.addWidget(self.refresh_ports_button)
        port_label = QLabel("Serial port")
        port_label.setObjectName("FieldLabel")
        layout.addWidget(port_label)
        layout.addLayout(port_row)

        self.baud_input = QLineEdit("1000000")
        self.baud_input.textChanged.connect(lambda value: self.selected_baud_value.setText(value.strip() or "1000000"))
        baud_label = QLabel("Baudrate")
        baud_label.setObjectName("FieldLabel")
        layout.addWidget(baud_label)
        layout.addWidget(self.baud_input)

        connect_row = QHBoxLayout()
        self.connect_button = QPushButton("Connect Serial")
        self.connect_button.setObjectName("PrimaryButton")
        self.connect_button.clicked.connect(self._toggle_exo_connection)
        self.refresh_exo_button = QPushButton("Refresh Device Info")
        self.refresh_exo_button.clicked.connect(self._refresh_exo_status)
        self.home_button = QPushButton("Home")
        self.home_button.clicked.connect(self._send_home)
        connect_row.addWidget(self.connect_button)
        connect_row.addWidget(self.refresh_exo_button)
        connect_row.addWidget(self.home_button)
        layout.addLayout(connect_row)

        self.exo_status_value = QLabel("Disconnected")
        self.exo_mode_value = QLabel("n/a")
        self.exo_gesture_value = QLabel("n/a")
        self.exo_motors_value = QLabel("n/a")

        metrics_grid = QGridLayout()
        metrics_grid.setHorizontalSpacing(10)
        metrics_grid.setVerticalSpacing(10)
        metrics_grid.addWidget(self._create_metric_card("Link", self.exo_status_value), 0, 0)
        metrics_grid.addWidget(self._create_metric_card("Mode", self.exo_mode_value), 0, 1)
        metrics_grid.addWidget(self._create_metric_card("Gesture", self.exo_gesture_value), 1, 0)
        metrics_grid.addWidget(self._create_metric_card("Motors", self.exo_motors_value), 1, 1)
        layout.addLayout(metrics_grid)

        self.exo_name_value = QLabel("n/a")
        self.exo_version_value = QLabel("n/a")

        detail_fields = [
            ("Device", self.exo_name_value),
            ("Version", self.exo_version_value),
            ("Selected port", QLabel(self.port_combo.currentText().strip() or "n/a")),
            ("Baudrate", QLabel(self.baud_input.text().strip() or "1000000")),
        ]

        self.selected_port_value = detail_fields[2][1]
        self.selected_baud_value = detail_fields[3][1]
        layout.addLayout(self._create_field_grid(detail_fields))

        hint = QLabel("This panel manages only the USB serial link and basic exo status queries.")
        hint.setWordWrap(True)
        hint.setObjectName("TinyMuted")
        layout.addWidget(hint)

        layout.addStretch(1)
        return self.link_card

    def _build_log_panel(self) -> QWidget:
        card, layout = self._create_card(
            "Runtime Log",
            "MindRove stream events, calibration notes, and exo transport status updates appear here in one place.",
        )
        self.log_view = QTextEdit()
        self.log_view.setObjectName("LogView")
        self.log_view.setReadOnly(True)
        self.log_view.setMinimumHeight(150)
        layout.addWidget(self.log_view)
        return card

    def _append_log(self, text: str) -> None:
        self.log_view.append(text)

    def _start_mindrove_stream(self) -> None:
        if self.board_shim is not None:
            self._append_log("MindRove stream is already active.")
            return

        try:
            BoardShim.enable_dev_board_logger()
            params = MindRoveInputParams()
            self.board_shim = BoardShim(BOARD_ID, params)
            self.board_shim.prepare_session()
            self.board_shim.start_stream()

            self.board_id = self.board_shim.get_board_id()
            self.exg_channels = BoardShim.get_exg_channels(self.board_id)
            self.accel_channels = get_accel_channels_safe(self.board_id)
            self.gyro_channels = get_gyro_channels_safe(self.board_id)
            self.fs = BoardShim.get_sampling_rate(self.board_id)
            self.window_points = int(self.fs * 0.50)

            self.poll_timer.start()
            self._append_log("MindRove stream started.")
            self._update_mindrove_labels()
        except Exception as exc:
            self._append_log(f"MindRove start failed: {exc}")
            QMessageBox.critical(self, "MindRove Error", str(exc))
            self._release_board()

    def _stop_mindrove_stream(self) -> None:
        self.poll_timer.stop()
        self._release_board()
        self.classifier = GrabRestClassifier()
        self._update_mindrove_labels()
        self._append_log("MindRove stream stopped.")

    def _release_board(self) -> None:
        if self.board_shim is None:
            return
        try:
            if hasattr(self.board_shim, "stop_stream"):
                self.board_shim.stop_stream()
        except Exception:
            pass
        try:
            if self.board_shim.is_prepared():
                self.board_shim.release_session()
        except Exception:
            pass
        self.board_shim = None
        self.board_id = None
        self.exg_channels = []
        self.accel_channels = []
        self.gyro_channels = []
        self.fs = 0
        self.window_points = 0

    def _start_calibration(self) -> None:
        if self.board_shim is None or not self.exg_channels or not self.fs:
            QMessageBox.warning(self, "MindRove", "Start the MindRove stream before calibrating.")
            return
        if self.calibration_worker is not None and self.calibration_worker.isRunning():
            return

        self.poll_timer.stop()
        self.calibration_value.setText("Calibrating...")
        self.calibrate_button.setEnabled(False)
        self.start_stream_button.setEnabled(False)
        self.stop_stream_button.setEnabled(False)
        self._append_log("Calibration started. Keep the terminal nearby for guided rest/grab prompts.")

        self.calibration_worker = CalibrationWorker(self.classifier, self.board_shim, self.exg_channels, self.fs)
        self.calibration_worker.finished.connect(self._finish_calibration)
        self.calibration_worker.start()

    def _finish_calibration(self, success: bool, message: str) -> None:
        self.calibrate_button.setEnabled(True)
        self.start_stream_button.setEnabled(True)
        self.stop_stream_button.setEnabled(True)
        if self.board_shim is not None:
            self.poll_timer.start()
        self._append_log(message)
        self._update_mindrove_labels()
        if not success:
            QMessageBox.warning(self, "Calibration", message)

    def _poll_mindrove(self) -> None:
        if self.board_shim is None or not self.window_points:
            return

        try:
            data = get_latest_window(self.board_shim, self.window_points)
            if data is None:
                return

            if self.classifier.calibrated:
                emg_score, debug = extract_activity_score(data, self.exg_channels, self.fs)
                movement_score = extract_movement_score(data, self.accel_channels, self.gyro_channels)
                state, smooth_score, smooth_movement, freeze_active = self.classifier.classify(
                    emg_score,
                    movement_score,
                    IMU_FREEZE_THRESHOLD,
                    USE_IMU_FREEZE,
                )
                self.state_value.setText(state)
                self.freeze_value.setText("FREEZE" if freeze_active else "LIVE")
                self.emg_value.setText(f"{smooth_score:.4f}")
                self.imu_value.setText(f"{smooth_movement:.4f}")
                self.band_value.setText(f"{debug['band_mean']:.4f}")
                self.rms_value.setText(f"{debug['rms_mean']:.4f}")
            else:
                self.freeze_value.setText("Awaiting calibration")
        except Exception as exc:
            self._append_log(f"MindRove poll failed: {exc}")
            self.poll_timer.stop()

    def _update_mindrove_labels(self) -> None:
        stream_active = self.board_shim is not None
        self.stream_status_value.setText("Streaming" if stream_active else "Offline")
        self.board_value.setText(str(self.board_id) if self.board_id is not None else str(BOARD_ID))
        self.fs_value.setText(f"{self.fs} Hz" if self.fs else "n/a")
        if self.exg_channels:
            self.channels_value.setText(
                f"EXG {len(self.exg_channels)} | ACC {len(self.accel_channels)} | GYRO {len(self.gyro_channels)}"
            )
        else:
            self.channels_value.setText("n/a")
        self.calibration_value.setText("Ready" if self.classifier.calibrated else "Not calibrated")
        self.stream_chip_value.setText("Live" if stream_active else "Offline")
        self.stream_chip_value.setProperty("connected", stream_active)
        self._set_connected_style(self.stream_chip_frame, stream_active)
        self.stream_chip_value.style().unpolish(self.stream_chip_value)
        self.stream_chip_value.style().polish(self.stream_chip_value)
        classifier_ready = self.classifier.calibrated
        self.classifier_chip_value.setText("Ready" if classifier_ready else "Uncal")
        self.classifier_chip_value.setProperty("connected", classifier_ready)
        self._set_connected_style(self.classifier_chip_frame, classifier_ready)
        self.classifier_chip_value.style().unpolish(self.classifier_chip_value)
        self.classifier_chip_value.style().polish(self.classifier_chip_value)
        if not self.classifier.calibrated:
            self.state_value.setText("REST")
            self.freeze_value.setText("n/a")
            self.emg_value.setText("n/a")
            self.imu_value.setText("n/a")
            self.band_value.setText("n/a")
            self.rms_value.setText("n/a")

    def _refresh_serial_ports(self) -> None:
        current = self.port_combo.currentText().strip()
        ports: list[str] = []
        if list_ports is not None:
            try:
                ports = sorted({str(port.device).strip() for port in list_ports.comports() if str(port.device).strip()})
            except Exception:
                ports = []

        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        if ports:
            self.port_combo.addItems(ports)
        if current:
            self.port_combo.setCurrentText(current)
        elif ports:
            self.port_combo.setCurrentText(ports[0])
        self.port_combo.blockSignals(False)
        self.selected_port_value.setText(self.port_combo.currentText().strip() or "n/a")

        if not ports and list_ports is None:
            self._append_log("Serial port discovery unavailable. Enter the COM port manually.")

    def _toggle_exo_connection(self) -> None:
        if self.exo is not None and self.exo.connected:
            self._disconnect_exo()
        else:
            self._connect_exo()

    def _connect_exo(self) -> None:
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "Exo Serial", "Choose or enter a COM port first.")
            return

        try:
            baudrate = int(float(self.baud_input.text().strip() or "1000000"))
            self.selected_baud_value.setText(str(baudrate))
            comm = SerialComm(port=port, baudrate=baudrate, verbose=False)
            self.exo = HandExo(comm, verbose=False)
            self.exo.connect()
            self._append_log(f"Exo connected on {port}.")
            self._refresh_exo_status()
        except Exception as exc:
            self.exo = None
            self._update_exo_labels()
            self._append_log(f"Exo connection failed: {exc}")
            QMessageBox.critical(self, "Exo Serial", str(exc))

    def _disconnect_exo(self) -> None:
        if self.exo is None:
            return
        try:
            self.exo.close()
        except Exception:
            pass
        self.exo = None
        self._update_exo_labels()
        self._append_log("Exo disconnected.")

    def _refresh_exo_status(self) -> None:
        if self.exo is None or not self.exo.connected:
            self._update_exo_labels()
            return

        try:
            info = self.exo.info()
            mode = self.exo.get_exo_mode() or "n/a"
            gesture = self.exo.get_gesture() or "n/a"
            self._update_exo_labels(info=info, mode=mode, gesture=gesture)
            self._append_log("Exo status refreshed.")
        except Exception as exc:
            self._append_log(f"Exo status refresh failed: {exc}")
            QMessageBox.warning(self, "Exo Serial", str(exc))

    def _send_home(self) -> None:
        if self.exo is None or not self.exo.connected:
            QMessageBox.warning(self, "Exo Serial", "Connect the exo before sending home.")
            return
        try:
            self.exo.home("all")
            self._append_log("Sent home command to the exo.")
        except Exception as exc:
            self._append_log(f"Home command failed: {exc}")
            QMessageBox.warning(self, "Exo Serial", str(exc))

    def _update_exo_labels(self, info: dict | None = None, mode: str = "n/a", gesture: str = "n/a") -> None:
        connected = self.exo is not None and self.exo.connected
        self.exo_status_value.setText("Connected" if connected else "Disconnected")
        self.connect_button.setText("Disconnect Serial" if connected else "Connect Serial")
        self.refresh_exo_button.setEnabled(connected)
        self.home_button.setEnabled(connected)
        self.link_chip_value.setText("Live" if connected else "Offline")
        self.link_chip_value.setProperty("connected", connected)
        self._set_connected_style(self.link_chip_frame, connected)
        self._set_connected_style(self.link_card, connected)
        self.link_chip_value.style().unpolish(self.link_chip_value)
        self.link_chip_value.style().polish(self.link_chip_value)
        self.selected_port_value.setText(self.port_combo.currentText().strip() or "n/a")
        self.selected_baud_value.setText(self.baud_input.text().strip() or "1000000")

        if not connected or not info:
            self.exo_name_value.setText("n/a")
            self.exo_version_value.setText("n/a")
            self.exo_mode_value.setText("n/a")
            self.exo_gesture_value.setText("n/a")
            self.exo_motors_value.setText("n/a")
            return

        self.exo_name_value.setText(str(info.get("name", "n/a")))
        self.exo_version_value.setText(str(info.get("version", "n/a")))
        self.exo_mode_value.setText(mode)
        self.exo_gesture_value.setText(gesture)
        self.exo_motors_value.setText(str(info.get("n_motors", len(info.get("motors", {})))))

    def closeEvent(self, event) -> None:
        self.poll_timer.stop()
        if self.calibration_worker is not None and self.calibration_worker.isRunning():
            self.calibration_worker.wait(250)
        self._disconnect_exo()
        self._release_board()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    window = MindRoveExoControlPanel()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
