"""Participant-facing EMG intent discovery and decoder GUI.

This application is intentionally separate from ``emg_centroid_decoder_gui``.
It reuses the LSL worker and the modular ``nml_hand_exo.decoding`` package while
keeping exoskeleton hardware commands in the hand-exo application.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLayout,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QProgressBar,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from nml_hand_exo.applications._emg_lsl_helpers import ChunkBuffer, EmgStreamWorker, LslScanWorker
from nml_hand_exo.decoding import (
    FeatureConfig,
    IntentCaptureSession,
    IntentDecoderPipeline,
    IntentOutputStabilizer,
    PreprocessConfig,
    StreamLayout,
    assess_signal_quality,
    extract_emg_features,
    orientation_from_accel,
    parse_channel_spec,
    preprocess_emg,
    rank_intent_pairs,
    import_xdf_session,
)


INTENT_STYLE = """
QWidget {
    background: #080a0d;
    color: #f5f7fa;
    font-family: "Segoe UI", "SF Pro Display", sans-serif;
    font-size: 16px;
}
QMainWindow { background: #080a0d; }
QScrollArea { background: #080a0d; border: 0; }
QScrollArea > QWidget > QWidget { background: #080a0d; }
QTabWidget::pane { border: 0; background: #080a0d; }
QTabBar { background: #080a0d; }
QTabBar::tab {
    background: transparent;
    color: #727983;
    border: 0;
    border-bottom: 2px solid #20252d;
    min-width: 160px;
    min-height: 52px;
    padding: 0 12px;
    font-weight: 600;
}
QTabBar::tab:selected { color: #ffffff; border-bottom: 2px solid #4da3ff; }
QTabBar::tab:hover:!selected { color: #bcc3cc; }
QGroupBox {
    background: #111419;
    border: 1px solid #252a33;
    border-radius: 12px;
    margin-top: 18px;
    padding: 0;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 16px;
    padding: 0 8px;
    color: #f5f7fa;
}
QLabel { background: transparent; }
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
    background: #191d23;
    border: 1px solid #303640;
    border-radius: 8px;
    min-height: 42px;
    padding: 0 10px;
    selection-background-color: #2f80ed;
}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {
    border: 1px solid #4da3ff;
}
QComboBox::drop-down { border: 0; width: 28px; }
QPushButton {
    background: #1b2027;
    color: #f5f7fa;
    border: 1px solid #343b46;
    border-radius: 8px;
    min-height: 46px;
    padding: 0 16px;
    font-weight: 600;
}
QPushButton:hover { background: #242a33; border-color: #4a5361; }
QPushButton:pressed { background: #15191e; }
QPushButton:disabled { color: #59616d; background: #111419; border-color: #20252d; }
QPushButton[accent="true"], QPushButton[role="primary"] {
    background: #2f80ed;
    border-color: #2f80ed;
    color: white;
}
QPushButton[accent="true"]:hover, QPushButton[role="primary"]:hover {
    background: #4094ff;
    border-color: #4094ff;
}
QPushButton[role="danger"] { background: #35171b; color: #ffb8bf; border-color: #713039; }
QPushButton[role="danger"]:hover { background: #512027; border-color: #a04450; }
QToolButton {
    background: transparent;
    color: #929aa5;
    border: 0;
    min-height: 34px;
    padding: 0 4px;
    text-align: left;
    font-weight: 600;
}
QToolButton:hover { color: #ffffff; }
QTableWidget, QTextEdit {
    background: #0d1014;
    border: 1px solid #252a33;
    border-radius: 10px;
    alternate-background-color: #11151a;
    outline: 0;
}
QHeaderView::section {
    background: #15191f;
    color: #aeb6c1;
    border: 0;
    border-bottom: 1px solid #303640;
    min-height: 42px;
    padding: 0 8px;
    font-weight: 600;
}
QTableWidget::item { border-bottom: 1px solid #20252d; padding: 8px; }
QTableWidget::item:selected { background: #17365c; }
QScrollBar:vertical { background: transparent; width: 10px; margin: 2px; }
QScrollBar::handle:vertical { background: #343b46; min-height: 32px; border-radius: 5px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QToolTip { background: #eef3f8; color: #111419; border: 0; padding: 6px; }
"""

IMU_FRESHNESS_TIMEOUT_S = 0.5

DEVICE_PRESETS = {
    # The combined MindRove stream begins with a constant package/status
    # channel. Its eight EMG channels are columns 1-8; columns 9-14 are IMU.
    "MindRove 8 + IMU": (
        "1-8", "9-11", "12-14", "EEG", "MindRoveStream", "EEG", "MindRoveStream"
    ),
    # The playback application publishes already-split streams whose channel
    # numbering starts at zero within each outlet.
    "MindRove XDF playback": (
        "0-7", "0-2", "3-5", "EMG", "MindRove_EMG", "IMU", "MindRove_IMU"
    ),
    "8-channel EMG": ("0-7", "", "", "EMG", "", "IMU", ""),
    "128-channel HD-EMG": ("0-127", "", "", "EMG", "", "IMU", ""),
}


class XdfSessionImportWorker(QThread):
    progress_changed = pyqtSignal(int, int, str)
    session_ready = pyqtSignal(object, object, str)
    import_failed = pyqtSignal(str)

    def __init__(self, paths: list[str], participant_id: str, output_path: str):
        super().__init__()
        self._paths = list(paths)
        self._participant_id = participant_id
        self._output_path = output_path

    def run(self):
        try:
            session, summary = import_xdf_session(
                self._paths,
                participant_id=self._participant_id,
                progress=self._report_progress,
            )
            if self.isInterruptionRequested():
                return
            session.save(self._output_path)
            self.session_ready.emit(session, summary, self._output_path)
        except Exception as exc:
            if not self.isInterruptionRequested():
                self.import_failed.emit(str(exc))

    def _report_progress(self, index: int, total: int, path: Path):
        if self.isInterruptionRequested():
            raise RuntimeError("XDF import cancelled")
        self.progress_changed.emit(index, total, path.name)


class EmgIntentDecoderWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NML EMG Intent Discovery")
        self.resize(1280, 880)
        self.setMinimumSize(1024, 720)
        self._worker: EmgStreamWorker | None = None
        self._lsl_scan_worker: LslScanWorker | None = None
        self._xdf_import_worker: XdfSessionImportWorker | None = None
        self._buffer = ChunkBuffer(5000)
        self._stream_meta: dict[str, object] = {}
        self._imu_worker: EmgStreamWorker | None = None
        self._imu_buffer = ChunkBuffer(1000)
        self._imu_meta: dict[str, object] = {}
        self._session = IntentCaptureSession()
        self._rankings = []
        self._pipeline: IntentDecoderPipeline | None = None
        self._output_stabilizer = IntentOutputStabilizer()
        self._outlet = None
        self._test_signal_active = False
        self._test_signal_started_monotonic = 0.0
        self._last_chunk_monotonic = 0.0
        self._last_imu_chunk_monotonic = 0.0

        self._build_ui()
        QTimer.singleShot(0, self._refresh_tab_heights)
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)

        header = QWidget()
        header.setStyleSheet("background:#080a0d;border-bottom:1px solid #252a33;")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(26, 14, 26, 14)
        title_stack = QVBoxLayout()
        overline = QLabel("NML  /  HUMAN-MACHINE INTERFACE")
        overline.setStyleSheet(
            "color:#8a929d;font-size:12px;font-weight:700;letter-spacing:2px;"
        )
        title = QLabel("Intent Discovery")
        title.setStyleSheet("font-size:29px;font-weight:600;letter-spacing:0.3px;")
        subtitle = QLabel("Find a reliable two-direction control signal for this participant")
        subtitle.setStyleSheet("color:#a5adb8;font-size:15px;")
        title_stack.addWidget(overline)
        title_stack.addWidget(title)
        title_stack.addWidget(subtitle)
        header_layout.addLayout(title_stack)
        header_layout.addStretch()
        self.workflow_status = QLabel("SETUP")
        self.workflow_status.setAlignment(Qt.AlignCenter)
        self.workflow_status.setMinimumWidth(112)
        self.workflow_status.setMaximumHeight(36)
        self.workflow_status.setStyleSheet(
            "color:#9bc8ff;background:#112840;border:1px solid #235286;"
            "border-radius:15px;padding:7px 14px;font-size:11px;font-weight:700;"
        )
        header_layout.addWidget(self.workflow_status)
        outer.addWidget(header)

        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.currentChanged.connect(self._on_tab_changed)
        outer.addWidget(self.tabs)
        self._build_setup_tab()
        self._build_discovery_tab()
        self._build_selection_tab()
        self._build_run_tab()

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(120)
        self.log.setVisible(False)
        self.log_toggle = QToolButton()
        self.log_toggle.setText("Show session activity")
        self.log_toggle.setCheckable(True)
        self.log_toggle.setArrowType(Qt.RightArrow)
        self.log_toggle.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.log_toggle.toggled.connect(self._toggle_log)
        outer.addWidget(self.log_toggle)
        outer.addWidget(self.log)

    def _tab(self, title: str):
        page = QWidget()
        page_layout = QVBoxLayout(page)
        page_layout.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(18, 14, 18, 14)
        layout.setSpacing(10)
        scroll.setWidget(widget)
        page_layout.addWidget(scroll)
        if not hasattr(self, "_tab_scrolls"):
            self._tab_scrolls = []
            self._tab_contents = []
            self._tab_layouts = []
        self._tab_scrolls.append(scroll)
        self._tab_contents.append(widget)
        self._tab_layouts.append(layout)
        self.tabs.addTab(page, title)
        return layout

    def _on_tab_changed(self, index: int):
        labels = ("SETUP", "SESSION", "VALIDATE", "MONITOR")
        if 0 <= index < len(labels):
            self.workflow_status.setText(labels[index])

    def _toggle_log(self, visible: bool):
        self.log.setVisible(visible)
        self.log_toggle.setArrowType(Qt.DownArrow if visible else Qt.RightArrow)
        self.log_toggle.setText("Hide session activity" if visible else "Show session activity")

    def _toggle_mapping(self, visible: bool):
        self.mapping_panel.setVisible(visible)
        self.mapping_toggle.setArrowType(Qt.DownArrow if visible else Qt.RightArrow)
        self.mapping_panel.updateGeometry()
        self.participant_group.updateGeometry()
        QTimer.singleShot(0, lambda: self._after_mapping_toggle(visible))

    def _after_mapping_toggle(self, visible: bool):
        self._refresh_tab_heights()
        if visible and self._tab_scrolls:
            self._tab_scrolls[0].ensureWidgetVisible(self.mapping_panel, 20, 20)

    def _refresh_tab_heights(self):
        if not hasattr(self, "_tab_contents"):
            return
        for content, layout in zip(self._tab_contents, self._tab_layouts):
            content.setMinimumWidth(0)
            content.setMinimumHeight(layout.minimumSize().height())

    @staticmethod
    def _refresh_style(widget: QWidget):
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def _build_setup_tab(self):
        layout = self._tab("1. Setup")
        intro = QLabel("Connect the participant's signals")
        intro.setStyleSheet("font-size:25px;font-weight:600;margin-top:6px;")
        helper = QLabel("Choose the device, connect EMG, then optionally add orientation compensation.")
        helper.setStyleSheet("color:#a5adb8;font-size:16px;")
        layout.addWidget(intro)
        layout.addWidget(helper)

        participant = QGroupBox("Participant")
        self.participant_group = participant
        grid = QGridLayout(participant)
        grid.setSizeConstraint(QLayout.SetMinimumSize)
        grid.setContentsMargins(20, 28, 20, 20)
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(16)
        grid.setColumnMinimumWidth(0, 132)
        self.participant_edit = QLineEdit()
        self.participant_edit.setPlaceholderText("Participant or session identifier")
        self.device_combo = QComboBox()
        self.device_combo.addItems([
            "MindRove 8 + IMU",
            "MindRove XDF playback",
            "8-channel EMG",
            "128-channel HD-EMG",
            "Custom",
        ])
        self.device_combo.currentTextChanged.connect(self._apply_device_preset)
        self.emg_channels_edit = QLineEdit("1-8")
        self.accel_channels_edit = QLineEdit("9-11")
        self.gyro_channels_edit = QLineEdit("12-14")
        grid.addWidget(QLabel("Participant"), 0, 0)
        grid.addWidget(self.participant_edit, 0, 1, 1, 3)
        grid.addWidget(QLabel("Device"), 1, 0)
        grid.addWidget(self.device_combo, 1, 1, 1, 3)
        self.mapping_toggle = QToolButton()
        self.mapping_toggle.setText("Advanced channel mapping")
        self.mapping_toggle.setCheckable(True)
        self.mapping_toggle.setArrowType(Qt.RightArrow)
        self.mapping_toggle.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        grid.addWidget(self.mapping_toggle, 2, 0, 1, 4)
        self.mapping_panel = QWidget()
        mapping_grid = QGridLayout(self.mapping_panel)
        mapping_grid.setSizeConstraint(QLayout.SetMinimumSize)
        mapping_grid.setContentsMargins(0, 8, 0, 0)
        mapping_grid.setHorizontalSpacing(14)
        mapping_grid.setVerticalSpacing(14)
        mapping_grid.addWidget(QLabel("EMG channels"), 0, 0)
        mapping_grid.addWidget(self.emg_channels_edit, 0, 1)
        mapping_grid.addWidget(QLabel("Accelerometer channels"), 0, 2)
        mapping_grid.addWidget(self.accel_channels_edit, 0, 3)
        mapping_grid.addWidget(QLabel("Gyroscope channels"), 1, 0)
        mapping_grid.addWidget(self.gyro_channels_edit, 1, 1)
        self.mapping_panel.setVisible(False)
        self.mapping_toggle.toggled.connect(self._toggle_mapping)
        grid.addWidget(self.mapping_panel, 3, 0, 1, 4)
        layout.addWidget(participant)

        connection = QGroupBox("EMG LSL stream")
        connection_grid = QGridLayout(connection)
        connection_grid.setSizeConstraint(QLayout.SetMinimumSize)
        connection_grid.setContentsMargins(20, 28, 20, 20)
        connection_grid.setHorizontalSpacing(14)
        connection_grid.setVerticalSpacing(14)
        connection_grid.setColumnStretch(0, 1)
        connection_grid.setColumnStretch(1, 1)
        self.stream_type_edit = QLineEdit("EEG")
        self.stream_name_edit = QLineEdit("MindRoveStream")
        self.stream_type_edit.setMinimumWidth(0)
        self.stream_name_edit.setMinimumWidth(0)
        self.connect_btn = QPushButton("Connect EMG")
        self.connect_btn.setMaximumWidth(180)
        self.connect_btn.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.connect_btn.setProperty("accent", True)
        self.connect_btn.clicked.connect(self._toggle_connection)
        self.connection_status = QLabel("Not connected")
        self.connection_status.setWordWrap(True)
        self.connection_status.setMinimumWidth(0)
        self.quality_status = QLabel("Signal quality: waiting for stream")
        self.quality_status.setWordWrap(True)
        connection_grid.addWidget(QLabel("Stream type"), 0, 0)
        connection_grid.addWidget(QLabel("Stream name"), 0, 1)
        connection_grid.addWidget(self.stream_type_edit, 1, 0)
        connection_grid.addWidget(self.stream_name_edit, 1, 1)
        connection_grid.addWidget(self.connect_btn, 2, 0, Qt.AlignLeft)
        connection_grid.addWidget(self.connection_status, 2, 1)
        connection_grid.addWidget(self.quality_status, 3, 0, 1, 2)
        sources_row = QHBoxLayout()
        sources_row.setSpacing(14)
        sources_row.addWidget(connection, 1)

        orientation = QGroupBox("Orientation LSL stream (optional)")
        orientation_grid = QGridLayout(orientation)
        orientation_grid.setSizeConstraint(QLayout.SetMinimumSize)
        orientation_grid.setContentsMargins(20, 28, 20, 20)
        orientation_grid.setHorizontalSpacing(14)
        orientation_grid.setVerticalSpacing(14)
        orientation_grid.setColumnStretch(0, 1)
        orientation_grid.setColumnStretch(1, 1)
        self.imu_stream_type_edit = QLineEdit("IMU")
        self.imu_stream_name_edit = QLineEdit()
        self.imu_stream_type_edit.setMinimumWidth(0)
        self.imu_stream_name_edit.setMinimumWidth(0)
        self.imu_stream_name_edit.setPlaceholderText("Optional exact stream name")
        self.imu_connect_btn = QPushButton("Connect IMU")
        self.imu_connect_btn.setMaximumWidth(160)
        self.imu_connect_btn.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.imu_connect_btn.clicked.connect(self._toggle_imu_connection)
        self.imu_connection_status = QLabel("Not connected - using global baseline")
        self.imu_connection_status.setWordWrap(True)
        self.imu_connection_status.setMinimumWidth(0)
        self.use_orientation_cb = QCheckBox(
            "Use live IMU orientation compensation"
        )
        self.use_orientation_cb.setChecked(False)
        self.use_orientation_cb.setToolTip(
            "Off (recommended without a live IMU): fit and run the decoder "
            "with a global EMG baseline. On: fit the orientation adapter and "
            "require fresh live IMU samples while decoding."
        )
        orientation_grid.addWidget(QLabel("Stream type"), 0, 0)
        orientation_grid.addWidget(QLabel("Stream name"), 0, 1)
        orientation_grid.addWidget(self.imu_stream_type_edit, 1, 0)
        orientation_grid.addWidget(self.imu_stream_name_edit, 1, 1)
        orientation_grid.addWidget(self.imu_connect_btn, 2, 0, Qt.AlignLeft)
        orientation_grid.addWidget(self.imu_connection_status, 2, 1)
        orientation_grid.addWidget(self.use_orientation_cb, 3, 0, 1, 2)
        sources_row.addWidget(orientation, 1)
        layout.addLayout(sources_row)

        discovery = QGroupBox("Available LSL streams")
        discovery_layout = QVBoxLayout(discovery)
        discovery_layout.setContentsMargins(20, 28, 20, 20)
        scan_row = QHBoxLayout()
        self.scan_lsl_btn = QPushButton("Scan LSL Streams")
        self.scan_lsl_btn.setMaximumWidth(190)
        self.scan_lsl_btn.clicked.connect(self._scan_lsl_streams)
        self.scan_lsl_status = QLabel("Not scanned")
        self.scan_lsl_status.setStyleSheet("color:#a5adb8;")
        scan_row.addWidget(self.scan_lsl_btn)
        scan_row.addWidget(self.scan_lsl_status)
        scan_row.addStretch()
        discovery_layout.addLayout(scan_row)

        self.lsl_stream_table = QTableWidget(0, 5)
        self.lsl_stream_table.setHorizontalHeaderLabels([
            "Name", "Type", "Channels", "Rate", "Source ID",
        ])
        self.lsl_stream_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.lsl_stream_table.verticalHeader().setVisible(False)
        self.lsl_stream_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.lsl_stream_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.lsl_stream_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.lsl_stream_table.setAlternatingRowColors(True)
        self.lsl_stream_table.setMinimumHeight(150)
        self.lsl_stream_table.setMaximumHeight(230)
        self.lsl_stream_table.cellDoubleClicked.connect(self._use_scanned_stream_automatically)
        discovery_layout.addWidget(self.lsl_stream_table)

        assign_row = QHBoxLayout()
        assign_row.addStretch()
        self.use_scan_emg_btn = QPushButton("Use Selected as EMG")
        self.use_scan_imu_btn = QPushButton("Use Selected as IMU")
        self.use_scan_emg_btn.clicked.connect(lambda: self._use_scanned_stream("emg"))
        self.use_scan_imu_btn.clicked.connect(lambda: self._use_scanned_stream("imu"))
        assign_row.addWidget(self.use_scan_emg_btn)
        assign_row.addWidget(self.use_scan_imu_btn)
        discovery_layout.addLayout(assign_row)
        layout.addWidget(discovery)

        note = QLabel("SAFE BY DESIGN  /  This app only publishes intent. It cannot arm or command the exoskeleton.")
        note.setWordWrap(True)
        note.setStyleSheet(
            "color:#aeb6c1;background:#111419;border:1px solid #252a33;"
            "border-radius:8px;padding:11px 14px;font-size:12px;font-weight:600;"
        )
        layout.addWidget(note)
        self.setup_continue_btn = QPushButton("Continue to session data")
        self.setup_continue_btn.setProperty("role", "primary")
        self.setup_continue_btn.setEnabled(True)
        self.setup_continue_btn.setToolTip("A live stream is optional until monitor and replay")
        self.setup_continue_btn.setMaximumWidth(290)
        self.setup_continue_btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.setup_continue_btn.clicked.connect(lambda: self.tabs.setCurrentIndex(1))
        continue_row = QHBoxLayout()
        continue_row.addStretch()
        continue_row.addWidget(self.setup_continue_btn)
        layout.addLayout(continue_row)
        layout.addStretch()

    def _build_discovery_tab(self):
        layout = self._tab("2. Session Data")
        intro = QLabel("Load recorded intent data")
        intro.setStyleSheet("font-size:25px;font-weight:600;margin-top:6px;")
        helper = QLabel(
            "Use a decoder session, or build one from the event-marked XDF files recorded by the Task GUI and LabRecorder."
        )
        helper.setStyleSheet("color:#a5adb8;font-size:16px;")
        helper.setWordWrap(True)
        layout.addWidget(intro)
        layout.addWidget(helper)

        source_box = QGroupBox("Session source")
        source_layout = QVBoxLayout(source_box)
        source_layout.setContentsMargins(20, 28, 20, 20)
        source_help = QLabel(
            "A decoder NPZ is the processed, reloadable session. If you only have recordings, select the folder containing the Task GUI's XDF files and this app will create the NPZ."
        )
        source_help.setWordWrap(True)
        source_help.setStyleSheet("color:#a5adb8;")
        source_layout.addWidget(source_help)
        source_actions = QHBoxLayout()
        self.load_session_btn = QPushButton("Load Decoder Session (.npz)")
        self.load_session_btn.setProperty("accent", True)
        self.load_session_btn.clicked.connect(self._load_session)
        self.import_xdf_btn = QPushButton("Build Session from XDF Folder")
        self.import_xdf_btn.clicked.connect(self._import_xdf_folder)
        source_actions.addWidget(self.load_session_btn)
        source_actions.addWidget(self.import_xdf_btn)
        source_actions.addStretch()
        source_layout.addLayout(source_actions)
        self.session_source_status = QLabel("No decoder session loaded")
        self.session_source_status.setWordWrap(True)
        self.session_source_status.setStyleSheet("color:#a5adb8;")
        source_layout.addWidget(self.session_source_status)
        self.xdf_import_progress = QProgressBar()
        self.xdf_import_progress.setRange(0, 1)
        self.xdf_import_progress.setValue(0)
        self.xdf_import_progress.setVisible(False)
        source_layout.addWidget(self.xdf_import_progress)
        layout.addWidget(source_box)

        contents_box = QGroupBox("Session contents")
        contents_layout = QVBoxLayout(contents_box)
        contents_layout.setContentsMargins(20, 28, 20, 20)
        self.counts_label = QLabel("No samples captured")
        self.counts_label.setWordWrap(True)
        contents_layout.addWidget(self.counts_label)
        self.session_contents_table = QTableWidget(0, 3)
        self.session_contents_table.setHorizontalHeaderLabels(["Intent", "Windows", "Repetitions"])
        self.session_contents_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.session_contents_table.verticalHeader().setVisible(False)
        self.session_contents_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.session_contents_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.session_contents_table.setMaximumHeight(300)
        contents_layout.addWidget(self.session_contents_table)
        layout.addWidget(contents_box)

        self.session_continue_btn = QPushButton("Continue to pair selection")
        self.session_continue_btn.setProperty("role", "primary")
        self.session_continue_btn.setEnabled(False)
        self.session_continue_btn.setToolTip(
            "Requires at least two rest repetitions and two candidate intents with two repetitions each"
        )
        self.session_continue_btn.setMaximumWidth(290)
        self.session_continue_btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.session_continue_btn.clicked.connect(lambda: self.tabs.setCurrentIndex(2))
        continue_row = QHBoxLayout()
        continue_row.addStretch()
        continue_row.addWidget(self.session_continue_btn)
        layout.addLayout(continue_row)

    def _build_selection_tab(self):
        layout = self._tab("3. Select and Validate")
        intro = QLabel("Choose the most reliable control pair")
        intro.setStyleSheet("font-size:25px;font-weight:600;margin-top:6px;")
        helper = QLabel(
            "Ranking holds out complete recordings and penalizes unintended activation."
        )
        helper.setStyleSheet("color:#a5adb8;font-size:16px;")
        layout.addWidget(intro)
        layout.addWidget(helper)
        controls = QHBoxLayout()
        self.rank_btn = QPushButton("Rank Intent Pairs")
        self.rank_btn.setProperty("accent", True)
        self.rank_btn.clicked.connect(self._rank_pairs)
        self.folds_spin = QSpinBox()
        self.folds_spin.setRange(2, 10)
        self.folds_spin.setValue(5)
        controls.addWidget(self.rank_btn)
        controls.addWidget(QLabel("Recording folds"))
        controls.addWidget(self.folds_spin)
        controls.addStretch()
        layout.addLayout(controls)
        self.ranking_table = QTableWidget(0, 7)
        self.ranking_table.setHorizontalHeaderLabels([
            "Intent pair", "Balanced accuracy", "Rest false activation",
            "Reject false activation", "Direction confusion", "Stability", "Score",
        ])
        self.ranking_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.ranking_table.verticalHeader().setVisible(False)
        self.ranking_table.verticalHeader().setDefaultSectionSize(44)
        self.ranking_table.setAlternatingRowColors(True)
        self.ranking_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.ranking_table.itemSelectionChanged.connect(self._ranking_selected)
        layout.addWidget(self.ranking_table)
        mapping = QGroupBox("Control mapping")
        mapping_grid = QGridLayout(mapping)
        mapping_grid.setSizeConstraint(QLayout.SetMinimumSize)
        mapping_grid.setContentsMargins(20, 28, 20, 20)
        mapping_grid.setHorizontalSpacing(14)
        mapping_grid.setVerticalSpacing(14)
        self.open_combo = QComboBox()
        self.close_combo = QComboBox()
        self.mapping_confirm_cb = QCheckBox(
            "I verified that these gestures mean physical hand OPEN and CLOSE"
        )
        self.open_combo.setToolTip(
            "Any captured intent may be assigned to the decoder's -1 / OPEN output."
        )
        self.close_combo.setToolTip(
            "Any captured intent may be assigned to the decoder's +1 / CLOSE output."
        )
        self.fit_btn = QPushButton("Fit selected intent mapping")
        self.fit_btn.setProperty("role", "primary")
        self.fit_btn.setMaximumWidth(330)
        self.fit_btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.fit_btn.clicked.connect(self._fit_selected)
        self.open_combo.currentIndexChanged.connect(self._mapping_changed)
        self.close_combo.currentIndexChanged.connect(self._mapping_changed)
        mapping_grid.addWidget(QLabel("Intent mapped to OPEN (-1)"), 0, 0)
        mapping_grid.addWidget(self.open_combo, 0, 1)
        mapping_grid.addWidget(QLabel("Intent mapped to CLOSE (+1)"), 0, 2)
        mapping_grid.addWidget(self.close_combo, 0, 3)
        mapping_grid.addWidget(self.mapping_confirm_cb, 1, 0, 1, 3)
        mapping_grid.addWidget(self.fit_btn, 1, 3, Qt.AlignRight)
        layout.addWidget(mapping)

    def _build_run_tab(self):
        layout = self._tab("4. Monitor and Run")
        intro = QLabel("Confirm behavior before going live")
        intro.setStyleSheet("font-size:25px;font-weight:600;margin-top:6px;")
        helper = QLabel("Monitor the decoded state first. Publishing remains a separate, explicit action.")
        helper.setStyleSheet("color:#a5adb8;font-size:16px;")
        layout.addWidget(intro)
        layout.addWidget(helper)
        self.state_label = QLabel("MONITOR - NO MODEL")
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("font-size:42px;font-weight:bold;color:#777;padding:30px;")
        layout.addWidget(self.state_label)
        self.detail_label = QLabel("Fit a validated participant-specific model before publishing.")
        self.detail_label.setAlignment(Qt.AlignCenter)
        self.detail_label.setWordWrap(True)
        layout.addWidget(self.detail_label)

        projection_box = QGroupBox("Continuous LDA projection")
        projection_layout = QVBoxLayout(projection_box)
        projection_layout.setContentsMargins(20, 28, 20, 20)
        self.projection_plot = pg.PlotWidget()
        self.projection_plot.setBackground("#0d1014")
        self.projection_plot.showGrid(x=True, y=False, alpha=0.22)
        self.projection_plot.setLabel(
            "bottom", "Normalized intent axis (-open MVC ... rest ... +close MVC)"
        )
        self.projection_plot.setXRange(-1.08, 1.08, padding=0.0)
        self.projection_plot.setYRange(-1.55, 1.55, padding=0.0)
        self.projection_plot.setMinimumHeight(250)
        self.projection_plot.getAxis("left").setTicks(
            [[(-1.0, "open"), (0.0, "rest"), (1.0, "close")]]
        )
        for y_value in (-1.0, 0.0, 1.0):
            self.projection_plot.addLine(
                y=y_value, pen=pg.mkPen("#303640", style=Qt.DashLine)
            )
        for x_value, color in ((-1.0, "#4da3ff"), (0.0, "#7f8792"), (1.0, "#cf102d")):
            self.projection_plot.addLine(
                x=x_value, pen=pg.mkPen(color, width=1, style=Qt.DashLine)
            )
        self._projection_y = {"open": -1.0, "rest": 0.0, "close": 1.0}
        self._projection_scatter = {}
        self._projection_medians = {}
        for role, color in (
            ("open", "#4da3ff"),
            ("rest", "#9aa1aa"),
            ("close", "#cf102d"),
        ):
            scatter = pg.ScatterPlotItem(
                pen=pg.mkPen(None), brush=pg.mkBrush(color + "78"), size=6
            )
            median = pg.ScatterPlotItem(
                pen=pg.mkPen("#ffffff", width=1),
                brush=pg.mkBrush(color),
                size=15,
                symbol="d",
            )
            self.projection_plot.addItem(scatter)
            self.projection_plot.addItem(median)
            self._projection_scatter[role] = scatter
            self._projection_medians[role] = median
        self._projection_live_line = pg.InfiniteLine(
            pos=0.0, angle=90, pen=pg.mkPen("#ffd166", width=3)
        )
        self._projection_live_marker = pg.ScatterPlotItem(
            pen=pg.mkPen("#ffffff", width=2),
            brush=pg.mkBrush("#ffd166"),
            size=18,
            symbol="o",
        )
        self.projection_plot.addItem(self._projection_live_line)
        self.projection_plot.addItem(self._projection_live_marker)
        projection_layout.addWidget(self.projection_plot)
        self.projection_value_label = QLabel(
            "Fit a model to populate the projection axis. The yellow marker is the live output."
        )
        self.projection_value_label.setAlignment(Qt.AlignCenter)
        self.projection_value_label.setWordWrap(True)
        self.projection_value_label.setStyleSheet(
            "color:#d4d8de;background:#15191f;border-radius:7px;padding:9px;"
        )
        projection_layout.addWidget(self.projection_value_label)
        layout.addWidget(projection_box)

        test_box = QGroupBox("Synthetic intent test")
        test_layout = QGridLayout(test_box)
        test_layout.setContentsMargins(20, 28, 20, 20)
        self.test_amplitude_spin = QDoubleSpinBox()
        self.test_amplitude_spin.setRange(0.05, 1.0)
        self.test_amplitude_spin.setDecimals(2)
        self.test_amplitude_spin.setSingleStep(0.05)
        self.test_amplitude_spin.setValue(0.25)
        self.test_amplitude_spin.setToolTip(
            "Peak normalized intent. Start at 0.25 with a low exo velocity limit."
        )
        self.test_period_spin = QDoubleSpinBox()
        self.test_period_spin.setRange(4.0, 60.0)
        self.test_period_spin.setDecimals(1)
        self.test_period_spin.setSingleStep(1.0)
        self.test_period_spin.setValue(10.0)
        self.test_period_spin.setSuffix(" s")
        self.test_period_spin.setToolTip(
            "Seconds per complete open-close cycle. The 4 s minimum prevents rapid reversals."
        )
        self.test_signal_btn = QPushButton("START SINE TEST")
        self.test_signal_btn.setProperty("role", "danger")
        self.test_signal_btn.setEnabled(False)
        self.test_signal_btn.clicked.connect(self._toggle_test_signal)
        test_note = QLabel(
            "Publishes a bounded sine wave through the normal NMLIntentV1 outlet. "
            "Fitting the mapping enables this control; starting it creates the outlet automatically. "
            "It does not bypass exo-side arming, per-motor limits, watchdogs, or STOP controls."
        )
        test_note.setWordWrap(True)
        test_note.setStyleSheet("color:#aeb6c1;")
        test_layout.addWidget(QLabel("Amplitude:"), 0, 0)
        test_layout.addWidget(self.test_amplitude_spin, 0, 1)
        test_layout.addWidget(QLabel("Cycle period:"), 0, 2)
        test_layout.addWidget(self.test_period_spin, 0, 3)
        test_layout.addWidget(self.test_signal_btn, 0, 4)
        test_layout.addWidget(test_note, 1, 0, 1, 5)
        test_layout.setColumnStretch(1, 1)
        test_layout.setColumnStretch(3, 1)
        layout.addWidget(test_box)
        publish_row = QHBoxLayout()
        self.publish_btn = QPushButton("Start Publishing NMLIntentV1")
        self.publish_btn.setProperty("role", "primary")
        self.publish_btn.setMaximumWidth(300)
        self.publish_btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.publish_btn.setEnabled(False)
        self.publish_btn.clicked.connect(self._toggle_publish)
        self.stop_btn = QPushButton("STOP OUTPUT")
        self.stop_btn.setProperty("role", "danger")
        self.stop_btn.setMaximumWidth(180)
        self.stop_btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.stop_btn.clicked.connect(self._stop_publish)
        publish_row.addStretch()
        publish_row.addWidget(self.publish_btn)
        publish_row.addWidget(self.stop_btn)
        layout.addLayout(publish_row)
        safety = QLabel(
            "Global-baseline mode needs only EMG. If orientation compensation "
            "is explicitly enabled, fresh IMU is required and missing IMU "
            "publishes zero. Uncertain predictions, stale EMG, and explicit "
            "stop also publish zero. Exo-side safety remains independent."
        )
        safety.setWordWrap(True)
        safety.setStyleSheet(
            "color:#aeb6c1;background:#111419;border:1px solid #252a33;"
            "border-radius:8px;padding:11px 14px;"
        )
        layout.addWidget(safety)
        layout.addStretch()

    def _apply_device_preset(self, name: str):
        if name in DEVICE_PRESETS:
            emg, accel, gyro, stream_type, stream_name, imu_type, imu_name = DEVICE_PRESETS[name]
            self.emg_channels_edit.setText(emg)
            self.accel_channels_edit.setText(accel)
            self.gyro_channels_edit.setText(gyro)
            self.stream_type_edit.setText(stream_type)
            self.stream_name_edit.setText(stream_name)
            self.imu_stream_type_edit.setText(imu_type)
            self.imu_stream_name_edit.setText(imu_name)

    def _layout(self, channel_count: int) -> StreamLayout:
        return StreamLayout.from_specs(
            channel_count,
            self.emg_channels_edit.text(),
        )

    def _scan_lsl_streams(self):
        if self._lsl_scan_worker is not None and self._lsl_scan_worker.isRunning():
            return
        self.scan_lsl_btn.setEnabled(False)
        self.scan_lsl_status.setText("Scanning for LSL streams...")
        self.scan_lsl_status.setStyleSheet("color:#f39c12;")
        self.lsl_stream_table.setRowCount(0)
        worker = LslScanWorker(wait_time=1.0)
        self._lsl_scan_worker = worker
        worker.results_ready.connect(self._on_lsl_scan_results)
        worker.scan_failed.connect(self._on_lsl_scan_failed)
        worker.finished.connect(self._on_lsl_scan_finished)
        worker.start()

    def _on_lsl_scan_results(self, records: object):
        streams = list(records)
        self.lsl_stream_table.setRowCount(len(streams))
        for row, record in enumerate(streams):
            rate = float(record.get("sample_rate", 0.0))
            values = (
                str(record.get("name", "")),
                str(record.get("type", "")),
                str(record.get("channel_count", "")),
                "irregular" if rate <= 0.0 else f"{rate:g} Hz",
                str(record.get("source_id", "")),
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if column == 0:
                    item.setData(Qt.UserRole, dict(record))
                self.lsl_stream_table.setItem(row, column, item)
        if streams:
            self.lsl_stream_table.selectRow(0)
            self.scan_lsl_status.setText(f"Found {len(streams)} LSL stream(s)")
            self.scan_lsl_status.setStyleSheet("color:#27ae60;")
        else:
            self.scan_lsl_status.setText("No LSL streams found")
            self.scan_lsl_status.setStyleSheet("color:#c0392b;")
        self._log(f"LSL scan found {len(streams)} stream(s)")

    def _on_lsl_scan_failed(self, message: str):
        self.scan_lsl_status.setText(f"Scan failed: {message}")
        self.scan_lsl_status.setStyleSheet("color:#c0392b;")
        self._log(f"LSL scan failed: {message}")

    def _on_lsl_scan_finished(self):
        self.scan_lsl_btn.setEnabled(True)
        worker = self._lsl_scan_worker
        self._lsl_scan_worker = None
        if worker is not None:
            worker.deleteLater()

    def _selected_scanned_stream(self) -> dict | None:
        row = self.lsl_stream_table.currentRow()
        if row < 0:
            QMessageBox.information(self, "Select a stream", "Select an LSL stream first.")
            return None
        item = self.lsl_stream_table.item(row, 0)
        record = item.data(Qt.UserRole) if item is not None else None
        return dict(record) if isinstance(record, dict) else None

    def _use_scanned_stream(self, role: str):
        record = self._selected_scanned_stream()
        if record is None:
            return
        stream_type = str(record.get("type", ""))
        stream_name = str(record.get("name", ""))
        if role == "imu":
            self.imu_stream_type_edit.setText(stream_type or "IMU")
            self.imu_stream_name_edit.setText(stream_name)
            self._log(f"Selected LSL IMU stream: {stream_name} ({stream_type})")
        else:
            self.stream_type_edit.setText(stream_type or "EMG")
            self.stream_name_edit.setText(stream_name)
            self._log(f"Selected LSL EMG stream: {stream_name} ({stream_type})")

    def _use_scanned_stream_automatically(self, row: int, _column: int):
        self.lsl_stream_table.selectRow(row)
        item = self.lsl_stream_table.item(row, 0)
        record = item.data(Qt.UserRole) if item is not None else {}
        stream_type = str(record.get("type", "")).strip().lower() if isinstance(record, dict) else ""
        self._use_scanned_stream("imu" if stream_type == "imu" else "emg")

    def _toggle_connection(self):
        if self._worker is not None:
            self._disconnect()
            return
        stream_type = self.stream_type_edit.text().strip() or "EMG"
        stream_name = self.stream_name_edit.text().strip()
        self._worker = EmgStreamWorker(stream_type, stream_name)
        self._worker.status_changed.connect(self._on_stream_status)
        self._worker.stream_ready.connect(self._on_stream_ready)
        self._worker.chunk_received.connect(self._on_chunk)
        self._worker.start()
        self.connect_btn.setText("Disconnect EMG")
        self.connect_btn.setProperty("accent", False)
        self._refresh_style(self.connect_btn)

    def _disconnect(self):
        worker = self._worker
        if worker is not None:
            worker.stop()
            worker.wait(2000)
        self._worker = None
        self.connect_btn.setText("Connect EMG")
        self.connect_btn.setProperty("accent", True)
        self._refresh_style(self.connect_btn)
        self.connection_status.setText("Not connected")
        self._stream_meta = {}
        self._last_chunk_monotonic = 0.0
        self._output_stabilizer.reset()
        self._stop_publish()

    def _toggle_imu_connection(self):
        if self._imu_worker is not None:
            self._disconnect_imu()
            return
        stream_type = self.imu_stream_type_edit.text().strip() or "IMU"
        stream_name = self.imu_stream_name_edit.text().strip()
        self._imu_worker = EmgStreamWorker(stream_type, stream_name)
        self._imu_worker.status_changed.connect(self._on_imu_status)
        self._imu_worker.stream_ready.connect(self._on_imu_stream_ready)
        self._imu_worker.chunk_received.connect(self._on_imu_chunk)
        self._imu_worker.start()
        self.imu_connect_btn.setText("Disconnect IMU")

    def _disconnect_imu(self):
        worker = self._imu_worker
        if worker is not None:
            worker.stop()
            worker.wait(2000)
        self._imu_worker = None
        self._imu_meta = {}
        self._imu_buffer.clear()
        self._last_imu_chunk_monotonic = 0.0
        self.imu_connect_btn.setText("Connect IMU")
        self.imu_connection_status.setText("Not connected - using global baseline")

    def _on_stream_status(self, message: str, color: str):
        self.connection_status.setText(message)
        self.connection_status.setStyleSheet(f"color:{color};")
        self._log(message)

    def _on_stream_ready(self, info: object):
        self._stream_meta = dict(info)
        count = int(self._stream_meta.get("channel_count", 0))
        try:
            self._layout(count)
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid channel layout", str(exc))
            self._disconnect()
            return
        fs = int(self._stream_meta.get("sample_rate", 500))
        self._buffer.set_max_samples(max(1, fs * 8))
        self._session.device_name = self.device_combo.currentText()
        self._session.channel_count = len(self._layout(count).emg_channels)
        self.setup_continue_btn.setEnabled(True)
        self._log(f"Stream ready: {count} channels at {fs} Hz")

    def _on_imu_status(self, message: str, color: str):
        self.imu_connection_status.setText(message)
        self.imu_connection_status.setStyleSheet(f"color:{color};")
        self._log(f"IMU: {message}")

    def _on_imu_stream_ready(self, info: object):
        self._imu_meta = dict(info)
        count = int(self._imu_meta.get("channel_count", 0))
        try:
            accel = parse_channel_spec(self.accel_channels_edit.text(), count)
            gyro = parse_channel_spec(self.gyro_channels_edit.text(), count)
            if len(accel) != 3 or (gyro and len(gyro) != 3):
                raise ValueError("IMU requires three accelerometer and zero or three gyro channels")
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid IMU layout", str(exc))
            self._disconnect_imu()
            return
        fs = int(self._imu_meta.get("sample_rate", 100))
        self._imu_buffer.set_max_samples(max(1, fs * 4))
        self._log(f"IMU stream ready: {count} channels at {fs} Hz")

    def _on_chunk(self, data: object, timestamps: object):
        del timestamps
        chunk = np.asarray(data, dtype=np.float64)
        if chunk.ndim == 2 and chunk.shape[1] > 0:
            self._buffer.append(chunk)
            self._last_chunk_monotonic = time.monotonic()

    def _on_imu_chunk(self, data: object, timestamps: object):
        del timestamps
        chunk = np.asarray(data, dtype=np.float64)
        if chunk.ndim == 2 and chunk.shape[1] > 0:
            self._imu_buffer.append(chunk)
            self._last_imu_chunk_monotonic = time.monotonic()

    def _latest_orientation(self):
        if not EmgIntentDecoderWindow._imu_is_fresh(self):
            return orientation_from_accel(np.full(3, np.nan))
        snapshot = self._imu_buffer.snapshot()
        if snapshot is None or not self._imu_meta:
            return orientation_from_accel(np.full(3, np.nan))
        count = snapshot.shape[0]
        accel_indices = parse_channel_spec(self.accel_channels_edit.text(), count)
        gyro_indices = parse_channel_spec(self.gyro_channels_edit.text(), count)
        accel = np.mean(snapshot[list(accel_indices), -20:], axis=1)
        gyro = np.mean(snapshot[list(gyro_indices), -20:], axis=1) if gyro_indices else None
        return orientation_from_accel(accel, gyro)

    def _imu_is_fresh(self, now: float | None = None) -> bool:
        """Return whether a live IMU sample is available for optional adaptation."""
        if self._imu_worker is None or not self._imu_meta:
            return False
        last_sample = float(self._last_imu_chunk_monotonic)
        if last_sample <= 0.0:
            return False
        current = time.monotonic() if now is None else float(now)
        return 0.0 <= current - last_sample <= IMU_FRESHNESS_TIMEOUT_S

    def _latest_feature(self):
        snapshot = self._buffer.snapshot()
        if snapshot is None or not self._stream_meta:
            return None
        fs = int(self._stream_meta.get("sample_rate", 500))
        samples = max(8, int(round(0.25 * fs)))
        if snapshot.shape[1] < samples:
            return None
        window = snapshot[:, -samples:]
        layout = self._layout(window.shape[0])
        emg = layout.select_emg(window)
        processed = preprocess_emg(emg, PreprocessConfig(sample_rate_hz=fs))
        feature = extract_emg_features(processed, FeatureConfig(common_mode="median"))
        orientation = self._latest_orientation()
        return feature, orientation, emg

    def _tick(self):
        if self._test_signal_active:
            self._tick_test_signal()
            return
        if (
            self._worker is not None
            and self._last_chunk_monotonic > 0.0
            and time.monotonic() - self._last_chunk_monotonic > 0.5
        ):
            self.quality_status.setText("Signal stale - intent forced to zero")
            self._output_stabilizer.reset()
            self._show_zero_state("stale input stream")
            self._publish_zero()
            return
        if (
            self._pipeline is not None
            and self._pipeline.require_orientation
            and not EmgIntentDecoderWindow._imu_is_fresh(self)
        ):
            self.quality_status.setText(
                "IMU compensation enabled but IMU is unavailable - intent forced to zero"
            )
            self._output_stabilizer.reset()
            self._show_zero_state("fresh IMU required by selected decoder mode")
            self._publish_zero()
            return
        try:
            latest = self._latest_feature()
        except Exception as exc:
            self.quality_status.setText(f"Signal configuration error: {exc}")
            return
        if latest is None:
            return
        feature, orientation, emg = latest
        quality = assess_signal_quality(emg)
        bad = sorted(set(quality.flat_channels) | set(quality.noisy_channels) | set(quality.saturated_channels))
        self.quality_status.setText(
            f"Signal quality: {quality.usable_fraction * 100:.0f}% usable | "
            f"{emg.shape[0]} EMG channels | bad candidates: {bad if bad else 'none'} | "
            + (
                f"roll={orientation.roll_deg:.1f} deg (IMU adapted)"
                if orientation.roll_deg is not None
                else "orientation=global EMG baseline"
            )
        )
        if self._pipeline is not None:
            decision = self._pipeline.predict(feature, orientation)
            decision = self._output_stabilizer.update(
                decision,
                open_label=self._pipeline.open_label,
                close_label=self._pipeline.close_label,
                rest_label=self._pipeline.rest_label,
            )
            self._show_decision(decision)
            if self._outlet is not None:
                try:
                    active = 0.0 if decision.rejected or decision.signed_intent == 0 else 1.0
                    self._outlet.push_sample([
                        float(decision.signed_intent),
                        float(abs(decision.signed_intent)),
                        float(decision.confidence),
                        active,
                    ])
                except Exception as exc:
                    self._log(f"LSL publish failed: {exc}")
                    self._stop_publish()

    @staticmethod
    def _sine_test_value(elapsed_s: float, amplitude: float, period_s: float) -> float:
        period = max(4.0, float(period_s))
        bounded_amplitude = max(0.0, min(1.0, float(amplitude)))
        return float(
            bounded_amplitude
            * np.sin(2.0 * np.pi * max(0.0, float(elapsed_s)) / period)
        )

    def _toggle_test_signal(self):
        if self._test_signal_active:
            self._stop_test_signal()
            return
        if self._outlet is None:
            if self._pipeline is None:
                QMessageBox.information(
                    self,
                    "Fit the mapping first",
                    "Choose the open and close states and fit the decoder before enabling the sine test.",
                )
                return
            self._toggle_publish()
            if self._outlet is None:
                return
        self._test_signal_active = True
        self._test_signal_started_monotonic = time.monotonic()
        self.test_signal_btn.setText("STOP SINE TEST")
        self.workflow_status.setText("SYNTHETIC TEST")
        self._log(
            f"Synthetic sine test started: amplitude={self.test_amplitude_spin.value():.2f}, "
            f"period={self.test_period_spin.value():.1f} s"
        )

    def _tick_test_signal(self):
        if not self._test_signal_active or self._outlet is None:
            self._stop_test_signal(send_zero=False)
            return
        elapsed = time.monotonic() - self._test_signal_started_monotonic
        value = self._sine_test_value(
            elapsed,
            self.test_amplitude_spin.value(),
            self.test_period_spin.value(),
        )
        try:
            self._outlet.push_sample([value, abs(value), 1.0, 1.0])
        except Exception as exc:
            self._log(f"Synthetic sine publish failed: {exc}")
            self._stop_publish()
            return
        role = "close" if value > 0.0 else "open" if value < 0.0 else "rest"
        self._projection_live_line.setValue(value)
        self._projection_live_marker.setData(x=[value], y=[self._projection_y[role]])
        self.state_label.setText(f"PUBLISHING - SYNTHETIC {role.upper()}")
        self.state_label.setStyleSheet(
            "font-size:42px;font-weight:bold;color:#ffd166;padding:30px;"
        )
        self.detail_label.setText(
            f"Sine test: amplitude={self.test_amplitude_spin.value():.2f}, "
            f"period={self.test_period_spin.value():.1f} s, output={value:+.3f}"
        )
        self.projection_value_label.setText(
            f"SYNTHETIC TEST SIGNAL  |  sin(2πt/T) × "
            f"{self.test_amplitude_spin.value():.2f}  |  signed output={value:+.3f}"
        )

    def _stop_test_signal(self, *, send_zero: bool = True):
        was_active = self._test_signal_active
        self._test_signal_active = False
        self._test_signal_started_monotonic = 0.0
        if send_zero and self._outlet is not None:
            try:
                self._outlet.push_sample([0.0, 0.0, 1.0, 0.0])
            except Exception:
                pass
        if hasattr(self, "test_signal_btn"):
            self.test_signal_btn.setText("START SINE TEST")
        if hasattr(self, "_projection_live_line"):
            self._projection_live_line.setValue(0.0)
            self._projection_live_marker.setData(x=[0.0], y=[0.0])
        if was_active:
            self._log("Synthetic sine test stopped; zero intent published")
        if self._outlet is not None and hasattr(self, "workflow_status"):
            self.workflow_status.setText("LIVE OUTPUT")

    def _update_counts(self):
        counts = self._session.class_counts()
        groups_by_label: dict[str, set[str]] = {}
        for label, group in zip(self._session.labels, self._session.groups):
            groups_by_label.setdefault(label, set()).add(group)
        total = sum(counts.values())
        self.counts_label.setText(
            f"{total} windows across {len(counts)} classes"
            if counts else "No decoder session loaded"
        )
        self.session_contents_table.setRowCount(len(counts))
        for row, label in enumerate(sorted(counts)):
            values = (label, str(counts[label]), str(len(groups_by_label.get(label, set()))))
            for column, value in enumerate(values):
                self.session_contents_table.setItem(row, column, QTableWidgetItem(value))
        candidates = [
            label for label, groups in groups_by_label.items()
            if label not in {"rest", "reject"} and len(groups) >= 2
        ]
        self.session_continue_btn.setEnabled(
            len(groups_by_label.get("rest", set())) >= 2 and len(candidates) >= 2
        )

    def _rank_pairs(self):
        X, y, groups, roll, pitch = self._session.arrays()
        try:
            self._rankings = rank_intent_pairs(
                X,
                y,
                groups,
                roll,
                pitch,
                folds=int(self.folds_spin.value()),
                use_orientation=self.use_orientation_cb.isChecked(),
            )
        except Exception as exc:
            QMessageBox.warning(self, "Cannot rank intents", str(exc))
            return
        if not self._rankings:
            QMessageBox.information(
                self,
                "More repetitions needed",
                "Load rest and at least two candidate intents with two or more marked repetitions.",
            )
            return
        self.ranking_table.setRowCount(len(self._rankings))
        for row, result in enumerate(self._rankings):
            stability = max(0.0, 1.0 - 2.0 * result.balanced_accuracy_std)
            values = (
                f"{result.open_label} / {result.close_label}",
                f"{result.balanced_accuracy * 100:.1f}% +/- {result.balanced_accuracy_std * 100:.1f}",
                f"{result.rest_false_activation_rate * 100:.1f}%",
                f"{result.reject_false_activation_rate * 100:.1f}%",
                f"{result.direction_confusion_rate * 100:.1f}%",
                f"{stability * 100:.1f}%",
                f"{result.composite_score:.3f}",
            )
            for column, value in enumerate(values):
                self.ranking_table.setItem(row, column, QTableWidgetItem(value))
        self.ranking_table.selectRow(0)
        mode = "orientation compensated" if self.use_orientation_cb.isChecked() else "global EMG baseline"
        self._log(
            f"Ranked {len(self._rankings)} candidate pairs by held-out recording [{mode}]"
        )

    def _ranking_selected(self):
        row = self.ranking_table.currentRow()
        if row < 0 or row >= len(self._rankings):
            return
        result = self._rankings[row]
        suggested_open, suggested_close = self._semantic_mapping_suggestion(
            result.open_label, result.close_label
        )
        self._refresh_mapping_choices(suggested_open, suggested_close)

    @staticmethod
    def _semantic_mapping_suggestion(
        first: str, second: str
    ) -> tuple[str | None, str | None]:
        """Suggest physical semantics only when labels explicitly say open/close."""
        pair = (str(first), str(second))
        open_matches = [label for label in pair if "open" in label.lower()]
        close_matches = [label for label in pair if "close" in label.lower()]
        if len(open_matches) == 1 and len(close_matches) == 1:
            return open_matches[0], close_matches[0]
        return None, None

    def _mapping_changed(self, *_args):
        if hasattr(self, "mapping_confirm_cb"):
            self.mapping_confirm_cb.setChecked(False)

    def _refresh_mapping_choices(
        self,
        preferred_open: str | None = None,
        preferred_close: str | None = None,
    ):
        """Show every captured intent while optionally applying a ranked preset."""
        labels = sorted(set(self._session.labels) - {"rest", "reject"})
        if not labels:
            return
        inferred_open = next(
            (label for label in labels if "open" in label.lower()), None
        )
        inferred_close = next(
            (label for label in labels if "close" in label.lower()), None
        )
        current_open = preferred_open or inferred_open or self.open_combo.currentText()
        current_close = preferred_close or inferred_close or self.close_combo.currentText()
        for combo, selected in (
            (self.open_combo, current_open),
            (self.close_combo, current_close),
        ):
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(labels)
            if selected in labels:
                combo.setCurrentText(selected)
            combo.blockSignals(False)
        if self.open_combo.currentText() == self.close_combo.currentText() and len(labels) > 1:
            self.close_combo.setCurrentIndex(1)
        if hasattr(self, "mapping_confirm_cb"):
            self.mapping_confirm_cb.setChecked(False)

    def _fit_selected(self):
        open_label = self.open_combo.currentText()
        close_label = self.close_combo.currentText()
        if not open_label or not close_label or open_label == close_label:
            QMessageBox.warning(self, "Invalid mapping", "Open and close must use different intents.")
            return
        if not self.mapping_confirm_cb.isChecked():
            QMessageBox.warning(
                self,
                "Confirm physical mapping",
                "Verify which recorded gesture physically opens and closes the hand, "
                "then check the confirmation box before fitting.",
            )
            return
        X, y, groups, roll, pitch = self._session.arrays()
        del groups
        keep = np.isin(y, ["rest", "reject", open_label, close_label])
        use_orientation = self.use_orientation_cb.isChecked()
        if use_orientation and not np.any(
            np.isfinite(roll[keep]) & np.isfinite(pitch[keep])
        ):
            QMessageBox.warning(
                self,
                "No recorded orientation",
                "Orientation compensation was selected, but this session has no "
                "usable IMU orientation samples.",
            )
            return
        fit_roll, fit_pitch = EmgIntentDecoderWindow._decoder_orientation_arrays(
            roll[keep], pitch[keep], use_orientation
        )
        try:
            self._pipeline = EmgIntentDecoderWindow._make_runtime_pipeline(
                open_label, close_label, use_orientation
            ).fit(X[keep], y[keep], fit_roll, fit_pitch)
        except Exception as exc:
            QMessageBox.warning(self, "Fit failed", str(exc))
            return
        self._update_projection_training_plot()
        self._output_stabilizer.reset()
        self.publish_btn.setEnabled(True)
        self.test_signal_btn.setEnabled(True)
        self.tabs.setCurrentIndex(3)
        self.workflow_status.setText("MODEL READY")
        self.state_label.setText("MONITOR - REST")
        self.detail_label.setText(
            f"Open: {open_label} | Close: {close_label} | Publishing remains off"
        )
        self._log(
            f"Fitted continuous rest-to-MVC LDA decoder: "
            f"open={open_label}, close={close_label}; "
            + (
                "fresh live IMU required"
                if use_orientation
                else "global EMG baseline; no live IMU required"
            )
        )

    @staticmethod
    def _make_runtime_pipeline(
        open_label: str,
        close_label: str,
        use_orientation: bool = False,
    ) -> IntentDecoderPipeline:
        """Build a decoder whose train/runtime orientation modes match."""
        return IntentDecoderPipeline(
            open_label=open_label,
            close_label=close_label,
            require_orientation=bool(use_orientation),
        )

    @staticmethod
    def _decoder_orientation_arrays(
        roll: np.ndarray, pitch: np.ndarray, use_orientation: bool
    ) -> tuple[np.ndarray, np.ndarray]:
        if use_orientation:
            return (
                np.asarray(roll, dtype=np.float64),
                np.asarray(pitch, dtype=np.float64),
            )
        return (
            np.full(np.asarray(roll).shape, np.nan, dtype=np.float64),
            np.full(np.asarray(pitch).shape, np.nan, dtype=np.float64),
        )

    def _update_projection_training_plot(self):
        """Render fitted session windows on the normalized signed control axis."""
        if self._pipeline is None:
            return
        X, y, _groups, roll, pitch = self._session.arrays()
        labels_by_role = {
            "open": self._pipeline.open_label,
            "rest": self._pipeline.rest_label,
            "close": self._pipeline.close_label,
        }
        keep = np.isin(y, list(labels_by_role.values()))
        if not np.any(keep):
            return
        plot_roll, plot_pitch = EmgIntentDecoderWindow._decoder_orientation_arrays(
            roll[keep], pitch[keep], self._pipeline.require_orientation
        )
        projected = self._pipeline.project_continuous(
            X[keep], plot_roll, plot_pitch
        )["signed_intent"]
        plotted_labels = y[keep]
        rng = np.random.default_rng(0)
        summaries = []
        for role, label in labels_by_role.items():
            values = projected[plotted_labels == label]
            if len(values) == 0:
                self._projection_scatter[role].setData([], [])
                self._projection_medians[role].setData([], [])
                continue
            # Keep rendering cheap for HD-EMG sessions with many windows.
            if len(values) > 1200:
                indices = np.linspace(0, len(values) - 1, 1200, dtype=int)
                display_values = values[indices]
            else:
                display_values = values
            y_center = self._projection_y[role]
            jitter = rng.uniform(-0.25, 0.25, len(display_values))
            self._projection_scatter[role].setData(
                x=display_values, y=y_center + jitter
            )
            median = float(np.median(values))
            self._projection_medians[role].setData(x=[median], y=[y_center])
            summaries.append(f"{role} median {median:+.3f}")
        self._projection_live_line.setValue(0.0)
        self._projection_live_marker.setData(x=[0.0], y=[0.0])
        self.projection_value_label.setText(
            "Fitted distributions: " + "  |  ".join(summaries)
            + ". Yellow shows the live signed projection."
        )

    def _update_live_projection(self, decision):
        if not hasattr(self, "_projection_live_line"):
            return
        value = 0.0 if decision.rejected else float(decision.signed_intent)
        role = (
            "close" if value > 0.0 else "open" if value < 0.0 else "rest"
        )
        self._projection_live_line.setValue(value)
        self._projection_live_marker.setData(
            x=[value], y=[self._projection_y[role]]
        )
        open_probability = decision.probabilities.get(
            self._pipeline.open_label, 0.0
        )
        close_probability = decision.probabilities.get(
            self._pipeline.close_label, 0.0
        )
        direction = "REJECT" if decision.rejected else role.upper()
        self.projection_value_label.setText(
            f"LDA direction: {direction}  |  "
            f"P(open)={open_probability:.3f}, P(close)={close_probability:.3f}  |  "
            f"open activation={decision.open_activation:.3f}, "
            f"close activation={decision.close_activation:.3f}  |  "
            f"signed output={value:+.3f}"
        )

    def _show_decision(self, decision):
        self._update_live_projection(decision)
        if decision.rejected:
            text, color = "REJECT / ZERO", "#f1c40f"
        elif decision.state == self._pipeline.open_label:
            text, color = "OPEN", "#4da3ff"
        elif decision.state == self._pipeline.close_label:
            text, color = "CLOSE", "#27ae60"
        else:
            text, color = "REST", "#aaaaaa"
        prefix = "PUBLISHING" if self._outlet is not None else "MONITOR"
        self.state_label.setText(f"{prefix} - {text}")
        self.state_label.setStyleSheet(
            f"font-size:42px;font-weight:bold;color:{color};padding:30px;"
        )
        self.detail_label.setText(
            f"confidence={decision.confidence:.3f} | "
            f"signed intent={decision.signed_intent:+.3f} | "
            f"open activation={decision.open_activation:.3f} | "
            f"close activation={decision.close_activation:.3f}"
            + (f" | {decision.reason}" if decision.reason else "")
        )

    def _show_zero_state(self, reason: str):
        if self._pipeline is None:
            return
        prefix = "PUBLISHING" if self._outlet is not None else "MONITOR"
        self.state_label.setText(f"{prefix} - REJECT / ZERO")
        self.state_label.setStyleSheet(
            "font-size:42px;font-weight:bold;color:#f1c40f;padding:30px;"
        )
        self.detail_label.setText(reason)
        if hasattr(self, "_projection_live_line"):
            self._projection_live_line.setValue(0.0)
            self._projection_live_marker.setData(x=[0.0], y=[0.0])
            self.projection_value_label.setText(
                f"Signed output forced to +0.000: {reason}"
            )

    def _publish_zero(self):
        if self._outlet is None:
            return
        try:
            self._outlet.push_sample([0.0, 0.0, 0.0, 0.0])
        except Exception as exc:
            self._log(f"LSL zero-intent publish failed: {exc}")
            self._stop_publish()

    def _toggle_publish(self):
        if self._outlet is not None:
            self._stop_publish()
            return
        if self._pipeline is None:
            return
        try:
            from pylsl import StreamInfo, StreamOutlet
            info = StreamInfo(
                "NMLIntentV1", "NMLIntent", 4, 10, "float32",
                "nml-emg-centroid-intent-v1",
            )
            channels = info.desc().append_child("channels")
            for label in ("signed_intent", "effort", "confidence", "state_code"):
                channel = channels.append_child("channel")
                channel.append_child_value("label", label)
                channel.append_child_value("unit", "normalized")
            info.desc().append_child_value("schema", "nml.intent.v1")
            self._outlet = StreamOutlet(info)
        except Exception as exc:
            QMessageBox.warning(self, "Cannot publish", str(exc))
            return
        self.publish_btn.setText("Stop Publishing")
        self.publish_btn.setProperty("role", "danger")
        self._refresh_style(self.publish_btn)
        self.test_signal_btn.setEnabled(True)
        self.workflow_status.setText("LIVE OUTPUT")
        self._log("Publishing NMLIntentV1; exo-side arming remains independent")

    def _stop_publish(self):
        self._stop_test_signal(send_zero=True)
        self._output_stabilizer.reset()
        self._outlet = None
        if hasattr(self, "publish_btn"):
            self.publish_btn.setText("Start Publishing NMLIntentV1")
            self.publish_btn.setProperty("role", "primary")
            self._refresh_style(self.publish_btn)
        if hasattr(self, "test_signal_btn"):
            self.test_signal_btn.setEnabled(self._pipeline is not None)
        if hasattr(self, "workflow_status") and self._pipeline is not None:
            self.workflow_status.setText("MONITOR")
        self._log("Intent output stopped")

    def _load_session(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load intent session", "", "Intent session (*.npz)")
        if not path:
            return
        try:
            session = IntentCaptureSession.load(path)
        except Exception as exc:
            QMessageBox.warning(self, "Load failed", str(exc))
            return
        self._install_session(session, path)
        self.session_source_status.setText(f"Loaded decoder session: {path}")
        self.session_source_status.setStyleSheet("color:#27ae60;")
        self._log(f"Loaded session: {path}")

    def _import_xdf_folder(self):
        if self._xdf_import_worker is not None and self._xdf_import_worker.isRunning():
            return
        folder = QFileDialog.getExistingDirectory(self, "Choose folder containing event-marked XDF files")
        if not folder:
            return
        files = sorted(str(path) for path in Path(folder).glob("*.xdf"))
        if not files:
            QMessageBox.information(self, "No XDF recordings", "The selected folder contains no .xdf files.")
            return
        default_output = str(Path(folder) / f"{Path(folder).name}_intent_session.npz")
        output, _ = QFileDialog.getSaveFileName(
            self,
            "Save decoder session",
            default_output,
            "Intent session (*.npz)",
        )
        if not output:
            return
        if not output.lower().endswith(".npz"):
            output += ".npz"
        self.load_session_btn.setEnabled(False)
        self.import_xdf_btn.setEnabled(False)
        self.session_continue_btn.setEnabled(False)
        self.xdf_import_progress.setVisible(True)
        self.xdf_import_progress.setRange(0, len(files))
        self.xdf_import_progress.setValue(0)
        self.session_source_status.setText(f"Importing {len(files)} XDF recording(s)...")
        self.session_source_status.setStyleSheet("color:#f39c12;")
        worker = XdfSessionImportWorker(
            files,
            self.participant_edit.text().strip(),
            output,
        )
        self._xdf_import_worker = worker
        worker.progress_changed.connect(self._on_xdf_import_progress)
        worker.session_ready.connect(self._on_xdf_session_ready)
        worker.import_failed.connect(self._on_xdf_import_failed)
        worker.finished.connect(self._on_xdf_import_finished)
        worker.start()

    def _on_xdf_import_progress(self, index: int, total: int, filename: str):
        self.xdf_import_progress.setRange(0, total)
        self.xdf_import_progress.setValue(max(0, index - 1))
        self.session_source_status.setText(f"Importing {index}/{total}: {filename}")

    def _on_xdf_session_ready(self, session: object, summary: object, output_path: str):
        self._install_session(session, output_path)
        details = dict(summary)
        errors = list(details.get("errors", []))
        self.xdf_import_progress.setValue(self.xdf_import_progress.maximum())
        self.session_source_status.setText(
            f"Created {output_path} from {details.get('files', 0)} XDF files, "
            f"{details.get('trials', 0)} marked phases, and {details.get('windows', 0)} windows"
            + (f" ({len(errors)} file error(s))" if errors else "")
        )
        self.session_source_status.setStyleSheet("color:#27ae60;" if not errors else "color:#f39c12;")
        self._log(f"Created decoder session: {output_path}")

    def _on_xdf_import_failed(self, message: str):
        self.session_source_status.setText(f"XDF import failed: {message}")
        self.session_source_status.setStyleSheet("color:#c0392b;")
        QMessageBox.warning(self, "XDF import failed", message)

    def _on_xdf_import_finished(self):
        self.load_session_btn.setEnabled(True)
        self.import_xdf_btn.setEnabled(True)
        worker = self._xdf_import_worker
        self._xdf_import_worker = None
        if worker is not None:
            worker.deleteLater()

    def _install_session(self, session: IntentCaptureSession, source_path: str):
        self._stop_publish()
        self._pipeline = None
        self._rankings = []
        self._session = session
        self.participant_edit.setText(self._session.participant_id)
        if hasattr(self, "ranking_table"):
            self.ranking_table.setRowCount(0)
        if hasattr(self, "publish_btn"):
            self.publish_btn.setEnabled(False)
        if hasattr(self, "test_signal_btn"):
            self.test_signal_btn.setEnabled(False)
        self._refresh_mapping_choices()
        self._update_counts()
        self.session_continue_btn.setToolTip(f"Session ready: {source_path}")

    def _log(self, message: str):
        self.log.append(message)

    def closeEvent(self, event):
        self._stop_publish()
        self._disconnect()
        self._disconnect_imu()
        if self._lsl_scan_worker is not None and self._lsl_scan_worker.isRunning():
            self._lsl_scan_worker.wait(1500)
        if self._xdf_import_worker is not None and self._xdf_import_worker.isRunning():
            self._xdf_import_worker.requestInterruption()
            if not self._xdf_import_worker.wait(5000):
                event.ignore()
                return
        event.accept()


def main(argv: list[str] | None = None) -> int:
    del argv
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyleSheet(INTENT_STYLE)
    window = EmgIntentDecoderWindow()
    window.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
