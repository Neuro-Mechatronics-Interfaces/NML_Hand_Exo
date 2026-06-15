"""
NML EXO -Hand Exoskeleton Control GUI

Dark-themed PyQt5 application for controlling the NML Hand Exoskeleton.
Features: device connection, motor control, gesture control, interactive
calibration, and ROM assessment -all from the UI.
"""

import csv
import json
import math
import os
import queue
import re
import socket
import statistics
import sys
import threading
import time
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QLineEdit, QTextEdit, QGridLayout, QMessageBox, QGroupBox, QComboBox,
    QDialog, QInputDialog, QScrollArea, QFrame, QSizePolicy, QSpacerItem,
    QTabWidget, QTabBar, QTableWidget, QTableWidgetItem, QHeaderView, QCheckBox,
    QSpinBox, QDoubleSpinBox,
)
from PyQt5.QtCore import Qt, QEvent, QSettings, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QFontMetrics

from serial.tools import list_ports
from nml_hand_exo.interface import HandExo, SerialComm
from nml_hand_exo.interface._telemetry_streaming import (
    NumericLSLTelemetryOutlet,
    UDPTelemetryPublisher,
)

# websockets is optional; teleop tab gracefully degrades if missing.
try:
    import websockets.sync.client as _ws_sync_client
    _WEBSOCKETS_AVAILABLE = True
except ImportError:
    _ws_sync_client = None          # type: ignore[assignment]
    _WEBSOCKETS_AVAILABLE = False


# -- Paths -----------------------------------------------------------------

def _repo_root():
    return os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))

PROFILES_DIR = os.path.join(_repo_root(), "examples", "calibration", "profiles")
CONFIG_FILE = os.path.join(PROFILES_DIR, "config.json")
OUTPUT_DIR = os.path.join(_repo_root(), "output_data")


# -- Helpers ---------------------------------------------------------------

def list_profiles() -> list[str]:
    os.makedirs(PROFILES_DIR, exist_ok=True)
    names = []
    for f in os.listdir(PROFILES_DIR):
        if f.endswith(".json") and f != "config.json":
            names.append(f.removesuffix(".json"))
    return sorted(names)


def load_profile(name: str) -> dict | None:
    path = os.path.join(PROFILES_DIR, f"{name}.json")
    if not os.path.exists(path):
        return None
    with open(path, "r") as f:
        return json.load(f)


def get_default_profile_name(side: str = "right") -> str | None:
    if not os.path.exists(CONFIG_FILE):
        return None
    with open(CONFIG_FILE, "r") as f:
        cfg = json.load(f)
    # Check side-specific key first, then legacy "default" (right-hand compat)
    return cfg.get(f"default_{side}") or cfg.get("default")


def save_profile(name: str, data: dict, side: str = "right"):
    os.makedirs(PROFILES_DIR, exist_ok=True)
    path = os.path.join(PROFILES_DIR, f"{name}.json")
    data_with_side = {"side": side, **data}
    with open(path, "w") as f:
        json.dump(data_with_side, f, indent=2)


def set_default_profile(name: str, side: str = "right"):
    cfg = {}
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r") as f:
            cfg = json.load(f)
    cfg[f"default_{side}"] = name
    # Maintain legacy "default" key for right-hand backward compat
    if side == "right":
        cfg["default"] = name
    os.makedirs(PROFILES_DIR, exist_ok=True)
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=2)


def normalize_angle(absolute: float, home: float, flip: bool) -> float:
    if flip:
        return home - absolute
    else:
        return absolute - home


def determine_run_number(participant: str, date_str: str) -> int:
    prefix = f"{participant}_rom_{date_str}_"
    run = 1
    if os.path.isdir(OUTPUT_DIR):
        for fname in os.listdir(OUTPUT_DIR):
            if fname.startswith(prefix) and fname.endswith(".csv"):
                try:
                    n = int(fname.removeprefix(prefix).removesuffix(".csv"))
                    run = max(run, n + 1)
                except ValueError:
                    pass
    return run


def _format_port_label(port) -> str:
    description = port.description or ""
    hwid = getattr(port, "hwid", "") or ""
    desc_lower = description.lower()
    hwid_lower = hwid.lower()

    tags = []
    if "bluetooth" in desc_lower or "rfcomm" in hwid_lower or "bthenum" in hwid_lower:
        tags.append("BT")
    if "usb serial device" in desc_lower or "usb" in desc_lower:
        tags.append("USB")
    if "nml_exo" in desc_lower or "nml_exo" in hwid_lower:
        tags.append("NML_EXO")

    parts = [port.device]
    if tags:
        parts.append(f"[{', '.join(tags)}]")
    if description:
        parts.append(description)
    if getattr(port, "manufacturer", None):
        parts.append(port.manufacturer)
    if getattr(port, "serial_number", None):
        parts.append(f"SN:{port.serial_number}")
    if getattr(port, "vid", None) is not None and getattr(port, "pid", None) is not None:
        parts.append(f"VID:{port.vid:04X} PID:{port.pid:04X}")
    if getattr(port, "hwid", None):
        parts.append(port.hwid)
    return " - ".join(parts)


# -- Stylesheet ------------------------------------------------------------

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
QPushButton[accent="true"]:disabled {
    background-color: #3a2020;
    color: #666666;
    border-color: #442222;
}
QLineEdit, QComboBox {
    background-color: #2a2a2a;
    color: #e0e0e0;
    border: 1px solid #444444;
    border-radius: 4px;
    padding: 4px 8px;
}
QLineEdit:focus, QComboBox:focus {
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
QTextEdit {
    background-color: #111111;
    color: #aaaaaa;
    border: 1px solid #333333;
    border-radius: 4px;
    font-family: "Cascadia Code", "Consolas", "Courier New", monospace;
}
QScrollArea {
    border: none;
    background-color: #1a1a1a;
}
QScrollBar:vertical {
    background: #1a1a1a;
    width: 10px;
}
QScrollBar::handle:vertical {
    background: #444444;
    border-radius: 4px;
    min-height: 20px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}
QLabel#title {
    color: #ffffff;
    font-weight: bold;
}
QLabel#accent-line {
    background-color: #c0392b;
    max-height: 2px;
    min-height: 2px;
}
QLabel#status-connected {
    color: #27ae60;
    font-weight: bold;
}
QLabel#status-disconnected {
    color: #c0392b;
    font-weight: bold;
}
QFrame#motor-row {
    background-color: #252525;
    border-radius: 4px;
    padding: 4px;
}
QDialog {
    background-color: #1a1a1a;
    color: #e0e0e0;
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
QTabBar::tab:hover:!selected {
    background-color: #3a3a3a;
}
QTableWidget {
    background-color: #1a1a1a;
    alternate-background-color: #222222;
    color: #e0e0e0;
    gridline-color: #333333;
    border: 1px solid #333333;
}
QTableWidget::item {
    color: #e0e0e0;
    padding: 4px;
}
QHeaderView::section {
    background-color: #2e2e2e;
    color: #e0e0e0;
    border: 1px solid #333333;
    padding: 4px 8px;
    font-weight: bold;
}
QCheckBox {
    color: #e0e0e0;
    spacing: 6px;
}
QCheckBox::indicator {
    width: 14px;
    height: 14px;
    border: 1px solid #555555;
    background-color: #2a2a2a;
    border-radius: 2px;
}
QCheckBox::indicator:checked {
    background-color: #c0392b;
    border-color: #c0392b;
}
"""


# ==========================================================================
#  Calibration Dialog
# ==========================================================================

# Anatomical prompts for each known motor name: (extension_desc, flexion_desc).
# Used for standalone motors. Wrist + wrist2 paired prompts are set in
# _build_cal_steps(); these entries serve as fallback if only one is present.
# Unknown motor names fall back to generic "<name> extended / flexed" text.
class CalibrationDialog(QDialog):
    """Interactive calibration dialog — two global streaming phases (extension then flexion)."""

    # Phase instruction text
    _PHASE_INFO = [
        (
            "Phase 1 of 2 — Extension / Open\n"
            "Move each joint through its extension/open extreme while recording is active,\n"
            "then press Stop Recording.\n"
            "(Wrist: extend upward. Fingers: open fully. Thumb: extend/abduct.)"
        ),
        (
            "Phase 2 of 2 — Flexion / Close\n"
            "Move each joint through its flexion/close extreme while recording is active,\n"
            "then press Stop Recording.\n"
            "(Wrist: flex downward. Fingers: close fully. Thumb: flex/adduct.)"
        ),
    ]
    _PHASE_BTN = ["Start Recording Extension", "Start Recording Flexion"]

    def __init__(self, exo: HandExo, motor_names: list[str],
                 profile_name: str, side: str = "right",
                 dxl_ids: list | None = None, parent=None):
        super().__init__(parent)
        self.exo = exo
        self.motor_names = motor_names
        self.profile_name = profile_name
        self._side = side
        self.setWindowTitle("Calibration Protocol")
        self.setMinimumWidth(500)

        # _phase: 0 = extension/open global window, 1 = flexion/close global window
        self._phase = 0

        # Precomputed index map: motor name → Dynamixel ID.
        # get_absolute_motor_angle('all') returns {DXL_ID: value}.
        # dxl_ids must be provided and match motor_names order; if omitted the
        # dialog falls back to list-index lookup (works only when DXL IDs happen
        # to equal 0-based indices, which they generally do NOT).
        if dxl_ids is not None and len(dxl_ids) == len(motor_names):
            self._motor_idx: dict[str, int] = {
                name: dxl_ids[i] for i, name in enumerate(motor_names)
            }
        else:
            # Fallback: list index — incorrect for non-zero-based DXL IDs but
            # kept so the dialog can still be constructed without crashing.
            self._motor_idx = {name: i for i, name in enumerate(motor_names)}

        # Streaming state — full sample lists kept for Option B profile derivation.
        # Keys are motor names; lists accumulate during each recording window.
        self._recording = False
        self._samples_buf: dict[str, list[float]] = {}  # current window accumulator
        self._open_samples: dict[str, list[float]] = {}   # committed after extension stop
        self._close_samples: dict[str, list[float]] = {}  # committed after flexion stop

        layout = QVBoxLayout(self)

        self.info_label = QLabel("")
        self.info_label.setWordWrap(True)
        self.info_label.setStyleSheet("font-size: 14px; padding: 8px;")
        layout.addWidget(self.info_label)

        self.result_label = QLabel("")
        self.result_label.setWordWrap(True)
        self.result_label.setStyleSheet("color: #aaaaaa; padding: 4px;")
        layout.addWidget(self.result_label)

        btn_row = QHBoxLayout()
        self.record_btn = QPushButton("")
        self.record_btn.setProperty("accent", True)
        self.record_btn.clicked.connect(self._toggle_recording)
        btn_row.addStretch()
        btn_row.addWidget(self.record_btn)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        self.sample_label = QLabel("Samples: 0")
        self.sample_label.setStyleSheet("color: #777777; padding: 2px 8px;")
        layout.addWidget(self.sample_label)

        # Dialog-owned timer — 100 ms, same rate as ROMDialog. Never shares the
        # main window's _angle_timer.
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll_angles)

        # Disable only the motors being calibrated so inactive-side motors
        # (already disabled at connect time) are not accidentally broadcast over.
        for _cal_dxl_id in self._motor_idx.values():
            try:
                self.exo.disable_motor(_cal_dxl_id)
            except Exception:
                pass

        self._show_phase_prompt()

    def _show_phase_prompt(self):
        """Set info_label and record_btn text for the current phase (0 or 1)."""
        self.info_label.setText(self._PHASE_INFO[self._phase])
        self.sample_label.setText("Samples: 0")
        self.record_btn.setText(self._PHASE_BTN[self._phase])

    # -- Streaming recording -----------------------------------------------

    def _toggle_recording(self):
        if not self._recording:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        self._samples_buf = {name: [] for name in self.motor_names}
        self._recording = True
        self._timer.start(100)
        self.record_btn.setText("Stop Recording")
        phase_word = "extension" if self._phase == 0 else "flexion"
        self.result_label.setText(f"Recording {phase_word}... click Stop when done.")

    def _stop_recording(self):
        self._timer.stop()
        self._recording = False

        # Guard: require at least 3 samples per motor before committing.
        min_samples = min(
            len(self._samples_buf.get(name, [])) for name in self.motor_names
        )
        if min_samples < 3:
            QMessageBox.warning(
                self, "Too Few Samples",
                f"Only {min_samples} sample(s) collected — need at least 3.\n"
                "Move each joint through its range while recording is active."
            )
            # Reset for a retry: leave _phase unchanged, restore prompt.
            self._show_phase_prompt()
            return

        if self._phase == 0:
            # Commit extension samples for all motors
            self._open_samples = {name: list(self._samples_buf[name])
                                   for name in self.motor_names}
            self.result_label.setText(
                f"Extension phase recorded: {min_samples} samples per motor."
            )
            self._phase = 1
            self._show_phase_prompt()
        else:
            # Commit flexion samples for all motors
            self._close_samples = {name: list(self._samples_buf[name])
                                    for name in self.motor_names}
            self.result_label.setText(
                f"Flexion phase recorded: {min_samples} samples per motor."
            )
            try:
                self._save_profile()
            except RuntimeError as e:
                QMessageBox.critical(self, "Calibration Error", str(e))
                # Leave dialog open — user must close manually.
                # accept() is never called so dlg.result() == Rejected
                # and HandExoGUI will not offer to apply.
                self.info_label.setText("Calibration failed — see error above.")
                self.sample_label.setText("")
                self.record_btn.setEnabled(False)
                return
            self.info_label.setText(
                f"Calibration complete!\n"
                f"Profile '{self.profile_name}' saved ({len(self.motor_names)} motors)."
            )
            self.sample_label.setText("")
            self.record_btn.setText("Close")
            self.record_btn.clicked.disconnect()
            self.record_btn.clicked.connect(self.accept)

    def _poll_angles(self):
        """Timer callback — appends one angle reading per motor to the current buffer."""
        try:
            angles = self.exo.get_absolute_motor_angle('all')
        except Exception:
            return
        for name, idx in self._motor_idx.items():
            val = angles.get(idx)
            if val is not None:
                self._samples_buf[name].append(float(val))
        count = min(
            len(self._samples_buf.get(name, [])) for name in self.motor_names
        )
        self.sample_label.setText(f"Samples: {count}")

    # Minimum observed range (deg) that is considered physiologically plausible.
    # Below this threshold the user is warned, but saving is not blocked.
    _MIN_RANGE_DEG = 2.0

    def _validate_profile(self, data: dict):
        """Check derived profile values before writing to disk.

        Blocking errors (raise RuntimeError — save is aborted):
          - limit_min == limit_max: motor never moved or hardware offline.
          - home outside [limit_min, limit_max]: self-inconsistent profile.

        Warnings (QMessageBox.warning — save continues after the user acknowledges):
          - observed range < _MIN_RANGE_DEG: motor moved very little.

        All messages include the motor name so grouped-step motors (e.g. wrist2)
        are identified individually.
        """
        warn_msgs = []
        for name, vals in data["motors"].items():
            lo = vals["limit_min"]
            hi = vals["limit_max"]
            home = vals["home"]
            observed_range = hi - lo

            if lo == hi:
                raise RuntimeError(
                    f"Motor '{name}': limit_min == limit_max ({lo:.2f}°).\n"
                    "The motor did not move during calibration — check hardware connection.\n"
                    "Profile not saved."
                )

            if not (lo <= home <= hi):
                raise RuntimeError(
                    f"Motor '{name}': home ({home:.2f}°) is outside "
                    f"[limit_min={lo:.2f}°, limit_max={hi:.2f}°].\n"
                    "Profile is self-inconsistent and not saved."
                )

            if observed_range < self._MIN_RANGE_DEG:
                warn_msgs.append(
                    f"  {name}: range = {observed_range:.2f}° "
                    f"(min={lo:.2f}°, max={hi:.2f}°)"
                )

        if warn_msgs:
            QMessageBox.warning(
                self, "Suspiciously Small Range",
                "The following motors moved less than "
                f"{self._MIN_RANGE_DEG:.0f}° during calibration:\n\n"
                + "\n".join(warn_msgs)
                + "\n\nThe profile will be saved, but verify hardware before use.",
            )

    def _save_profile(self):
        """Derive calibration values from full sample lists (Option B).

        home      = median of extension samples  (stable resting position)
        flip      = flexion median < extension median  (direction detection)
        limit_min = true minimum across extension + flexion samples
        limit_max = true maximum across extension + flexion samples

        Profile schema is unchanged.
        """
        data = {"motors": {}}
        for name in self.motor_names:
            o_vals = self._open_samples.get(name, [])
            c_vals = self._close_samples.get(name, [])
            # Guard: both lists must be non-empty (enforced by _stop_recording,
            # but protect _save_profile against any unexpected code path).
            if not o_vals or not c_vals:
                raise RuntimeError(
                    f"Motor '{name}': missing samples — extension or flexion was "
                    "not recorded. Profile not saved."
                )
            o_med = statistics.median(o_vals)
            c_med = statistics.median(c_vals)
            all_vals = o_vals + c_vals
            data["motors"][name] = {
                "home":      round(o_med, 2),
                "flip":      c_med < o_med,
                "limit_min": round(min(all_vals), 2),
                "limit_max": round(max(all_vals), 2),
            }

        # Validate before writing — raises RuntimeError on blocking problems,
        # shows QMessageBox.warning for non-blocking concerns.
        self._validate_profile(data)

        save_profile(self.profile_name, data, side=self._side)

        # Always set the newly calibrated profile as the default so that
        # _ensure_gesture_ready() applies the correct profile on the next gesture.
        set_default_profile(self.profile_name, side=self._side)


# ==========================================================================
#  ROM Assessment Dialog
# ==========================================================================

class ROMDialog(QDialog):
    """ROM assessment dialog with in-GUI recording (no terminal input)."""

    def __init__(self, exo: HandExo, motor_names: list[str],
                 participant: str, side: str = "right",
                 dxl_ids: list | None = None, parent=None):
        super().__init__(parent)
        self.exo = exo
        self.motor_names = motor_names
        self.participant = participant
        self._side = side
        self.setWindowTitle("ROM Assessment")
        self.setMinimumWidth(600)

        # Motor name → Dynamixel ID lookup for _poll_angles.
        # get_absolute_motor_angle('all') returns {DXL_ID: value} so we need
        # the real ID, not the 0-based list index.
        if dxl_ids is not None and len(dxl_ids) == len(motor_names):
            self._motor_dxl_lookup: dict[str, int] = {
                name: dxl_ids[i] for i, name in enumerate(motor_names)
            }
        else:
            self._motor_dxl_lookup = {name: i for i, name in enumerate(motor_names)}

        # Auto-detect motor orientation from applied calibration or profile
        self.orientation = self._detect_orientation()

        # State
        self._recording = False
        self._samples = {}
        self._phase = 0  # 0-3 -> (unassisted open, unassisted closed, assisted open, assisted closed)
        self._phase_data = [{}, {}, {}, {}]  # collected samples per phase

        self.PHASE_LABELS = [
            "Phase 1 - Unassisted ROM: OPEN hand",
            "Phase 1 - Unassisted ROM: CLOSED hand",
            "Phase 2 - Assisted ROM: OPEN hand",
            "Phase 2 - Assisted ROM: CLOSED hand",
        ]

        layout = QVBoxLayout(self)

        self.phase_label = QLabel(self.PHASE_LABELS[0])
        self.phase_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 6px;")
        self.phase_label.setWordWrap(True)
        layout.addWidget(self.phase_label)

        self.instruction_label = QLabel("Position the hand, then click Start Recording.")
        self.instruction_label.setWordWrap(True)
        self.instruction_label.setStyleSheet("color: #aaaaaa; padding: 4px;")
        layout.addWidget(self.instruction_label)

        btn_row = QHBoxLayout()
        self.start_btn = QPushButton("Start Recording")
        self.start_btn.setProperty("accent", True)
        self.start_btn.clicked.connect(self._toggle_recording)
        btn_row.addStretch()
        btn_row.addWidget(self.start_btn)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        self.sample_label = QLabel("Samples: 0")
        self.sample_label.setStyleSheet("color: #777777;")
        layout.addWidget(self.sample_label)

        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setMaximumHeight(200)
        layout.addWidget(self.result_text)

        # Recording timer
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll_angles)

        # Disable only the motors being assessed (side-specific IDs).
        for _rom_dxl_id in self._motor_dxl_lookup.values():
            try:
                self.exo.disable_motor(_rom_dxl_id)
            except Exception:
                pass

    def _detect_orientation(self) -> dict:
        """Auto-detect motor orientation from the default/applied calibration profile."""
        orientation = {}
        # Load the default profile for the correct side so left ROM assessments
        # use a left profile rather than the global (right) default.
        default_name = get_default_profile_name(side=self._side)
        cal = load_profile(default_name) if default_name else None

        # If no default, try any profile that exists
        if cal is None:
            for name in list_profiles():
                cal = load_profile(name)
                if cal:
                    break

        for name in self.motor_names:
            if cal and name in cal.get("motors", {}):
                m = cal["motors"][name]
                orientation[name] = {"home": m["home"], "flip": m["flip"]}
            else:
                # Fallback: try reading home from device
                try:
                    home = self.exo.get_home(name)
                    # Without a profile, we can't know flip -default to False
                    orientation[name] = {"home": float(home), "flip": False}
                except Exception:
                    orientation[name] = {"home": 0.0, "flip": False}
        return orientation

    def _toggle_recording(self):
        if not self._recording:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        self._recording = True
        self._samples = {name: [] for name in self.motor_names}
        self.start_btn.setText("Stop Recording")
        self.instruction_label.setText("Recording... click Stop when done.")
        self._timer.start(100)

    def _stop_recording(self):
        self._timer.stop()
        self._recording = False
        self._phase_data[self._phase] = dict(self._samples)
        count = min(len(v) for v in self._samples.values()) if self._samples else 0
        self.sample_label.setText(f"Samples: {count}")
        self.start_btn.setText("Start Recording")

        self._phase += 1
        if self._phase < 4:
            self.phase_label.setText(self.PHASE_LABELS[self._phase])
            self.instruction_label.setText("Position the hand, then click Start Recording.")
        else:
            self._finish()

    def _poll_angles(self):
        try:
            angles = self.exo.get_absolute_motor_angle('all')
            for name in self.motor_names:
                dxl_id = self._motor_dxl_lookup.get(name)
                val = angles.get(dxl_id) if dxl_id is not None else None
                if val is not None:
                    self._samples[name].append(float(val))
        except Exception:
            pass
        total = min(len(v) for v in self._samples.values()) if self._samples else 0
        self.sample_label.setText(f"Samples: {total}")

    def _finish(self):
        self.start_btn.setEnabled(False)
        self.instruction_label.setText("Assessment complete.")
        self.phase_label.setText("ROM Assessment Complete")

        # Compute results
        unassisted = self._compute_phase(self._phase_data[0], self._phase_data[1])
        assisted = self._compute_phase(self._phase_data[2], self._phase_data[3])

        # Display summary
        lines = ["Motor          | Unassisted ROM | Assisted ROM",
                 "-------------- | -------------- | ------------"]
        for name in self.motor_names:
            u_rom = unassisted[name]["rom"]
            a_rom = assisted[name]["rom"]
            lines.append(f"{name:<14} | {u_rom:>13.2f} deg | {a_rom:>11.2f} deg")
        self.result_text.setText("\n".join(lines))

        # Save CSV
        filepath = self._save_csv(unassisted, assisted)
        self.instruction_label.setText(f"Saved to: {filepath}")

        # Offer to derive a calibration profile from the assisted ROM data
        self.saved_profile_name = None
        ans = QMessageBox.question(
            self, "Save Calibration Profile",
            "Save a calibration profile derived from the assisted ROM data?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )
        if ans == QMessageBox.Yes:
            default_name = self.participant
            name, ok = QInputDialog.getText(
                self, "Profile Name", "Profile name:", text=default_name
            )
            if ok and name.strip():
                self._derive_and_save_cal_profile(name.strip().lower())

    def _derive_and_save_cal_profile(self, name: str):
        """Derive a calibration profile from the assisted ROM data and save it.

        Uses median of the assisted open samples as `home` and median of
        assisted close samples as the flex position.  Arithmetic is identical
        to CalibrationDialog._save_profile(): flip detected from data,
        limit_min/max from the two medians.  Profile schema is unchanged.
        """
        open_samples = self._phase_data[2]   # assisted open
        close_samples = self._phase_data[3]  # assisted close

        data = {"motors": {}}
        for motor_name in self.motor_names:
            o_vals = open_samples.get(motor_name, [])
            c_vals = close_samples.get(motor_name, [])
            o = statistics.median(o_vals) if o_vals else 0.0
            c = statistics.median(c_vals) if c_vals else 0.0
            data["motors"][motor_name] = {
                "home":      round(o, 2),
                "limit_min": round(min(o, c), 2),
                "limit_max": round(max(o, c), 2),
                "flip":      c < o,
            }

        save_profile(name, data, side=self._side)
        if get_default_profile_name(self._side) is None:
            set_default_profile(name, side=self._side)

        self.saved_profile_name = name

    def _compute_phase(self, open_samples: dict, closed_samples: dict) -> dict:
        results = {}
        for name in self.motor_names:
            home = self.orientation[name]["home"]
            flip = self.orientation[name]["flip"]

            o_vals = open_samples.get(name, [])
            c_vals = closed_samples.get(name, [])

            o_abs_max = max(o_vals) if o_vals else 0.0
            o_abs_min = min(o_vals) if o_vals else 0.0
            c_abs_max = max(c_vals) if c_vals else 0.0
            c_abs_min = min(c_vals) if c_vals else 0.0

            o_norm = [normalize_angle(v, home, flip) for v in o_vals] if o_vals else [0.0]
            c_norm = [normalize_angle(v, home, flip) for v in c_vals] if c_vals else [0.0]

            o_norm_max = max(o_norm)
            o_norm_min = min(o_norm)
            c_norm_max = max(c_norm)
            c_norm_min = min(c_norm)

            rom = c_norm_max - o_norm_min

            results[name] = {
                "open_abs_max": o_abs_max, "open_abs_min": o_abs_min,
                "closed_abs_max": c_abs_max, "closed_abs_min": c_abs_min,
                "open_norm_max": o_norm_max, "open_norm_min": o_norm_min,
                "closed_norm_max": c_norm_max, "closed_norm_min": c_norm_min,
                "rom": rom, "flip": flip,
            }
        return results

    def _save_csv(self, unassisted: dict, assisted: dict) -> str:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        date_str = datetime.now().strftime("%Y-%m-%d")
        run = determine_run_number(self.participant, date_str)
        filename = f"{self.participant}_rom_{date_str}_{run}.csv"
        filepath = os.path.join(OUTPUT_DIR, filename)

        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "participant", "date", "run",
                "motor", "flip", "phase",
                "open_abs_max", "open_abs_min",
                "closed_abs_max", "closed_abs_min",
                "open_norm_max", "open_norm_min",
                "closed_norm_max", "closed_norm_min",
                "rom_deg",
            ])
            for phase_label, data in [("unassisted", unassisted),
                                      ("assisted", assisted)]:
                for name in self.motor_names:
                    r = data[name]
                    writer.writerow([
                        self.participant, date_str, run,
                        name, r["flip"], phase_label,
                        f"{r['open_abs_max']:.2f}", f"{r['open_abs_min']:.2f}",
                        f"{r['closed_abs_max']:.2f}", f"{r['closed_abs_min']:.2f}",
                        f"{r['open_norm_max']:.2f}", f"{r['open_norm_min']:.2f}",
                        f"{r['closed_norm_max']:.2f}", f"{r['closed_norm_min']:.2f}",
                        f"{r['rom']:.2f}",
                    ])
        return filepath


# ==========================================================================
#  Hand State Visualisation Widget
# ==========================================================================

class HandSkeletonWidget(QWidget):
    """
    2D dorsal-view stick-figure hand skeleton, driven by normalised motor states.

    Call update_motor_states(t_dict, connected) after each angle poll.
    Each t-value is in [0, 1]:  0 = extended / open,  1 = flexed / closed.

    Motor → visual role
    ───────────────────
      index / middle / ring / pinky  →  2-segment finger curl (MCP + PIP joints)
      thumbflex                      →  thumb tip curl
      thumbadd                       →  thumb lateral abduction angle
      thumbrot                       →  thumb metacarpal in-plane rotation (±7°)
      wrist                          →  tilt of the whole hand assembly (±9°)
      wrist2                         →  omitted (v1; pronation/supination not shown)
    """

    # (MCP x, MCP y, proximal length, distal length) in hand-unit space.
    # Dorsal right-hand view: index on the left, pinky on the right.
    _FINGER_CFG = {
        "index":  (-0.110, 0.330, 0.175, 0.115),
        "middle": (-0.037, 0.355, 0.195, 0.130),
        "ring":   ( 0.037, 0.345, 0.180, 0.120),
        "pinky":  ( 0.115, 0.310, 0.135, 0.090),
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._t: dict[str, float] = {}   # normalised motor states
        self._connected = False
        self.setMinimumSize(260, 320)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_motor_states(self, t_dict: dict[str, float], connected: bool = True):
        """Push new normalised [0, 1] states and request a repaint."""
        self._t = dict(t_dict)
        self._connected = connected
        self.update()

    # -- Paint ---------------------------------------------------------------

    def paintEvent(self, event):
        from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QPolygonF
        from PyQt5.QtCore import QPointF

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        if not self._connected:
            painter.setPen(QColor("#555555"))
            painter.setFont(QFont("Segoe UI", 10))
            painter.drawText(self.rect(), Qt.AlignCenter,
                             "Connect a device\nto view hand state")
            return

        # ── Scale / origin ───────────────────────────────────────────────────
        # Hand bounding box (unit space): ~0.44 wide (−0.24 … +0.20),
        #   ~0.78 tall (−0.10 forearm … +0.68 middle tip at full extension).
        PAD = 18
        scale = min((w - 2 * PAD) / 0.44, (h - 2 * PAD) / 0.78)
        # ox: keep the 0.44-unit box horizontally centred; thumb extends left so
        # we bias the origin 0.24 units from the left edge of that box.
        ox = PAD + 0.24 * scale + max(0.0, (w - 2 * PAD - 0.44 * scale) / 2)
        oy = h - PAD - 0.08 * scale  # wrist origin near widget bottom

        def _px(x, y):
            """Hand-unit coords → screen pixels (y-axis flipped for screen)."""
            return ox + x * scale, oy - y * scale

        # ── Pens / brushes ───────────────────────────────────────────────────
        lw = max(2, int(scale * 0.022))
        bone_pen  = QPen(QColor("#c0392b"), lw, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        thin_pen  = QPen(QColor("#3a3a3a"), max(1, lw - 1), Qt.SolidLine, Qt.RoundCap)
        joint_brush  = QBrush(QColor("#777777"))
        accent_brush = QBrush(QColor("#c0392b"))
        palm_brush   = QBrush(QColor("#222222"))
        jr = max(3, int(scale * 0.028))  # joint dot radius (px)

        # ── Wrist tilt ───────────────────────────────────────────────────────
        # t=0 → extended (hand tilts back), t=0.5 → neutral, t=1 → flexed.
        # The whole hand assembly rotates ±9° around the wrist origin.
        t_wrist = self._t.get("wrist", 0.5)
        tilt_rad = math.radians((t_wrist - 0.5) * 18.0)
        cos_t, sin_t = math.cos(tilt_rad), math.sin(tilt_rad)

        def _rot(x, y):
            """Rotate by wrist tilt around (0, 0)."""
            return x * cos_t - y * sin_t, x * sin_t + y * cos_t

        # ── Chain-walking helper ─────────────────────────────────────────────
        def _walk(x0, y0, dir_deg, segs):
            """
            Walk a segment chain starting at (x0, y0).
            dir_deg: initial direction in degrees (90 = straight up).
            segs: list of (length, bend_deg); each bend_deg is subtracted from
                  the current direction so positive bend curls toward the palm.
            Returns a list of (x, y) unit-space points (start included),
            with wrist-tilt rotation already applied.
            """
            pts = [(x0, y0)]
            d = dir_deg
            for length, bend in segs:
                d -= bend
                xi = pts[-1][0] + length * math.cos(math.radians(d))
                yi = pts[-1][1] + length * math.sin(math.radians(d))
                pts.append((xi, yi))
            return [_rot(xi, yi) for xi, yi in pts]

        # ── Drawing helpers ──────────────────────────────────────────────────
        def _chain(pts):
            """Draw bone segments between consecutive rotated unit-space points."""
            painter.setPen(bone_pen)
            for i in range(len(pts) - 1):
                x1, y1 = _px(*pts[i])
                x2, y2 = _px(*pts[i + 1])
                painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        def _dot(rx, ry, r=None, brush=None):
            """Draw joint dot at rotated unit-space position (rx, ry)."""
            painter.setPen(Qt.NoPen)
            painter.setBrush(brush or joint_brush)
            cr = r or jr
            cx_, cy_ = _px(rx, ry)
            painter.drawEllipse(int(cx_ - cr), int(cy_ - cr), cr * 2, cr * 2)

        # ── Forearm stub (fixed — does not rotate with wrist tilt) ───────────
        painter.setPen(thin_pen)
        ax, ay = _px(0.0, -0.08)
        bx, by = _px(0.0,  0.00)
        painter.drawLine(int(ax), int(ay), int(bx), int(by))
        # Wrist joint dot in accent colour, drawn here so it sits on top of
        # the palm polygon painted next.
        _dot(0.0, 0.0, jr + 2, accent_brush)

        # ── Palm body ────────────────────────────────────────────────────────
        # Trapezoid: narrower at wrist, wider at the knuckle row.
        palm_corners = [(-0.10, 0.02), (-0.155, 0.32),
                        ( 0.155, 0.32), ( 0.10, 0.02)]
        r_palm = [_rot(x, y) for x, y in palm_corners]
        poly = QPolygonF([QPointF(*_px(x, y)) for x, y in r_palm])
        painter.setPen(thin_pen)
        painter.setBrush(palm_brush)
        painter.drawPolygon(poly)
        painter.setPen(bone_pen)

        # ── Fingers ──────────────────────────────────────────────────────────
        # Live angles come from get_angle:all → firmware getRelativeAngle.
        # Normalised t: 0 = home/extended, 1 = calibrated maximum flexion.
        # MCP bends up to 70°, PIP adds up to 50° — producing a natural curl.
        for name, (mx, my, prox, dist) in self._FINGER_CFG.items():
            t = self._t.get(name, 0.0)
            pts = _walk(mx, my, 90.0, [
                (prox, t * 70.0),   # MCP joint
                (dist, t * 50.0),   # PIP joint (anatomically coupled for v1)
            ])
            _chain(pts)
            for pt in pts[1:]:  # skip the knuckle — it's on the palm edge
                _dot(*pt)

        # ── Thumb ────────────────────────────────────────────────────────────
        # CMC joint on the radial (left) side of palm, mid-height.
        # thumbadd: abduction spread → base direction 125° … 153° from +x.
        # thumbrot: metacarpal rotation ±7°.
        # thumbflex: proximal phalanx curl 0° … 60°.
        t_add  = self._t.get("thumbadd",  0.0)
        t_rot  = self._t.get("thumbrot",  0.5)
        t_flex = self._t.get("thumbflex", 0.0)

        cmc_x, cmc_y = -0.145, 0.130
        base_dir  = 125.0 + t_add * 28.0 + (t_rot - 0.5) * 14.0
        thumb_pts = _walk(cmc_x, cmc_y, base_dir, [
            (0.130, 0.0),            # metacarpal — no intrinsic bend
            (0.115, t_flex * 60.0),  # proximal phalanx — flexion curl
        ])
        _chain(thumb_pts)
        for pt in thumb_pts:
            _dot(*pt, r=jr - 1)


# ==========================================================================
#  Teleop WebSocket Worker
# ==========================================================================

class TeleopWorker(QThread):
    """
    Background QThread that owns the WebSocket client connection for teleop
    streaming.

    The main GUI thread's ``_teleop_timer`` (100 ms QTimer) calls
    ``enqueue(payload)`` with a compact JSON string.  This worker drains
    the queue and fires each frame over the socket.

    The send queue is bounded (maxsize=3).  When the network falls behind,
    the oldest frame is silently dropped so the downstream controller always
    receives the *freshest* reading — never a seconds-old backlog.

    Signals
    -------
    status_changed(msg, color_hex)
        Emitted on every connection-state transition.
        color_hex is one of:
          ``#27ae60``  green  — connected and streaming
          ``#f39c12``  amber  — connecting (in-flight)
          ``#c0392b``  red    — refused or runtime error
          ``#888888``  grey   — cleanly disconnected / idle
    """

    status_changed = pyqtSignal(str, str)
    frame_sent = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._url: str = "ws://localhost:8765"
        self._queue: queue.Queue = queue.Queue(maxsize=3)
        self._stop_evt: threading.Event = threading.Event()

    # -- Public API (called from the GUI thread) ----------------------------

    def configure(self, url: str):
        """Set the WebSocket server URL.  Must be called before start()."""
        self._url = url

    def enqueue(self, payload: str):
        """
        Add a JSON frame to the send queue.

        If the queue is already full (network slow or worker stalled), the
        oldest queued frame is dropped and the new one takes its place —
        this keeps end-to-end latency at most one period (100 ms).
        """
        if self._stop_evt.is_set():
            return
        if self._queue.full():
            try:
                self._queue.get_nowait()      # drop oldest
            except queue.Empty:
                pass
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            pass                              # race — discard silently

    def stop(self):
        """Signal the send loop to exit and the WebSocket to close."""
        self._stop_evt.set()

    # -- Thread entry point -------------------------------------------------

    def run(self):
        """
        Runs entirely on the worker thread.  Opens a synchronous WebSocket
        connection (websockets.sync.client — available since websockets 12),
        then drains the queue until stop() is called or the connection breaks.

        All status updates are emitted as Qt signals so the GUI can update
        safely from the main thread.
        """
        if not _WEBSOCKETS_AVAILABLE:
            self.status_changed.emit("websockets package not installed", "#c0392b")
            return

        self._stop_evt.clear()

        # Drain stale frames left over from a previous session.
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break

        self.status_changed.emit("Connecting\u2026", "#f39c12")

        final_msg, final_color = "Disconnected", "#888888"
        try:
            with _ws_sync_client.connect(self._url, open_timeout=5) as ws:
                self.status_changed.emit("Connected", "#27ae60")
                while not self._stop_evt.is_set():
                    try:
                        msg = self._queue.get(timeout=0.05)
                        ws.send(msg)
                        self.frame_sent.emit(msg)
                    except queue.Empty:
                        pass    # nothing ready — yield back and check stop flag
        except OSError as exc:
            final_msg = f"Refused: {exc.strerror or exc}"
            final_color = "#c0392b"
        except Exception as exc:
            final_msg = f"Error: {exc}"
            final_color = "#c0392b"

        self.status_changed.emit(final_msg, final_color)


class UDPCommandWorker(QThread):
    """Receive UDP command datagrams without blocking the GUI thread."""

    command_received = pyqtSignal(str, str)
    status_changed = pyqtSignal(str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._host = "0.0.0.0"
        self._port = 10001
        self._stop_event = threading.Event()
        self._sock = None

    def configure(self, host: str, port: int):
        self._host = host.strip() or "0.0.0.0"
        self._port = int(port)

    def stop(self):
        self._stop_event.set()
        sock = self._sock
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def run(self):
        self._stop_event.clear()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock = sock
        try:
            sock.settimeout(0.25)
            sock.bind((self._host, self._port))
            self.status_changed.emit(
                f"Listening on {self._host}:{self._port}", "#27ae60"
            )
            while not self._stop_event.is_set():
                try:
                    data, addr = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                message = data.decode("utf-8", errors="ignore").strip()
                if message:
                    self.command_received.emit(message, f"{addr[0]}:{addr[1]}")
        except OSError as exc:
            self.status_changed.emit(f"UDP command error: {exc}", "#c0392b")
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self._sock = None
            if self._stop_event.is_set():
                self.status_changed.emit("Command receiver stopped", "#888888")


class SynchronizedHandExo:
    """Serialize complete HandExo method calls across GUI and worker threads."""

    def __init__(self, exo):
        self._exo = exo
        self._lock = threading.RLock()

    def __getattr__(self, name):
        attr = getattr(self._exo, name)
        if not callable(attr):
            return attr

        def synchronized_call(*args, **kwargs):
            with self._lock:
                return attr(*args, **kwargs)

        return synchronized_call

    def run_locked(self, callback):
        """Run a callback while holding the serial access lock exactly once."""
        with self._lock:
            return callback(self._exo)


class DevicePollWorker(QThread):
    """Perform blocking serial telemetry reads away from the Qt event loop."""

    completed = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._exo = None
        self._include_telemetry = False

    def configure(self, exo, include_telemetry: bool):
        self._exo = exo
        self._include_telemetry = bool(include_telemetry)

    def run(self):
        exo = self._exo
        result = {
            "relative": None,
            "positions": None,
            "torques": None,
            "currents": None,
        }
        if exo is None:
            self.completed.emit(result)
            return
        try:
            result["relative"] = exo.get_motor_angle("all")
        except Exception:
            pass
        if self._include_telemetry:
            try:
                result["positions"] = exo.get_absolute_motor_angle("all")
            except Exception:
                pass
            try:
                result["torques"] = exo.get_motor_torque("all")
            except Exception:
                pass
            try:
                result["currents"] = exo.get_motor_current("all")
            except Exception:
                pass
        self.completed.emit(result)


# ==========================================================================
#  Main GUI
# ==========================================================================

class HandExoGUI(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("NML EXO")
        self.exo = None
        self.exo_connected = False
        self.n_motors = 0
        self.motor_names = []
        self.motor_widgets = []  # list of dicts per motor row
        self._gesture_ready = False  # set True after calibration + enable for gestures

        # Per-motor lookup maps (populated on connect, cleared on disconnect)
        self._motor_idx: dict[str, int] = {}  # motor name → serial index (0-based)
        self._motor_row: dict[str, int] = {}  # motor name → telemetry table row
        # Cached after every successful apply_calibration; used by Hand State tab
        # to normalise live relative angles against per-motor calibrated limits.
        self._active_cal_profile: dict | None = None

        # Dual-mode per-side calibration profiles (populated separately for each side).
        # In single mode these are None; _active_cal_profile is used instead.
        self._active_cal_left:  dict | None = None
        self._active_cal_right: dict | None = None

        # Per-side motor name lists for dual-mode calibration / ROM dialogs.
        self._left_motor_names:  list[str] = []
        self._right_motor_names: list[str] = []

        # Dynamixel ID for each widget index — needed to look up telemetry/angle
        # results since HandExo returns dicts keyed by Dynamixel ID, not 0-based index.
        self._motor_dxl_id: list[int] = []
        self._last_telemetry_update_monotonic: float | None = None
        self._telemetry_rate_ema: float | None = None
        self._direct_armed_ids: set[int] = set()
        self._direct_mode: str | None = None
        self._direct_command_active = False
        self._suspend_device_poll_requests = False

        self._udp_telemetry = UDPTelemetryPublisher()
        self._lsl_angles = NumericLSLTelemetryOutlet(
            "NMLHandExoJointAngles", "JointAngles", "degrees"
        )
        self._lsl_torque = NumericLSLTelemetryOutlet(
            "NMLHandExoMotorTorque", "MotorTorque", "Nm"
        )
        self._udp_command_worker = UDPCommandWorker(self)
        self._udp_command_worker.command_received.connect(self._on_udp_command)
        self._udp_command_worker.status_changed.connect(self._on_udp_command_status)
        self._udp_stream_pending: dict[tuple[str, int], tuple[str, str, str]] = {}
        self._udp_stream_last_status = 0.0
        self._udp_stream_sent_since_status = 0
        self._udp_stream_timer = QTimer(self)
        self._udp_stream_timer.setInterval(20)
        self._udp_stream_timer.timeout.connect(self._flush_udp_stream_commands)
        self._udp_stream_timer.start()
        self._device_poll_worker = DevicePollWorker(self)
        self._device_poll_worker.completed.connect(self._on_device_poll_completed)

        self._build_ui()

        # Motor angle poll timer (Controls tab)
        self._angle_timer = QTimer(self)
        self._angle_timer.timeout.connect(self._request_device_poll)
        self._direct_command_timer = QTimer(self)
        self._direct_command_timer.setInterval(50)
        self._direct_command_timer.timeout.connect(self._send_direct_command_tick)
        self._udp_direct_idle_timer = QTimer(self)
        self._udp_direct_idle_timer.setSingleShot(True)
        self._udp_direct_idle_timer.timeout.connect(self._resume_normal_polling)

        # ------------------------------------------------------------------
        # Teleop state
        # ------------------------------------------------------------------
        # True while the configured teleop tick is running (motors disabled,
        # exo used as sensor only).
        self._teleop_streaming: bool = False
        # True while the WebSocket connection is established (green status).
        self._teleop_ws_connected: bool = False
        # Worker thread — persistent for the lifetime of the window so that
        # connect/disconnect cycles don't leave dangling threads.
        self._teleop_worker = TeleopWorker(self)
        self._teleop_worker.status_changed.connect(self._on_teleop_status)
        self._teleop_worker.frame_sent.connect(self._on_teleop_frame_sent)
        # Teleop tick timer, active only while streaming. It replaces
        # _angle_timer so serial polls never overlap.
        self._teleop_timer = QTimer(self)
        self._teleop_timer.timeout.connect(self._teleop_tick)
        self._load_stream_settings()

    # -- UI Construction ---------------------------------------------------

    def _build_ui(self):
        # Outer scroll area for screen scaling
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._main_scroll = scroll
        outer.addWidget(scroll)

        container = QWidget()
        self.main_layout = QVBoxLayout(container)
        self.main_layout.setSpacing(10)
        self.main_layout.setContentsMargins(16, 12, 16, 12)
        scroll.setWidget(container)

        self._build_header()
        self._build_connection_section()

        # Tab widget: Controls | Telemetry
        self.main_tabs = QTabWidget()
        self.main_tabs.setUsesScrollButtons(True)
        self.main_tabs.setElideMode(Qt.ElideRight)
        self.main_tabs.tabBar().installEventFilter(self)
        self.main_layout.addWidget(self.main_tabs)

        # Build the Controls tab by temporarily redirecting self.main_layout so
        # all existing _build_*_section() methods add their boxes to it unchanged.
        controls_container = QWidget()
        controls_layout = QVBoxLayout(controls_container)
        controls_layout.setSpacing(10)
        controls_layout.setContentsMargins(0, 4, 0, 4)
        _saved_layout = self.main_layout
        self.main_layout = controls_layout
        self._build_motor_section()
        self._build_gesture_section()
        self._build_calibration_section()
        self._build_rom_section()
        self.main_layout.addStretch()
        self.main_layout = _saved_layout

        self.main_tabs.addTab(controls_container, "Controls")
        self.main_tabs.addTab(self._build_telemetry_tab(), "Telemetry")
        self.main_tabs.addTab(self._build_visualization_tab(), "Hand State")
        self.main_tabs.addTab(self._build_direct_control_tab(), "Direct Control")
        self.main_tabs.addTab(self._build_teleop_tab(), "Teleop")
        self.main_tabs.addTab(self._build_settings_tab(), "Settings")

        self._build_log_section()
        for button in self.findChildren(QPushButton):
            if button is not self.refresh_btn:
                self._fit_button_text(button)
        for input_widget in (
            self.findChildren(QComboBox)
            + self.findChildren(QSpinBox)
            + self.findChildren(QDoubleSpinBox)
        ):
            input_widget.installEventFilter(self)
        self._update_enabled_state()

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Wheel and isinstance(obj, QTabBar):
            # Require explicit click to change tabs; ignore wheel cycling.
            return True
        if (
            event.type() == QEvent.Wheel
            and isinstance(obj, (QComboBox, QSpinBox, QDoubleSpinBox))
        ):
            bar = self._main_scroll.verticalScrollBar()
            steps = event.angleDelta().y() / 120.0
            bar.setValue(int(bar.value() - steps * bar.singleStep() * 3))
            return True
        return super().eventFilter(obj, event)

    def _build_telemetry_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # Control row
        ctrl_row = QHBoxLayout()
        self._telem_refresh_btn = QPushButton("Refresh")
        self._telem_refresh_btn.clicked.connect(self._poll_telemetry)
        self._telem_status_lbl = QLabel("Not connected")
        self._telem_status_lbl.setStyleSheet("color: #888888;")
        ctrl_row.addWidget(self._telem_refresh_btn)
        ctrl_row.addStretch()
        ctrl_row.addWidget(self._telem_status_lbl)
        layout.addLayout(ctrl_row)

        self._telem_table = QTableWidget(0, 4)
        self._telem_table.setHorizontalHeaderLabels(
            ["Motor", "Position (°)", "Torque", "Current (mA)"]
        )
        hdr = self._telem_table.horizontalHeader()
        hdr.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        for col in (1, 2, 3):
            hdr.setSectionResizeMode(col, QHeaderView.Stretch)
        self._telem_table.verticalHeader().setVisible(False)
        self._telem_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._telem_table.setSelectionMode(QTableWidget.NoSelection)
        self._telem_table.setAlternatingRowColors(True)
        self._telem_table.setMinimumHeight(200)

        layout.addWidget(self._telem_table)
        return widget

    def _build_visualization_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        self._vis_status_lbl = QLabel("No profile loaded — showing home position")
        self._vis_status_lbl.setStyleSheet("color: #888888; font-size: 10px;")
        self._vis_status_lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(self._vis_status_lbl)

        self._hand_vis = HandSkeletonWidget()
        layout.addWidget(self._hand_vis, stretch=1)
        return widget

    def _build_direct_control_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        warning = QLabel(
            "Direct velocity/current control bypasses position targets. Firmware "
            "joint-limit checks and a command watchdog remain active, but use this "
            "mode only with the mechanism clear and an emergency stop available."
        )
        warning.setWordWrap(True)
        warning.setStyleSheet(
            "color: #f39c12; font-weight: bold; padding: 6px;"
        )
        layout.addWidget(warning)

        mode_box = QGroupBox("Mode and Watchdog")
        mode_layout = QVBoxLayout(mode_box)
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Mode:"))
        self._direct_mode_combo = QComboBox()
        self._direct_mode_combo.addItems(["Velocity", "Current / Torque"])
        self._direct_mode_combo.currentTextChanged.connect(
            self._on_direct_mode_selection_changed
        )
        mode_row.addWidget(self._direct_mode_combo)
        mode_row.addWidget(QLabel("Watchdog:"))
        self._direct_timeout_spin = QSpinBox()
        self._direct_timeout_spin.setRange(50, 5000)
        self._direct_timeout_spin.setValue(250)
        self._direct_timeout_spin.setSuffix(" ms")
        mode_row.addWidget(self._direct_timeout_spin)
        self._direct_apply_mode_btn = QPushButton("Apply Mode")
        self._direct_apply_mode_btn.setProperty("accent", True)
        self._direct_apply_mode_btn.clicked.connect(self._apply_direct_mode)
        mode_row.addWidget(self._direct_apply_mode_btn)
        mode_row.addStretch()
        mode_layout.addLayout(mode_row)
        mode_status_row = QHBoxLayout()
        self._direct_mode_status = QLabel("Not configured")
        self._direct_mode_status.setStyleSheet("color: #888888;")
        mode_status_row.addWidget(self._direct_mode_status, 1)
        self._direct_position_btn = QPushButton("Return to Position Control")
        self._direct_position_btn.clicked.connect(self._restore_position_control)
        mode_status_row.addWidget(self._direct_position_btn)
        mode_layout.addLayout(mode_status_row)
        layout.addWidget(mode_box)

        command_box = QGroupBox("Per-Motor Command")
        command_layout = QVBoxLayout(command_box)
        target_row = QHBoxLayout()
        target_row.addWidget(QLabel("Motor:"))
        self._direct_motor_combo = QComboBox()
        self._direct_motor_combo.currentIndexChanged.connect(
            self._update_direct_arm_status
        )
        target_row.addWidget(self._direct_motor_combo, 1)
        self._direct_arm_btn = QCheckBox("Arm Selected Motor")
        self._direct_arm_btn.toggled.connect(self._on_direct_arm_toggled)
        target_row.addWidget(self._direct_arm_btn)
        self._direct_arm_confirm_cb = QCheckBox("Require arm confirmation")
        self._direct_arm_confirm_cb.setChecked(True)
        target_row.addWidget(self._direct_arm_confirm_cb)
        command_layout.addLayout(target_row)

        command_row = QHBoxLayout()
        command_row.addWidget(QLabel("Command:"))
        self._direct_command_spin = QDoubleSpinBox()
        self._direct_command_spin.setDecimals(2)
        self._direct_command_spin.setSingleStep(0.25)
        command_row.addWidget(self._direct_command_spin)
        self._direct_send_btn = QPushButton("Hold to Command")
        self._direct_send_btn.setProperty("accent", True)
        self._direct_send_btn.pressed.connect(self._start_direct_command)
        self._direct_send_btn.released.connect(self._zero_direct_target)
        command_row.addWidget(self._direct_send_btn)
        command_layout.addLayout(command_row)

        stop_row = QHBoxLayout()
        self._direct_arm_status = QLabel("No motor armed")
        self._direct_arm_status.setStyleSheet("color: #888888;")
        stop_row.addWidget(self._direct_arm_status, 1)
        self._direct_zero_btn = QPushButton("Zero Target")
        self._direct_zero_btn.clicked.connect(self._zero_direct_target)
        stop_row.addWidget(self._direct_zero_btn)
        self._direct_stop_all_btn = QPushButton("STOP ALL")
        self._direct_stop_all_btn.setStyleSheet(
            "background-color: #8e1b1b; color: white; font-weight: bold;"
        )
        self._direct_stop_all_btn.clicked.connect(self._stop_all_direct_control)
        stop_row.addWidget(self._direct_stop_all_btn)
        command_layout.addLayout(stop_row)
        layout.addWidget(command_box)

        detail = QLabel(
            "Velocity commands use signed rpm and are limited to +/-10 rpm. "
            "Current commands use signed mA and are limited to +/-200 mA. "
            "Each nonzero target must be refreshed before the watchdog expires."
        )
        detail.setWordWrap(True)
        detail.setStyleSheet("color: #888888; font-size: 10px;")
        layout.addWidget(detail)
        layout.addStretch()

        self._on_direct_mode_selection_changed(
            self._direct_mode_combo.currentText()
        )
        return widget

    def _build_teleop_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # -- Optional warning when websockets is absent --------------------
        if not _WEBSOCKETS_AVAILABLE:
            warn = QLabel(
                "\u26a0  websockets package not found.\n"
                "Install it with:  pip install websockets"
            )
            warn.setStyleSheet(
                "color: #f39c12; font-weight: bold; padding: 6px;"
            )
            warn.setWordWrap(True)
            layout.addWidget(warn)

        # -- Outbound WebSocket telemetry group ----------------------------
        ws_box = QGroupBox("WebSocket Telemetry Client (Outbound)")
        ws_layout = QVBoxLayout()

        ws_note = QLabel(
            "Connects to a WebSocket server and sends normalized joint-state "
            "frames. It does not receive motor commands; inbound commands use "
            "UDP Command Input in Settings."
        )
        ws_note.setWordWrap(True)
        ws_note.setStyleSheet("color: #888888; font-size: 10px;")
        ws_layout.addWidget(ws_note)

        addr_row = QHBoxLayout()
        addr_row.addWidget(QLabel("Address:"))
        self._teleop_addr_edit = QLineEdit("ws://localhost:8765")
        self._teleop_addr_edit.setPlaceholderText("ws://host:port")
        addr_row.addWidget(self._teleop_addr_edit, 1)
        ws_layout.addLayout(addr_row)

        ws_btn_row = QHBoxLayout()
        self._teleop_connect_btn = QPushButton("Connect")
        self._teleop_connect_btn.setProperty("accent", True)
        self._teleop_connect_btn.clicked.connect(self._on_teleop_connect)
        self._teleop_ws_disconnect_btn = QPushButton("Disconnect")
        self._teleop_ws_disconnect_btn.clicked.connect(self._on_teleop_ws_disconnect)
        self._teleop_ws_disconnect_btn.setEnabled(False)
        self._teleop_ws_status_lbl = QLabel("\u25cf  Not connected")
        self._teleop_ws_status_lbl.setStyleSheet("color: #888888;")
        ws_btn_row.addWidget(self._teleop_connect_btn)
        ws_btn_row.addWidget(self._teleop_ws_disconnect_btn)
        ws_btn_row.addStretch()
        ws_btn_row.addWidget(self._teleop_ws_status_lbl)
        ws_layout.addLayout(ws_btn_row)
        self._teleop_last_sent_lbl = QLabel("Last frame sent: none")
        self._teleop_last_sent_lbl.setWordWrap(True)
        self._teleop_last_sent_lbl.setStyleSheet("color: #888888; font-size: 10px;")
        ws_layout.addWidget(self._teleop_last_sent_lbl)
        ws_box.setLayout(ws_layout)
        layout.addWidget(ws_box)

        # -- Streaming group -----------------------------------------------
        stream_box = QGroupBox("Streaming")
        stream_layout = QVBoxLayout()

        stream_ctrl = QHBoxLayout()
        self._teleop_start_btn = QPushButton("Start Streaming")
        self._teleop_start_btn.setProperty("accent", True)
        self._teleop_start_btn.clicked.connect(self._on_teleop_start)
        self._teleop_start_btn.setEnabled(False)
        self._teleop_stop_btn = QPushButton("Stop Streaming")
        self._teleop_stop_btn.clicked.connect(self._on_teleop_stop)
        self._teleop_stop_btn.setEnabled(False)
        self._teleop_stream_status_lbl = QLabel("\u25cf  Idle")
        self._teleop_stream_status_lbl.setStyleSheet("color: #888888;")
        stream_ctrl.addWidget(self._teleop_start_btn)
        stream_ctrl.addWidget(self._teleop_stop_btn)
        stream_ctrl.addStretch()
        stream_ctrl.addWidget(self._teleop_stream_status_lbl)
        stream_layout.addLayout(stream_ctrl)

        note = QLabel(
            "When streaming starts, all motors are torque-off and the exo acts "
            "as a pure joint-angle sensor.  Motors are NOT re-enabled automatically "
            "when streaming stops — use the Controls tab to re-enable them."
        )
        note.setStyleSheet("color: #777777; font-size: 10px;")
        note.setWordWrap(True)
        stream_layout.addWidget(note)
        stream_box.setLayout(stream_layout)
        layout.addWidget(stream_box)

        # -- Live normalised states table ----------------------------------
        state_box = QGroupBox("Live Normalised Joint States")
        state_layout = QVBoxLayout()

        state_note = QLabel("0 = open/extended, 1 = closed/flexed")
        state_note.setWordWrap(True)
        state_note.setStyleSheet("color: #888888; font-size: 10px;")
        state_layout.addWidget(state_note)

        self._teleop_state_table = QTableWidget(0, 2)
        self._teleop_state_table.setHorizontalHeaderLabels(["Joint", "Value [0\u20131]"])
        hdr = self._teleop_state_table.horizontalHeader()
        hdr.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(1, QHeaderView.Stretch)
        self._teleop_state_table.verticalHeader().setVisible(False)
        self._teleop_state_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._teleop_state_table.setSelectionMode(QTableWidget.NoSelection)
        self._teleop_state_table.setAlternatingRowColors(True)
        self._teleop_state_table.setMaximumHeight(260)
        state_layout.addWidget(self._teleop_state_table)
        state_box.setLayout(state_layout)
        layout.addWidget(state_box)

        layout.addStretch()
        return widget

    def _build_settings_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        lsl_box = QGroupBox("Telemetry Sampling and LSL")
        lsl_layout = QVBoxLayout(lsl_box)
        sampling_row = QHBoxLayout()
        self._lsl_enabled_cb = QCheckBox("Enable LSL streaming")
        self._lsl_enabled_cb.setChecked(False)
        self._lsl_enabled_cb.toggled.connect(self._on_lsl_enabled_toggled)
        self._telemetry_rate_spin = QSpinBox()
        self._telemetry_rate_spin.setRange(1, 20)
        self._telemetry_rate_spin.setValue(2)
        self._telemetry_rate_spin.setSuffix(" Hz")
        self._lsl_angles_cb = QCheckBox("Publish joint angles")
        self._lsl_angles_cb.setChecked(True)
        self._lsl_torque_cb = QCheckBox("Publish motor torque")
        self._lsl_torque_cb.setChecked(True)
        self._lsl_status_lbl = QLabel("Disabled")
        self._lsl_status_lbl.setStyleSheet("color: #888888;")
        sampling_row.addWidget(self._lsl_enabled_cb)
        sampling_row.addWidget(QLabel("Broadcast Rate:"))
        sampling_row.addWidget(self._telemetry_rate_spin)
        sampling_row.addStretch()
        lsl_layout.addLayout(sampling_row)
        lsl_row = QHBoxLayout()
        lsl_row.addWidget(self._lsl_angles_cb)
        lsl_row.addWidget(self._lsl_torque_cb)
        lsl_row.addStretch()
        lsl_row.addWidget(self._lsl_status_lbl)
        lsl_layout.addLayout(lsl_row)
        layout.addWidget(lsl_box)

        udp_out_box = QGroupBox("UDP Telemetry Output")
        udp_out_layout = QVBoxLayout(udp_out_box)
        udp_out_row = QHBoxLayout()
        self._udp_telem_cb = QCheckBox("Enable")
        self._udp_telem_host = QLineEdit("127.0.0.1")
        self._udp_telem_port = QSpinBox()
        self._udp_telem_port.setRange(1, 65535)
        self._udp_telem_port.setValue(10002)
        self._udp_telem_status_lbl = QLabel("Disabled")
        self._udp_telem_status_lbl.setStyleSheet("color: #888888;")
        udp_out_row.addWidget(self._udp_telem_cb)
        udp_out_row.addWidget(QLabel("Host:"))
        udp_out_row.addWidget(self._udp_telem_host, 1)
        udp_out_row.addWidget(QLabel("Port:"))
        udp_out_row.addWidget(self._udp_telem_port)
        udp_out_row.addWidget(QLabel("Status:"))
        udp_out_row.addWidget(self._udp_telem_status_lbl, 1)
        udp_out_layout.addLayout(udp_out_row)
        layout.addWidget(udp_out_box)

        udp_in_box = QGroupBox("UDP Command Input")
        udp_in_layout = QVBoxLayout(udp_in_box)
        udp_in_row = QHBoxLayout()
        self._udp_cmd_cb = QCheckBox("Enable")
        self._udp_cmd_host = QLineEdit("0.0.0.0")
        self._udp_cmd_port = QSpinBox()
        self._udp_cmd_port.setRange(1, 65535)
        self._udp_cmd_port.setValue(10001)
        self._udp_cmd_advanced_cb = QCheckBox(
            "Allow advanced protocol commands"
        )
        self._udp_cmd_advanced_cb.setToolTip(
            "Safety-sensitive motion and configuration commands remain blocked."
        )
        self._udp_cmd_status_lbl = QLabel("Disabled")
        self._udp_cmd_status_lbl.setStyleSheet("color: #888888;")
        udp_in_row.addWidget(self._udp_cmd_cb)
        udp_in_row.addWidget(QLabel("Listen:"))
        udp_in_row.addWidget(self._udp_cmd_host, 1)
        udp_in_row.addWidget(QLabel("Port:"))
        udp_in_row.addWidget(self._udp_cmd_port)
        udp_in_row.addWidget(QLabel("Status:"))
        udp_in_row.addWidget(self._udp_cmd_status_lbl, 1)
        udp_in_layout.addLayout(udp_in_row)
        udp_options_row = QHBoxLayout()
        udp_options_row.addWidget(self._udp_cmd_advanced_cb)
        self._udp_last_command_lbl = QLabel("Last received: none")
        self._udp_last_command_lbl.setWordWrap(True)
        self._udp_last_command_lbl.setStyleSheet("color: #888888;")
        udp_options_row.addStretch()
        udp_options_row.addWidget(self._udp_last_command_lbl, 1)
        udp_in_layout.addLayout(udp_options_row)
        layout.addWidget(udp_in_box)

        note = QLabel(
            "Command datagrams may be plain protocol text or JSON such as "
            '{"command":"set_gesture:pinch_index:close"}. By default only '
            "set_gesture commands are accepted."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #888888; font-size: 10px;")
        layout.addWidget(note)

        apply_btn = QPushButton("Apply Streaming Settings")
        apply_btn.setProperty("accent", True)
        apply_btn.clicked.connect(self._apply_stream_settings)
        layout.addWidget(apply_btn)
        layout.addStretch()
        return widget

    def _load_stream_settings(self):
        settings = QSettings("NML", "HandExoGUI")
        if not settings.contains("telemetry/rate_hz"):
            settings.setValue("telemetry/rate_hz", 2)
        elif not settings.contains("telemetry/rate_hz_migrated"):
            # Migrate the legacy default of 20 Hz to the new 2 Hz default once.
            if settings.value("telemetry/rate_hz", 2, type=int) == 20:
                settings.setValue("telemetry/rate_hz", 2)
            settings.setValue("telemetry/rate_hz_migrated", True)
        self._lsl_enabled_cb.setChecked(
            settings.value("lsl/enabled", False, type=bool)
        )
        self._telemetry_rate_spin.setValue(
            settings.value("telemetry/rate_hz", 2, type=int)
        )
        self._lsl_angles_cb.setChecked(settings.value("lsl/angles", True, type=bool))
        self._lsl_torque_cb.setChecked(settings.value("lsl/torque", True, type=bool))
        self._udp_telem_cb.setChecked(settings.value("udp_telemetry/enabled", False, type=bool))
        self._udp_telem_host.setText(settings.value("udp_telemetry/host", "127.0.0.1"))
        self._udp_telem_port.setValue(settings.value("udp_telemetry/port", 10002, type=int))
        self._udp_cmd_cb.setChecked(settings.value("udp_command/enabled", False, type=bool))
        self._udp_cmd_host.setText(settings.value("udp_command/host", "0.0.0.0"))
        self._udp_cmd_port.setValue(settings.value("udp_command/port", 10001, type=int))
        self._udp_cmd_advanced_cb.setChecked(
            settings.value("udp_command/advanced", False, type=bool)
        )
        self._on_lsl_enabled_toggled(self._lsl_enabled_cb.isChecked())
        self._apply_stream_settings()

    def _apply_stream_settings(self):
        settings = QSettings("NML", "HandExoGUI")
        settings.setValue("lsl/enabled", self._lsl_enabled_cb.isChecked())
        settings.setValue("telemetry/rate_hz", self._telemetry_rate_spin.value())
        settings.setValue("lsl/angles", self._lsl_angles_cb.isChecked())
        settings.setValue("lsl/torque", self._lsl_torque_cb.isChecked())
        settings.setValue("udp_telemetry/enabled", self._udp_telem_cb.isChecked())
        settings.setValue("udp_telemetry/host", self._udp_telem_host.text().strip())
        settings.setValue("udp_telemetry/port", self._udp_telem_port.value())
        settings.setValue("udp_command/enabled", self._udp_cmd_cb.isChecked())
        settings.setValue("udp_command/host", self._udp_cmd_host.text().strip())
        settings.setValue("udp_command/port", self._udp_cmd_port.value())
        settings.setValue("udp_command/advanced", self._udp_cmd_advanced_cb.isChecked())

        interval_ms = max(50, round(1000 / self._telemetry_rate_spin.value()))
        self._angle_timer.setInterval(interval_ms)
        self._teleop_timer.setInterval(interval_ms)
        if self.exo_connected and not self._teleop_streaming:
            self._angle_timer.start(interval_ms)

        self._udp_telemetry.configure(
            self._udp_telem_cb.isChecked(),
            self._udp_telem_host.text(),
            self._udp_telem_port.value(),
        )
        if self._udp_telem_cb.isChecked():
            self._udp_telem_status_lbl.setText(
                f"Sending to {self._udp_telemetry.host}:{self._udp_telemetry.port}"
            )
            self._udp_telem_status_lbl.setStyleSheet("color: #27ae60;")
        else:
            self._udp_telem_status_lbl.setText("Disabled")
            self._udp_telem_status_lbl.setStyleSheet("color: #888888;")

        self._configure_lsl_outlets()

        if self._udp_command_worker.isRunning():
            self._udp_command_worker.stop()
            self._udp_command_worker.wait(1000)
        if self._udp_cmd_cb.isChecked():
            self._udp_command_worker.configure(
                self._udp_cmd_host.text(), self._udp_cmd_port.value()
            )
            self._udp_cmd_status_lbl.setText("Starting...")
            self._udp_cmd_status_lbl.setStyleSheet("color: #f39c12;")
            self._udp_command_worker.start()
        else:
            self._udp_cmd_status_lbl.setText("Disabled")
            self._udp_cmd_status_lbl.setStyleSheet("color: #888888;")

    def _on_lsl_enabled_toggled(self, enabled: bool):
        for widget in (self._lsl_angles_cb, self._lsl_torque_cb):
            widget.setEnabled(enabled)
        self._lsl_status_lbl.setText("Disabled" if not enabled else "Enabled")
        self._lsl_status_lbl.setStyleSheet(
            "color: #888888;" if not enabled else "color: #27ae60;"
        )
        if hasattr(self, "_telemetry_rate_spin"):
            self._apply_stream_settings()

    def _configure_lsl_outlets(self):
        nominal_rate = float(self._telemetry_rate_spin.value())
        enabled = self._lsl_enabled_cb.isChecked()
        self._lsl_angles.configure(
            enabled and self._lsl_angles_cb.isChecked(),
            self.motor_names,
            nominal_srate=nominal_rate,
        )
        self._lsl_torque.configure(
            enabled and self._lsl_torque_cb.isChecked(),
            self.motor_names,
            nominal_srate=nominal_rate,
        )
        errors = [e for e in (self._lsl_angles.last_error, self._lsl_torque.last_error) if e]
        if errors:
            self._lsl_status_lbl.setText(f"LSL unavailable: {errors[0]}")
            self._lsl_status_lbl.setStyleSheet("color: #c0392b;")
        elif enabled and (self._lsl_angles_cb.isChecked() or self._lsl_torque_cb.isChecked()):
            if self.motor_names:
                self._lsl_status_lbl.setText(
                    f"Publishing {len(self.motor_names)} channel(s) at telemetry refresh rate"
                )
                self._lsl_status_lbl.setStyleSheet("color: #27ae60;")
            else:
                self._lsl_status_lbl.setText("Enabled; waiting for device connection")
                self._lsl_status_lbl.setStyleSheet("color: #f39c12;")
        else:
            self._lsl_status_lbl.setText("Disabled")
            self._lsl_status_lbl.setStyleSheet("color: #888888;")

    def _on_udp_command_status(self, text: str, color: str):
        self._udp_cmd_status_lbl.setText(text)
        self._udp_cmd_status_lbl.setStyleSheet(f"color: {color};")
        self._log(f"[UDP command] {text}")

    def _set_udp_command_feedback(
        self, payload: str, sender: str, outcome: str, color: str
    ):
        preview = " ".join(payload.split())
        if len(preview) > 120:
            preview = preview[:117] + "..."
        ts = datetime.now().strftime("%H:%M:%S")
        self._udp_last_command_lbl.setText(
            f"Last received {ts} from {sender}: {preview} [{outcome}]"
        )
        self._udp_last_command_lbl.setStyleSheet(
            f"color: {color}; font-size: 10px;"
        )

    def _on_udp_command(self, payload: str, sender: str):
        if not self.exo_connected:
            self._set_udp_command_feedback(
                payload, sender, "ignored: device disconnected", "#c0392b"
            )
            self._log(f"[UDP command] Ignored from {sender}: device is not connected")
            return
        command = payload
        if payload.startswith("{"):
            try:
                command = str(json.loads(payload).get("command", "")).strip()
            except (TypeError, ValueError, json.JSONDecodeError):
                self._set_udp_command_feedback(
                    payload, sender, "rejected: invalid JSON", "#c0392b"
                )
                self._log(f"[UDP command] Invalid JSON from {sender}")
                return
        if not command:
            self._set_udp_command_feedback(
                payload, sender, "rejected: empty command", "#c0392b"
            )
            return

        if not command.startswith("set_gesture:"):
            if not self._udp_cmd_advanced_cb.isChecked():
                self._set_udp_command_feedback(
                    payload, sender, "rejected: gesture commands only", "#c0392b"
                )
                self._log(f"[UDP command] Rejected non-gesture command from {sender}: {command}")
                return
            allowed_prefixes = (
                "get_", "info", "version", "set_exo_mode:", "enable:",
                "disable:", "home:", "oled:", "set_velocity:", "set_current:",
                "stop:", "set_control_mode:", "set_command_timeout:",
            )
            if not command.startswith(allowed_prefixes):
                self._set_udp_command_feedback(
                    payload, sender, "rejected: command blocked", "#c0392b"
                )
                self._log(f"[UDP command] Blocked command from {sender}: {command}")
                return
            for prefix in ("enable:", "disable:", "home:"):
                if command.startswith(prefix):
                    target = command.removeprefix(prefix).strip()
                    try:
                        target_id = int(target)
                    except ValueError:
                        self._set_udp_command_feedback(
                            payload,
                            sender,
                            "rejected: active motor ID required",
                            "#c0392b",
                        )
                        self._log(
                            f"[UDP command] {prefix[:-1]} requires an active motor ID; "
                            f"rejected '{target}' from {sender}"
                        )
                        return
            if command.startswith(("set_velocity:", "set_current:")):
                parts = command.split(":")
                try:
                    target_id = int(parts[1])
                    target_value = float(parts[2])
                except (IndexError, ValueError):
                    self._set_udp_command_feedback(
                        payload, sender, "rejected: expected ID and value", "#c0392b"
                    )
                    return
                max_abs = 10.0 if command.startswith("set_velocity:") else 200.0
                if target_id not in self._motor_dxl_id or abs(target_value) > max_abs:
                    self._set_udp_command_feedback(
                        payload,
                        sender,
                        "rejected: inactive ID or unsafe value",
                        "#c0392b",
                    )
                    return
                required_mode = (
                    "velocity" if command.startswith("set_velocity:") else "current"
                )
                if self._direct_mode != required_mode:
                    self._set_udp_command_feedback(
                        payload,
                        sender,
                        f"rejected: set {required_mode} mode first",
                        "#c0392b",
                    )
                    return
                if target_id not in self._direct_armed_ids:
                    self._set_udp_command_feedback(
                        payload,
                        sender,
                        "rejected: motor not armed for direct control",
                        "#c0392b",
                    )
                    return
                # High-rate direct streaming commands are coalesced to avoid
                # flooding the GUI thread and serial link with stale packets.
                key = (parts[0], target_id)
                self._udp_stream_pending[key] = (command, payload, sender)
                self._angle_timer.stop()
                self._udp_direct_idle_timer.start(
                    self._direct_timeout_spin.value() + 100
                )
                now = time.monotonic()
                if now - self._udp_stream_last_status >= 0.25:
                    pending = len(self._udp_stream_pending)
                    self._set_udp_command_feedback(
                        payload,
                        sender,
                        f"streaming ({pending} latest target(s) queued)",
                        "#27ae60",
                    )
                    self._udp_stream_last_status = now
                return
            if command.startswith("set_control_mode:"):
                parts = command.split(":")
                if (
                    len(parts) != 3
                    or parts[1] != "all"
                    or parts[2] not in ("position", "current_position", "velocity", "current")
                ):
                    self._set_udp_command_feedback(
                        payload, sender, "rejected: invalid control mode", "#c0392b"
                    )
                    return
        self._set_udp_command_feedback(payload, sender, "received", "#f39c12")
        try:
            self.exo.send_command(command)
            if command.startswith("set_control_mode:all:"):
                selected_mode = command.rsplit(":", 1)[-1]
                if selected_mode in ("velocity", "current"):
                    self._direct_mode = selected_mode
                    self._angle_timer.stop()
                else:
                    self._direct_mode = None
                    self._resume_normal_polling()
            self._set_udp_command_feedback(payload, sender, "accepted", "#27ae60")
            self._log(f"[UDP command] {sender} -> {command}")
        except Exception as exc:
            self._set_udp_command_feedback(
                payload, sender, f"failed: {exc}", "#c0392b"
            )
            self._log(f"[UDP command] Failed from {sender}: {exc}")

    def _flush_udp_stream_commands(self):
        if not self.exo_connected or not self._udp_stream_pending:
            return
        pending_items = list(self._udp_stream_pending.values())
        self._udp_stream_pending.clear()
        for command, payload, sender in pending_items:
            try:
                self.exo.send_command(command)
                self._udp_stream_sent_since_status += 1
            except Exception as exc:
                self._set_udp_command_feedback(
                    payload, sender, f"failed: {exc}", "#c0392b"
                )
                self._log(f"[UDP command] Failed from {sender}: {exc}")

        now = time.monotonic()
        if now - self._udp_stream_last_status >= 0.5:
            sent = self._udp_stream_sent_since_status
            self._udp_stream_sent_since_status = 0
            self._udp_stream_last_status = now
            self._udp_last_command_lbl.setText(
                f"Streaming direct commands active ({sent} command(s) sent / 0.5 s)"
            )
            self._udp_last_command_lbl.setStyleSheet("color: #27ae60; font-size: 10px;")

    def _rebuild_teleop_table(self):
        """Repopulate the normalised-states table from the current motor list."""
        self._teleop_state_table.setRowCount(len(self.motor_names))
        for row, name in enumerate(self.motor_names):
            for col, text in enumerate([name, "\u2014"]):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                self._teleop_state_table.setItem(row, col, item)

    def _set_active_profile(self, name: str, profile: dict | None):
        """Cache the active calibration profile and update the Hand State status label."""
        self._active_cal_profile = profile
        if profile is not None:
            self._vis_status_lbl.setText(f"Profile: {name}")
            self._vis_status_lbl.setStyleSheet("color: #27ae60; font-size: 10px;")
        else:
            self._vis_status_lbl.setText("No profile loaded — showing home position")
            self._vis_status_lbl.setStyleSheet("color: #888888; font-size: 10px;")

    def _update_vis_status_dual(self):
        """Update the Hand State status label to reflect dual-mode profiles."""
        parts = []
        if self._active_cal_left:
            parts.append("L: loaded")
        if self._active_cal_right:
            parts.append("R: loaded")
        if parts:
            self._vis_status_lbl.setText("Profiles — " + ", ".join(parts))
            self._vis_status_lbl.setStyleSheet("color: #27ae60; font-size: 10px;")
        else:
            self._vis_status_lbl.setText("No profiles loaded — showing home position")
            self._vis_status_lbl.setStyleSheet("color: #888888; font-size: 10px;")

    def _rebuild_telem_table(self):
        """Populate telemetry table rows from self.motor_names after connect."""
        self._telem_table.setRowCount(len(self.motor_names))
        for row, name in enumerate(self.motor_names):
            for col in range(4):
                text = name if col == 0 else "—"
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                self._telem_table.setItem(row, col, item)

    def _poll_telemetry(self):
        self._request_device_poll(force_telemetry=True)

    def _run_bulk_serial_action(self, action_callback):
        """Run a bulk serial action with poll scheduling paused, then refresh UI."""
        was_angle_timer_active = self._angle_timer.isActive()
        self._angle_timer.stop()
        self._suspend_device_poll_requests = True
        try:
            # Let any in-flight poll finish so bulk commands don't queue behind it.
            if self._device_poll_worker.isRunning():
                self._device_poll_worker.wait(1200)
            action_callback()
        finally:
            self._suspend_device_poll_requests = False
            if was_angle_timer_active:
                self._resume_normal_polling()
        self._request_device_poll(force_telemetry=True)

    def _request_device_poll(self, force_telemetry: bool = False):
        if not self.exo_connected:
            return
        if self._suspend_device_poll_requests:
            return
        if self._device_poll_worker.isRunning():
            return
        # Telemetry now always follows the configured broadcast rate.
        include_telemetry = True
        self._device_poll_worker.configure(self.exo, include_telemetry)
        self._device_poll_worker.start()

    def _on_device_poll_completed(self, result: dict):
        if not self.exo_connected:
            return
        relative = result.get("relative")
        if relative is not None:
            self._apply_motor_angles(relative)

        positions = result.get("positions")
        torques = result.get("torques")
        currents = result.get("currents")
        if positions is None and torques is None and currents is None:
            ts = datetime.now().strftime("%H:%M:%S")
            self._telem_status_lbl.setText(f"Read failed  {ts}")
            self._telem_status_lbl.setStyleSheet("color: #c0392b;")
            return
        self._apply_telemetry_result(positions, torques, currents)

    def _apply_telemetry_result(self, positions, torques, currents):
        positions_by_name = {}
        torque_by_name = {}
        current_by_name = {}
        for name, row in self._motor_row.items():
            i      = self._motor_idx[name]
            dxl_id = self._motor_dxl_id[i] if i < len(self._motor_dxl_id) else None
            pos  = positions.get(dxl_id) if (positions is not None and dxl_id is not None) else None
            torq = torques.get(dxl_id)   if (torques   is not None and dxl_id is not None) else None
            curr = currents.get(dxl_id)  if (currents  is not None and dxl_id is not None) else None
            positions_by_name[name] = pos
            torque_by_name[name] = torq
            current_by_name[name] = curr
            self._telem_table.item(row, 1).setText(
                f"{pos:.2f}"  if pos  is not None else "—"
            )
            self._telem_table.item(row, 2).setText(
                f"{torq:.4f}" if torq is not None else "—"
            )
            self._telem_table.item(row, 3).setText(
                f"{curr:.1f}" if curr is not None else "—"
            )

        now = time.monotonic()
        actual_rate = None
        if self._last_telemetry_update_monotonic is not None:
            elapsed = now - self._last_telemetry_update_monotonic
            if elapsed > 0:
                instantaneous = 1.0 / elapsed
                if self._telemetry_rate_ema is None:
                    self._telemetry_rate_ema = instantaneous
                else:
                    self._telemetry_rate_ema = (
                        0.25 * instantaneous + 0.75 * self._telemetry_rate_ema
                    )
                actual_rate = self._telemetry_rate_ema
        self._last_telemetry_update_monotonic = now
        ts = datetime.now().strftime("%H:%M:%S")
        if actual_rate is None:
            status = (
                f"Last update OK {ts} "
                f"({self._telemetry_rate_spin.value()} Hz target)"
            )
        else:
            status = (
                f"Last update OK {ts} "
                f"({actual_rate:.1f} Hz actual / "
                f"{self._telemetry_rate_spin.value()} Hz target)"
            )
        self._telem_status_lbl.setText(status)
        self._telem_status_lbl.setStyleSheet("color: #27ae60;")
        self._publish_telemetry(
            positions_by_name, torque_by_name, current_by_name
        )

    def _publish_telemetry(
        self,
        positions: dict[str, float | None],
        torques: dict[str, float | None],
        currents: dict[str, float | None],
    ):
        self._lsl_angles.publish(positions)
        self._lsl_torque.publish(torques)
        if not self._udp_telemetry.enabled:
            return
        frame = {
            "timestamp": time.time(),
            "source": "nml_hand_exo",
            "side": (
                "dual" if self.mode_combo.currentText() == "Dual"
                else self.mode_combo.currentText().split()[0].lower()
            ),
            "joint_angles_deg": positions,
            "motor_torque_nm": torques,
            "motor_current_ma": currents,
        }
        try:
            self._udp_telemetry.publish(frame)
        except (OSError, ValueError) as exc:
            self._udp_telem_status_lbl.setText(f"Send error: {exc}")
            self._udp_telem_status_lbl.setStyleSheet("color: #c0392b;")

    def _build_header(self):
        title = QLabel("NML EXO")
        title.setObjectName("title")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Segoe UI", 28, QFont.Bold))
        self.main_layout.addWidget(title)

        line = QLabel()
        line.setObjectName("accent-line")
        line.setFixedHeight(2)
        self.main_layout.addWidget(line)
        self.main_layout.addSpacing(4)

    # -- Connection --------------------------------------------------------

    def _build_connection_section(self):
        box = QGroupBox("Connection")
        outer = QVBoxLayout()

        # --- Row 0: Mode selector ---
        row0 = QHBoxLayout()
        row0.addWidget(QLabel("Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Right Only", "Left Only", "Dual"])
        self.mode_combo.setToolTip(
            "Right Only / Left Only: single exo — shows only that side's motors.\n"
            "Dual: both exos on ONE shared port (left IDs 1-9, right IDs 11-19)."
        )
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        row0.addWidget(self.mode_combo)
        row0.addStretch()
        outer.addLayout(row0)

        # --- Row 1: Serial port. Give long USB descriptions the full width. ---
        row1 = QHBoxLayout()
        self.port_label = QLabel("Port (R):")
        self.port_combo = QComboBox()
        self.port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.port_combo.setSizeAdjustPolicy(
            QComboBox.AdjustToMinimumContentsLengthWithIcon
        )
        self.port_combo.setMinimumContentsLength(24)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.setToolTip("Rescan serial ports")
        self.refresh_btn.clicked.connect(self._refresh_ports)

        self.probe_btn = QPushButton("Probe Ports")
        self.probe_btn.setToolTip(
            "Try each serial port at the selected baud and mark the ones that answer the exo info command."
        )
        self.probe_btn.clicked.connect(self._probe_ports)

        row1.addWidget(self.port_label)
        row1.addWidget(self.port_combo, 1)
        row1.addWidget(self.refresh_btn)
        row1.addWidget(self.probe_btn)
        outer.addLayout(row1)

        # --- Row 2: Baud, connection actions, and status. ---
        row2 = QHBoxLayout()
        self.baud_combo = QComboBox()
        for b in ["9600", "57600", "115200", "230400"]:
            self.baud_combo.addItem(b)
        self.baud_combo.setCurrentText("115200")
        self.baud_combo.setMinimumContentsLength(6)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setProperty("accent", True)
        self.connect_btn.clicked.connect(self._connect)
        self._fit_button_text(self.connect_btn)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self._disconnect)
        self._fit_button_text(self.disconnect_btn)

        self.status_label = QLabel("Disconnected")
        self.status_label.setObjectName("status-disconnected")
        self.status_label.setWordWrap(True)
        self.status_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        row2.addWidget(QLabel("Baud:"))
        row2.addWidget(self.baud_combo)
        row2.addWidget(self.connect_btn)
        row2.addWidget(self.disconnect_btn)
        row2.addSpacing(8)
        row2.addWidget(self.status_label, 1)
        outer.addLayout(row2)

        # Populate port combo now that all widgets exist.
        self._refresh_ports()

        box.setLayout(outer)
        self.main_layout.addWidget(box)

    @staticmethod
    def _fit_button_text(button: QPushButton):
        """Prevent layouts from compressing a button below its readable text."""
        button.setMinimumWidth(button.sizeHint().width())
        button.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)

    def _refresh_ports(self):
        previous_port = self.port_combo.currentData()
        ports = list_ports.comports()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        for p in ports:
            self.port_combo.addItem(_format_port_label(p), p.device)
        if previous_port is not None:
            previous_index = self.port_combo.findData(previous_port)
            if previous_index >= 0:
                self.port_combo.setCurrentIndex(previous_index)
        self.port_combo.blockSignals(False)
        if hasattr(self, "status_label") and not self.exo_connected:
            self.status_label.setText(
                f"Disconnected - {len(ports)} serial port(s) found"
                if ports
                else "Disconnected - no serial ports found"
            )
            self.status_label.setObjectName("status-disconnected")
            self.status_label.setStyle(self.status_label.style())
        if hasattr(self, "connect_btn"):
            has_ports = self.port_combo.count() > 0
            self.connect_btn.setEnabled((not self.exo_connected) and has_ports)
            self.port_combo.setEnabled((not self.exo_connected) and has_ports)
            self.probe_btn.setEnabled((not self.exo_connected) and has_ports)

    def _probe_ports(self):
        ports = list_ports.comports()
        if not ports:
            QMessageBox.information(self, "No Ports", "No serial ports were found.")
            return
        baud = int(self.baud_combo.currentText())
        matches = []
        for port in ports:
            try:
                comm = SerialComm(port=port.device, baudrate=baud, response_timeout=0.5)
                exo = HandExo(
                    comm,
                    auto_connect=True,
                    verbose=False,
                    command_delimiter='\r\n',
                )
                info = exo.info(timeout=1.5)
                motors = info.get("motors", {}) if isinstance(info, dict) else {}
                if motors:
                    matches.append((port.device, port.description, info.get("side", "unknown")))
                exo.close()
            except Exception:
                try:
                    comm.close()
                except Exception:
                    pass

        if not matches:
            QMessageBox.information(
                self,
                "Probe Result",
                f"No ports answered an exo info request at {baud} baud.\n\n"
                "Tip: the USB cable is usually the USB Serial Device entry; the Bluetooth link entries often show as 'Standard Serial over Bluetooth link'.",
            )
            return

        lines = []
        for device, description, side in matches:
            label = f"{device} - {description}" if description else device
            lines.append(f"{label}  [{side}]")
        QMessageBox.information(
            self,
            "Probe Result",
            "Ports that answered the exo info command:\n\n" + "\n".join(lines),
        )

    def _on_mode_changed(self, mode_text: str):
        """Update port label and dual-mode widgets based on mode."""
        is_dual = (mode_text == "Dual")
        if mode_text == "Left Only":
            self.port_label.setText("Port (L):")
        elif mode_text == "Right Only":
            self.port_label.setText("Port (R):")
        else:  # Dual
            self.port_label.setText("Port:")
        if hasattr(self, "_cal_side_row"):
            self._cal_side_row.setVisible(is_dual)
        if hasattr(self, "_gesture_target_row"):
            self._gesture_target_row.setVisible(is_dual)
        self._refresh_profiles()

    # -- Motor Control -----------------------------------------------------

    def _build_motor_section(self):
        self.motor_box = QGroupBox("Motors")
        self.motor_layout = QVBoxLayout()

        btn_row = QHBoxLayout()
        self.enable_all_btn = QPushButton("Enable All")
        self.enable_all_btn.clicked.connect(lambda: self._motor_all("enable"))
        self.disable_all_btn = QPushButton("Disable All")
        self.disable_all_btn.clicked.connect(lambda: self._motor_all("disable"))
        self.home_all_btn = QPushButton("Home All")
        self.home_all_btn.clicked.connect(self._home_all)
        btn_row.addWidget(self.enable_all_btn)
        btn_row.addWidget(self.disable_all_btn)
        btn_row.addWidget(self.home_all_btn)
        btn_row.addStretch()
        self.motor_layout.addLayout(btn_row)

        # Panel container — populated by _build_motor_rows() on connect.
        # In single-exo mode: single-column layout with header + rows.
        # In dual mode: two side-by-side panels (Left Exo | Right Exo).
        self.motor_panel_container = QWidget()
        self._motor_panel_v = QVBoxLayout(self.motor_panel_container)
        self._motor_panel_v.setContentsMargins(0, 0, 0, 0)
        self._motor_panel_v.setSpacing(0)

        self.no_motors_label = QLabel("Connect to a device to see motors.")
        self.no_motors_label.setStyleSheet("color: #555555; padding: 8px;")
        self._motor_panel_v.addWidget(self.no_motors_label)

        self.motor_layout.addWidget(self.motor_panel_container)
        self.motor_box.setLayout(self.motor_layout)
        self.main_layout.addWidget(self.motor_box)

    def _build_motor_rows(self):
        """Rebuild the motor panel after connecting.

        Dispatches to a single-column layout (single-exo mode) or a
        two-column side-by-side layout (Dual mode).  Motor widget dicts
        carry ``name`` (display, e.g. "L:wrist"), ``cmd_name`` (bare serial
        name, always "wrist"), and ``dxl_id`` (integer Dynamixel ID).
        """
        self._clear_layout(self._motor_panel_v)
        self.motor_widgets = []

        mode = self.mode_combo.currentText() if hasattr(self, "mode_combo") else "Right Only"
        if mode == "Dual":
            self._build_motor_rows_dual()
        else:
            self._build_motor_rows_single()

    def _build_motor_rows_single(self):
        """Single-exo layout: column headers + motor rows in one column."""
        col_header = QHBoxLayout()
        for text, stretch in [("Motor", 2), ("Angle", 2), ("Status", 1), ("", 1)]:
            lbl = QLabel(text)
            lbl.setStyleSheet("color: #9a9a9a; font-size: 13px; font-weight: 600;")
            col_header.addWidget(lbl, stretch)
        self._motor_panel_v.addLayout(col_header)

        rows_v = QVBoxLayout()
        rows_v.setSpacing(0)
        rows_v.setContentsMargins(0, 0, 0, 0)
        for i, name in enumerate(self.motor_names):
            cmd_name = name[2:] if name.startswith(("L:", "R:")) else name
            row_frame, w = self._make_motor_row(i, name, cmd_name, display_name=name)
            rows_v.addWidget(row_frame)
            self.motor_widgets.append(w)
        self._motor_panel_v.addLayout(rows_v)

    def _build_motor_rows_dual(self):
        """Dual-exo layout: Left Exo and Right Exo panels side-by-side."""
        panels_h = QHBoxLayout()
        panels_h.setSpacing(10)
        panels_h.setContentsMargins(0, 4, 0, 0)

        left_panel,  left_rows  = self._make_exo_panel("Left Exo",  "left")
        right_panel, right_rows = self._make_exo_panel("Right Exo", "right")

        panels_h.addWidget(left_panel)
        panels_h.addWidget(right_panel)

        # Build widgets in motor_names order (left first, then right).
        # Each row goes into the appropriate panel's rows layout.
        for i, name in enumerate(self.motor_names):
            cmd_name = name[2:] if name.startswith(("L:", "R:")) else name
            row_frame, w = self._make_motor_row(i, name, cmd_name, display_name=cmd_name)
            if name.startswith("L:"):
                left_rows.addWidget(row_frame)
            else:
                right_rows.addWidget(row_frame)
            self.motor_widgets.append(w)

        # Add a stretch at the bottom of each panel so rows stay top-aligned
        left_rows.addStretch()
        right_rows.addStretch()

        wrapper = QWidget()
        wrapper.setLayout(panels_h)
        self._motor_panel_v.addWidget(wrapper)

    def _make_exo_panel(self, title: str, side: str):
        """Build one exo panel (header + col headers + rows area).

        Returns ``(panel_widget, rows_layout)`` where ``rows_layout`` is the
        QVBoxLayout the caller should append motor rows into.
        """
        panel = QFrame()
        panel.setFrameShape(QFrame.StyledPanel)
        panel.setStyleSheet(
            "QFrame { border: 1px solid #333333; border-radius: 4px; background: transparent; }"
        )
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(4)

        # Title row with per-exo enable/disable buttons
        hdr = QHBoxLayout()
        title_lbl = QLabel(title)
        title_lbl.setStyleSheet("font-weight: bold; color: #cccccc; font-size: 12px;")
        hdr.addWidget(title_lbl)
        hdr.addStretch()
        en_btn = QPushButton(f"Enable {title}")
        en_btn.setMinimumHeight(28)
        en_btn.setStyleSheet("padding: 4px 12px;")
        en_btn.clicked.connect(lambda _, s=side: self._motor_side("enable", s))
        dis_btn = QPushButton(f"Disable {title}")
        dis_btn.setMinimumHeight(28)
        dis_btn.setStyleSheet("padding: 4px 12px;")
        dis_btn.clicked.connect(lambda _, s=side: self._motor_side("disable", s))
        hdr.addWidget(en_btn)
        hdr.addWidget(dis_btn)
        layout.addLayout(hdr)

        # Thin separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("border: none; background: #333333; max-height: 1px;")
        sep.setFixedHeight(1)
        layout.addWidget(sep)

        # Column headers
        col_hdr = QHBoxLayout()
        for text, stretch in [("Motor", 2), ("Angle", 2), ("Status", 1), ("", 1)]:
            lbl = QLabel(text)
            lbl.setStyleSheet("color: #9a9a9a; font-size: 13px; font-weight: 600;")
            col_hdr.addWidget(lbl, stretch)
        layout.addLayout(col_hdr)

        # Motor rows area
        rows_layout = QVBoxLayout()
        rows_layout.setSpacing(0)
        rows_layout.setContentsMargins(0, 2, 0, 0)
        layout.addLayout(rows_layout)

        return panel, rows_layout

    def _make_motor_row(self, i: int, name: str, cmd_name: str, display_name: str = ""):
        """Build one motor row widget.

        Returns ``(row_frame, widget_dict)``.  ``display_name`` is what
        appears in the Motor label; defaults to ``name`` if not supplied.
        """
        if not display_name:
            display_name = name

        row = QFrame()
        row.setObjectName("motor-row")
        row.setFrameShape(QFrame.NoFrame)
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(6, 3, 6, 3)

        name_lbl   = QLabel(display_name)
        name_lbl.setStyleSheet("font-weight: bold;")
        angle_lbl  = QLabel("--")
        status_lbl = QLabel("--")

        toggle_btn = QPushButton("Enable")
        toggle_btn.setMinimumWidth(102)
        toggle_btn.setStyleSheet("padding: 4px 12px;")
        dxl_id = self._motor_dxl_id[i] if i < len(self._motor_dxl_id) else None
        toggle_btn.clicked.connect(self._make_motor_toggle(i, dxl_id))

        row_layout.addWidget(name_lbl,   2)
        row_layout.addWidget(angle_lbl,  2)
        row_layout.addWidget(status_lbl, 1)
        row_layout.addWidget(toggle_btn, 1)

        widget_dict = {
            "name":       name,      # full display name ("L:wrist" / "wrist")
            "cmd_name":   cmd_name,  # bare serial name ("wrist")
            "dxl_id":     dxl_id,   # integer Dynamixel ID; use for per-motor commands
            "angle_lbl":  angle_lbl,
            "status_lbl": status_lbl,
            "toggle_btn": toggle_btn,
            # Cached GUI belief about device torque state.
            "enabled": False,
            # Persistent user-intent lock.
            "user_disabled": False,
        }
        return row, widget_dict

    def _make_motor_toggle(self, idx, dxl_id):
        """Return a handler that enables/disables one motor by Dynamixel ID.

        Using the integer DXL ID (not the name) avoids duplicate-name
        collisions when both left (IDs 1-9) and right (IDs 11-19) motors
        share identical names (e.g. both have a motor called "wrist").
        """
        def handler():
            if not self.exo_connected:
                return
            w = self.motor_widgets[idx]
            motor_ref = dxl_id if dxl_id is not None else w["cmd_name"]
            try:
                if w["enabled"]:
                    self.exo.disable_motor(motor_ref)
                    w["enabled"] = False
                    w["user_disabled"] = True   # explicit user action — block gesture re-enable
                    if dxl_id in self._direct_armed_ids:
                        self._direct_armed_ids.discard(dxl_id)
                        self._update_direct_arm_status()
                    w["toggle_btn"].setText("Enable")
                    w["status_lbl"].setText("OFF")
                    w["status_lbl"].setStyleSheet("color: #c0392b;")
                    self._log(f"Disabled motor {motor_ref}")
                else:
                    if not self._ensure_position_control():
                        raise RuntimeError("Could not restore current-position mode")
                    self.exo.enable_motor(motor_ref)
                    w["enabled"] = True
                    w["user_disabled"] = False  # explicit user action — clear the block
                    if self._direct_mode is not None and dxl_id is not None:
                        self._direct_armed_ids.add(dxl_id)
                        w["status_lbl"].setText("DIRECT")
                        w["status_lbl"].setStyleSheet("color: #f39c12;")
                        self._update_direct_arm_status()
                    else:
                        w["status_lbl"].setText("ON")
                        w["status_lbl"].setStyleSheet("color: #27ae60;")
                    w["toggle_btn"].setText("Disable")
                    self._log(f"Enabled motor {motor_ref}")
            except Exception as e:
                self._log(f"Error toggling motor {motor_ref}: {e}")
        return handler

    # -- Gesture Control ---------------------------------------------------

    def _build_gesture_section(self):
        box = QGroupBox("Gestures")
        outer = QVBoxLayout()

        # Target selector — shown in Dual mode to choose which exo receives
        # the gesture command.  Hidden in single-exo modes.
        self._gesture_target_row = QWidget()
        target_row_layout = QHBoxLayout(self._gesture_target_row)
        target_row_layout.setContentsMargins(0, 0, 0, 0)
        target_row_layout.addWidget(QLabel("Target:"))
        self._gesture_target_combo = QComboBox()
        self._gesture_target_combo.addItems(["Both", "Left Only", "Right Only"])
        self._gesture_target_combo.setToolTip(
            "Which exo to send gesture commands to.\n"
            "'Both' enables all motors and gestures both sides."
        )
        target_row_layout.addWidget(self._gesture_target_combo)
        target_row_layout.addStretch()
        self._gesture_target_row.setVisible(False)  # shown when mode == "Dual"
        outer.addWidget(self._gesture_target_row)

        grid = QGridLayout()
        gestures = [
            ("Grasp",        "grasp"),
            ("Keygrip",      "keygrip"),
            ("Pinch Index",  "pinch_index"),
            ("Pinch Middle", "pinch_middle"),
            ("Pinch Ring",   "pinch_ring"),
            ("Peace",        "peace"),
        ]

        for row, (label, cmd) in enumerate(gestures):
            name_lbl = QLabel(label)
            name_lbl.setStyleSheet("font-weight: bold;")
            open_btn  = QPushButton("Open")
            close_btn = QPushButton("Close")
            close_btn.setProperty("accent", True)

            open_btn.clicked.connect(self._make_gesture_handler(cmd, "open"))
            close_btn.clicked.connect(self._make_gesture_handler(cmd, "close"))

            grid.addWidget(name_lbl,  row, 0)
            grid.addWidget(open_btn,  row, 1)
            grid.addWidget(close_btn, row, 2)

        grid.setColumnStretch(0, 2)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        outer.addLayout(grid)

        box.setLayout(outer)
        self.main_layout.addWidget(box)

    def _ensure_gesture_ready(self, target: str = "Both"):
        """Enable motors and apply calibration if needed before gestures.

        Parameters
        ----------
        target : str
            In Dual mode, which exo to prepare: ``"Both"``, ``"Left Only"``,
            or ``"Right Only"``.  Ignored in single-exo modes.

        Invariant: user-disabled motors must remain disabled and must not be
        moved by gesture commands until explicitly re-enabled.  Only motors
        whose ``user_disabled`` flag is False are enabled here.
        """
        if not self._ensure_position_control():
            raise RuntimeError("Could not restore current-position mode")
        if not self._gesture_ready:
            mode = self.mode_combo.currentText() if hasattr(self, "mode_combo") else "Right Only"

            # --- Apply calibration per side -----------------------------------
            if mode == "Dual":
                # Apply left profile if targeting left or both
                if target in ("Both", "Left Only"):
                    left_profile = get_default_profile_name(side="left")
                    if left_profile:
                        try:
                            self.exo.apply_calibration(left_profile,
                                                       name_to_id=self._make_name_to_id("left"))
                            self._active_cal_left = load_profile(left_profile)
                            self._log(f"Applied left calibration profile '{left_profile}'.")
                        except Exception as e:
                            self._log(f"Warning: could not apply left calibration: {e}")
                    else:
                        self._log("Warning: no default left calibration profile found.")
                # Apply right profile if targeting right or both
                if target in ("Both", "Right Only"):
                    right_profile = get_default_profile_name(side="right")
                    if right_profile:
                        try:
                            self.exo.apply_calibration(right_profile,
                                                       name_to_id=self._make_name_to_id("right"))
                            self._active_cal_right = load_profile(right_profile)
                            self._log(f"Applied right calibration profile '{right_profile}'.")
                        except Exception as e:
                            self._log(f"Warning: could not apply right calibration: {e}")
                    else:
                        self._log("Warning: no default right calibration profile found.")
                self._update_vis_status_dual()
            else:
                default_profile = get_default_profile_name()
                if default_profile:
                    try:
                        self.exo.apply_calibration(default_profile,
                                                   name_to_id=self._make_name_to_id())
                        self._set_active_profile(default_profile, load_profile(default_profile))
                        self._log(f"Applied calibration profile '{default_profile}' for gestures.")
                    except Exception as e:
                        self._log(f"Warning: could not apply calibration: {e}")
                else:
                    self._log("Warning: no calibration profile found. Gestures may not work correctly.")

            # --- Enable target motors (respect user_disabled invariant) -------
            # Determine which prefix(es) to enable in dual mode
            if mode == "Dual":
                target_prefixes = set()
                if target in ("Both", "Left Only"):
                    target_prefixes.add("L:")
                if target in ("Both", "Right Only"):
                    target_prefixes.add("R:")
            else:
                target_prefixes = None  # all motors belong to the single exo

            try:
                enabled_count = 0
                for w in self.motor_widgets:
                    if w["user_disabled"]:
                        continue
                    # In dual mode skip motors not in the target side
                    if target_prefixes is not None:
                        if not any(w["name"].startswith(p) for p in target_prefixes):
                            continue
                    motor_ref = w.get("dxl_id") or w["cmd_name"]
                    self.exo.enable_motor(motor_ref)
                    w["enabled"] = True
                    w["toggle_btn"].setText("Disable")
                    w["status_lbl"].setText("ON")
                    w["status_lbl"].setStyleSheet("color: #27ae60;")
                    enabled_count += 1
                skipped = sum(1 for w in self.motor_widgets if w["user_disabled"])
                msg = f"Enabled {enabled_count} motor(s) for gesture control."
                if skipped:
                    msg += f" {skipped} user-disabled motor(s) left off."
                self._log(msg)
            except Exception as e:
                self._log(f"Warning: could not enable motors: {e}")

            # Set gesture mode
            try:
                self.exo.send_command("set_exo_mode:gesture_fixed")
                time.sleep(0.15)
            except Exception:
                pass

            self._gesture_ready = True

    def _make_gesture_handler(self, gesture, state):
        def handler():
            if not self.exo_connected:
                QMessageBox.warning(self, "Not Connected", "Connect to a device first.")
                return
            try:
                mode = self.mode_combo.currentText()
                if mode == "Dual":
                    target = self._gesture_target_combo.currentText()
                    # One-time: apply calibration profiles and initial motor enable.
                    self._ensure_gesture_ready(target=target)
                    # Every gesture: enforce target motor enable/disable state so
                    # changing the target combo actually takes effect on each press.
                    self._apply_gesture_target_motors(target)
                else:
                    target = "single"
                    self._ensure_gesture_ready()
                cmd = f"set_gesture:{gesture}:{state}"
                self._log(f"[Gesture] target={target}  cmd={cmd}")
                self.exo.send_command(cmd)
                self._log(f"Gesture: {gesture} -> {state}")
            except Exception as e:
                self._log(f"Gesture error: {e}")
        return handler

    def _apply_gesture_target_motors(self, target: str):
        """Enable/disable motors to match the current gesture target.

        Called on *every* gesture button press so target-combo changes take
        effect immediately without a reconnect or re-calibration.

        Rules
        -----
        - ``user_disabled`` motors are never touched (user intent wins).
        - Target-side motors that are OFF  → enabled.
        - Non-target motors that are ON    → disabled (temporarily, no user intent).
        - ``user_disabled`` flag is NOT set for auto-disables (only user toggle sets it).
        """
        for w in self.motor_widgets:
            if w["user_disabled"]:
                continue  # never override explicit user action
            name = w["name"]
            motor_ref = w.get("dxl_id") or w["cmd_name"]
            is_left  = name.startswith("L:")
            is_right = name.startswith("R:")
            should_enable = (
                target == "Both"
                or (target == "Left Only"  and is_left)
                or (target == "Right Only" and is_right)
            )
            try:
                if should_enable and not w["enabled"]:
                    self.exo.enable_motor(motor_ref)
                    w["enabled"] = True
                    w["toggle_btn"].setText("Disable")
                    w["status_lbl"].setText("ON")
                    w["status_lbl"].setStyleSheet("color: #27ae60;")
                    self._log(f"  [target-enable] motor {motor_ref}")
                elif not should_enable and w["enabled"]:
                    self.exo.disable_motor(motor_ref)
                    w["enabled"] = False
                    w["toggle_btn"].setText("Enable")
                    w["status_lbl"].setText("OFF")
                    w["status_lbl"].setStyleSheet("color: #c0392b;")
                    self._log(f"  [target-disable] motor {motor_ref}")
            except Exception as e:
                self._log(f"  [target motor error] {motor_ref}: {e}")

    # -- Calibration -------------------------------------------------------

    def _build_calibration_section(self):
        box = QGroupBox("Calibration")
        layout = QVBoxLayout()

        # Side selector — visible only in Dual mode; targets which exo to
        # run calibration or apply a profile for.
        self._cal_side_row = QWidget()
        side_row_layout = QHBoxLayout(self._cal_side_row)
        side_row_layout.setContentsMargins(0, 0, 0, 0)
        side_row_layout.addWidget(QLabel("Side:"))
        self.cal_side_combo = QComboBox()
        self.cal_side_combo.addItems(["Left", "Right"])
        self.cal_side_combo.currentTextChanged.connect(self._refresh_profiles)
        side_row_layout.addWidget(self.cal_side_combo)
        side_row_layout.addStretch()
        self._cal_side_row.setVisible(False)  # shown when mode == "Dual"
        layout.addWidget(self._cal_side_row)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Profile Name:"))
        self.cal_name_input = QLineEdit()
        self.cal_name_input.setPlaceholderText("e.g. zach")
        row1.addWidget(self.cal_name_input, 2)
        self.cal_run_btn = QPushButton("Run Calibration")
        self.cal_run_btn.setProperty("accent", True)
        self.cal_run_btn.clicked.connect(self._run_calibration)
        row1.addWidget(self.cal_run_btn)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Apply Profile:"))
        self.profile_combo = QComboBox()
        self.profile_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        row2.addWidget(self.profile_combo, 2)
        self.profile_refresh_btn = QPushButton("Refresh")
        self.profile_refresh_btn.clicked.connect(self._refresh_profiles)
        row2.addWidget(self.profile_refresh_btn)
        self.apply_profile_btn = QPushButton("Apply")
        self.apply_profile_btn.clicked.connect(self._apply_profile)
        row2.addWidget(self.apply_profile_btn)
        layout.addLayout(row2)

        box.setLayout(layout)
        self.main_layout.addWidget(box)

    def _refresh_profiles(self):
        """Repopulate the profile combo filtered by the current connection mode and side.

        - Left Only  → show profiles tagged side="left" (legacy untagged profiles
                        default to "right" and are excluded, by design).
        - Right Only → show profiles tagged side="right" or with no side tag
                        (legacy backward compat: untagged profiles are right-hand).
        - Dual       → show profiles matching the cal_side_combo selection.

        Legacy profiles without a ``"side"`` key default to ``"right"``; they
        appear in Right Only and Dual/Right views but not in Left views.
        """
        self.profile_combo.clear()
        mode = self.mode_combo.currentText() if hasattr(self, "mode_combo") else "Right Only"

        if mode == "Left Only":
            filter_side = "left"
        elif mode == "Right Only":
            filter_side = "right"
        elif mode == "Dual" and hasattr(self, "cal_side_combo"):
            filter_side = self.cal_side_combo.currentText().lower()
        else:
            filter_side = None  # show all if mode is unknown

        # Use the side-specific default for the status suffix.
        default = get_default_profile_name(filter_side or "right")

        for name in list_profiles():
            profile = load_profile(name)
            p_side = (profile or {}).get("side", "right") if profile else "right"
            if filter_side is not None and p_side != filter_side:
                continue
            suffix = " (default)" if name == default else ""
            side_tag = f"[{p_side[0].upper()}] " if mode == "Dual" else ""
            self.profile_combo.addItem(f"{side_tag}{name}{suffix}", name)

    # -- ROM Assessment ----------------------------------------------------

    def _build_rom_section(self):
        box = QGroupBox("ROM Assessment")
        layout = QHBoxLayout()

        layout.addWidget(QLabel("Participant:"))
        self.rom_name_input = QLineEdit()
        self.rom_name_input.setPlaceholderText("e.g. participant01")
        layout.addWidget(self.rom_name_input, 2)

        self.rom_run_btn = QPushButton("Run ROM Test")
        self.rom_run_btn.setProperty("accent", True)
        self.rom_run_btn.clicked.connect(self._run_rom)
        layout.addWidget(self.rom_run_btn)

        box.setLayout(layout)
        self.main_layout.addWidget(box)

    # -- Log ---------------------------------------------------------------

    def _build_log_section(self):
        box = QGroupBox("Log")
        layout = QVBoxLayout()

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(80)
        self.log_text.setMaximumHeight(160)
        layout.addWidget(self.log_text)

        box.setLayout(layout)
        self.main_layout.addWidget(box)

    # -- Actions -----------------------------------------------------------

    def _connect(self):
        if self.exo_connected:
            return
        if self.port_combo.count() == 0:
            QMessageBox.warning(self, "No Ports", "No serial ports found. Click Refresh.")
            return

        mode = self.mode_combo.currentText()
        port = self.port_combo.currentData() or self.port_combo.currentText().split()[0]
        baud = int(self.baud_combo.currentText())

        try:
            # One controller, one serial connection for all modes.
            # Both hands share the same OpenRB-150 board and Dynamixel bus.
            side = "left" if mode == "Left Only" else ("right" if mode == "Right Only" else None)
            comm = SerialComm(port=port, baudrate=baud, response_timeout=0.5)
            self.exo = SynchronizedHandExo(
                HandExo(comm, side=side, auto_connect=True,
                        verbose=False, command_delimiter='\r\n')
            )
            conn_desc = f"{port} @ {baud} [{mode}]"

            self._log(f"Connecting: mode={mode}, port={port}, baud={baud}, "
                      f"expected_side={side or 'all'}")

            info = self.exo.info(timeout=5.0)
            motors_dict = info.get("motors", {})  # keyed by Dynamixel ID

            # ID ranges that define handedness on the shared bus.
            LEFT_IDS  = range(1, 10)   # IDs 1-9  → left hand
            RIGHT_IDS = range(11, 20)  # IDs 11-19 → right hand

            # Build motor_names in sorted-Dynamixel-ID order.
            # In Dual mode add L:/R: prefix; single-side modes use bare names.
            self.motor_names        = []
            self._left_motor_names  = []
            self._right_motor_names = []
            self._motor_dxl_id      = []   # _motor_dxl_id[widget_i] = Dynamixel ID

            for dxl_id in sorted(motors_dict.keys()):
                # Filter by selected mode: single-side modes skip the other side.
                # This lets dual firmware work with single-side GUI selection.
                if mode == "Left Only" and dxl_id not in LEFT_IDS:
                    continue
                if mode == "Right Only" and dxl_id not in RIGHT_IDS:
                    continue

                md = motors_dict[dxl_id]
                bare_name = md.get("name", f"motor_{dxl_id}")

                if dxl_id in LEFT_IDS:
                    self._left_motor_names.append(bare_name)
                    display_name = f"L:{bare_name}" if mode == "Dual" else bare_name
                elif dxl_id in RIGHT_IDS:
                    self._right_motor_names.append(bare_name)
                    display_name = f"R:{bare_name}" if mode == "Dual" else bare_name
                else:
                    display_name = bare_name  # unknown ID range: use bare

                self.motor_names.append(display_name)
                self._motor_dxl_id.append(dxl_id)

            self.n_motors = len(self.motor_names)
            if self.n_motors == 0:
                detected_ids = sorted(motors_dict.keys())
                reported_side = info.get("side", "unknown")
                reported_count = info.get("n_motors", "unknown")
                if detected_ids:
                    raise ConnectionError(
                        f"The device reported motor IDs {detected_ids} "
                        f"(firmware side: {reported_side}), but {mode} expects "
                        f"{'IDs 1-9' if mode == 'Left Only' else 'IDs 11-19'}. "
                        "Select the matching GUI mode or flash the intended firmware build."
                    )

                raw_preview = " ".join(info.get("_raw", "").split())[:240]
                detail = (
                    f" Parsed header: side={reported_side}, "
                    f"motor_count={reported_count}."
                    if info else ""
                )
                preview = f" Response preview: {raw_preview}" if raw_preview else ""
                raise ConnectionError(
                    f"No complete motor records were received from {port} at {baud} baud."
                    f"{detail}{preview} The firmware in this repository defaults the "
                    "OpenRB USB serial port to 115200 baud; use the exact baud that shows "
                    "readable output in Arduino Serial Monitor."
                )

            self._log(f"Detected motor IDs: {self._motor_dxl_id}")
            self._log(f"Left motors ({len(self._left_motor_names)}): {self._left_motor_names}")
            self._log(f"Right motors ({len(self._right_motor_names)}): {self._right_motor_names}")

            # Safety: disable every motor that is on the bus but outside the
            # active control subset.  The firmware's set_gesture command
            # broadcasts to ALL managed motors regardless of torque state, so
            # inactive-side motors must be disabled here to prevent them from
            # receiving and responding to gesture commands.  This only applies
            # in Left Only / Right Only mode; Dual mode includes all detected
            # IDs so the inactive set is empty.
            all_detected_ids = set(motors_dict.keys())
            active_ids       = set(self._motor_dxl_id)
            inactive_ids     = sorted(all_detected_ids - active_ids)
            if inactive_ids:
                self._log(
                    f"Disabling {len(inactive_ids)} inactive-side motor(s) "
                    f"to prevent gesture leakage: IDs {inactive_ids}"
                )
                for _inactive_id in inactive_ids:
                    try:
                        self.exo.disable_motor(_inactive_id)
                    except Exception as _e:
                        self._log(f"  Warning: could not disable inactive motor {_inactive_id}: {_e}")

            self.exo_connected = True
            self._gesture_ready = False
            self._active_cal_profile = None
            self._active_cal_left    = None
            self._active_cal_right   = None

            self.status_label.setText(f"Connected — {self.n_motors} motors")
            self.status_label.setObjectName("status-connected")
            self.status_label.setStyle(self.status_label.style())
            self._log(f"Connected: {conn_desc} — {self.n_motors} motors: {', '.join(self.motor_names)}")

            # Precompute motor lookup maps used by telemetry polling.
            # Keys are display names (with L:/R: prefix in Dual mode).
            self._motor_idx = {name: i for i, name in enumerate(self.motor_names)}
            self._motor_row = {name: row for row, name in enumerate(self.motor_names)}

            self._build_motor_rows()
            self._sync_motor_enabled_states_after_connect()
            self._rebuild_telem_table()
            self._rebuild_teleop_table()
            self._rebuild_direct_motor_combo()
            self._configure_lsl_outlets()
            self._last_telemetry_update_monotonic = None
            self._telemetry_rate_ema = None
            self._telem_status_lbl.setText("Connected — waiting for first poll")
            self._telem_status_lbl.setStyleSheet("color: #888888;")
            self._refresh_profiles()
            self._angle_timer.start(self._angle_timer.interval() or 50)
            self._request_device_poll()
        except Exception as e:
            try:
                if self.exo:
                    self.exo.close()
            except Exception:
                pass
            self.exo = None
            self.exo_connected = False
            QMessageBox.critical(self, "Connection Error", str(e))
            self._log(f"Connection failed: {e}")

        self._update_enabled_state()

    def _disconnect(self):
        # Stop teleop streaming first so the tick timer doesn't fire after
        # the serial port closes.  Also signal the WebSocket worker to exit
        # (non-blocking — its status_changed slot will clean up the Teleop UI).
        if self._teleop_streaming:
            self._on_teleop_stop()
        if self._teleop_worker.isRunning():
            self._teleop_worker.stop()
        self._stop_all_direct_control()
        self._angle_timer.stop()
        try:
            if self.exo:
                self.exo.close()
        except Exception:
            pass
        self.exo = None
        self.exo_connected = False
        self._gesture_ready = False
        self._motor_idx = {}
        self._motor_row = {}
        self._left_motor_names  = []
        self._right_motor_names = []
        self._motor_dxl_id      = []
        self.motor_names        = []
        self._direct_motor_combo.clear()
        self._direct_mode = None
        self._direct_mode_status.setText("Not configured")
        self._direct_mode_status.setStyleSheet("color: #888888;")
        self._configure_lsl_outlets()
        self._active_cal_left   = None
        self._active_cal_right  = None
        self._set_active_profile("", None)
        self._hand_vis.update_motor_states({}, connected=False)
        # Reset telemetry value cells; leave motor-name column intact
        for row in range(self._telem_table.rowCount()):
            for col in (1, 2, 3):
                item = self._telem_table.item(row, col)
                if item:
                    item.setText("—")
        self._telem_status_lbl.setText("Not connected")
        self._telem_status_lbl.setStyleSheet("color: #888888;")
        self._last_telemetry_update_monotonic = None
        self._telemetry_rate_ema = None
        self.status_label.setText("Disconnected")
        self.status_label.setObjectName("status-disconnected")
        self.status_label.setStyle(self.status_label.style())
        self._log("Disconnected.")
        self._update_enabled_state()

    def _motor_all(self, action):
        """Enable or disable all active-mode motors.

        Sends per-motor commands using the DXL IDs from ``_motor_dxl_id``
        (which is already filtered to the selected mode at connect time) instead
        of the firmware-level ``enable:all`` / ``disable:all``.  Sending ``:all``
        would act on the entire Dynamixel bus and move motors from the wrong side
        when running in Left Only or Right Only mode.
        """
        if not self.exo_connected:
            return
        try:
            if action == "enable":
                # Do not rely only on cached _direct_mode. UDP direct commands
                # can leave firmware in velocity/current mode; force position mode
                # before torque-enable so motors do not feel limp.
                self.exo.set_control_mode("current_position")
                self._direct_mode = None
                self._direct_armed_ids.clear()
                self._update_direct_arm_status()
                self._log("Set control mode to current_position before enabling motors.")
                def _enable_all(exo):
                    for dxl_id in self._motor_dxl_id:
                        exo.enable_motor(dxl_id)

                self._run_bulk_serial_action(lambda: self.exo.run_locked(_enable_all))
                for w in self.motor_widgets:
                    w["enabled"] = True
                    w["user_disabled"] = False  # explicit "Enable All" clears user-disabled
                    w["toggle_btn"].setText("Disable")
                    w["status_lbl"].setText("ON")
                    w["status_lbl"].setStyleSheet("color: #27ae60;")
                self._log(f"Enabled all motors: IDs {self._motor_dxl_id}")
            else:
                def _disable_all(exo):
                    for dxl_id in self._motor_dxl_id:
                        exo.disable_motor(dxl_id)

                self._run_bulk_serial_action(lambda: self.exo.run_locked(_disable_all))
                for w in self.motor_widgets:
                    w["enabled"] = False
                    w["user_disabled"] = True   # explicit "Disable All" marks all user-disabled
                    w["toggle_btn"].setText("Enable")
                    w["status_lbl"].setText("OFF")
                    w["status_lbl"].setStyleSheet("color: #c0392b;")
                self._log(f"Disabled all motors: IDs {self._motor_dxl_id}")
        except Exception as e:
            self._log(f"Error: {e}")

    def _sync_motor_enabled_states_after_connect(self):
        """Read per-motor torque state from firmware and update row widgets.

        Runs once after connect so each row reflects the real startup state
        instead of assuming all motors are OFF.
        """
        if not self.exo_connected or not self.motor_widgets:
            return
        try:
            enabled_by_id = self.exo.run_locked(lambda exo: exo.is_enabled("all"))
        except Exception as exc:
            self._log(f"Warning: could not read initial motor enabled states: {exc}")
            return

        if not isinstance(enabled_by_id, dict):
            self._log("Warning: unexpected enabled-state response; leaving motor row states unchanged.")
            return

        resolved = 0
        enabled_count = 0
        unresolved_ids = []
        for w in self.motor_widgets:
            dxl_id = w.get("dxl_id")
            if dxl_id is None:
                continue

            state = enabled_by_id.get(dxl_id)
            if state is None:
                unresolved_ids.append(dxl_id)
                continue

            is_enabled = bool(state)
            w["enabled"] = is_enabled
            # Keep user_disabled reserved for explicit user actions only.
            w["user_disabled"] = False
            if is_enabled:
                enabled_count += 1
                w["toggle_btn"].setText("Disable")
                w["status_lbl"].setText("ON")
                w["status_lbl"].setStyleSheet("color: #27ae60;")
            else:
                w["toggle_btn"].setText("Enable")
                w["status_lbl"].setText("OFF")
                w["status_lbl"].setStyleSheet("color: #c0392b;")
            resolved += 1

        if resolved:
            self._log(
                f"Synced initial motor states: {enabled_count}/{resolved} enabled."
            )
        if unresolved_ids:
            self._log(
                f"Warning: could not resolve enabled state for motor IDs {sorted(unresolved_ids)}."
            )

    def _motor_side(self, action: str, side: str):
        """Enable or disable all motors belonging to one exo side.

        Sends individual motor commands using bare ``cmd_name`` values.
        Only operates on the widget rows whose display name carries the
        matching side prefix (``L:`` for left, ``R:`` for right).
        """
        if not self.exo_connected:
            return
        prefix = "L:" if side == "left" else "R:"
        side_widgets = [w for w in self.motor_widgets if w["name"].startswith(prefix)]
        if not side_widgets:
            self._log(f"No {side} motors to {action}.")
            return
        try:
            if action == "enable":
                # Force position mode before side enable to avoid stale direct mode.
                self.exo.set_control_mode("current_position")
                self._direct_mode = None
                self._direct_armed_ids.clear()
                self._update_direct_arm_status()
                self._log(
                    f"Set control mode to current_position before enabling {side} motors."
                )
            side_ids = [w.get("dxl_id") for w in side_widgets if w.get("dxl_id")]
            def _toggle_side(exo):
                for dxl_id in side_ids:
                    if action == "enable":
                        exo.enable_motor(dxl_id)
                    else:
                        exo.disable_motor(dxl_id)

            self._run_bulk_serial_action(lambda: self.exo.run_locked(_toggle_side))
            for w in side_widgets:
                if action == "enable":
                    w["enabled"]       = True
                    w["user_disabled"] = False
                    w["toggle_btn"].setText("Disable")
                    w["status_lbl"].setText("ON")
                    w["status_lbl"].setStyleSheet("color: #27ae60;")
                else:
                    w["enabled"]       = False
                    w["user_disabled"] = True
                    w["toggle_btn"].setText("Enable")
                    w["status_lbl"].setText("OFF")
                    w["status_lbl"].setStyleSheet("color: #c0392b;")
            self._log(f"{action.capitalize()}d all {side} motors: IDs {side_ids}")
        except Exception as e:
            self._log(f"Error {action}ing {side} motors: {e}")

    def _make_name_to_id(self, side: str | None = None) -> dict:
        """Return a {bare_motor_name: dxl_id} map for the given side.

        Passed to ``apply_calibration(name_to_id=...)`` so calibration
        commands use explicit integer IDs instead of ambiguous bare motor
        names.  In dual firmware both "wrist" motors (left ID 1, right ID 11)
        share the same name; without explicit IDs the firmware always resolves
        to the first match (left), making right-side calibration impossible.

        Parameters
        ----------
        side : 'left', 'right', or None
            Which side to build the map for.  ``None`` is valid in Left Only /
            Right Only mode where all active motors belong to one side.
        """
        LEFT_IDS  = range(1, 10)
        RIGHT_IDS = range(11, 20)
        mode = self.mode_combo.currentText() if hasattr(self, "mode_combo") else "Right Only"

        if mode == "Dual":
            left_dxl_ids  = [i for i in self._motor_dxl_id if i in LEFT_IDS]
            right_dxl_ids = [i for i in self._motor_dxl_id if i in RIGHT_IDS]
            if side == "left":
                return dict(zip(self._left_motor_names, left_dxl_ids))
            else:  # "right" or None in Dual
                return dict(zip(self._right_motor_names, right_dxl_ids))
        else:
            # Single-side mode: motor_names are already bare and _motor_dxl_id
            # is already filtered to the active side at connect time.
            return dict(zip(self.motor_names, self._motor_dxl_id))

    def _home_all(self):
        """Home all active-mode motors.

        Sends per-motor ``home:<id>`` commands using DXL IDs from
        ``_motor_dxl_id`` (filtered to the selected mode at connect time).
        Sending ``home:all`` would move motors from the wrong side when running
        in Left Only or Right Only mode with both exos physically on the bus.
        """
        if not self.exo_connected:
            return
        try:
            if not self._ensure_position_control():
                raise RuntimeError("Could not restore current-position mode")
            for dxl_id in self._motor_dxl_id:
                self.exo.home(dxl_id)
            self._log("Homed all motors.")
        except Exception as e:
            self._log(f"Home error: {e}")

    def _apply_motor_angles(self, angles: dict):
        if not self.exo_connected:
            return
        # Source: get_angle:all → firmware getRelativeAngle (zeroed at home, flip applied).
        # HandExo returns {Dynamixel_ID: angle}; map to widget index via _motor_dxl_id.
        for i, w in enumerate(self.motor_widgets):
            dxl_id = self._motor_dxl_id[i] if i < len(self._motor_dxl_id) else None
            val = angles.get(dxl_id) if dxl_id is not None else None
            if val is not None:
                w["angle_lbl"].setText(f"{float(val):.2f} deg")

        # Normalise each relative angle to [0, 1] for the Hand State visualisation.
        t_dict: dict[str, float] = {}
        mode = self.mode_combo.currentText()

        for i, w in enumerate(self.motor_widgets):
            name   = w["name"]
            dxl_id = self._motor_dxl_id[i] if i < len(self._motor_dxl_id) else None
            val    = angles.get(dxl_id) if dxl_id is not None else None
            m      = None

            if mode == "Dual":
                # Strip L:/R: prefix to look up bare motor name in per-side profile.
                if name.startswith("L:"):
                    bare = name[2:]
                    m = (self._active_cal_left or {}).get("motors", {}).get(bare)
                elif name.startswith("R:"):
                    bare = name[2:]
                    m = (self._active_cal_right or {}).get("motors", {}).get(bare)
            else:
                m = (self._active_cal_profile or {}).get("motors", {}).get(name)

            if val is not None and m is not None:
                rel_a = normalize_angle(m["limit_min"], m["home"], m["flip"])
                rel_b = normalize_angle(m["limit_max"], m["home"], m["flip"])
                lo, hi = min(rel_a, rel_b), max(rel_a, rel_b)
                span = hi - lo
                t_dict[name] = (
                    max(0.0, min(1.0, (float(val) - lo) / span)) if span > 0 else 0.0
                )
            else:
                t_dict[name] = 0.0  # no data or no profile: show home position

        # HandSkeletonWidget uses bare motor names (right-hand view).
        # In Dual mode, pass only the right-side (or left-side if only left is present).
        if mode == "Dual":
            vis_side_pfx = "R:" if self._right_motor_names else "L:"
            bare_t_dict = {
                k[2:]: v for k, v in t_dict.items() if k.startswith(vis_side_pfx)
            }
        else:
            bare_t_dict = t_dict  # already bare names in single mode

        self._hand_vis.update_motor_states(bare_t_dict, connected=True)

    def _run_calibration(self):
        if not self.exo_connected:
            QMessageBox.warning(self, "Not Connected", "Connect to a device first.")
            return
        name = self.cal_name_input.text().strip().lower()
        if not name:
            QMessageBox.warning(self, "No Name", "Enter a profile name.")
            return

        mode = self.mode_combo.currentText()
        LEFT_IDS  = range(1, 10)
        RIGHT_IDS = range(11, 20)
        if mode == "Dual":
            cal_side = self.cal_side_combo.currentText().lower()
            side_motor_names = (
                self._left_motor_names if cal_side == "left" else self._right_motor_names
            )
            if not side_motor_names:
                QMessageBox.warning(self, "No Motors",
                                    f"No {cal_side} motors found on the connected device.")
                return
            id_range = LEFT_IDS if cal_side == "left" else RIGHT_IDS
            side_dxl_ids = [i for i in self._motor_dxl_id if i in id_range]
            dlg = CalibrationDialog(self.exo, side_motor_names, name, side=cal_side,
                                    dxl_ids=side_dxl_ids, parent=self)
        else:
            side = "left" if mode == "Left Only" else "right"
            dlg = CalibrationDialog(self.exo, self.motor_names, name, side=side,
                                    dxl_ids=list(self._motor_dxl_id), parent=self)

        dlg.exec_()
        # CalibrationDialog disables motors. Reset the flag so the next gesture
        # call re-runs _ensure_gesture_ready() and re-enables them.
        self._gesture_ready = False
        self._refresh_profiles()

        if dlg.result() == QDialog.Accepted:
            self._log(f"Calibration profile '{name}' saved.")
            ans = QMessageBox.question(
                self, "Apply Profile",
                f"Apply calibration profile '{name}' to the device now?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes,
            )
            if ans == QMessageBox.Yes:
                try:
                    if mode == "Dual":
                        self.exo.apply_calibration(name,
                                                   name_to_id=self._make_name_to_id(cal_side))
                        profile = load_profile(name)
                        if cal_side == "left":
                            self._active_cal_left = profile
                        else:
                            self._active_cal_right = profile
                        self._update_vis_status_dual()
                    else:
                        self.exo.apply_calibration(name,
                                                   name_to_id=self._make_name_to_id())
                        self._set_active_profile(name, load_profile(name))
                    self._log(f"Applied calibration profile: {name}")
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Failed to apply profile:\n{e}")
                    self._log(f"Apply profile error: {e}")

    def _apply_profile(self):
        if not self.exo_connected:
            QMessageBox.warning(self, "Not Connected", "Connect to a device first.")
            return
        name = self.profile_combo.currentData()
        if not name:
            QMessageBox.warning(self, "No Profile", "Select a profile to apply.")
            return
        mode = self.mode_combo.currentText()
        try:
            if mode == "Dual":
                cal_side = self.cal_side_combo.currentText().lower()
                self.exo.apply_calibration(name,
                                           name_to_id=self._make_name_to_id(cal_side))
                profile = load_profile(name)
                if cal_side == "left":
                    self._active_cal_left = profile
                else:
                    self._active_cal_right = profile
                self._update_vis_status_dual()
            else:
                self.exo.apply_calibration(name,
                                           name_to_id=self._make_name_to_id())
                self._set_active_profile(name, load_profile(name))
            self._log(f"Applied calibration profile: {name}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to apply profile:\n{e}")
            self._log(f"Apply profile error: {e}")

    def _run_rom(self):
        if not self.exo_connected:
            QMessageBox.warning(self, "Not Connected", "Connect to a device first.")
            return
        participant = self.rom_name_input.text().strip().lower()
        if not participant:
            QMessageBox.warning(self, "No Name", "Enter a participant name.")
            return

        mode = self.mode_combo.currentText()
        LEFT_IDS  = range(1, 10)
        RIGHT_IDS = range(11, 20)
        if mode == "Dual":
            cal_side = self.cal_side_combo.currentText().lower()
            side_motor_names = (
                self._left_motor_names if cal_side == "left" else self._right_motor_names
            )
            if not side_motor_names:
                QMessageBox.warning(self, "No Motors",
                                    f"No {cal_side} motors found on the connected device.")
                return
            id_range = LEFT_IDS if cal_side == "left" else RIGHT_IDS
            side_dxl_ids = [i for i in self._motor_dxl_id if i in id_range]
            dlg = ROMDialog(self.exo, side_motor_names, participant, side=cal_side,
                            dxl_ids=side_dxl_ids, parent=self)
        else:
            side = "left" if mode == "Left Only" else "right"
            dlg = ROMDialog(self.exo, self.motor_names, participant, side=side,
                            dxl_ids=list(self._motor_dxl_id), parent=self)

        dlg.exec_()
        self._log(f"ROM assessment complete for '{participant}'.")

        profile_name = getattr(dlg, "saved_profile_name", None)
        if profile_name:
            self._refresh_profiles()
            self._log(f"ROM-derived calibration profile '{profile_name}' saved.")
            ans = QMessageBox.question(
                self, "Apply Calibration Profile",
                f"Apply calibration profile '{profile_name}' to the device now?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes,
            )
            if ans == QMessageBox.Yes:
                try:
                    if mode == "Dual":
                        self.exo.apply_calibration(profile_name,
                                                   name_to_id=self._make_name_to_id(cal_side))
                        profile = load_profile(profile_name)
                        if cal_side == "left":
                            self._active_cal_left = profile
                        else:
                            self._active_cal_right = profile
                        self._update_vis_status_dual()
                    else:
                        self.exo.apply_calibration(profile_name,
                                                   name_to_id=self._make_name_to_id())
                        self._set_active_profile(profile_name, load_profile(profile_name))
                    self._log(f"Applied calibration profile: {profile_name}")
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Failed to apply profile:\n{e}")
                    self._log(f"Apply profile error: {e}")

    def _update_enabled_state(self):
        on = self.exo_connected
        has_ports = self.port_combo.count() > 0
        self.connect_btn.setEnabled((not on) and has_ports)
        self.disconnect_btn.setEnabled(on)
        # Disable connection controls while connected so the serial target is stable.
        self.mode_combo.setEnabled(not on)
        self.port_combo.setEnabled((not on) and has_ports)
        self.baud_combo.setEnabled(not on)
        self.refresh_btn.setEnabled(not on)
        self.enable_all_btn.setEnabled(on)
        self.disable_all_btn.setEnabled(on)
        self.home_all_btn.setEnabled(on)
        self.cal_run_btn.setEnabled(on)
        self.apply_profile_btn.setEnabled(on)
        self.rom_run_btn.setEnabled(on)
        # Teleop: Start Streaming requires both exo and WS to be connected.
        # If exo just disconnected while streaming was active, _on_teleop_stop()
        # has already been called from _disconnect(), so _teleop_streaming=False.
        self._teleop_start_btn.setEnabled(
            on and self._teleop_ws_connected and not self._teleop_streaming
        )
        for widget in (
            self._direct_apply_mode_btn,
            self._direct_position_btn,
            self._direct_arm_btn,
            self._direct_send_btn,
            self._direct_zero_btn,
            self._direct_stop_all_btn,
        ):
            widget.setEnabled(on)

    # -- Direct control tab handlers --------------------------------------

    def _on_direct_mode_selection_changed(self, text: str):
        self._zero_direct_target()
        if text == "Velocity":
            self._direct_command_spin.setRange(-10.0, 10.0)
            self._direct_command_spin.setSuffix(" rpm")
        else:
            self._direct_command_spin.setRange(-200.0, 200.0)
            self._direct_command_spin.setSuffix(" mA")

    def _rebuild_direct_motor_combo(self):
        self._direct_motor_combo.clear()
        for name, dxl_id in zip(self.motor_names, self._motor_dxl_id):
            self._direct_motor_combo.addItem(f"{name} (ID {dxl_id})", dxl_id)
        self._update_direct_arm_status()

    def _selected_direct_motor_id(self) -> int | None:
        value = self._direct_motor_combo.currentData()
        return int(value) if value is not None else None

    def _apply_direct_mode(self):
        if not self.exo_connected:
            return
        self._stop_all_direct_control()
        mode = (
            "velocity"
            if self._direct_mode_combo.currentText() == "Velocity"
            else "current"
        )
        try:
            for dxl_id in self._motor_dxl_id:
                self.exo.disable_motor(dxl_id)
            self.exo.set_direct_command_timeout(self._direct_timeout_spin.value())
            self.exo.set_control_mode(mode)
            self._direct_mode = mode
            self._angle_timer.stop()
            self._direct_armed_ids.clear()
            for motor in self.motor_widgets:
                motor["enabled"] = False
                motor["toggle_btn"].setText("Enable")
                motor["status_lbl"].setText("OFF")
                motor["status_lbl"].setStyleSheet("color: #c0392b;")
            self._direct_mode_status.setText(
                f"{mode.title()} mode; torque off"
            )
            self._direct_mode_status.setStyleSheet("color: #f39c12;")
            self._update_direct_arm_status()
            self._log(
                f"[Direct] Applied {mode} mode with "
                f"{self._direct_timeout_spin.value()} ms watchdog."
            )
        except Exception as exc:
            self._direct_mode = None
            self._direct_mode_status.setText(f"Mode error: {exc}")
            self._direct_mode_status.setStyleSheet("color: #c0392b;")
            self._log(f"[Direct] Mode change failed: {exc}")

    def _restore_position_control(self) -> bool:
        if not self.exo_connected:
            return False
        self._stop_all_direct_control()
        try:
            for dxl_id in self._motor_dxl_id:
                self.exo.disable_motor(dxl_id)
            self.exo.set_control_mode("current_position")
            self._direct_mode = None
            self._direct_armed_ids.clear()
            self._direct_mode_status.setText(
                "Current-position mode; torque off"
            )
            self._direct_mode_status.setStyleSheet("color: #27ae60;")
            self._update_direct_arm_status()
            self._log("[Direct] Returned to current-position control.")
            self._resume_normal_polling(force_refresh=True)
            return True
        except Exception as exc:
            self._log(f"[Direct] Could not restore position control: {exc}")
            return False

    def _ensure_position_control(self) -> bool:
        if self._direct_mode is None:
            return True
        return self._restore_position_control()

    def _on_direct_arm_toggled(self, checked: bool):
        if not checked:
            self._set_direct_motor_armed(False)
            return
        if self._set_direct_motor_armed(True):
            return
        # Revert toggle when arming failed or was cancelled.
        self._direct_arm_btn.blockSignals(True)
        self._direct_arm_btn.setChecked(False)
        self._direct_arm_btn.blockSignals(False)

    def _set_direct_motor_armed(self, armed: bool) -> bool:
        if not self.exo_connected or self._direct_mode is None:
            if armed:
                QMessageBox.warning(
                    self,
                    "Direct Mode Not Ready",
                    "Apply a direct-control mode before arming a motor.",
                )
            return False

        dxl_id = self._selected_direct_motor_id()
        if dxl_id is None:
            return False

        if armed and self._direct_arm_confirm_cb.isChecked():
            answer = QMessageBox.warning(
                self,
                "Arm Direct Control",
                f"Enable direct {self._direct_mode} control for motor ID {dxl_id}?\n\n"
                "Keep the mechanism clear. Releasing Hold to Command sends zero.",
                QMessageBox.Yes | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            if answer != QMessageBox.Yes:
                return False

        try:
            if armed:
                self.exo.stop_direct_control(dxl_id)
                self.exo.enable_motor(dxl_id)
                self._direct_armed_ids.add(dxl_id)
            else:
                self.exo.stop_direct_control(dxl_id)
                self.exo.disable_motor(dxl_id)
                self._direct_armed_ids.discard(dxl_id)

            for motor in self.motor_widgets:
                if motor.get("dxl_id") != dxl_id:
                    continue
                motor["enabled"] = armed
                motor["toggle_btn"].setText("Disable" if armed else "Enable")
                motor["status_lbl"].setText("DIRECT" if armed else "OFF")
                motor["status_lbl"].setStyleSheet(
                    "color: #f39c12;" if armed else "color: #c0392b;"
                )
                break

            self._update_direct_arm_status()
            self._log(
                f"[Direct] {'Armed' if armed else 'Disarmed'} motor ID {dxl_id}."
            )
            return True
        except Exception as exc:
            self._log(
                f"[Direct] Could not {'arm' if armed else 'disarm'} motor ID {dxl_id}: {exc}"
            )
            return False

    def _arm_direct_motor(self):
        # Backward-compatible shim: arm currently selected motor.
        self._set_direct_motor_armed(True)

    def _update_direct_arm_status(self):
        if not hasattr(self, "_direct_arm_status"):
            return
        dxl_id = self._selected_direct_motor_id()
        armed = dxl_id is not None and dxl_id in self._direct_armed_ids
        if hasattr(self, "_direct_arm_btn"):
            self._direct_arm_btn.blockSignals(True)
            self._direct_arm_btn.setChecked(armed)
            self._direct_arm_btn.blockSignals(False)
        if armed:
            self._direct_arm_status.setText(f"Motor ID {dxl_id} armed")
            self._direct_arm_status.setStyleSheet("color: #27ae60;")
        else:
            self._direct_arm_status.setText("Selected motor is not armed")
            self._direct_arm_status.setStyleSheet("color: #888888;")

    def _start_direct_command(self):
        dxl_id = self._selected_direct_motor_id()
        if (
            not self.exo_connected
            or self._direct_mode is None
            or dxl_id not in self._direct_armed_ids
        ):
            self._direct_send_btn.setDown(False)
            self._update_direct_arm_status()
            return
        try:
            # Direct mode changes torque state globally; force selected motor on
            # at command start so "DIRECT" never maps to a torque-off motor.
            self.exo.enable_motor(dxl_id)
            for motor in self.motor_widgets:
                if motor.get("dxl_id") == dxl_id:
                    motor["enabled"] = True
                    motor["user_disabled"] = False
                    motor["toggle_btn"].setText("Disable")
                    motor["status_lbl"].setText("DIRECT")
                    motor["status_lbl"].setStyleSheet("color: #f39c12;")
                    break
        except Exception as exc:
            self._log(f"[Direct] Could not enable motor ID {dxl_id}: {exc}")
            self._direct_send_btn.setDown(False)
            return
        self._direct_command_active = True
        self._angle_timer.stop()
        self._send_direct_command_tick()
        self._direct_command_timer.start()

    def _send_direct_command_tick(self):
        if not self._direct_command_active or not self.exo_connected:
            return
        dxl_id = self._selected_direct_motor_id()
        if dxl_id is None or dxl_id not in self._direct_armed_ids:
            self._zero_direct_target()
            return
        value = self._direct_command_spin.value()
        try:
            if self._direct_mode == "velocity":
                self.exo.set_direct_velocity(dxl_id, value)
            elif self._direct_mode == "current":
                self.exo.set_direct_current(dxl_id, value)
        except Exception as exc:
            self._log(f"[Direct] Command failed for motor ID {dxl_id}: {exc}")
            self._zero_direct_target()

    def _zero_direct_target(self):
        self._direct_command_active = False
        if hasattr(self, "_direct_command_timer"):
            self._direct_command_timer.stop()
        if not self.exo_connected:
            return
        dxl_id = self._selected_direct_motor_id()
        if dxl_id is None:
            return
        try:
            self.exo.stop_direct_control(dxl_id)
        except Exception as exc:
            self._log(f"[Direct] Zero target failed for motor ID {dxl_id}: {exc}")
        self._resume_normal_polling()

    def _stop_all_direct_control(self):
        self._direct_command_active = False
        if hasattr(self, "_direct_command_timer"):
            self._direct_command_timer.stop()
        if self.exo_connected:
            try:
                self.exo.stop_direct_control("all")
                for dxl_id in list(self._direct_armed_ids):
                    self.exo.disable_motor(dxl_id)
            except Exception as exc:
                self._log(f"[Direct] Stop all failed: {exc}")
        self._direct_armed_ids.clear()
        self._update_direct_arm_status()
        if hasattr(self, "_direct_mode_status") and self._direct_mode is not None:
            self._direct_mode_status.setText(
                f"{self._direct_mode.title()} mode; all targets stopped"
            )
            self._direct_mode_status.setStyleSheet("color: #f39c12;")
        self._resume_normal_polling()

    def _resume_normal_polling(self, force_refresh: bool = False):
        if (
            self.exo_connected
            and not self._teleop_streaming
            and self._direct_mode is None
        ):
            self._angle_timer.start(self._angle_timer.interval() or 50)
            if force_refresh:
                self._request_device_poll(force_telemetry=True)

    # -- Teleop tab handlers -----------------------------------------------

    def _on_teleop_connect(self):
        """Connect to the WebSocket server in the background worker thread."""
        if not _WEBSOCKETS_AVAILABLE:
            QMessageBox.critical(
                self, "Missing Package",
                "Install websockets first:\n\n    pip install websockets"
            )
            return
        url = self._teleop_addr_edit.text().strip()
        if not (url.startswith("ws://") or url.startswith("wss://")):
            QMessageBox.warning(
                self, "Invalid Address",
                "WebSocket address must start with ws:// or wss://"
            )
            return
        if self._teleop_worker.isRunning():
            return

        self._teleop_worker.configure(url)
        self._teleop_connect_btn.setEnabled(False)
        self._teleop_addr_edit.setEnabled(False)
        self._teleop_ws_status_lbl.setText("\u25cf  Connecting\u2026")
        self._teleop_ws_status_lbl.setStyleSheet("color: #f39c12;")
        self._teleop_worker.start()
        self._teleop_ws_disconnect_btn.setEnabled(True)

    def _on_teleop_ws_disconnect(self):
        """Disconnect from the WebSocket server and stop streaming if active."""
        self._on_teleop_stop()
        self._teleop_worker.stop()
        # Non-blocking: the status_changed signal will fire from the worker
        # thread once it closes and will finish resetting the UI.
        self._teleop_ws_disconnect_btn.setEnabled(False)

    def _on_teleop_status(self, msg: str, color: str):
        """
        Slot for TeleopWorker.status_changed.  Runs on the GUI thread.

        Green  → WS is open; enable Start Streaming if exo is also connected.
        Other  → WS closed or errored; stop streaming, re-enable Connect.
        """
        self._teleop_ws_status_lbl.setText(f"\u25cf  {msg}")
        self._teleop_ws_status_lbl.setStyleSheet(f"color: {color};")
        self._teleop_ws_connected = (color == "#27ae60")

        if self._teleop_ws_connected:
            # Only allow Start Streaming if an exo is also connected.
            self._teleop_start_btn.setEnabled(
                self.exo_connected and not self._teleop_streaming
            )
        else:
            # Connection lost or failed — stop streaming and reset connect UI.
            self._on_teleop_stop()
            self._teleop_start_btn.setEnabled(False)
            self._teleop_connect_btn.setEnabled(True)
            self._teleop_addr_edit.setEnabled(True)
            self._teleop_ws_disconnect_btn.setEnabled(False)

        self._log(f"[Teleop WS] {msg}")

    def _on_teleop_frame_sent(self, payload: str):
        try:
            frame = json.loads(payload)
            if frame.get("side") == "dual":
                joints = sum(
                    len(frame.get(key, {})) for key in ("left", "right")
                )
            else:
                joints = len(frame.get("joints", {}))
            detail = f"{frame.get('side', 'unknown')} side, {joints} joints"
        except (TypeError, ValueError, json.JSONDecodeError):
            detail = f"{len(payload)} bytes"
        ts = datetime.now().strftime("%H:%M:%S")
        self._teleop_last_sent_lbl.setText(f"Last frame sent {ts}: {detail}")
        self._teleop_last_sent_lbl.setStyleSheet(
            "color: #27ae60; font-size: 10px;"
        )

    def _on_teleop_start(self):
        """
        Begin teleop streaming.

        Safety procedure (mirrors CalibrationDialog / ROMDialog):
          1. Disable torque on all motors — exo becomes a pure sensor.
          2. Reset gesture_ready so the next gesture call re-enables motors
             intentionally through _ensure_gesture_ready().
          3. Suspend _angle_timer so the serial bus carries exactly one
             get_angle:all request per configured teleop tick.
        """
        if not self.exo_connected:
            QMessageBox.warning(
                self, "Not Connected",
                "Connect to the exoskeleton first."
            )
            return
        if self._teleop_streaming:
            return

        # -- Disable all motors (torque-off) --------------------------------
        try:
            for dxl_id in self._motor_dxl_id:
                self.exo.disable_motor(dxl_id)
            for w in self.motor_widgets:
                w["enabled"] = False
                w["user_disabled"] = True   # block automatic re-enable
                w["toggle_btn"].setText("Enable")
                w["status_lbl"].setText("OFF")
                w["status_lbl"].setStyleSheet("color: #c0392b;")
        except Exception as exc:
            self._log(f"[Teleop] Warning: could not disable motors: {exc}")

        # Reset gesture state so gestures don't fire while streaming.
        self._gesture_ready = False

        # Suspend normal polling; the configured teleop tick takes over.
        self._angle_timer.stop()

        # Populate the live-states table with current motor names.
        self._rebuild_teleop_table()

        # -- Start streaming ------------------------------------------------
        self._teleop_streaming = True
        self._teleop_timer.start(self._teleop_timer.interval() or 50)

        self._teleop_start_btn.setEnabled(False)
        self._teleop_stop_btn.setEnabled(True)
        self._teleop_stream_status_lbl.setText(
            f"\u25cf  Streaming  ({self._telemetry_rate_spin.value()} Hz target)"
        )
        self._teleop_stream_status_lbl.setStyleSheet("color: #27ae60;")
        self._log("[Teleop] Streaming started — motors disabled.")

    def _on_teleop_stop(self):
        """Stop streaming.  Motors stay disabled until the user re-enables them."""
        if not self._teleop_streaming:
            return
        self._teleop_streaming = False
        self._teleop_timer.stop()

        # Restart normal polling at the configured target if still connected.
        if self.exo_connected:
            self._angle_timer.start(self._angle_timer.interval() or 50)

        self._teleop_start_btn.setEnabled(
            self.exo_connected and self._teleop_ws_connected
        )
        self._teleop_stop_btn.setEnabled(False)
        self._teleop_stream_status_lbl.setText("\u25cf  Idle")
        self._teleop_stream_status_lbl.setStyleSheet("color: #888888;")
        self._log("[Teleop] Streaming stopped.")

    def _teleop_tick(self):
        """
        Timer callback at the configured target rate. Runs on the GUI thread.

        Responsibilities while streaming is active:
          1. Poll relative motor angles from the device (one serial round-trip).
          2. Normalise each angle to [0, 1] against the active calibration profile.
          3. Update the motor angle labels in the Controls tab (same as
             _poll_motor_angles normally does at 500 ms).
          4. Push normalised values to the Hand State visualisation.
          5. Refresh the live Teleop state table.
          6. Enqueue a compact JSON frame for the WebSocket worker thread.

        Normalisation formula (identical to _poll_motor_angles):
            rel_a  = normalize_angle(limit_min, home, flip)
            rel_b  = normalize_angle(limit_max, home, flip)
            lo, hi = min(rel_a, rel_b), max(rel_a, rel_b)
            t      = clamp( (relative_angle - lo) / (hi - lo), 0, 1 )

        Convention: 0 = fully open / extended, 1 = fully closed / flexed.
        This matches the convention used by HandSkeletonWidget.

        Joints that lack a calibration entry are transmitted as ``null`` in
        the JSON payload so downstream consumers can detect and ignore them
        rather than acting on a meaningless zero.

        JSON payload structure
        ----------------------
        Single-exo mode:
        {
          "timestamp": <float, Unix seconds>,
          "source":    "hand_exo",
          "side":      "left" | "right",
          "joints": {
            "<name>": <float 0-1> | null,   -- null = no calibration
            ...
          }
        }

        Dual-exo mode (both hands on one shared controller):
        {
          "timestamp": <float, Unix seconds>,
          "source":    "hand_exo",
          "side":      "dual",
          "left":  { "<name>": <float 0-1> | null, ... },
          "right": { "<name>": <float 0-1> | null, ... }
        }
        """
        if not self.exo_connected:
            return

        # -- 1. Poll relative angles from device ---------------------------
        # HandExo returns {Dynamixel_ID: angle}; map to widget via _motor_dxl_id.
        try:
            angles: dict = self.exo.get_motor_angle('all')
        except Exception:
            return

        mode = self.mode_combo.currentText()

        # -- 2 & 3. Normalise and update angle labels ----------------------
        t_dict: dict[str, float] = {}
        joints_left:  dict = {}
        joints_right: dict = {}
        joints_single: dict = {}

        for i, w in enumerate(self.motor_widgets):
            name   = w["name"]
            bare   = w.get("cmd_name", name)   # bare name for cal lookup
            dxl_id = self._motor_dxl_id[i] if i < len(self._motor_dxl_id) else None
            val    = angles.get(dxl_id) if dxl_id is not None else None

            # Update Controls-tab angle label (same display as _poll_motor_angles)
            if val is not None:
                w["angle_lbl"].setText(f"{float(val):.2f} deg")

            # Select the correct calibration profile entry for this motor
            if mode == "Dual":
                if name.startswith("L:"):
                    m = (self._active_cal_left or {}).get("motors", {}).get(bare)
                else:
                    m = (self._active_cal_right or {}).get("motors", {}).get(bare)
            else:
                m = (self._active_cal_profile or {}).get("motors", {}).get(name)

            if val is not None and m is not None:
                rel_a = normalize_angle(m["limit_min"], m["home"], m["flip"])
                rel_b = normalize_angle(m["limit_max"], m["home"], m["flip"])
                lo, hi = min(rel_a, rel_b), max(rel_a, rel_b)
                span = hi - lo
                t = (
                    max(0.0, min(1.0, (float(val) - lo) / span))
                    if span > 0 else 0.0
                )
                t_dict[name] = t
                norm_val = round(t, 4)
            else:
                t_dict[name] = 0.0
                norm_val = None  # no calibration data

            if mode == "Dual":
                if name.startswith("L:"):
                    joints_left[bare]  = norm_val
                else:
                    joints_right[bare] = norm_val
            else:
                joints_single[name] = norm_val

        # -- 4. Hand State visualisation -----------------------------------
        if mode == "Dual":
            vis_side_pfx = "R:" if self._right_motor_names else "L:"
            bare_t = {k[2:]: v for k, v in t_dict.items() if k.startswith(vis_side_pfx)}
        else:
            bare_t = t_dict
        self._hand_vis.update_motor_states(bare_t, connected=True)

        # -- 5. Live Teleop state table ------------------------------------
        for row, mw in enumerate(self.motor_widgets):
            item = self._teleop_state_table.item(row, 1)
            if item is None:
                continue
            bare = mw.get("cmd_name", mw["name"])
            if mode == "Dual":
                v = (joints_left if mw["name"].startswith("L:") else joints_right).get(bare)
            else:
                v = joints_single.get(mw["name"])
            item.setText(f"{v:.3f}" if v is not None else "no cal")

        # -- 6. Enqueue JSON frame for the WebSocket worker ----------------
        if self._teleop_worker.isRunning():
            if mode == "Dual":
                frame = {
                    "timestamp": time.time(),
                    "source": "hand_exo",
                    "side": "dual",
                    "left":  joints_left,
                    "right": joints_right,
                }
            else:
                frame = {
                    "timestamp": time.time(),
                    "source": "hand_exo",
                    "side": self.exo.side or "right",
                    "joints": joints_single,
                }
            self._teleop_worker.enqueue(json.dumps(frame, separators=(",", ":")))

    def closeEvent(self, event):
        self._angle_timer.stop()
        self._teleop_timer.stop()
        self._direct_command_timer.stop()
        self._stop_all_direct_control()
        if self._teleop_worker.isRunning():
            self._teleop_worker.stop()
            self._teleop_worker.wait(1000)
        if self._udp_command_worker.isRunning():
            self._udp_command_worker.stop()
            self._udp_command_worker.wait(1000)
        self._udp_telemetry.close()
        self._lsl_angles.close()
        self._lsl_torque.close()
        try:
            if self.exo:
                self.exo.close()
        except Exception:
            pass
        event.accept()

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")

    def _clear_layout(self, layout):
        while layout.count():
            item = layout.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()
            elif item.layout():
                self._clear_layout(item.layout())


# ==========================================================================
#  Entry point
# ==========================================================================

def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLE)
    # Qt already performs DPI scaling on Windows. Scaling the point size again
    # makes controls oversized and causes otherwise responsive rows to clip.
    app.setFont(QFont("Segoe UI", 10))

    screen = app.primaryScreen()

    window = HandExoGUI()

    # Keep controls readable below this size, but never demand more space than
    # the current screen can provide.
    if screen:
        geom = screen.availableGeometry()
        min_width = min(900, geom.width())
        min_height = min(650, geom.height())
        window.setMinimumSize(min_width, min_height)
        window.resize(
            min(geom.width(), max(min_width, int(geom.width() * 0.85))),
            min(geom.height(), max(min_height, int(geom.height() * 0.85))),
        )
    else:
        window.setMinimumSize(900, 650)
        window.resize(1100, 760)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
