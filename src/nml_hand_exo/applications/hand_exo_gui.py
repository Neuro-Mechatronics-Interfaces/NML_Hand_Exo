"""
NML EXO -Hand Exoskeleton Control GUI

Dark-themed PyQt5 application for controlling the NML Hand Exoskeleton.
Features: device connection, motor control, gesture control, interactive
calibration, and ROM assessment -all from the UI.
"""

import csv
import json
import os
import re
import sys
import time
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QLineEdit, QTextEdit, QGridLayout, QMessageBox, QGroupBox, QComboBox,
    QDialog, QScrollArea, QFrame, QSizePolicy, QSpacerItem,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QFontMetrics

from serial.tools import list_ports
from nml_hand_exo.interface import HandExo, SerialComm


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


def get_default_profile_name() -> str | None:
    if not os.path.exists(CONFIG_FILE):
        return None
    with open(CONFIG_FILE, "r") as f:
        return json.load(f).get("default")


def save_profile(name: str, data: dict):
    os.makedirs(PROFILES_DIR, exist_ok=True)
    path = os.path.join(PROFILES_DIR, f"{name}.json")
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def set_default_profile(name: str):
    cfg = {}
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r") as f:
            cfg = json.load(f)
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
"""


# ==========================================================================
#  Calibration Dialog
# ==========================================================================

class CalibrationDialog(QDialog):
    """Interactive calibration dialog -walks through open/closed positions."""

    def __init__(self, exo: HandExo, motor_names: list[str],
                 profile_name: str, parent=None):
        super().__init__(parent)
        self.exo = exo
        self.motor_names = motor_names
        self.profile_name = profile_name
        self.setWindowTitle("Calibration Protocol")
        self.setMinimumWidth(500)
        self._step = 0  # 0=open, 1=closed, 2=done
        self.open_angles = {}
        self.close_angles = {}

        layout = QVBoxLayout(self)

        self.info_label = QLabel("Step 1: Move ALL fingers to the FULLY OPEN position.")
        self.info_label.setWordWrap(True)
        self.info_label.setStyleSheet("font-size: 14px; padding: 8px;")
        layout.addWidget(self.info_label)

        self.result_label = QLabel("")
        self.result_label.setWordWrap(True)
        self.result_label.setStyleSheet("color: #aaaaaa; padding: 4px;")
        layout.addWidget(self.result_label)

        btn_row = QHBoxLayout()
        self.record_btn = QPushButton("Record Open Position")
        self.record_btn.setProperty("accent", True)
        self.record_btn.clicked.connect(self._record)
        btn_row.addStretch()
        btn_row.addWidget(self.record_btn)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        # Disable motors for free movement
        try:
            self.exo.disable_motor('all')
        except Exception:
            pass

    def _record(self):
        if self._step == 0:
            # Record open positions
            try:
                angles = self.exo.get_absolute_motor_angle('all')
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to read motor angles:\n{e}")
                return

            for mid, name in enumerate(self.motor_names):
                val = angles.get(mid, angles.get(name, 0.0))
                self.open_angles[name] = float(val)

            lines = [f"  {n:<12} {self.open_angles[n]:.2f} deg" for n in self.motor_names]
            self.result_label.setText("Open positions recorded:\n" + "\n".join(lines))

            self._step = 1
            self.info_label.setText("Step 2: Move ALL fingers to the FULLY CLOSED position.")
            self.record_btn.setText("Record Closed Position")

        elif self._step == 1:
            # Record closed positions
            try:
                angles = self.exo.get_absolute_motor_angle('all')
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to read motor angles:\n{e}")
                return

            for mid, name in enumerate(self.motor_names):
                val = angles.get(mid, angles.get(name, 0.0))
                self.close_angles[name] = float(val)

            lines = [f"  {n:<12} {self.close_angles[n]:.2f} deg" for n in self.motor_names]
            self.result_label.setText("Closed positions recorded:\n" + "\n".join(lines))

            # Compute and save profile
            self._save_profile()
            self._step = 2
            self.info_label.setText(f"Calibration complete! Profile '{self.profile_name}' saved.")
            self.record_btn.setText("Close")
            self.record_btn.clicked.disconnect()
            self.record_btn.clicked.connect(self.accept)

    def _save_profile(self):
        data = {"motors": {}}
        for name in self.motor_names:
            o = self.open_angles[name]
            c = self.close_angles[name]
            lo = min(o, c)
            hi = max(o, c)
            flip = c < o
            data["motors"][name] = {
                "home": round(o, 2),
                "limit_min": round(lo, 2),
                "limit_max": round(hi, 2),
                "flip": flip,
            }
        save_profile(self.profile_name, data)

        # Set as default if first profile
        if get_default_profile_name() is None:
            set_default_profile(self.profile_name)


# ==========================================================================
#  ROM Assessment Dialog
# ==========================================================================

class ROMDialog(QDialog):
    """ROM assessment dialog with in-GUI recording (no terminal input)."""

    def __init__(self, exo: HandExo, motor_names: list[str],
                 participant: str, parent=None):
        super().__init__(parent)
        self.exo = exo
        self.motor_names = motor_names
        self.participant = participant
        self.setWindowTitle("ROM Assessment")
        self.setMinimumWidth(600)

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

        # Disable motors
        try:
            self.exo.disable_motor('all')
        except Exception:
            pass

    def _detect_orientation(self) -> dict:
        """Auto-detect motor orientation from the default/applied calibration profile."""
        orientation = {}
        # Try loading the default profile
        default_name = get_default_profile_name()
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
            for i, name in enumerate(self.motor_names):
                val = angles.get(i, None)
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

        self._build_ui()

        # Motor angle poll timer
        self._angle_timer = QTimer(self)
        self._angle_timer.timeout.connect(self._poll_motor_angles)

    # -- UI Construction ---------------------------------------------------

    def _build_ui(self):
        # Outer scroll area for screen scaling
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        outer.addWidget(scroll)

        container = QWidget()
        self.main_layout = QVBoxLayout(container)
        self.main_layout.setSpacing(10)
        self.main_layout.setContentsMargins(16, 12, 16, 12)
        scroll.setWidget(container)

        self._build_header()
        self._build_connection_section()
        self._build_motor_section()
        self._build_gesture_section()
        self._build_calibration_section()
        self._build_rom_section()
        self._build_log_section()

        self.main_layout.addStretch()
        self._update_enabled_state()

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
        layout = QHBoxLayout()

        self.port_combo = QComboBox()
        self.port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._refresh_ports()

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)

        self.baud_combo = QComboBox()
        for b in ["57600", "115200", "230400"]:
            self.baud_combo.addItem(b)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setProperty("accent", True)
        self.connect_btn.clicked.connect(self._connect)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self._disconnect)

        self.status_label = QLabel("Disconnected")
        self.status_label.setObjectName("status-disconnected")

        layout.addWidget(QLabel("Port:"))
        layout.addWidget(self.port_combo, 3)
        layout.addWidget(self.refresh_btn)
        layout.addWidget(QLabel("Baud:"))
        layout.addWidget(self.baud_combo, 1)
        layout.addWidget(self.connect_btn)
        layout.addWidget(self.disconnect_btn)
        layout.addWidget(self.status_label, 2)
        box.setLayout(layout)
        self.main_layout.addWidget(box)

    def _refresh_ports(self):
        self.port_combo.clear()
        for p in list_ports.comports():
            self.port_combo.addItem(f"{p.device} -{p.description}", p.device)

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

        # Header row
        header = QHBoxLayout()
        for text, stretch in [("Motor", 2), ("Angle", 2), ("Status", 1), ("", 1)]:
            lbl = QLabel(text)
            lbl.setStyleSheet("color: #888888; font-size: 11px;")
            header.addWidget(lbl, stretch)
        self.motor_layout.addLayout(header)

        # Motor rows placeholder
        self.motor_rows_layout = QVBoxLayout()
        self.motor_layout.addLayout(self.motor_rows_layout)

        self.no_motors_label = QLabel("Connect to a device to see motors.")
        self.no_motors_label.setStyleSheet("color: #555555; padding: 8px;")
        self.motor_rows_layout.addWidget(self.no_motors_label)

        self.motor_box.setLayout(self.motor_layout)
        self.main_layout.addWidget(self.motor_box)

    def _build_motor_rows(self):
        # Clear existing
        while self.motor_rows_layout.count():
            item = self.motor_rows_layout.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()
            elif item.layout():
                self._clear_layout(item.layout())

        self.motor_widgets = []
        for i, name in enumerate(self.motor_names):
            row = QFrame()
            row.setObjectName("motor-row")
            row.setFrameShape(QFrame.NoFrame)
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(6, 3, 6, 3)

            name_lbl = QLabel(name)
            name_lbl.setStyleSheet("font-weight: bold;")
            angle_lbl = QLabel("--")
            status_lbl = QLabel("--")

            toggle_btn = QPushButton("Enable")
            toggle_btn.setFixedWidth(80)
            toggle_btn.clicked.connect(self._make_motor_toggle(i, name))

            row_layout.addWidget(name_lbl, 2)
            row_layout.addWidget(angle_lbl, 2)
            row_layout.addWidget(status_lbl, 1)
            row_layout.addWidget(toggle_btn, 1)

            self.motor_rows_layout.addWidget(row)
            self.motor_widgets.append({
                "name": name,
                "angle_lbl": angle_lbl,
                "status_lbl": status_lbl,
                "toggle_btn": toggle_btn,
                "enabled": False,
            })

    def _make_motor_toggle(self, idx, name):
        def handler():
            if not self.exo_connected:
                return
            w = self.motor_widgets[idx]
            try:
                if w["enabled"]:
                    self.exo.disable_motor(idx)
                    w["enabled"] = False
                    w["toggle_btn"].setText("Enable")
                    w["status_lbl"].setText("OFF")
                    w["status_lbl"].setStyleSheet("color: #c0392b;")
                    self._log(f"Disabled motor {name}")
                else:
                    self.exo.enable_motor(idx)
                    w["enabled"] = True
                    w["toggle_btn"].setText("Disable")
                    w["status_lbl"].setText("ON")
                    w["status_lbl"].setStyleSheet("color: #27ae60;")
                    self._log(f"Enabled motor {name}")
            except Exception as e:
                self._log(f"Error toggling motor {name}: {e}")
        return handler

    # -- Gesture Control ---------------------------------------------------

    def _build_gesture_section(self):
        box = QGroupBox("Gestures")
        layout = QGridLayout()

        gestures = [
            ("Grasp", "grasp"),
            ("Pinch Index", "pinch_index"),
            ("Pinch Middle", "pinch_middle"),
            ("Pinch Ring", "pinch_ring"),
        ]

        for row, (label, cmd) in enumerate(gestures):
            name_lbl = QLabel(label)
            name_lbl.setStyleSheet("font-weight: bold;")
            open_btn = QPushButton("Open")
            close_btn = QPushButton("Close")
            close_btn.setProperty("accent", True)

            open_btn.clicked.connect(self._make_gesture_handler(cmd, "open"))
            close_btn.clicked.connect(self._make_gesture_handler(cmd, "close"))

            layout.addWidget(name_lbl, row, 0)
            layout.addWidget(open_btn, row, 1)
            layout.addWidget(close_btn, row, 2)

        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)

        box.setLayout(layout)
        self.main_layout.addWidget(box)

    def _ensure_gesture_ready(self):
        """Enable motors and apply calibration if needed before gestures."""
        if not self._gesture_ready:
            # Apply calibration so gesture angles scale correctly
            default_profile = get_default_profile_name()
            if default_profile:
                try:
                    self.exo.apply_calibration(default_profile)
                    self._log(f"Applied calibration profile '{default_profile}' for gestures.")
                except Exception as e:
                    self._log(f"Warning: could not apply calibration: {e}")
            else:
                self._log("Warning: no calibration profile found. Gestures may not work correctly.")

            # Enable all motors
            try:
                self.exo.enable_motor('all')
                for w in self.motor_widgets:
                    w["enabled"] = True
                    w["toggle_btn"].setText("Disable")
                    w["status_lbl"].setText("ON")
                    w["status_lbl"].setStyleSheet("color: #27ae60;")
                self._log("Enabled all motors for gesture control.")
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
                self._ensure_gesture_ready()
                self.exo.send_command(f"set_gesture:{gesture}:{state}")
                self._log(f"Gesture: {gesture} -> {state}")
            except Exception as e:
                self._log(f"Gesture error: {e}")
        return handler

    # -- Calibration -------------------------------------------------------

    def _build_calibration_section(self):
        box = QGroupBox("Calibration")
        layout = QVBoxLayout()

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
        self.profile_combo.clear()
        default = get_default_profile_name()
        for name in list_profiles():
            suffix = " (default)" if name == default else ""
            self.profile_combo.addItem(f"{name}{suffix}", name)

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

        port = self.port_combo.currentData() or self.port_combo.currentText().split()[0]
        baud = int(self.baud_combo.currentText())

        try:
            comm = SerialComm(port=port, baudrate=baud)
            self.exo = HandExo(comm, auto_connect=True, verbose=False,
                               command_delimiter='\r\n')
            info = self.exo.info()
            self.n_motors = info.get("n_motors", 0)

            # Extract motor names
            motors_dict = info.get("motors", {})
            self.motor_names = []
            for mid in range(self.n_motors):
                md = motors_dict.get(mid, {})
                name = md.get("name", f"motor_{mid}")
                self.motor_names.append(name)

            self.exo_connected = True
            self._gesture_ready = False
            self.status_label.setText(f"Connected -{self.n_motors} motors")
            self.status_label.setObjectName("status-connected")
            self.status_label.setStyle(self.status_label.style())  # force re-style
            self._log(f"Connected to {port} @ {baud} -{self.n_motors} motors: {', '.join(self.motor_names)}")

            self._build_motor_rows()
            self._refresh_profiles()
            self._angle_timer.start(500)
        except Exception as e:
            self.exo = None
            self.exo_connected = False
            QMessageBox.critical(self, "Connection Error", str(e))
            self._log(f"Connection failed: {e}")

        self._update_enabled_state()

    def _disconnect(self):
        self._angle_timer.stop()
        try:
            if self.exo:
                self.exo.close()
        except Exception:
            pass
        self.exo = None
        self.exo_connected = False
        self._gesture_ready = False
        self.status_label.setText("Disconnected")
        self.status_label.setObjectName("status-disconnected")
        self.status_label.setStyle(self.status_label.style())
        self._log("Disconnected.")
        self._update_enabled_state()

    def _motor_all(self, action):
        if not self.exo_connected:
            return
        try:
            if action == "enable":
                self.exo.enable_motor('all')
                for w in self.motor_widgets:
                    w["enabled"] = True
                    w["toggle_btn"].setText("Disable")
                    w["status_lbl"].setText("ON")
                    w["status_lbl"].setStyleSheet("color: #27ae60;")
                self._log("Enabled all motors.")
            else:
                self.exo.disable_motor('all')
                for w in self.motor_widgets:
                    w["enabled"] = False
                    w["toggle_btn"].setText("Enable")
                    w["status_lbl"].setText("OFF")
                    w["status_lbl"].setStyleSheet("color: #c0392b;")
                self._log("Disabled all motors.")
        except Exception as e:
            self._log(f"Error: {e}")

    def _home_all(self):
        if not self.exo_connected:
            return
        try:
            self.exo.home('all')
            self._log("Homed all motors.")
        except Exception as e:
            self._log(f"Home error: {e}")

    def _poll_motor_angles(self):
        if not self.exo_connected:
            return
        try:
            angles = self.exo.get_motor_angle('all')
            for i, w in enumerate(self.motor_widgets):
                val = angles.get(i, None)
                if val is not None:
                    w["angle_lbl"].setText(f"{float(val):.2f} deg")
        except Exception:
            pass

    def _run_calibration(self):
        if not self.exo_connected:
            QMessageBox.warning(self, "Not Connected", "Connect to a device first.")
            return
        name = self.cal_name_input.text().strip().lower()
        if not name:
            QMessageBox.warning(self, "No Name", "Enter a profile name.")
            return

        dlg = CalibrationDialog(self.exo, self.motor_names, name, parent=self)
        dlg.exec_()
        self._refresh_profiles()
        self._log(f"Calibration profile '{name}' saved.")

    def _apply_profile(self):
        if not self.exo_connected:
            QMessageBox.warning(self, "Not Connected", "Connect to a device first.")
            return
        name = self.profile_combo.currentData()
        if not name:
            QMessageBox.warning(self, "No Profile", "Select a profile to apply.")
            return
        try:
            self.exo.apply_calibration(name)
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

        dlg = ROMDialog(self.exo, self.motor_names, participant, parent=self)
        dlg.exec_()
        self._log(f"ROM assessment complete for '{participant}'.")

    def _update_enabled_state(self):
        on = self.exo_connected
        self.connect_btn.setEnabled(not on)
        self.disconnect_btn.setEnabled(on)
        self.enable_all_btn.setEnabled(on)
        self.disable_all_btn.setEnabled(on)
        self.home_all_btn.setEnabled(on)
        self.cal_run_btn.setEnabled(on)
        self.apply_profile_btn.setEnabled(on)
        self.rom_run_btn.setEnabled(on)

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

    # Scale font to screen DPI
    screen = app.primaryScreen()
    if screen:
        dpi = screen.logicalDotsPerInch()
        base_size = max(9, int(10 * dpi / 96))
        app.setFont(QFont("Segoe UI", base_size))

    window = HandExoGUI()

    # Size to 85% of screen, launch maximized
    if screen:
        geom = screen.availableGeometry()
        window.resize(int(geom.width() * 0.85), int(geom.height() * 0.85))
    window.showMaximized()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
