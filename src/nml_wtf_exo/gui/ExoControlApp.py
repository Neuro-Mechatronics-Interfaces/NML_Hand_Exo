import sys, os, time, traceback, re
from datetime import datetime
import xml.etree.ElementTree as ET
from nml_wtf_exo.gui.LSLSettingsDialog import LSLSettingsDialog
from nml_wtf_exo.utils.OneEuroFilter import OneEuro
from nml_wtf_exo.utils.paths import PATHS
from pathlib import Path
CONFIG_PATH = Path(os.path.join(PATHS['configs_dir'], ".nml_hand_exo_gui.xml" ))

from PyQt5.QtWidgets import (
    QDialog, QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QLineEdit, QMessageBox, QGroupBox, QSpinBox,
    QDoubleSpinBox, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer
from nml_wtf_exo.gui.LivePlotWindow import LSLLivePlotWindow

from pylsl import resolve_streams, StreamInlet, proc_ALL
from serial.tools import list_ports

# Hand Exo API (installed via pip)
from nml_hand_exo.interface import HandExo, SerialComm

# Optional: your logger
from nml_wtf_exo.lsl.StreamLogger import StreamLogger


class ExoControlApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hand Exo — LSL/Manual Controller")
        self.setGeometry(120, 120, 560, 520)

        # --- State ---
        self.exo = None
        self.exo_connected = False
        self.lsl_view = None


        self.current_inlet = None
        self.current_stream_info = None
        self.available_streams = []
        self.lsl_poll_ms = 10  # ~100 Hz to oversample

        self.logger = None
        self.log_dir = "exo_logs"
        os.makedirs(self.log_dir, exist_ok=True)

        # UI
        root = QVBoxLayout(self)
        root.addWidget(self._build_exo_box())
        root.addWidget(self._build_mode_box())
        root.addWidget(self._build_manual_box())
        root.addWidget(self._build_lsl_box())
        root.addWidget(self._build_raw_cmd_box())
        root.addStretch(1)

        # Timers
        self.lsl_timer = QTimer(self)
        self.lsl_timer.timeout.connect(self._lsl_tick)
        self.motor_angles = {}   # id → current absolute angle
        self.motor_enabled = {}   # id -> bool
        self.nmotors = 6          # updated on connect
        self.angle_timer = QTimer(self)
        self.angle_timer.timeout.connect(self._update_motor_angles)
        self._lsl_disabled_warned = set()
        # Calibration
        self.lsl_calibrated = False
        self.lsl_home = {i: 0.0 for i in range(self.nmotors)}               # id -> home (filtered LSL angle at calibration)

        # Per-joint gains (default 1.0)
        self.lsl_gains = {i: 1.0 for i in range(self.nmotors)}

        # OneEuro filter params & filters
        self.oe_params = {"min_cutoff": 1.5, "beta": 0.07, "d_cutoff": 10.0}
        self.lsl_freq = max(1.0, 1000.0 / self.lsl_poll_ms)  # Hz from your poll period
        self.oe_filters = {}             # id -> OneEuro

        # Initial state
        self._update_enabled()
        self._apply_saved_serial()

    # ---------------- UI Builders ----------------
    def _build_raw_cmd_box(self):
        box = QGroupBox("Raw Command (advanced)")
        v = QVBoxLayout()

        row = QHBoxLayout()
        self.raw_input = QLineEdit()
        self.raw_input.setPlaceholderText("e.g., info   (press Enter or click Send)")
        self.raw_send_btn = QPushButton("Send")

        # OPTIONAL presets
        self.raw_presets = QComboBox()
        self.raw_presets.addItems(["(choose…)", "info", "help"])
        self.raw_presets.currentTextChanged.connect(self._raw_fill_from_preset)

        # NEW: timeout spinbox (seconds)
        self.raw_timeout = QDoubleSpinBox()
        self.raw_timeout.setRange(0.0, 5.0)
        self.raw_timeout.setDecimals(2)
        self.raw_timeout.setSingleStep(0.1)
        self.raw_timeout.setValue(0.50)  # default 500 ms

        row.addWidget(QLabel("Command:"))
        row.addWidget(self.raw_input, 3)
        row.addWidget(self.raw_presets, 1)
        row.addSpacing(8)
        row.addWidget(QLabel("Timeout (s):"))
        row.addWidget(self.raw_timeout)
        row.addWidget(self.raw_send_btn)

        self.raw_status = QLabel("")

        v.addLayout(row)
        v.addWidget(self.raw_status)
        box.setLayout(v)

        self.raw_send_btn.clicked.connect(self._send_raw_cmd)
        self.raw_input.returnPressed.connect(self._send_raw_cmd)
        return box


    def _raw_fill_from_preset(self, txt: str):
        if txt and not txt.startswith("("):
            self.raw_input.setText(txt)

    def _send_raw_cmd(self):
        if not (self.exo and self.exo_connected):
            QMessageBox.warning(self, "No Exo", "Connect to the Exo before sending commands.")
            return

        cmd = self.raw_input.text().strip()
        if not cmd:
            return
        try:
            # Send (your HandExo handles delimiter + small send delay internally)
            self.exo.send_command(cmd)
            # Drain anything that arrives within timeout and print to terminal
            timeout_s = float(self.raw_timeout.value())
            txt = self._drain_exo_response(timeout_s=timeout_s)
            if txt:
                print(txt)  # <-- prints to terminal
                self.raw_status.setText(f"Sent: {cmd}  |  {len(txt)} chars received")
            else:
                self.raw_status.setText(f"Sent: {cmd}  |  no response within {timeout_s:.2f}s")
            self.raw_input.clear()
        except Exception as e:
            self.raw_status.setText(f"Send error: {e}")

    def _drain_exo_response(self, timeout_s: float = 0.5) -> str:
        """
        Collect any response text from the exo for up to timeout_s seconds and return it.
        Tries several known APIs:
        - HandExo._receive(...)            (private but available in your code)
        - HandExo.receive(...)             (if exposed)
        - HandExo.device.receive(...)      (SerialComm)
        - HandExo.device.read*/readline    (pyserial-style)
        """
        end = time.time() + max(0.0, timeout_s)
        chunks = []

        def _to_str(x):
            if x is None:
                return ""
            if isinstance(x, bytes):
                return x.decode(errors="ignore")
            return str(x)

        def _try_once():
            # 1) HandExo._receive (seen in your info() implementation)
            fn = getattr(self.exo, "_receive", None)
            if callable(fn):
                for kwargs in ({"timeout": 0.05}, {"wait_until_return": False}, {}):
                    try:
                        out = fn(**kwargs)
                        if out:
                            return _to_str(out)
                    except TypeError:
                        continue
                    except Exception:
                        break  # don't spam on repeated errors

            # 2) HandExo.receive (if present)
            fn = getattr(self.exo, "receive", None)
            if callable(fn):
                for kwargs in ({"timeout": 0.05}, {}):
                    try:
                        out = fn(**kwargs)
                        if out:
                            return _to_str(out)
                    except TypeError:
                        continue
                    except Exception:
                        break

            # 3) Device-level receive/read
            dev = getattr(self.exo, "device", None)
            if dev:
                # Prefer a 'receive' if wrapper provides it
                fn = getattr(dev, "receive", None)
                if callable(fn):
                    try:
                        out = fn(timeout=0.05)
                        if out:
                            return _to_str(out)
                    except TypeError:
                        try:
                            out = fn()
                            if out:
                                return _to_str(out)
                        except Exception:
                            pass
                    except Exception:
                        pass

                # Common pyserial-style reads
                for meth in ("read_all", "readall", "readline"):
                    fn = getattr(dev, meth, None)
                    if callable(fn):
                        try:
                            out = fn()
                            if out:
                                return _to_str(out)
                        except Exception:
                            pass

            return ""

        # Poll quickly until timeout
        while time.time() < end:
            out = _try_once()
            if out:
                chunks.append(out)
                # keep looping in case more arrives within the window
            else:
                time.sleep(0.02)

        return "".join(chunks)


    def _build_exo_box(self):
        box = QGroupBox("1) Exo Connection")
        layout = QHBoxLayout()

        self.port_combo = QComboBox()
        self._refresh_ports()
        self.refresh_ports_btn = QPushButton("Refresh")
        self.refresh_ports_btn.clicked.connect(self._refresh_ports)

        self.baud_combo = QComboBox()
        for b in [57600, 115200, 230400]:
            self.baud_combo.addItem(str(b))
        self.baud_combo.setCurrentText("57600")

        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        self.connect_btn.clicked.connect(self._connect_exo)
        self.disconnect_btn.clicked.connect(self._disconnect_exo)

        self.exo_status = QLabel("Not connected")
        self.exo_status.setAlignment(Qt.AlignLeft)

        layout.addWidget(QLabel("Port:"))
        layout.addWidget(self.port_combo, 2)
        layout.addWidget(QLabel("Baud:"))
        layout.addWidget(self.baud_combo, 1)
        layout.addWidget(self.refresh_ports_btn)
        layout.addWidget(self.connect_btn)
        layout.addWidget(self.disconnect_btn)
        layout.addWidget(self.exo_status, 3)
        box.setLayout(layout)
        return box

    def _build_mode_box(self):
        box = QGroupBox("2) Control Mode")
        layout = QHBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Manual", "LSL"])
        self.mode_combo.currentTextChanged.connect(self._update_enabled)

        # NEW: global fix-range toggle (default True)
        self.fix_range_cb = QCheckBox("Fix range (θ = −(90−θ))")
        self.fix_range_cb.setChecked(True)

        self.log_checkbox = QCheckBox("Log LSL while controlling")
        self.log_checkbox.setChecked(True)

        layout.addWidget(QLabel("Mode:"))
        layout.addWidget(self.mode_combo)
        layout.addStretch(1)
        layout.addWidget(self.fix_range_cb)
        layout.addSpacing(12)
        layout.addWidget(self.log_checkbox)
        box.setLayout(layout)
        return box

    def _build_manual_box(self):
        box = QGroupBox("Manual Control (relative angle)")
        v = QVBoxLayout()

        row = QHBoxLayout()
        self.manual_motor_id = QSpinBox()
        self.manual_motor_id.setRange(0, 5)
        self.manual_motor_id.setValue(0)

        self.manual_delta_deg = QDoubleSpinBox()
        self.manual_delta_deg.setRange(-360.0, 360.0)
        self.manual_delta_deg.setDecimals(1)
        self.manual_delta_deg.setSingleStep(5.0)
        self.manual_delta_deg.setValue(10.0)

        self.current_angle_label = QLabel("Current: ?°")
        self.enabled_label = QLabel("Enabled: ?")

        self.nudge_btn_plus = QPushButton("+Δ")
        self.nudge_btn_plus.clicked.connect(self._manual_nudge_plus)
        self.nudge_btn_minus = QPushButton("-Δ")
        self.nudge_btn_minus.clicked.connect(self._manual_nudge_minus)

        row.addWidget(QLabel("Motor ID:"))
        row.addWidget(self.manual_motor_id)
        row.addSpacing(12)
        row.addWidget(QLabel("Δ angle (deg):"))
        row.addWidget(self.manual_delta_deg)
        row.addStretch(1)
        row.addWidget(QLabel("Nudge:"))
        row.addWidget(self.nudge_btn_plus)
        row.addWidget(self.nudge_btn_minus)
        v.addLayout(row)
        row2 = QHBoxLayout()
        row2.addStretch(1)
        row2.addWidget(self.current_angle_label)
        row2.addSpacing(12)
        row2.addWidget(self.enabled_label)
        v.addLayout(row2)
        self.manual_motor_id.valueChanged.connect(lambda _: self._refresh_selected_motor_labels())

        # NEW: Torque controls for the selected motor
        trow = QHBoxLayout()
        self.torque_on_btn = QPushButton("Torque ON (motor)")
        self.torque_off_btn = QPushButton("Torque OFF (motor)")
        self.torque_on_btn.clicked.connect(self._manual_torque_on)
        self.torque_off_btn.clicked.connect(self._manual_torque_off)
        self.led_on_btn = QPushButton("LED ON")
        self.led_off_btn = QPushButton("LED OFF")
        self.led_on_btn.clicked.connect(self._led_on)
        self.led_off_btn.clicked.connect(self._led_off)
        trow.addStretch(1)
        trow.addWidget(self.torque_on_btn)
        trow.addWidget(self.torque_off_btn)
        trow.addWidget(self.led_on_btn)
        trow.addWidget(self.led_off_btn)
        v.addLayout(trow)

        box.setLayout(v)
        return box

    def _build_lsl_box(self):
        box = QGroupBox("LSL Control")
        v = QVBoxLayout()

        # Stream row
        top = QHBoxLayout()
        self.stream_combo = QComboBox()
        self.stream_combo.currentIndexChanged.connect(self._on_stream_changed)
        self.refresh_streams_btn = QPushButton("Refresh Streams")
        self.refresh_streams_btn.clicked.connect(self._refresh_streams)
        top.addWidget(QLabel("Stream:"))
        top.addWidget(self.stream_combo, 2)
        top.addWidget(self.refresh_streams_btn)

        # Hand + mode row
        handrow = QHBoxLayout()
        self.hand_combo = QComboBox()
        self.hand_combo.addItems(["Left (L)", "Right (R)"])
        self.multi_joint_cb = QCheckBox("Use multi-joint map (L/R)")
        self.multi_joint_cb.setChecked(True)
        self.calib_btn = QPushButton("Calibrate LSL")
        self.calib_btn.clicked.connect(self._calibrate_lsl)
        self.calib_btn.setEnabled(False)
        self.settings_btn = QPushButton("LSL Settings…")
        self.settings_btn.clicked.connect(self._open_lsl_settings)
        self.calib_state = QLabel("Not calibrated")
        handrow.addWidget(QLabel("Hand:"))
        handrow.addWidget(self.hand_combo)
        handrow.addSpacing(10)
        handrow.addWidget(self.multi_joint_cb)
        handrow.addStretch(1)
        handrow.addWidget(self.calib_btn)
        handrow.addWidget(self.settings_btn)
        handrow.addWidget(self.calib_state)

        # Mapping (single-channel legacy controls; disabled if multi-joint)
        maprow = QHBoxLayout()
        self.channel_combo = QComboBox()
        self.motor_id_for_lsl = QSpinBox()
        self.motor_id_for_lsl.setRange(0, 5)
        self.motor_id_for_lsl.setValue(0)

        self.scale_spin = QDoubleSpinBox()
        self.scale_spin.setRange(-10.0, 10.0); self.scale_spin.setDecimals(3); self.scale_spin.setValue(1.0)
        self.offset_spin = QDoubleSpinBox()
        self.offset_spin.setRange(-360.0, 360.0); self.offset_spin.setDecimals(2); self.offset_spin.setValue(0.0)

        maprow.addWidget(QLabel("Channel:"))
        maprow.addWidget(self.channel_combo, 2)
        maprow.addSpacing(8)
        maprow.addWidget(QLabel("→ Motor ID:"))
        maprow.addWidget(self.motor_id_for_lsl)
        maprow.addSpacing(8)
        maprow.addWidget(QLabel("Scale:"))
        maprow.addWidget(self.scale_spin)
        maprow.addWidget(QLabel("Offset:"))
        maprow.addWidget(self.offset_spin)
        maprow.addStretch(1)

        # Start/Stop + torque for mapped motor
        bot = QHBoxLayout()
        self.start_lsl_btn = QPushButton("Start LSL Control")
        self.stop_lsl_btn = QPushButton("Stop LSL Control")
        self.stop_lsl_btn.setEnabled(False)
        self.start_lsl_btn.clicked.connect(self._start_lsl_control)
        self.stop_lsl_btn.clicked.connect(self._stop_lsl_control)

        self.lsl_torque_on_btn = QPushButton("Torque ON (mapped)")
        self.lsl_torque_off_btn = QPushButton("Torque OFF (mapped)")
        self.lsl_torque_on_btn.clicked.connect(self._lsl_torque_on)
        self.lsl_torque_off_btn.clicked.connect(self._lsl_torque_off)

        self.lsl_status = QLabel("Idle")

        bot.addWidget(self.start_lsl_btn)
        bot.addWidget(self.stop_lsl_btn)
        bot.addStretch(1)
        bot.addWidget(self.lsl_torque_on_btn)
        bot.addWidget(self.lsl_torque_off_btn)
        bot.addWidget(self.lsl_status, 3)

        v.addLayout(top)
        v.addLayout(handrow)
        v.addLayout(maprow)
        v.addLayout(bot)
        box.setLayout(v)
        return box


    # ---------------- Actions ----------------
    def _open_lsl_settings(self):
        dlg = LSLSettingsDialog(self, self.lsl_gains, self.oe_params)
        if dlg.exec_() == QDialog.Accepted:
            self.lsl_gains, self.oe_params = dlg.result_values()
            self._init_oe_filters()  # rebuild filters with new params

    def _init_oe_filters(self):
        self.oe_filters = {i: OneEuro(self.lsl_freq,
                                    self.oe_params["min_cutoff"],
                                    self.oe_params["beta"],
                                    self.oe_params["d_cutoff"])
                        for i in range(self.nmotors)}

    def _load_config(self) -> dict:
        """Return {'port': 'COM5', 'baud': '57600'} if present; else {}."""
        try:
            if CONFIG_PATH.exists():
                tree = ET.parse(CONFIG_PATH)
                root = tree.getroot()
                serial = root.find("Serial")
                if serial is not None:
                    return {
                        "port": serial.get("port") or "",
                        "baud": serial.get("baud") or "",
                    }
        except Exception as e:
            print(f"[config] load failed: {e}")
        return {}

    def _save_config(self, *, port: str, baud: int) -> None:
        """Write XML with last-used serial settings."""
        try:
            root = ET.Element("ExoAppConfig")
            serial = ET.SubElement(root, "Serial")
            serial.set("port", str(port))
            serial.set("baud", str(baud))
            tmp = CONFIG_PATH.with_suffix(".tmp")
            ET.ElementTree(root).write(tmp, encoding="utf-8", xml_declaration=True)
            tmp.replace(CONFIG_PATH)
        except Exception as e:
            print(f"[config] save failed: {e}")

    def _apply_saved_serial(self) -> None:
        """Select saved port/baud in widgets if available; add 'last used' if not present."""
        cfg = self._load_config()
        if not cfg:
            return

        port = cfg.get("port")
        baud = cfg.get("baud")

        if port:
            idx = -1
            for i in range(self.port_combo.count()):
                if self.port_combo.itemData(i) == port:
                    idx = i
                    break
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
            else:
                # Port not currently detected; show it anyway at top.
                self.port_combo.insertItem(0, f"{port} (last used — not detected)", port)
                self.port_combo.setCurrentIndex(0)

        if baud:
            btxt = str(baud)
            j = self.baud_combo.findText(btxt)
            if j == -1:
                self.baud_combo.addItem(btxt)
            self.baud_combo.setCurrentText(btxt)

    def _refresh_selected_motor_labels(self):
        if not self.exo_connected:
            self.current_angle_label.setText("Current: ?°")
            self.enabled_label.setText("Enabled: ?")
            return
        mid = int(self.manual_motor_id.value())
        ang = self.motor_angles.get(mid, float('nan'))
        en  = self.motor_enabled.get(mid, False)
        try:
            self.current_angle_label.setText(f"Current: {ang:.2f}°")
        except Exception:
            self.current_angle_label.setText("Current: ?°")
        self.enabled_label.setText(f"Enabled: {'Yes' if en else 'No'}")

    def _refresh_ports(self):
        self.port_combo.clear()
        for p in list_ports.comports():
            label = f"{p.device} - {p.description}"
            self.port_combo.addItem(label, p.device)

    def _connect_exo(self):
        if self.exo_connected:
            return
        if self.port_combo.count() == 0:
            QMessageBox.warning(self, "No Ports", "No serial ports found.")
            return

        port = self.port_combo.currentData() or self.port_combo.currentText().split()[0]
        try:
            baud = int(self.baud_combo.currentText())
            comm = SerialComm(port=port, baudrate=baud)
            self.exo = HandExo(comm, auto_connect=True, verbose=False, command_delimiter='\r\n')
            info = self.exo.info()
            # print("Exo Info:", info)
            self.nmotors = info.get("n_motors", 6)
            # Initialize caches
            self.motor_angles = {}
            self.motor_enabled = {}
            self.manual_motor_id.setRange(0, max(0, self.nmotors - 1))
            self.motor_id_for_lsl.setRange(0, max(0, self.nmotors - 1))
            self.exo_connected = True
            self.exo_status.setText(f"Connected ({port}, {baud}) — motors: {self.nmotors}")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self._save_config(port=port, baud=baud)
            # Prefer structured 'motors' block if present; fallback to motor_<id> keys
            motors_dict = info.get("motors", {})
            if not motors_dict:
                # legacy: motor_0..motor_N
                for mid in range(self.nmotors):
                    md = info.get(f"motor_{mid}", {}) or {}
                    self.motor_angles[mid] = float(md.get("angle", 0.0))
                    self.motor_enabled[mid] = bool(md.get("enabled", False))
            else:
                for mid in range(self.nmotors):
                    md = motors_dict.get(mid, {}) or {}
                    self.motor_angles[mid] = float(md.get("angle", 0.0))
                    self.motor_enabled[mid] = bool(md.get("enabled", False))
            self.angle_timer.start(250)
            self._refresh_selected_motor_labels()
        except Exception as e:
            self.exo = None
            self.exo_connected = False
            QMessageBox.critical(self, "Exo Error", f"Failed to connect to Exo:\n{e}")
        finally:
            self._update_enabled()

    def _disconnect_exo(self):
        try:
            self._stop_lsl_control()
            if self.exo:
                self.exo.close()
        except Exception:
            pass
        self.exo = None
        self.exo_connected = False
        self.exo_status.setText("Not connected")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self._update_enabled()
        self.angle_timer.stop()

    def _update_motor_angles(self):
        if not (self.exo and self.exo_connected):
            return
        try:
            # update only the currently selected motor’s label to keep UI light
            mid = int(self.manual_motor_id.value())
            ang = self.exo.get_motor_angle(mid)
            self.motor_angles[mid] = float(ang)
            self.current_angle_label.setText(f"Current: {ang:.2f}°")
            # do NOT modify self.motor_enabled[] here
        except Exception:
            # leave label as-is on error
            pass

    # ---- Manual control ----
    def _manual_nudge_plus(self):
        if not (self.exo and self.exo_connected): return
        mid = int(self.manual_motor_id.value())
        if not self.motor_enabled.get(mid, False):
            QMessageBox.information(self, "Motor Disabled",
                                    f"Motor {mid} torque is OFF; enable it to move.")
            return
        delta = float(self.manual_delta_deg.value())
        # Compute absolute target as (current cached angle + delta)
        base = float(self.motor_angles.get(mid, 0.0))
        target = base + delta
        # Optional clamp
        target = max(-180.0, min(180.0, target))

        try:
            # set_motor_angle expects an absolute target (per your API); we don’t update cache
            self.exo.set_motor_angle(mid, target)
            # Let the periodic poll update the displayed 'Current' once the move occurs
        except Exception as e:
            QMessageBox.critical(self, "Manual Control Error", str(e))

    def _manual_nudge_minus(self):
        if not (self.exo and self.exo_connected): return
        mid = int(self.manual_motor_id.value())
        if not self.motor_enabled.get(mid, False):
            QMessageBox.information(self, "Motor Disabled",
                                    f"Motor {mid} torque is OFF; enable it to move.")
            return
        delta = float(self.manual_delta_deg.value())
        # Compute absolute target as (current cached angle + delta)
        base = float(self.motor_angles.get(mid, 0.0))
        target = base - delta
        # Optional clamp
        target = max(-180.0, min(180.0, target))

        try:
            # set_motor_angle expects an absolute target (per your API); we don’t update cache
            self.exo.set_motor_angle(mid, target)
            # Let the periodic poll update the displayed 'Current' once the move occurs
        except Exception as e:
            QMessageBox.critical(self, "Manual Control Error", str(e))

    def _manual_torque_on(self):
        if not (self.exo and self.exo_connected): return
        mid = int(self.manual_motor_id.value())
        try:
            # accept either API name
            if hasattr(self.exo, "enable_torque"):
                self.exo.enable_torque(mid)
            elif hasattr(self.exo, "enable_motor"):
                self.exo.enable_motor(mid)
            self.motor_enabled[mid] = True
            self._lsl_disabled_warned.discard(mid)
            self._refresh_selected_motor_labels()
        except Exception as e:
            QMessageBox.critical(self, "Torque ON Error", str(e))

    def _manual_torque_off(self):
        if not (self.exo and self.exo_connected): return
        mid = int(self.manual_motor_id.value())
        try:
            if hasattr(self.exo, "disable_torque"):
                self.exo.disable_torque(mid)
            elif hasattr(self.exo, "disable_motor"):
                self.exo.disable_motor(mid)
            self.motor_enabled[mid] = False
            self._refresh_selected_motor_labels()
        except Exception as e:
            QMessageBox.critical(self, "Torque OFF Error", str(e))


    def _led_on(self):
        if not (self.exo and self.exo_connected):
            return
        led_id = int(self.manual_motor_id.value())
        try:
            self.exo.enable_led(led_id)
        except Exception as e:
            QMessageBox.critical(self, "LED ON Error", str(e))

    def _led_off(self):
        if not (self.exo and self.exo_connected):
            return
        led_id = int(self.manual_motor_id.value())
        try:
            self.exo.disable_led(led_id)
        except Exception as e:
            QMessageBox.critical(self, "LED OFF Error", str(e))

    # ---- LSL control ----
    def _refresh_streams(self):
        self.stream_combo.clear()
        self.channel_combo.clear()
        try:
            streams = resolve_streams()
            self.available_streams = streams
            for s in streams:
                self.stream_combo.addItem(f"{s.name()} [{s.source_id()}]")
            if streams:
                self._populate_channel_labels(streams[0])
        except Exception as e:
            QMessageBox.critical(self, "LSL Error", f"Failed to resolve streams:\n{e}")

    def _get_full_streaminfo(self, info):
        """Return a full StreamInfo (with <desc>) by briefly opening an inlet."""
        try:
            inlet = StreamInlet(info,
                                 max_buflen=2,          # seconds of buffer
                                 max_chunklen=256,      # internal chunk cap
                                 processing_flags=proc_ALL)
            finfo = inlet.info()  # this queries the outlet for the complete descriptor
            # Free the temporary inlet quickly
            try:
                inlet.close_stream()  # newer pylsl; safe to ignore if missing
            except Exception:
                pass
            del inlet
            return finfo
        except Exception:
            return info  # fallback to whatever we have

    def _get_stream_labels(self, info):
        """Return channel labels from StreamInfo XML, or [] if none found."""
        labels = []
        try:
            finfo = self._get_full_streaminfo(info)
            desc = finfo.desc()
            if desc.empty():
                print("[LSL] No <desc/> in StreamInfo; no labels available.", flush=True)
                return labels
            chans = desc.child("channels")
            if chans.empty():
                print("[LSL] No <channels> in StreamInfo; no labels available.", flush=True)
                return labels
            ch = chans.child("channel")
            idx = 0
            while not ch.empty():
                lab = ch.child_value("label") or f"chan_{idx}"
                print(f"[LSL] label[{idx}] = {lab}", flush=True)
                labels.append(lab)
                ch = ch.next_sibling()
                idx += 1
        except Exception as e:
            import traceback; traceback.print_exc()
        return labels

    def _populate_channel_labels(self, stream_info):
        self.channel_combo.clear()
        labels = self._get_stream_labels(stream_info)
        if not labels:
            n = stream_info.channel_count()
            labels = [f"chan_{i}" for i in range(n)]
            print(f"[LSL] Using fallback names; channel_count={n}", flush=True)
        for lab in labels:
            self.channel_combo.addItem(lab)

    def _pull_latest(self, inlet):
        # If your pylsl has samples_available(), use it to size the pull.
        try:
            n = inlet.samples_available()
        except Exception:
            n = 0
        if n <= 0:
            # single pass attempt anyway
            n = 512

        # Drain up to n samples in one call (non-blocking)
        samples, timestamps = inlet.pull_chunk(timeout=0.0, max_samples=n)
        if samples:
            return samples[-1], (timestamps[-1] if timestamps else None)
        return None, None

    def _start_lsl_control(self):
        if self.stream_combo.currentIndex() < 0:
            QMessageBox.warning(self, "No Stream", "Select an LSL stream.")
            return
        try:
            self.current_stream_info = self.available_streams[self.stream_combo.currentIndex()]
            self.current_inlet = StreamInlet(self.current_stream_info,
                                 max_buflen=2,          # seconds of buffer
                                 max_chunklen=256,      # internal chunk cap
                                 processing_flags=proc_ALL)
            if self.log_checkbox.isChecked():
                name = self.current_stream_info.name()
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.logger = StreamLogger(name, log_dir=self.log_dir, suffix=f"control_{ts}")
                self.logger.start()
            self.lsl_timer.start(self.lsl_poll_ms)
            self.start_lsl_btn.setEnabled(False)
            self.stop_lsl_btn.setEnabled(True)
            # Launch viewer if not already open
            try:
                rate = self.lsl_freq
            except Exception:
                rate = max(1.0, 1000.0 / self.lsl_poll_ms)
            if self.lsl_view is None:
                self.lsl_view = LSLLivePlotWindow(history_secs=10.0, rate_hz=rate, parent=None)
            self.lsl_view.clear()
            self.lsl_view.show()
            self.lsl_view.raise_()
            # Build channel map for chosen hand; validate presence
            hand_char = 'L' if self.hand_combo.currentText().startswith("Left") else 'R'
            self.lsl_idx_map, self._cached_labels, missing = self._build_channel_index_map(self.current_stream_info, hand_char)
            print("[LSL] idx_map:", self.lsl_idx_map, "missing:", missing, flush=True)  
            if missing:
                QMessageBox.critical(self, "LSL Channels Missing",
                                    f"Stream is missing required labels for hand {hand_char}: {', '.join(missing)}")
                self.current_inlet = None
                self.current_stream_info = None
                return

            # Ready to calibrate now that we have an inlet + mapping
            self.calib_btn.setEnabled(True)
            self.lsl_calibrated = False
            self.calib_state.setText("Not calibrated")
            self._init_oe_filters()
            self.lsl_status.setText(f"Running: {self.current_stream_info.name()}")
        except Exception as e:
            QMessageBox.critical(self, "LSL Control Error", f"Failed to start LSL control:\n{e}")

    def _stop_lsl_control(self):
        self.lsl_timer.stop()
        self.start_lsl_btn.setEnabled(True)
        self.stop_lsl_btn.setEnabled(False)
        self.current_inlet = None
        self.current_stream_info = None
        try:
            if self.logger:
                self.logger.stop()
                self.logger = None
        except Exception:
            pass
        self.lsl_status.setText("Idle")
        self.calib_btn.setEnabled(False)
        self.lsl_calibrated = False
        self.calib_state.setText("Not calibrated")
        if self.lsl_view is not None:
            # keep it open but clear data? or close it. Here we close:
            self.lsl_view.close()
            self.lsl_view = None


    def _on_stream_changed(self, idx: int):
        # repopulate channel labels preview; store for validation
        if idx < 0 or idx >= len(getattr(self, "available_streams", [])):
            self.channel_combo.clear()
            return
        self.current_stream_info = self.available_streams[self.stream_combo.currentIndex()]
        # print(self.current_stream_info.as_xml())
        self._populate_channel_labels(self.available_streams[idx])

    def _lsl_tick(self):
        if not (self.current_inlet):
            return
        try:
            sample, ts = self._pull_latest(self.current_inlet)
            if not sample:
                return

            if self.multi_joint_cb.isChecked():
                # Multi-joint path: each joint → matching motor
                if not getattr(self, "lsl_idx_map", None):
                    self.lsl_status.setText("No mapping")
                    return
                plot_vals = [0.0]*self.nmotors
                for mid in range(self.nmotors):
                    ch_idx = self.lsl_idx_map.get(mid, None)
                    if ch_idx is None or ch_idx >= len(sample):
                        continue
                    raw = float(sample[ch_idx])
                    if self.fix_range_cb.isChecked():
                        raw = self._fix_range(raw)
                    # Filter then compute deviation from home
                    fval = self.oe_filters[mid].filter(raw)
                    dev = self.lsl_home.get(mid, 0.0) - fval
                    plot_vals[mid] = (dev if self.lsl_calibrated else fval)
                    # Gain map to delta (relative desired change on exo)
                    delta = self.lsl_gains.get(mid, 1.0) * dev

                    base = float(self.motor_angles.get(mid, 0.0))
                    target = base + delta
                    target = max(-180.0, min(180.0, target))

                    # throttle sends: only if changed meaningfully
                    last = getattr(self, "_last_sent", {}).get(mid)
                    if last is None or abs(target - last) >= getattr(self, "min_send_step_deg", 2.5):
                        if (self.lsl_calibrated and self.motor_enabled.get(mid, False) and self.exo and self.exo_connected):
                            self.exo.set_motor_angle(mid, target)
                            if not hasattr(self, "_last_sent"): self._last_sent = {}
                            self._last_sent[mid] = target

                self.lsl_status.setText("OK")
                if self.lsl_view is not None:
                    try:
                        self.lsl_view.update_values(plot_vals, ts)
                    except Exception:
                        pass

            else:
                # Legacy single-channel path
                ch_idx = self.channel_combo.currentIndex()
                if ch_idx < 0 or ch_idx >= len(sample):
                    return
                raw_val = float(sample[ch_idx])
                if self.fix_range_cb.isChecked():
                    raw_val = self._fix_range(raw_val)

                scale = float(self.scale_spin.value())
                offset = float(self.offset_spin.value())
                cmd_angle = raw_val * scale + offset  # relative mapping
                mid = int(self.motor_id_for_lsl.value())

                if not self.motor_enabled.get(mid, False):
                    return

                base = float(self.motor_angles.get(mid, 0.0))
                target = base + cmd_angle
                target = max(-180.0, min(180.0, target))
                last = getattr(self, "_last_sent", {}).get(mid)
                if last is None or abs(target - last) >= getattr(self, "min_send_step_deg", 1.0):
                    self.exo.set_motor_angle(mid, target)
                    if not hasattr(self, "_last_sent"): self._last_sent = {}
                    self._last_sent[mid] = target

        except Exception as e:
            self.lsl_status.setText(f"Err: {e}")


    def _lsl_torque_on(self):
        if not (self.exo and self.exo_connected): return
        mid = int(self.motor_id_for_lsl.value())
        try:
            if hasattr(self.exo, "enable_torque"):
                self.exo.enable_torque(mid)
            elif hasattr(self.exo, "enable_motor"):
                self.exo.enable_motor(mid)
            self.motor_enabled[mid] = True
            # if selected motor matches, refresh UI label
            if mid == int(self.manual_motor_id.value()):
                self._refresh_selected_motor_labels()
        except Exception as e:
            QMessageBox.critical(self, "Torque ON Error", str(e))

    def _lsl_torque_off(self):
        if not (self.exo and self.exo_connected): return
        mid = int(self.motor_id_for_lsl.value())
        try:
            if hasattr(self.exo, "disable_torque"):
                self.exo.disable_torque(mid)
            elif hasattr(self.exo, "disable_motor"):
                self.exo.disable_motor(mid)
            self.motor_enabled[mid] = False
            if mid == int(self.manual_motor_id.value()):
                self._refresh_selected_motor_labels()
        except Exception as e:
            QMessageBox.critical(self, "Torque OFF Error", str(e))


    # ---------------- Helpers ----------------

    def _fix_range(self, theta):
        """Fix range of the input angles"""
        return float(theta) - 90.0

    def _update_enabled(self):
        on_exo = self.exo_connected
        is_manual = (self.mode_combo.currentText() == "Manual")
        in_lsl_mode = not is_manual

        # Manual widgets (exo required)
        self.manual_motor_id.setEnabled(on_exo and is_manual)
        self.manual_delta_deg.setEnabled(on_exo and is_manual)
        self.nudge_btn_plus.setEnabled(on_exo and is_manual)
        self.nudge_btn_minus.setEnabled(on_exo and is_manual)
        self.torque_on_btn.setEnabled(on_exo and is_manual)
        self.torque_off_btn.setEnabled(on_exo and is_manual)
        if hasattr(self, "led_on_btn"):  self.led_on_btn.setEnabled(on_exo and is_manual)
        if hasattr(self, "led_off_btn"): self.led_off_btn.setEnabled(on_exo and is_manual)

        # LSL widgets (independent of exo)
        self.stream_combo.setEnabled(in_lsl_mode)
        self.refresh_streams_btn.setEnabled(in_lsl_mode)
        self.start_lsl_btn.setEnabled(in_lsl_mode and (not self.lsl_timer.isActive()))
        self.stop_lsl_btn.setEnabled(in_lsl_mode and self.lsl_timer.isActive())
        self.log_checkbox.setEnabled(in_lsl_mode)

        single_ok = in_lsl_mode and not self.multi_joint_cb.isChecked()
        self.channel_combo.setEnabled(single_ok)
        self.motor_id_for_lsl.setEnabled(single_ok)
        self.scale_spin.setEnabled(single_ok)
        self.offset_spin.setEnabled(single_ok)

        self.settings_btn.setEnabled(in_lsl_mode)
        self.hand_combo.setEnabled(in_lsl_mode)
        self.multi_joint_cb.setEnabled(in_lsl_mode)
        # Allow calibration whenever in LSL mode and a stream is selected
        self.calib_btn.setEnabled(in_lsl_mode and (self.stream_combo.currentIndex() >= 0))

        # Raw command panel (exo-only)
        if hasattr(self, "raw_input"):     self.raw_input.setEnabled(on_exo)
        if hasattr(self, "raw_send_btn"):  self.raw_send_btn.setEnabled(on_exo)
        if hasattr(self, "raw_presets"):   self.raw_presets.setEnabled(on_exo)

        # Exo connect/disconnect
        self.connect_btn.setEnabled(not on_exo)
        self.disconnect_btn.setEnabled(on_exo)


    def _calibrate_lsl(self):
        # must be in LSL mode and have a stream selected
        if self.stream_combo.currentIndex() < 0:
            QMessageBox.warning(self, "No Stream", "Select an LSL stream first.")
            return

        # Build channel map (uses full StreamInfo)
        hand_char = 'L' if self.hand_combo.currentText().startswith("Left") else 'R'
        sinfo = self.available_streams[self.stream_combo.currentIndex()]
        self.lsl_idx_map, self._cached_labels, missing = self._build_channel_index_map(sinfo, hand_char)
        if missing:
            QMessageBox.critical(self, "LSL Channels Missing",
                                f"Missing for hand {hand_char}: {', '.join(missing)}")
            return

        # Use existing inlet if running, otherwise create a temporary one
        inlet = self.current_inlet or StreamInlet(sinfo,
                                 max_buflen=2,          # seconds of buffer
                                 max_chunklen=256,      # internal chunk cap
                                 processing_flags=proc_ALL)

        # Try to grab one sample
        sample, ts = inlet.pull_sample(timeout=0.2)
        if not sample:
            QMessageBox.information(self, "No Data", "No LSL sample available for calibration.")
            if inlet is not self.current_inlet:
                try: inlet.close_stream()
                except Exception: pass
            return

        # Reset filters and set home
        self._init_oe_filters()
        for mid, ch_idx in self.lsl_idx_map.items():
            val = float(sample[ch_idx])
            if self.fix_range_cb.isChecked():
                val = self._fix_range(val)
            fval = self.oe_filters[mid].filter(val)
            self.lsl_home[mid] = fval

        self.lsl_calibrated = True
        self.calib_state.setText("Calibrated")

        if inlet is not self.current_inlet:
            try: inlet.close_stream()
            except Exception: pass


    # -- HELPERS --
    def _required_basenames(self):
        # Canonical order → motors 0..5
        # Use lower-case canonical keys for matching
        return ["wrist", "thumb", "index", "middle", "ring", "pinky"]

    def _canonical_base(self, s: str) -> str:
        """Normalize a channel base label to one of the required basenames."""
        if not s:
            return ""
        # Remove separators and lowercase
        t = re.sub(r"[^A-Za-z]", "", s).lower()
        # Drop a trailing 'fe' (flex/extend) if present
        t = re.sub(r"fe$", "", t)
        # Map common aliases
        aliases = {
            "wristfe": "wrist", "wrist": "wrist",
            "thumbfe": "thumb", "thumb": "thumb",
            "indexfe": "index", "index": "index", "forefinger": "index", "pointer": "index",
            "middlefe": "middle", "middle": "middle",
            "ringfe": "ring", "ring": "ring",
            "pinkyfe": "pinky", "pinky": "pinky", "little": "pinky",
        }
        return aliases.get(t, t)

    def _split_side_and_base(self, label: str):
        """
        Extract side ('L'/'R') and base name from label variants like:
        'L.WristFE', 'RThumb', 'Wrist_L', 'WristFE-R', 'ThumbR', 'L-Index', etc.
        Returns (side, canonical_base) or (None, None) if not parsable.
        """
        if not label:
            return (None, None)
        lab = label.strip()

        # Try prefix side (L.Thumb, L_Thumb, LThumb, L-Thumb)
        m = re.match(r"^\s*([LR])(?:[._-]|(?=[A-Z]))(.*)$", lab, flags=re.I)
        if m:
            side = m.group(1).upper()
            base_raw = m.group(2)
            return (side, self._canonical_base(base_raw))

        # Try suffix side (Thumb_L, Thumb-L, ThumbL)
        m = re.match(r"^(.*?)(?:[._-]?)([LR])\s*$", lab, flags=re.I)
        if m:
            base_raw = m.group(1)
            side = m.group(2).upper()
            return (side, self._canonical_base(base_raw))

        # Try dotted middle (rare): WristFE.L
        m = re.match(r"^(.*?)[._-]([LR])\s*$", lab, flags=re.I)
        if m:
            base_raw = m.group(1)
            side = m.group(2).upper()
            return (side, self._canonical_base(base_raw))

        return (None, None)

    def _build_channel_index_map(self, sinfo, hand_char: str):
        """
        Build mid->channel_index map for the chosen hand using tolerant parsing.
        We will only accept exact base matches to the 6 required basenames after canonicalization.
        """
        labels = self._get_stream_labels(sinfo)
        if not labels:
            return None, [], [f"{hand_char}.{b.title()}" for b in self._required_basenames()]

        # Build a lookup: (side, canonical_base) -> first index
        lookup = {}
        for idx, lab in enumerate(labels):
            side, base = self._split_side_and_base(lab)
            if side and base:
                key = (side, base)
                # Keep the first occurrence
                if key not in lookup:
                    lookup[key] = idx

        req_bases = self._required_basenames()
        chosen_side = hand_char.upper()
        idx_map, missing = {}, []
        for mid, base in enumerate(req_bases):
            key = (chosen_side, base)
            idx = lookup.get(key, None)
            if idx is None:
                missing.append(f"{chosen_side}.{base.title()}")
            else:
                idx_map[mid] = idx

        return (idx_map if not missing else None), labels, missing


def main():
    app = QApplication(sys.argv)
    w = ExoControlApp()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
