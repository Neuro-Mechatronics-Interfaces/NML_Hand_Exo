import sys, os, math, time, json
from pathlib import Path
from nml_wtf_exo.utils.paths import PATHS
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QGroupBox, QComboBox
)
from PyQt5.QtCore import Qt, QTimer
from nml_hand_exo.interface import HandExo, SerialComm
from nml_wtf_exo.utils.MotionFunctions import MotionFunctionType, MotionSine
from nml_wtf_exo.gui.ui.JointConfigDialog import JointConfigDialog
from serial.tools import list_ports

JOINT_NAMES = [
    "wrist",
    "thumbflex",
    "thumbrot",
    "index",
    "middle",
    "ring",
    "pinky",
]

# --- Helper/Utility ---
def canonicalize(name: str):
    if not name:
        return None
    name = name.lower().strip()
    if name in JOINT_NAMES:
        return name
    return None

# ---------------------------------------------------------------------
# Demo Routine
# ---------------------------------------------------------------------
class DemoRoutine:
    def __init__(self, joint_name, home_angle, current_angle):
        self.joint = joint_name
        self.home = float(home_angle)
        self.current = float(current_angle)
        self.prev = self.current
        self.dx = 0.0
        self.on = False
        # start with a modest, slow sinusoid
        self.motion = MotionSine(amplitude=5.0, frequency=20.0)  # RPM
        self.dt_accum = 0.0

    def step(self, dt):
        if not self.on:
            return None
        self.prev = self.current
        offset = self.motion.step(dt)   # offset around home
        self.current = self.home + offset
        self.dx = self.current - self.prev
        return offset  # propagate offset only for motion API

    def set_motion(self, motion_obj: MotionFunctionType):
        self.motion = motion_obj

    def motion_to_json(self) -> str:
        return self.motion.to_json()

# ---------------------------------------------------------------------
# Main Demo App
# ---------------------------------------------------------------------
class DemoApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hand Exo — Demo Routines")
        self.setGeometry(200, 200, 480, 360)
        self.default_motion_cfg = self._load_default_motion_config()
        self.exo = None
        self.exo_connected = False
        self.joint_map = {}
        self.home_angles = {}
        self.routines = {}

        layout = QVBoxLayout(self)
        layout.addWidget(self._build_connect_box())
        layout.addWidget(self._build_demo_box())
        layout.addStretch(1)

        # 10 Hz update loop
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._step_routines)
        self.timer.start(100)
        self.last_t = time.time()

    def _load_default_motion_config(self) -> dict:
        """
        Load default motion configs per joint from JSON.
        Returns dict: joint_name -> config_dict.
        """
        cfg_path = Path(PATHS["exo_demo_defaults"])
        if not cfg_path.exists():
            return {}

        try:
            with cfg_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            return data
        except Exception as e:
            print(f"[Demo] Failed to load default motion config: {e}")
            return {}

    # --------------------------------------------------------------
    # UI: Connect Box
    # --------------------------------------------------------------
    def _build_connect_box(self):
        box = QGroupBox("1) Connect Exo")
        h = QHBoxLayout()

        self.port_combo = QComboBox()
        for p in list_ports.comports():
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)

        self.baud_combo = QComboBox()
        for b in [57600, 115200, 230400]:
            self.baud_combo.addItem(str(b))
        self.baud_combo.setCurrentText("57600")

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connect)
        self.status_lbl = QLabel("Not connected")

        h.addWidget(QLabel("Port:"))
        h.addWidget(self.port_combo)
        h.addWidget(QLabel("Baud:"))
        h.addWidget(self.baud_combo)
        h.addWidget(self.connect_btn)
        h.addWidget(self.status_lbl)

        box.setLayout(h)
        return box

    # --------------------------------------------------------------
    # UI: Demo Box
    # --------------------------------------------------------------
    def _build_demo_box(self):
        box = QGroupBox("2) Demo Routines")
        v = QVBoxLayout()

        self.buttons = {}   # joint→button
        self.motor_labels = {}  # joint→QLabel

        for j in JOINT_NAMES:
            row = QHBoxLayout()

            lbl = QLabel("[?]")     # motor index after connection
            self.motor_labels[j] = lbl
            row.addWidget(lbl)

            gear_btn = QPushButton("⚙")
            gear_btn.setFixedWidth(28)
            gear_btn.clicked.connect(lambda _, jj=j: self._configure_joint(jj))
            row.addWidget(gear_btn)

            btn = QPushButton(f"Turn ON ({j.title()})")
            btn.setEnabled(False)
            btn.clicked.connect(lambda _, jj=j: self._toggle_routine(jj))
            self.buttons[j] = btn

            row.addWidget(btn)
            v.addLayout(row)

        box.setLayout(v)
        return box

    def _configure_joint(self, joint):
        routine = self.routines.get(joint)
        if routine is None:
            return

        dlg = JointConfigDialog(self, routine)
        if dlg.exec_():
            updated_routine = dlg.apply()
            self.routines[joint] = updated_routine

    # --------------------------------------------------------------
    # Connect / Disconnect Logic
    # --------------------------------------------------------------
    def _toggle_connect(self):
        if self.exo_connected:
            self._disconnect_exo()
        else:
            self._connect_exo()

    def _connect_exo(self):
        if self.exo_connected:
            return

        port = self.port_combo.currentData()
        baud = int(self.baud_combo.currentText())

        try:
            # ------------------------------------------------------------
            # 1) Connect to the device
            # ------------------------------------------------------------
            comm = SerialComm(port=port, baudrate=baud)
            self.exo = HandExo(comm, auto_connect=True, command_delimiter="\r\n")

            # ------------------------------------------------------------
            # 2) Enable all motors on the exo
            # ------------------------------------------------------------
            try:
                self.exo.send_command("disable:all")
            except Exception:
                pass

            info = self.exo.info()
            print("Exo Info:", info)  # debug output

            # ------------------------------------------------------------
            # 3) Extract motors block or legacy motor_0..motor_N fallback
            # ------------------------------------------------------------
            motors_dict = info.get("motors", {})
            nmotors = info.get("n_motors", len(motors_dict))

            self.motor_angles = {}
            self.motor_enabled = {}

            if motors_dict:
                # New-format firmware
                for mid in range(nmotors):
                    md = motors_dict.get(mid, {}) or {}
                    self.motor_angles[mid] = float(md.get("angle_abs", 0.0))
                    self.motor_enabled[mid] = bool(md.get("enabled", False))

            else:
                # Legacy: motor_0, motor_1, ...
                for mid in range(nmotors):
                    md = info.get(f"motor_{mid}", {}) or {}
                    self.motor_angles[mid] = float(md.get("angle_abs", 0.0))
                    self.motor_enabled[mid] = bool(md.get("enabled", False))

            # ------------------------------------------------------------
            # 4) Build mapping: joint_name → motor_id
            # ------------------------------------------------------------
            self.joint_map = {}
            self.home_angles = {}
            self.routines = {}

            # Loop through motors and assign canonical names
            for mid in range(nmotors):
                md = motors_dict.get(mid, {}) if motors_dict else info.get(f"motor_{mid}", {})
                raw_name = md.get("name", None)
                if not raw_name:
                    continue

                # store the raw firmware joint name lowercase for internal indexing
                jname = raw_name.lower().strip()

                # store the exact firmware command version (UPPERCASE)
                cmd_name = raw_name.upper().strip()

                self.joint_map[jname] = {
                    "motor_id": mid,
                    "cmd": cmd_name,
                    "home_angle": float(md.get("angle", md.get("angle", 0.0)))
                }

                self.routines[jname] = DemoRoutine(
                    joint_name=jname,
                    home_angle=float(md.get("angle", md.get("angle", 0.0))), 
                    current_angle=float(md.get("abs_angle", md.get("angle", 0.0)))
                )

            for jname, routine in self.routines.items():
                cfg = self.default_motion_cfg.get(jname, None)
                if cfg:
                    motion = MotionFunctionType.from_config(cfg)
                    routine.set_motion(motion)
                    self.routines[jname] = routine

            # ------------------------------------------------------------
            # 5) Update UI: enable only the joints we detected
            # ------------------------------------------------------------
            for j, btn in self.buttons.items():
                if j in self.joint_map:
                    mid = self.joint_map[j]
                    self.motor_labels[j].setText(f"[{mid}]")
                    btn.setEnabled(True)
                else:
                    self.motor_labels[j].setText("[?]")
                    btn.setEnabled(False)

            # ------------------------------------------------------------
            # 6) Finalize state
            # ------------------------------------------------------------
            self.exo_connected = True
            self.status_lbl.setText(f"Connected ({port})")
            self.connect_btn.setText("Disconnect")

        except Exception as e:
            self.status_lbl.setText(f"Error: {e}")
            self.exo = None
            self.exo_connected = False
            self.connect_btn.setText("Connect")


    def _disconnect_exo(self):
        if self.exo:
            try:
                self.exo.close()
            except Exception:
                pass

        self.exo = None
        self.exo_connected = False
        self.status_lbl.setText("Not connected")
        self.connect_btn.setText("Connect")

        # Disable routines
        for j, btn in self.buttons.items():
            btn.setEnabled(False)
            self.motor_labels[j].setText("[?]")

    # --------------------------------------------------------------
    # Toggle Routine ON/OFF
    # --------------------------------------------------------------
    def _toggle_routine(self, joint):
        r = self.routines.get(joint)
        if not r:
            return

        r.on = not r.on
        btn = self.buttons[joint]

        if r.on:
            self.exo.send_command(f"enable:{r.joint.upper()}")
            btn.setText(f"Turn OFF ({joint.title()})")
        else:
            self.exo.send_command(f"disable:{r.joint.upper()}")
            btn.setText(f"Turn ON ({joint.title()})")

    # --------------------------------------------------------------
    # Step routine → send motor angles
    # --------------------------------------------------------------
    def _step_routines(self):
        if not (self.exo and self.exo_connected):
            return

        now = time.time()
        dt = now - self.last_t
        self.last_t = now

        for joint, routine in self.routines.items():
            if not routine.on:
                continue

            offset = routine.step(dt)
            if offset is None:
                continue

            cmd_name = self.joint_map[joint]["cmd"]
            home_abs = self.joint_map[joint]["home_angle"]
            target_abs_angle = home_abs + offset

            cmd = f"set_angle:{cmd_name}:{target_abs_angle:.2f}"
            try:
                self.exo.send_command(cmd)
            except Exception:
                print(f"Error sending command for joint {joint}")

    # --------------------------------------------------------------
    # Graceful close
    # --------------------------------------------------------------
    def closeEvent(self, event):
        self._disconnect_exo()
        event.accept()

    def __del__(self):
        try:
            self._disconnect_exo()
        except Exception:
            pass


# ---------------------------------------------------------------------
# Launcher
# ---------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    w = DemoApp()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
