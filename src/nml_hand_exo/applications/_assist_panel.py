"""Explicit opt-in assist controls; all serial work stays on SerialWorker."""
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTableWidget,
    QTableWidgetItem, QCheckBox, QDoubleSpinBox, QHeaderView,
)


class AssistPanel(QWidget):
    preparing = pyqtSignal()
    busy_changed = pyqtSignal(bool)
    log_line = pyqtSignal(str)

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.busy = False
        self.connected = self.supported = self.pending = self.stopping = False
        self.rows = {}
        layout = QVBoxLayout(self)
        explanation = QLabel(
            "Assist uses measured holding-current changes and joint deflection. "
            "Calibration selects current-position mode and enables only the joints selected below. "
            "Keep relaxed and still while calibrating bias. Lower threshold means greater sensitivity. "
            "Bias is valid within 5 motor degrees of this pose; recalibrate after repositioning."
        )
        explanation.setWordWrap(True)
        layout.addWidget(explanation)
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels([
            "Joint", "Threshold (mA)", "Gain (deg/mA)", "Current cap (mA)",
            "Bias / deadzone (mA)", "Effort / state",
        ])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setMaximumHeight(360)
        layout.addWidget(self.table)
        row = QHBoxLayout()
        self.calibrate = QPushButton("Calibrate relaxed bias")
        self.toggle = QPushButton("Assist off")
        self.toggle.setCheckable(True)
        self.stop = QPushButton("Stop / torque off")
        self.calibrate.clicked.connect(self._calibrate)
        self.toggle.clicked.connect(self._toggle)
        self.stop.clicked.connect(self.stop_session)
        for button in (self.calibrate, self.toggle, self.stop):
            row.addWidget(button)
        row.addStretch()
        layout.addLayout(row)
        self.status = QLabel("Connect firmware with assist-v1 support.")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addStretch()
        self.timer = QTimer(self)
        self.timer.setInterval(250)
        self.timer.timeout.connect(self._heartbeat)
        worker.assist_completed.connect(self._completed)
        self.set_connection(False)

    def set_connection(self, connected, motors=(), info=None):
        self.timer.stop()
        self.connected = bool(connected)
        self.supported = bool(info and info.get("assist_protocol") == "assist-v1")
        self.pending = self.stopping = False
        self.last_error = ""
        self._set_busy(False)
        self.rows.clear()
        self.table.setRowCount(len(motors))
        self.table.setFixedHeight(min(360, max(150,
            self.table.horizontalHeader().height() + len(motors)*self.table.verticalHeader().defaultSectionSize() + 2)))
        for row, (mid, name) in enumerate(motors):
            selected = QCheckBox(f"{name} ({mid})")
            controls = []
            for lo, hi, default, decimals, step in ((5, 60, 10, 1, 1), (.001, .05, .01, 3, .001), (10, 80, 40, 0, 5)):
                control = QDoubleSpinBox()
                control.setRange(lo, hi)
                control.setDecimals(decimals)
                control.setSingleStep(step)
                control.setValue(default)
                controls.append(control)
            for col, widget in enumerate((selected, *controls)):
                self.table.setCellWidget(row, col, widget)
            for col in (4, 5):
                item = QTableWidgetItem("—")
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(row, col, item)
            self.rows[mid] = (row, selected, *controls)
        self.calibrate.setEnabled(self.connected and self.supported)
        self.stop.setEnabled(self.connected and self.supported)
        self.toggle.setEnabled(False)
        self.status.setText("Select joints, then calibrate relaxed bias." if self.supported else
                            "Connect firmware with assist-v1 support (reflash the updated v0.9.1 build).")

    def _set_busy(self, busy):
        self.busy = busy
        self.table.setEnabled(not busy)
        self.calibrate.setEnabled(self.connected and self.supported and not busy)
        self.busy_changed.emit(busy)
        if not busy:
            self.toggle.setChecked(False)
            self.toggle.setText("Assist off")
            self.toggle.setEnabled(False)

    def _calibrate(self):
        configs = {
            mid: dict(threshold_mA=threshold.value(), gain_deg_per_mA=gain.value(), current_cap_mA=cap.value())
            for mid, (_, selected, threshold, gain, cap) in self.rows.items() if selected.isChecked()
        }
        if not configs:
            self.status.setText("Select at least one joint to enable and calibrate.")
            return
        self.preparing.emit()
        self.last_error = ""
        self.stopping = False
        self._set_busy(True)
        self.pending = True
        self.status.setText("Preparing current-position mode and capturing relaxed bias; keep still…")
        self.worker.request_assist("calibrate", configs)

    def _toggle(self, checked):
        if not checked:
            self.stop_session()
            return
        self.toggle.setEnabled(False)
        self.pending = True
        self.worker.request_assist("start")

    def _heartbeat(self):
        if self.busy and not self.pending and not self.stopping:
            self.pending = True
            self.worker.request_assist("heartbeat")

    def stop_session(self):
        self.timer.stop()
        self.stopping = True
        self.toggle.setChecked(False)
        self.toggle.setEnabled(False)
        if self.connected and self.supported:
            self.pending = True
            self.status.setText("Stopping assist…")
            self.worker.request_assist("stop")
        else:
            self._set_busy(False)

    def _completed(self, operation, status, error):
        if not self.connected or (self.stopping and operation != "stop"):
            return
        self.pending = False
        if error:
            self.last_error = error
            self.log_line.emit(f"[assist] {error}")
            self.status.setText(error)
            self.timer.stop()
            if operation != "stop":
                self.stop_session()
                self.status.setText(error + " — stopping assist.")
                return
            self._set_busy(True)
            self.status.setText(error + " ? stop not acknowledged; heartbeat stopped. Retry Stop or disconnect.")
            return
        state = status.get("state", 0)
        reason = status.get("reason", "unknown")
        self._set_busy(state != 0)
        self.status.setText({0: "Off", 1: "Calibrating relaxed bias", 2: "Bias ready — enable Assist", 3: "Assist active", 4: "Torque-off unconfirmed; retrying stop"}[state] + f" ({reason})")
        self.toggle.setEnabled(state in (2, 3))
        self.toggle.setChecked(state == 3)
        self.toggle.setText("Assist on" if state == 3 else "Assist off")
        for mid, joint in status.get("joints", {}).items():
            if mid not in self.rows:
                continue
            row = self.rows[mid][0]
            self.table.item(row, 4).setText(f"{joint['bias_mA']:.1f} / {joint['deadzone_mA']:.1f}")
            detail = "settling" if joint.get("moving") else f"{joint.get('samples', 0):.0f}/50 samples" if state == 1 else "sensing"
            self.table.item(row, 5).setText(f"{joint.get('effort_mA', 0):+.1f} mA / {detail}")
        if state:
            self.timer.start()
        else:
            self.timer.stop()
            if self.last_error:
                self.status.setText(self.last_error)
            self.log_line.emit(f"[assist] {reason}; assist off. Recalibrate before rearming.")
