"""One-click secondary-goal torque assistance; all serial work stays on the worker."""
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QDoubleSpinBox, QHeaderView, QCheckBox,
)


class AssistPanel(QWidget):
    preparing = pyqtSignal()
    busy_changed = pyqtSignal(bool)
    log_line = pyqtSignal(str)

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.busy = self.connected = self.supported = self.pending = self.stopping = False
        self.rows = {}
        self.last_error = ""
        layout = QVBoxLayout(self)
        explanation = QLabel(
            "Enable torque-assist for every joint on the active GUI side(s). Keep relaxed and still "
            "while startup waits for stationary readings (up to five seconds) and collects the "
            "initial baseline (about one second), then apply effort to move. "
            "Assistance advances a persistent position goal with bounded lead. If movement stalls, "
            "it releases; relax before trying again. Stored joint limits remain in force. "
            "Startup settling is based on measured velocity and angle, not resting current."
        )
        explanation.setWordWrap(True)
        layout.addWidget(explanation)
        self.parameters = QWidget()
        form = QFormLayout(self.parameters)

        def spin(lo, hi, default, decimals, step):
            widget = QDoubleSpinBox()
            widget.setRange(lo, hi)
            widget.setDecimals(decimals)
            widget.setSingleStep(step)
            widget.setValue(default)
            return widget

        # ``threshold`` is retained as the public widget attribute for callers
        # and tests. It is the configurable floor of the firmware's effective
        # current deadzone, not the startup motion-settling tolerance.
        self.threshold = spin(.00575, .230, .08, 5, .005)
        self.threshold.setToolTip(
            "Minimum effort required before assistance activates. The effective "
            "deadzone is max(this value, 0.00575 N m, 3 × measured resting-current noise). "
            "This does not relax the startup velocity/angle settling check."
        )
        self.settle_velocity = spin(.1, 10, 1.0, 2, .1)
        self.settle_velocity.setToolTip(
            "Startup qualification only: measured velocity must stay within this limit. "
            "This is not the activation deadzone or the Assist motion speed limit."
        )
        self.settle_drift = spin(.01, 5, .30, 2, .01)
        self.settle_drift.setToolTip(
            "Startup qualification only: observed pose span must stay within this tolerance. "
            "This is not the activation deadzone."
        )
        self.gain = spin(1, 434, 80, 2, 5)
        self.cap = spin(10, 910, 300, 0, 10)
        form.addRow("Activation deadzone floor (N m)", self.threshold)
        form.addRow("Startup settle velocity limit (deg/s)", self.settle_velocity)
        form.addRow("Startup pose drift tolerance (deg)", self.settle_drift)
        form.addRow("Goal velocity gain (deg/s per N m)", self.gain)
        form.addRow("Maximum current per joint (mA)", self.cap)
        self.model = QCheckBox("Use identified hardware dynamics (optional)")
        self.model.setChecked(False)
        model_contents = QWidget()
        model_form = QFormLayout(model_contents)
        self.model.toggled.connect(model_contents.setVisible)
        model_contents.setVisible(False)
        self.inertia = spin(0, 1, 0, 5, .001)
        self.viscous = spin(0, 1, 0, 5, .001)
        self.coulomb = spin(0, 1, 0, 5, .001)
        model_form.addRow("Inertia (N m s2/deg)", self.inertia)
        model_form.addRow("Viscous friction (N m s/deg)", self.viscous)
        model_form.addRow("Coulomb friction (N m)", self.coulomb)
        model_note = QLabel(
            "Defaults estimate effort from measured current. The simulator's plant inertia and "
            "friction are not identified hardware parameters. Enter measured motor-coordinate "
            "values here if available. N m values use the nominal 0.00115 N m/mA conversion."
        )
        model_note.setWordWrap(True)
        model_form.addRow(model_note)
        form.addRow(self.model)
        form.addRow(model_contents)
        layout.addWidget(self.parameters)
        self.budget = QLabel("The existing combined current budget and motor limits cap each joint; they are never raised here.")
        self.budget.setWordWrap(True)
        layout.addWidget(self.budget)
        row = QHBoxLayout()
        self.toggle = QPushButton("Enable Torque-Assist")
        self.toggle.setCheckable(True)
        self.toggle.clicked.connect(self._toggle)
        self.stop = QPushButton("Stop / torque off")
        self.stop.clicked.connect(self.stop_session)
        row.addWidget(self.toggle)
        row.addWidget(self.stop)
        row.addStretch()
        layout.addLayout(row)
        self.status = QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels([
            "Joint", "Effective cap (mA)", "Effort (N m)", "Goal / angle (deg)", "State",
        ])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        layout.addWidget(self.table)
        layout.addStretch()
        self.timer = QTimer(self)
        self.timer.setInterval(250)
        self.timer.timeout.connect(self._heartbeat)
        worker.assist_completed.connect(self._completed)
        self.set_connection(False)

    def set_connection(self, connected, motors=(), info=None):
        self.timer.stop()
        self.connected = bool(connected)
        self.supported = bool(info and info.get("assist_protocol") == "assist-v3")
        if info and str(info.get("usb_protocol", "")).startswith("protobuf-"):
            self.supported = self.supported and {'assist_binary', 'assist_enable'}.issubset(info.get('usb_features', []))
        self.pending = self.stopping = False
        self.last_error = ""
        self.rows = {mid: row for row, (mid, _) in enumerate(motors)}
        self.table.setRowCount(len(motors))
        self.table.setFixedHeight(min(360, max(150,
            self.table.horizontalHeader().height() + len(motors)*self.table.verticalHeader().defaultSectionSize() + 2)))
        for row, (mid, name) in enumerate(motors):
            for col, text in enumerate((f"{name} ({mid})", "-", "-", "-", "Off")):
                self.table.setItem(row, col, QTableWidgetItem(text))
        self._set_busy(False)
        self.stop.setEnabled(self.connected and self.supported)
        self.status.setText(
            f"Ready: {len(self.rows)} joints. Enable Torque-Assist to begin." if self.connected and self.supported else
            "Connect firmware with assist-v3 support; recompile/reflash the current build."
        )

    def _set_busy(self, busy):
        changed = self.busy != busy
        self.busy = busy
        self.parameters.setEnabled(not busy)
        self.toggle.setEnabled(self.connected and self.supported and not self.stopping)
        if not busy:
            self.toggle.setChecked(False)
            self.toggle.setText("Enable Torque-Assist")
        if changed:
            self.busy_changed.emit(busy)

    def _toggle(self, checked):
        if not checked:
            self.stop_session()
            return
        if not self.rows:
            self.toggle.setChecked(False)
            self.status.setText("No joints are available on the active GUI side.")
            return
        self.preparing.emit()
        self.last_error = ""
        self.stopping = False
        self._set_busy(True)
        self.pending = True
        self.toggle.setText("Disable Torque-Assist")
        self.status.setText("Waiting for stationary readings, then preparing a capped hold and baseline; keep relaxed and still...")
        dynamics = self.model.isChecked()
        self.worker.request_assist("enable", dict(ids=list(self.rows), parameters=dict(
            threshold_Nm=self.threshold.value(), velocity_gain_deg_s_per_Nm=self.gain.value(),
            current_cap_mA=self.cap.value(), inertia_Nm_s2_per_deg=self.inertia.value() if dynamics else 0.,
            viscous_Nm_s_per_deg=self.viscous.value() if dynamics else 0.,
            coulomb_Nm=self.coulomb.value() if dynamics else 0.,
            startup_velocity_limit_deg_s=self.settle_velocity.value(),
            startup_pose_drift_deg=self.settle_drift.value(),
        )))

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
            self.status.setText("Stopping torque-assist...")
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
            self.timer.stop()
            if operation != "stop":
                self.stop_session()
                self.status.setText(error + " - stopping torque-assist.")
            else:
                self._set_busy(True)
                self.status.setText(error + " - torque-off unconfirmed. Retry Stop or disconnect.")
            return
        state = status.get("state", 0)
        reason = status.get("reason", "unknown")
        if state == 0:
            self.stopping = False
        self._set_busy(state != 0)
        self.toggle.setEnabled(self.supported and state in (0, 1, 3) and not self.stopping)
        self.toggle.setChecked(state in (1, 2, 3))
        self.toggle.setText("Disable Torque-Assist" if state in (1, 2, 3) else "Enable Torque-Assist")
        labels = {0: "Torque-assist off", 1: "Collecting initial baseline: keep relaxed and still",
                  2: "Baseline ready", 3: "Torque-assist enabled: apply effort to move",
                  4: "Torque-off unconfirmed; retrying stop"}
        self.status.setText(labels.get(state, "Unknown assist state") + f" ({reason})")
        if 'total_current_budget_mA' in status:
            self.budget.setText(
                f"Combined budget: {status['total_current_budget_mA']} mA. Requested maximum: {self.cap.value():.0f} mA/joint. "
                "The table shows the installed caps after budget sharing and motor limits."
            )
        for mid, joint in status.get("joints", {}).items():
            if mid not in self.rows:
                continue
            row = self.rows[mid]
            detail = "Relax to rearm" if joint.get("rearm_blocked") else "Assisting" if joint.get("moving") else (
                f"Baseline {joint.get('samples', 0):.0f}/50" if state == 1 else "Sensing effort" if state == 3 else "Off")
            texts = (f"{joint.get('current_cap_mA', 0):.0f}", f"{joint.get('effort_mA', 0)*.00115:+.4f}",
                     f"{joint.get('goal', 0):.2f} / {joint.get('angle', 0):.2f}", detail)
            for col, text in enumerate(texts, 1):
                self.table.item(row, col).setText(text)
            self.table.item(row, 2).setToolTip(
                f"Effective activation threshold: {joint.get('deadzone_mA', 0)*.00115:.4f} N m\n"
                f"Baseline bias/noise: {joint.get('bias_mA', 0):.1f} / {joint.get('noise_mA', 0):.1f} mA\n"
                f"Measured sample age: {joint.get('age_ms', 0):.0f} ms"
            )
        if state:
            if not self.stopping:
                self.timer.start()
        else:
            self.timer.stop()
            if self.last_error:
                self.status.setText(self.last_error)
            elif reason not in ("off", "stopped", "stop_retried"):
                motor = f" (ID {status['reject_id']})" if 'reject_id' in status else ""
                self.status.setText(f"Torque-assist stopped: {reason}{motor}. Check the joints and parameters before enabling again.")
            self.log_line.emit(f"[assist] {reason}; torque-assist off.")
