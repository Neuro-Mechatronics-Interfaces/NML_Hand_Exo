from PyQt5.QtWidgets import (
    QDialog, QFormLayout, QDoubleSpinBox, QPushButton,
    QHBoxLayout, QComboBox
)

from nml_wtf_exo.utils.MotionFunctions import (
    MotionFunctionType,
    MotionSine,
    MotionTriangle,
    MotionAlternatingStep,
    MotionWhiteNoise,
    MotionPinkNoise,
    MOTION_CLASS_REGISTRY,
)

MOTION_TYPE_LABELS = {
    "sine": "Sinusoid",
    "triangle": "Triangle",
    "alt_step": "Alternating Step",
    "white_noise": "White Noise",
    "pink_noise": "Pink Noise",
}

LABEL_TO_TYPE = {v: k for k, v in MOTION_TYPE_LABELS.items()}


class JointConfigDialog(QDialog):
    def __init__(self, parent, routine):
        super().__init__(parent)
        self.setWindowTitle(f"Configure {routine.joint.title()}")
        self.routine = routine

        layout = QFormLayout(self)

        # Motion type dropdown
        self.type_box = QComboBox()
        self.type_box.addItems(list(MOTION_TYPE_LABELS.values()))

        # Preselect current motion type
        cur_type = getattr(routine.motion, "type_name", "sine")
        cur_label = MOTION_TYPE_LABELS.get(cur_type, "Sinusoid")
        idx = self.type_box.findText(cur_label)
        if idx >= 0:
            self.type_box.setCurrentIndex(idx)

        layout.addRow("Motion Type:", self.type_box)

        # Amplitude
        self.amp_box = QDoubleSpinBox()
        self.amp_box.setRange(0, 100)
        self.amp_box.setValue(routine.motion.amplitude)
        layout.addRow("Amplitude (deg):", self.amp_box)

        # Frequency (rpm)
        self.freq_box = QDoubleSpinBox()
        self.freq_box.setDecimals(1)
        self.freq_box.setRange(1, 120)
        self.freq_box.setSingleStep(0.5)
        self.freq_box.setValue(routine.motion.frequency)  # in RPM
        layout.addRow("Frequency (rpm):", self.freq_box)

        # Buttons
        btn_row = QHBoxLayout()
        ok_btn = QPushButton("Apply")
        cancel_btn = QPushButton("Cancel")
        btn_row.addWidget(ok_btn)
        btn_row.addWidget(cancel_btn)
        layout.addRow(btn_row)

        ok_btn.clicked.connect(self.accept)
        cancel_btn.clicked.connect(self.reject)

    def apply(self):
        # Map label → type key
        label = self.type_box.currentText()
        mtype = LABEL_TO_TYPE.get(label, "sine")
        MotionCls = MOTION_CLASS_REGISTRY.get(mtype, MotionSine)

        amp = self.amp_box.value()
        freq = self.freq_box.value()

        new_motion = MotionCls(amplitude=amp, frequency=freq)
        self.routine.set_motion(new_motion)
        return self.routine
