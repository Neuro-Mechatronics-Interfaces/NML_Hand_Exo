from PyQt5.QtWidgets import (
    QDialog, QFormLayout, QDialogButtonBox, QDoubleSpinBox
)

class LSLSettingsDialog(QDialog):
    def __init__(self, parent, gains: dict, oe_params: dict):
        super().__init__(parent)
        self.setWindowTitle("LSL Settings")
        self.gains = gains.copy()
        self.oe_params = oe_params.copy()
        form = QFormLayout(self)

        # per-joint gains 0..5
        self._gain_boxes = {}
        labels = ["Wrist", "Thumb", "Index", "Middle", "Ring", "Pinky"]
        for i, lab in enumerate(labels):
            sb = QDoubleSpinBox()
            sb.setRange(0.0, 3.0); sb.setDecimals(3); sb.setSingleStep(0.1)
            sb.setValue(float(self.gains.get(i, 1.0)))
            self._gain_boxes[i] = sb
            form.addRow(f"{lab} gain:", sb)

        # OneEuro params
        self._min_cutoff = QDoubleSpinBox(); self._min_cutoff.setRange(0.01, 10.0); self._min_cutoff.setDecimals(3); self._min_cutoff.setValue(float(self.oe_params.get("min_cutoff", 1.0)))
        self._beta       = QDoubleSpinBox(); self._beta.setRange(0.0, 5.0);        self._beta.setDecimals(3);        self._beta.setValue(float(self.oe_params.get("beta", 0.0)))
        self._d_cutoff   = QDoubleSpinBox(); self._d_cutoff.setRange(0.01, 10.0); self._d_cutoff.setDecimals(3);    self._d_cutoff.setValue(float(self.oe_params.get("d_cutoff", 1.0)))
        form.addRow("OneEuro min_cutoff:", self._min_cutoff)
        form.addRow("OneEuro beta:", self._beta)
        form.addRow("OneEuro d_cutoff:", self._d_cutoff)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=self)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def result_values(self):
        gains = {i: self._gain_boxes[i].value() for i in self._gain_boxes}
        oe = {"min_cutoff": self._min_cutoff.value(),
              "beta": self._beta.value(),
              "d_cutoff": self._d_cutoff.value()}
        return gains, oe
