# ~/nml/gui/LogViewer.py
import os
import sys
from typing import Optional

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QSplitter, QListWidget
import numpy as np

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from nml_wtf_exo.utils.LogReader import LogReader
from nml_wtf_exo.utils.paths import PATHS


class LandmarkCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(5, 5), dpi=100)
        super().__init__(self.fig)
        self.setParent(parent)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Landmarks (normalized)")
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.scatter = None
        self.flipY = True

    def set_highlight(self, index: int):
        """Make one point larger; reset others."""
        if self.scatter is None:
            return
        offs = self.scatter.get_offsets()
        n = len(offs)
        sizes = np.full(n, 20.0)
        if 0 <= index < n:
            sizes[index] = 80.0
        self.scatter.set_sizes(sizes)
        self.draw_idle()


    def init_points(self, xs: np.ndarray, ys: np.ndarray, flipY: bool = True):
        self.flipY = flipY
        ys_plot = 1.0 - ys if flipY else ys
        if self.scatter is None:
            self.scatter = self.ax.scatter(xs, ys_plot, s=8)
        else:
            self.scatter.set_offsets(np.c_[xs, ys_plot])
        self.draw_idle()

    def update_points(self, xs: np.ndarray, ys: np.ndarray):
        if self.scatter is None:
            self.init_points(xs, ys, self.flipY)
            return
        ys_plot = 1.0 - ys if self.flipY else ys
        self.scatter.set_offsets(np.c_[xs, ys_plot])
        self.draw_idle()


class LandmarkViewer(QtWidgets.QWidget):
    def __init__(self, default_dir: Optional[str] = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("NML Log Viewer")
        self.resize(900, 600)

        self.dirEdit = QtWidgets.QLineEdit(self)
        default_path = os.path.expanduser(default_dir or PATHS["landmarks_dir"])
        self.dirEdit.setText(default_path)
        self.refreshBtn = QtWidgets.QPushButton("Refresh", self)

        self.fileList = QtWidgets.QListWidget(self)
        self.fileList.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)

        self.loadBtn = QtWidgets.QPushButton("Load", self)
        self.playBtn = QtWidgets.QPushButton("Playback", self)
        self.playBtn.setEnabled(False)
        self.pauseBtn = QtWidgets.QPushButton("Pause", self)
        self.pauseBtn.setEnabled(False)

        self.flipCheck = QtWidgets.QCheckBox("Flip Y (UI coords)", self)
        self.flipCheck.setChecked(True)
        self.speedSpin = QtWidgets.QDoubleSpinBox(self)
        self.speedSpin.setRange(0.1, 5.0)
        self.speedSpin.setSingleStep(0.1)
        self.speedSpin.setValue(1.0)
        self.speedSpin.setSuffix("x")

        self.canvas = LandmarkCanvas(self)

        # --- Labels list (acts like a dock) ---
        self.labelList = QListWidget(self)
        self.labelList.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.labelList.setMinimumWidth(180)

        # Put canvas + labels into a splitter
        self.split = QSplitter(Qt.Horizontal, self)
        self.split.addWidget(self.canvas)
        self.split.addWidget(self.labelList)
        self.split.setStretchFactor(0, 1)
        self.split.setStretchFactor(1, 0)

        # Selection -> highlight point
        self.labelList.currentRowChanged.connect(self._on_label_selected)

        topRow = QtWidgets.QHBoxLayout()
        topRow.addWidget(QtWidgets.QLabel("Directory:"))
        topRow.addWidget(self.dirEdit, stretch=1)
        topRow.addWidget(self.refreshBtn)

        leftCol = QtWidgets.QVBoxLayout()
        leftCol.addLayout(topRow)
        leftCol.addWidget(self.fileList, stretch=1)
        btnRow = QtWidgets.QHBoxLayout()
        btnRow.addWidget(self.loadBtn)
        btnRow.addWidget(self.playBtn)
        btnRow.addWidget(self.pauseBtn)
        leftCol.addLayout(btnRow)
        optRow = QtWidgets.QHBoxLayout()
        optRow.addWidget(self.flipCheck)
        optRow.addWidget(QtWidgets.QLabel("Speed:"))
        optRow.addWidget(self.speedSpin)
        optRow.addStretch(1)
        leftCol.addLayout(optRow)

        main = QtWidgets.QHBoxLayout(self)
        main.addLayout(leftCol, stretch=0)
        main.addWidget(self.split, stretch=1)

        self.reader = None
        self.timer = QtCore.QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self._on_timer)

        self.refreshBtn.clicked.connect(self.refresh_file_list)
        self.loadBtn.clicked.connect(self.load_selected)
        self.playBtn.clicked.connect(self.start_playback)
        self.pauseBtn.clicked.connect(self.pause_playback)
        self.fileList.itemDoubleClicked.connect(lambda _: self.load_selected())
        self.flipCheck.toggled.connect(self._on_flip_change)

        self.refresh_file_list()

    def _labels_for_landmarks(self):
        """Collapse per-channel labels into one label per landmark."""
        if not self.reader:
            return []
        meta = getattr(self.reader, "meta", {})
        ch = meta.get("channels", [])
        step = getattr(self.reader, "dims_per_landmark", lambda: 2)()
        
        if ch and len(ch) >= step:
            # Take every 'step' channels, strip the .x/.y/.z suffix
            labels = []
            for i in range(0, len(ch), step):
                lab = ch[i].get("label", f"lm[{i//step}]")
                base = lab.rsplit(".", 1)[0]
                labels.append(base)
        else:
            # Fallback generic
            n = self.reader.landmark_count()
            labels = [f"lm[{i}]" for i in range(n)]
        return labels

    def _populate_label_list(self):
        self.labelList.clear()
        self.labelList.addItems(self._labels_for_landmarks())


    def _on_flip_change(self, checked: bool):
        if self.reader is None:
            return
        i = max(self.reader._i - 1, 0)
        if i >= self.reader.frame_count():
            return
        s = self.reader.samples[i]
        if self.reader.dims == 3:
            xs = s[0::3]; ys = s[1::3]
        else:
            xs = s[0::2]; ys = s[1::2]
        self.canvas.init_points(xs, ys, flipY=checked)

    def refresh_file_list(self):
        self.fileList.clear()
        d = os.path.expanduser(self.dirEdit.text().strip())
        try:
            items = sorted([p for p in os.listdir(d) if p.lower().endswith(".csv")])
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Cannot list directory:\n{e}")
            return
        for name in items:
            self.fileList.addItem(name)

    def _selected_path(self):
        d = os.path.expanduser(self.dirEdit.text().strip())
        item = self.fileList.currentItem()
        if not item:
            return None
        return os.path.join(d, item.text())

    def load_selected(self):
        path = self._selected_path()
        if not path:
            QtWidgets.QMessageBox.information(self, "Select a file", "Choose a CSV in the list first.")
            return
        try:
            self.reader = LogReader(path)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Load failed", f"{e}")
            self.reader = None
            self.playBtn.setEnabled(False)
            self.pauseBtn.setEnabled(False)
            return
        
        meta = getattr(self.reader, "meta", {})  # <— safe
        subset = meta.get("subset", "?")
        n = self.reader.landmark_count()
        d = self.reader.dims_per_landmark()
        self.setWindowTitle(f"NML Log Viewer — {os.path.basename(path)}  ({subset}, {n} landmarks, {d} dims)")

        # Auto-flip:
        auto_flip = getattr(self.reader, "recommended_flip_y", lambda: True)()
        self.flipCheck.setChecked(auto_flip)
        n = self.reader.landmark_count()
        d = self.reader.dims_per_landmark()
        self.setWindowTitle(f"NML Log Viewer — {os.path.basename(path)}  ({subset}, {n} landmarks, {d} dims)")
        self.reader.reset()
        step = self.reader.step()
        if step is None:
            QtWidgets.QMessageBox.warning(self, "Empty", "No frames in this log.")
            self.playBtn.setEnabled(False)
            self.pauseBtn.setEnabled(False)
            return

        t, xs, ys = step
        self.canvas.init_points(xs, ys, flipY=self.flipCheck.isChecked())
        self._populate_label_list()
        # Optional: auto-select the first point & highlight it
        if self.labelList.count() > 0:
            self.labelList.setCurrentRow(0)
            self._on_label_selected(0)

        self.playBtn.setEnabled(True)
        self.pauseBtn.setEnabled(False)

    def _on_label_selected(self, row: int):
        if row is None or row < 0:
            return
        self.canvas.set_highlight(row)

    def start_playback(self):
        if not self.reader:
            return
        if self.reader.is_done():
            self.reader.reset()
        self.playBtn.setEnabled(False)
        self.pauseBtn.setEnabled(True)
        self._schedule_next()

    def pause_playback(self):
        self.timer.stop()
        self.playBtn.setEnabled(True)
        self.pauseBtn.setEnabled(False)

    def _schedule_next(self):
        if not self.reader or self.reader.is_done():
            self.playBtn.setEnabled(True)
            self.pauseBtn.setEnabled(False)
            return
        ms = self.reader.peek_delta_to_next_ms(speed=self.speedSpin.value())
        self.timer.start(max(1, ms))

    def _on_timer(self):
        if not self.reader:
            return
        step = self.reader.step()
        if step is None:
            self.playBtn.setEnabled(True)
            self.pauseBtn.setEnabled(False)
            return
        _, xs, ys = step
        self.canvas.update_points(xs, ys)
        self._schedule_next()


def _maybe_fix_matplotlib_backend():
    import matplotlib
    backend = matplotlib.get_backend().lower()
    if "qt" not in backend:
        matplotlib.use("Qt5Agg")


def main():
    _maybe_fix_matplotlib_backend()
    app = QtWidgets.QApplication(sys.argv)
    w = LandmarkViewer()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
