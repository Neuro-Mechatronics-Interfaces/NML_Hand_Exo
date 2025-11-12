
from collections import deque
import math, time
from PyQt5.QtWidgets import QWidget, QVBoxLayout
import pyqtgraph as pg
import numpy as np

class LSLLivePlotWindow(QWidget):
    """
    High-FPS (pyqtgraph) viewer for six time-series:
      Wrist, Thumb, Index, Middle, Ring, Pinky.
    Expects values already filtered & with calibration applied
    (i.e., dev = filtered - home when calibrated; else filtered).
    """
    def __init__(self, history_secs=10.0, rate_hz=25.0, parent=None):
        super().__init__(parent)
        self.setWindowTitle("LSL Angles (filtered, calibrated)")
        self.resize(1000, 560)

        self.history_secs = float(history_secs)
        self.rate_hz = float(rate_hz)
        self.maxlen = max(32, int(self.history_secs * self.rate_hz))
        self._t0 = None
        self._frame_skip = 0  # draw every frame; raise to lighten CPU

        # UI
        pg.setConfigOptions(antialias=True)
        layout = QVBoxLayout(self)
        self.glw = pg.GraphicsLayoutWidget()
        layout.addWidget(self.glw)

        names = ["Wrist", "Thumb", "Index", "Middle", "Ring", "Pinky"]
        self.plots = []
        self.curves = []
        self.buff_t = deque(maxlen=self.maxlen)
        self.buff_y = [deque(maxlen=self.maxlen) for _ in range(6)]

        for i, nm in enumerate(names):
            r, c = divmod(i, 3)
            if c == 0:
                self.glw.nextRow()
            p = self.glw.addPlot(row=r, col=c, title=nm)
            p.showGrid(x=True, y=True, alpha=0.3)
            cvr = p.plot([], [], pen=pg.mkPen(width=2))
            self.plots.append(p)
            self.curves.append(cvr)

    def clear(self):
        self.buff_t.clear()
        for d in self.buff_y:
            d.clear()
        for p in self.plots:
            p.enableAutoRange(x=True, y=True)

    def update_values(self, values, ts=None):
        """
        values: iterable of 6 floats (deg) in motor order:
                Wrist, Thumb, Index, Middle, Ring, Pinky.
        ts: optional timestamp (float); else uses time.time()
        """
        t = float(ts) if ts is not None else time.time()
        if self._t0 is None:
            self._t0 = t
        t_rel = t - self._t0

        self.buff_t.append(t_rel)
        vals = list(values) + [float("nan")] * (6 - len(values))
        for i in range(6):
            self.buff_y[i].append(float(vals[i]))

        # Draw (every frame by default)
        x = np.fromiter(self.buff_t, dtype=float, count=len(self.buff_t))
        for i in range(6):
            y = np.fromiter(self.buff_y[i], dtype=float, count=len(self.buff_y[i]))
            self.curves[i].setData(x, y)
            if len(x) > 1:
                xmax = x[-1]
                xmin = max(0.0, xmax - self.history_secs)
                self.plots[i].setXRange(xmin, xmax, padding=0.0)
