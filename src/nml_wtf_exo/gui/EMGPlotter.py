import sys
import numpy as np
from pylsl import StreamInlet, resolve_stream
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSlot, QObject
import pyqtgraph as pg
from collections import deque
import time
from nml_wtf_exo.lsl.LSLWorker import LSLWorker


class EMGPlotter(QObject):
    max_samples: int = 15000

    def __init__(self, app, duration_secs=5):
        super().__init__()
        self.worker = None
        self.duration = duration_secs  # still 5 seconds


        print("Looking for EMG stream...")
        streams = resolve_stream('type', 'EMG')
        stream_info = streams[0]

        # Print stream metadata
        print("Resolved Stream Info:")
        print(f"  Name       : {stream_info.name()}")
        print(f"  Type       : {stream_info.type()}")
        print(f"  Channels   : {stream_info.channel_count()}")
        print(f"  SR (nominal): {stream_info.nominal_srate()} Hz")
        print(f"  Format     : {stream_info.channel_format()}")
        print(f"  UID        : {stream_info.uid()}")
        print(f"  Source ID  : {stream_info.source_id()}")
        self.inlet = StreamInlet(stream_info)

        # Buffers
        self.write_idx = 0
        self.timestamps = []
        self.amplitudes = []
        self.full = False
        self.sampling_rate = self.inlet.info().nominal_srate()

        # Setup plot
        self.win = pg.GraphicsLayoutWidget(title="Real-time EMG Channel 0 (Irregular)")
        self.plot = self.win.addPlot(title="Channel 0")
        self.curve = self.plot.plot(pen='y')
        self.plot.setLabel('left', 'Amplitude')
        self.plot.setYRange(-5, 5)
        self.plot.setXRange(-self.duration, 0)
        self.win.show()

    def add_worker(self, worker=None):
        if worker is None:
            worker = LSLWorker(self.inlet)
            auto_start = True
        else:
            auto_start = False
        self.worker = worker
        self.worker.new_data.connect(self.handle_new_data)
        if auto_start:
            self.worker.start()

    @pyqtSlot(list, list)
    def handle_new_data(self, chunk, timestamps):
        if not chunk or not timestamps:
            return

        samples = np.array([s[0] for s in chunk], dtype=np.float64)
        t = np.array(timestamps)

        self.timestamps.extend(t.tolist())
        self.amplitudes.extend(samples.tolist())

        now = t[-1]
        cutoff = now - self.duration

        # Keep only recent samples
        t = np.array(self.timestamps)
        y = np.array(self.amplitudes)
        mask = t >= cutoff
        t = t[mask]
        y = y[mask]

        # Update internal state
        self.timestamps = t.tolist()
        self.amplitudes = y.tolist()

        # Shift to scroll (latest at t=0)
        t_rel = t - t[-1]
        sort_idx = np.argsort(t_rel)
        self.curve.setData(t_rel[sort_idx], y[sort_idx])

if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    plotter = EMGPlotter(app)
    worker = LSLWorker(plotter.inlet)
    plotter.add_worker(worker)
    worker.start()
    sys.exit(app.exec_())
    