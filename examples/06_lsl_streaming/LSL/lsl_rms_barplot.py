import numpy as np
from pylsl import StreamInlet, resolve_byprop
from PyQt5 import QtWidgets
import pyqtgraph as pg
from collections import deque
from scipy.signal import iirnotch, butter, lfilter
import sys

# === Parameters ===
fs = 2000
window_sec = 0.2
window_samples = int(fs * window_sec)
n_channels = 128
noise_threshold = 100

# === Precomputed Filters ===
notch_ba = iirnotch(60.0, 30.0, fs)
bp_ba = butter(4, [10 / (0.5 * fs), 500 / (0.5 * fs)], btype='band')

def apply_filters(signal):
    signal = lfilter(*notch_ba, signal)
    signal = lfilter(*bp_ba, signal)
    return signal

def compute_rms(signal):
    return np.sqrt(np.mean(signal**2))

# === Setup LSL ===
print("Looking for an EMG stream...")
streams = resolve_byprop("type", "EMG")
if not streams:
    raise RuntimeError("No EMG LSL stream found.")
inlet = StreamInlet(streams[0])
print("EMG stream connected!")

# === Setup Buffers ===
buffers = [deque(maxlen=window_samples) for _ in range(n_channels)]

# === PyQtGraph UI Setup ===
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="Real-Time EMG RMS Monitor")
win.resize(1600, 500)
plot = win.addPlot(title="EMG RMS per Channel")
bar_item = pg.BarGraphItem(x=np.arange(n_channels), height=np.zeros(n_channels),
                           width=0.8, brush='dodgerblue')
plot.addItem(bar_item)
plot.setYRange(0, 500)
plot.setXRange(-1, n_channels)
plot.setLabel('left', "RMS Amplitude")
plot.setLabel('bottom', "Channel")
win.show()

# === Update Function ===
def update():
    chunk, _ = inlet.pull_chunk(max_samples=64)
    if chunk:
        chunk = np.array(chunk)
        for row in chunk:
            for ch in range(min(len(row), n_channels)):
                buffers[ch].append(row[ch])

    rms_vals = np.zeros(n_channels)
    for ch in range(n_channels):
        buf = np.array(buffers[ch])
        if len(buf) < window_samples:
            continue
        try:
            filtered = apply_filters(buf)
            rms_vals[ch] = compute_rms(filtered)
        except:
            rms_vals[ch] = 0

    # Update bar plot
    colors = ['red' if val > noise_threshold else 'dodgerblue' for val in rms_vals]
    #bar_item.setOpts(height=rms_vals, brush=colors)
    plot.clear()

    # Rebuild bars with color per channel
    for i, val in enumerate(rms_vals):
        color = 'red' if val > noise_threshold else 'dodgerblue'
        bar = pg.BarGraphItem(x=[i], height=[val], width=0.8, brush=color)
        plot.addItem(bar)

    plot.setYRange(0, 500)
    plot.setXRange(-1, n_channels)
    plot.setLabel('left', "RMS Amplitude")
    plot.setLabel('bottom', "Channel")

# === Set Timer ===
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(200)  # ms

# === Run App ===
if __name__ == '__main__':
    if sys.flags.interactive != 1:
        QtWidgets.QApplication.instance().exec_()
