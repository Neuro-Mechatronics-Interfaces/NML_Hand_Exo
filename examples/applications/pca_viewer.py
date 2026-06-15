from sklearn.decomposition import PCA
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

class PCALiveViewer:
    def __init__(self, n_channels=7, history=200):
        self.n_channels = n_channels
        self.buffer = np.zeros((history, n_channels))  # rolling window of RMS vectors
        self.ptr = 0

        # PyQtGraph setup
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title="Real-Time PCA")
        self.plot = self.win.addPlot(title="EMG PCA - PC1 vs PC2")
        self.scatter = self.plot.plot([], [], pen=None, symbol='o', symbolBrush='orange')
        self.win.show()

        # PCA model (re-fit periodically)
        self.pca = PCA(n_components=2)

    def update(self, rms_vector):
        self.buffer[self.ptr % len(self.buffer)] = rms_vector
        self.ptr += 1

        if self.ptr > len(self.buffer):  # only fit once buffer is filled
            transformed = self.pca.fit_transform(self.buffer)
            x, y = transformed[:, 0], transformed[:, 1]
            self.scatter.setData(x, y)

        self.app.processEvents()

    def run(self):
        QtWidgets.QApplication.instance().exec_()
