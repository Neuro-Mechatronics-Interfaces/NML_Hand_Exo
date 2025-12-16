from PyQt5.QtCore import QThread, pyqtSignal

class LSLWorker(QThread):
    new_data = pyqtSignal(list, list)  # use object for generic list/array

    def __init__(self, inlet, poll_interval=0.001):
        super().__init__()
        self.inlet = inlet
        self.poll_interval = poll_interval
        self._running = True

    def run(self):
        while self._running:
            chunk, timestamps = self.inlet.pull_chunk(timeout=self.poll_interval)
            if chunk:
                self.new_data.emit(chunk, timestamps)

    def stop(self):
        self._running = False
        self.wait()
