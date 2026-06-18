import sys
from PyQt5.QtWidgets import QApplication
from nml_hand_exo.interface import LSLClient
from nml_hand_exo.plotting import GridPlotter

if __name__ == "__main__":
    app = QApplication(sys.argv)
    client = LSLClient(stream_type="EMG")
    plotter = GridPlotter(
        client=client,
        buffer_ms=500,
        channels_to_plot=list(range(64)),
    )
    plotter.start()
    sys.exit(app.exec_())
