import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QSlider, QLineEdit, QTextEdit, QGridLayout, QMessageBox
)
from PyQt5.QtCore import Qt

from nml_hand_exo.interface import HandExo, SerialComm, TCPComm  # Assumes hand_exo.py is in the same directory


class HandExoGUI(QWidget):
    """    A GUI for controlling the HandExo device.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HandExo Control Panel")
        self.exo = None
        self.n_motors = 1 # Default to 1 motor until connected
        self.sliders = []
        self.angle_fields = []

        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Connection row
        conn_layout = QHBoxLayout()
        self.port_input = QLineEdit("COM1")
        self.port_input.setFixedWidth(60)
        self.baud_input = QLineEdit("57600")
        self.baud_input.setFixedWidth(120)
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_to_device)
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: red")

        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_input)
        conn_layout.addWidget(QLabel("Baud:"))
        conn_layout.addWidget(self.baud_input)
        conn_layout.addWidget(self.connect_button)
        conn_layout.addWidget(self.status_label)

        main_layout.addLayout(conn_layout)

        # Sliders panel
        self.slider_panel = QGridLayout()
        main_layout.addLayout(self.slider_panel)

        # By default, let's make one slider
        self.create_sliders()

        # Log window
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setFixedHeight(30)
        main_layout.addWidget(QLabel("Log:"))
        main_layout.addWidget(self.log)

        self.setLayout(main_layout)

    def connect_to_device(self):
        port = self.port_input.text()
        baud = int(self.baud_input.text())
        try:
            comm = SerialComm(port=port, baudrate=baud)
            self.exo = HandExo(comm)
            self.exo.connect()
            if self.exo.connected:
                info = self.exo.info()
                self.n_motors = info.get("n_motors", 0)
                self.status_label.setText("Connected")
                self.status_label.setStyleSheet("color: green")
                self.log.append("Connected to device. Found {} motors.".format(self.n_motors))
                self.create_sliders()
        except Exception as e:
            self.status_label.setText("Failed to Connect")
            self.status_label.setStyleSheet("color: red")
            self.log.append(f"Connection failed: {e}")
            QMessageBox.critical(self, "Connection Error", str(e))

    def create_sliders(self):
        # Clear any existing sliders
        for i in reversed(range(self.slider_panel.count())):
            widget = self.slider_panel.itemAt(i).widget()
            if widget:
                widget.setParent(None)

        self.sliders = []
        self.angle_fields = []

        for i in range(self.n_motors):
            slider = QSlider(Qt.Vertical)
            slider.setMinimum(-90)
            slider.setMaximum(90)
            slider.setValue(0)
            slider.valueChanged.connect(self.make_slider_handler(i))
            slider.setTickPosition(QSlider.TicksRight)
            slider.setTickInterval(10)

            angle_field = QLineEdit("0")
            angle_field.setFixedWidth(50)
            angle_field.returnPressed.connect(self.make_text_handler(i))

            self.sliders.append(slider)
            self.angle_fields.append(angle_field)

            self.slider_panel.addWidget(QLabel(f"Motor {i}"), 0, i)
            self.slider_panel.addWidget(slider, 1, i)
            self.slider_panel.addWidget(angle_field, 2, i)

    def make_slider_handler(self, idx):
        def handler(value):
            try:
                self.angle_fields[idx].setText(str(value))
                if self.exo and self.exo.connected:
                    self.exo.set_motor_angle(idx, value)
                    self.log.append(f"Set motor {idx} to {value}°")
            except Exception as e:
                self.log.append(f"Error: {e}")
        return handler

    def make_text_handler(self, idx):
        def handler():
            try:
                angle = float(self.angle_fields[idx].text())
                self.sliders[idx].blockSignals(True)  # Prevent slider update
                self.sliders[idx].setValue(int(angle))
                self.sliders[idx].blockSignals(False)  # Re-enable slider update
                if self.exo and self.exo.connected:
                    self.exo.set_motor_angle(idx, angle)
                    self.log.append(f"Set motor {idx} to {angle}°")
            except Exception as e:
                self.log.append(f"Error: {e}")
        return handler


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HandExoGUI()
    window.resize(800, 400)
    window.show()
    sys.exit(app.exec_())
