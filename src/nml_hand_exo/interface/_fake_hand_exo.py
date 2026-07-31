import time
import numpy as np

from ._interfaces import BaseComm


class FakeHandExo(object):
    """
    Fake HandExo class for testing purposes.
    This class simulates the behavior of the real HandExo device.
    """

    def __init__(self, comm: BaseComm, name: str = "FakeHandExo", verbose: bool = False):
        self.name = name
        self.comm = comm
        self.verbose = verbose
        self.connected = True  # Simulate a successful connection
        self._imu_data = np.zeros(3)  # Simulated IMU data (roll, pitch, yaw)
        self.motor_names = ['wrist', 'wrist2', 'thumbadd', 'thumbrot', 'thumbflex', 'index', 'middle', 'ring', 'pinky']
        self.motor_ids = {name: i for i, name in enumerate(self.motor_names)}
        self.home_position = np.zeros(len(self.motor_names))  # Simulated home position

    def logger(self, *argv, warning: bool = False):
        """
        Robust debugging print function

        Args:
            *argv             : (str) Messages to log.
            warning           : (bool) If True, prints the message in yellow.

        """
        if self.verbose:
            msg = ''.join(argv)
            msg = f"[{time.monotonic():.3f}][{self.name}] {msg}"

            # If a warning, print the text in yellow
            msg = f"\033[93m{msg}\033[0m" if warning else msg
            print(msg)

    def set_comm(self, comm: BaseComm):
        """
        Sets the communication interface for the exoskeleton.

        Args:
            comm (BaseComm): The communication interface to use.

        """
        self.device = comm
        if self.verbose:
            self.logger(f"Communication interface set to {comm.__class__.__name__}")

    def connect(self):
        """
        Simulate connecting to the HandExo device.
        """
        if self.verbose:
            self.logger("Connected to HandExo device.")

    def close(self):
        """
        Simulate closing the connection to the HandExo device.
        """
        self.connected = False
        if self.verbose:
            self.logger("Connection to HandExo device closed.")

    def get_imu_angles(self):
        """Simulate reading IMU angles."""
        return self._imu_data

    def set_imu_angles(self, roll, pitch, yaw):
        """Simulate setting IMU angles."""
        self._imu_data = np.array([roll, pitch, yaw])

    def get_motor_angles(self):
        """Simulate reading motor angles."""
        return np.random.uniform(0, 180, len(self.motor_names))

    def set_motor_angles(self, angles):
        """Simulate setting motor angles."""
        if len(angles) != len(self.motor_names):
            raise ValueError("Angles must match the number of motors.")
        self.logger(f"Setting motor angles: {angles}")

    def home(self):
        """Simulate homing the device."""
        self.set_motor_angles(self.home_position)
        if self.verbose:
            self.logger("Homing the device to the home position.")


    def set_gesture_state(self, gesture_state):
        """
        Simulate setting the gesture state.

        Args:
            gesture_state (str): The gesture state to set.
        """
        if self.verbose:
            if gesture_state not in ['rest', 'open', 'close']:
                self.logger(f"Invalid gesture state: {gesture_state}", warning=True)
                return
            else:
                self.logger(f"Setting gesture state to: {gesture_state}")