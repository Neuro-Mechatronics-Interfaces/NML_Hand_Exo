# Description: Example of reading IMU angles from a HandExo device using Python.

import time
from nml_hand_exo.interface import HandExo, SerialComm

# Initialize the HandExo device with the specified serial port and baudrate.
port = 'COM6'
baudrate = 1000000

comm = SerialComm(port=port, baudrate=baudrate)
exo = HandExo(comm, verbose=False)
exo.connect()

# Wait for the device to be ready.
time.sleep(1)

# Continuously read and print the roll, pitch, and yaw angles from the IMU.
try:
    while True:
        rpy = exo.get_imu_angles()
        print(rpy)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopping IMU readings...")
finally:
    exo.close()
