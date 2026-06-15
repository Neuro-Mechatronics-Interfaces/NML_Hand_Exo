from nml_hand_exo.interface import HandExo, SerialComm
import time

port = "COM6"
baudrate = 57600

comm = SerialComm(port=port, baudrate=baudrate, verbose=False)
exo = HandExo(comm, verbose=False)

STEP_RATE = 5  # Hz

exo.connect()
print("Testing motor control...")

exo.set_motor_angle(0,0)
time.sleep(1)
exo.set_motor_angle(0,30)
time.sleep(1)
exo.set_motor_angle(0,0)

time.sleep(1)
exo.close()

