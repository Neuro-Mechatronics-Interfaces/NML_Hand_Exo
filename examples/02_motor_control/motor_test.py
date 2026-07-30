from nml_hand_exo.interface import HandExo, SerialComm
import time

port = "COM6"
baudrate = 1000000

comm = SerialComm(port=port, baudrate=baudrate, verbose=False)
exo = HandExo(comm, verbose=False)

STEP_RATE = 5  # Hz

exo.connect()
print("Testing motor control...")

motors = exo.info().get("motors", {})
if not motors:
    raise RuntimeError("Firmware reported no motors; cannot run motor test.")
motor_id = sorted(motors)[0]
print(f"Using configured motor ID {motor_id}")

exo.set_motor_angle(motor_id, 0)
time.sleep(1)
exo.set_motor_angle(motor_id, 30)
time.sleep(1)
exo.set_motor_angle(motor_id, 0)

time.sleep(1)
exo.close()

