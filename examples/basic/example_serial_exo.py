from nml_hand_exo.interface import HandExo, SerialComm

# Serial usage
port = "COM12"
baudrate = 115200

comm = SerialComm(port=port, baudrate=baudrate)
exo = HandExo(comm, verbose=False)

try:
    exo.connect()
    print(exo.version())
    print(exo.get_exo_mode())
    print(exo.get_home())
    print(exo.info())
    print(exo.get_absolute_motor_angle())
    print(exo.get_motor_angle())
    print(exo.get_motor_velocity())
    print(exo.get_gesture())
    print(exo.get_gesture_state())
    print(exo.get_motor_torque())
except Exception as e:
    print(f"Error: {e}")
finally:
    exo.close()