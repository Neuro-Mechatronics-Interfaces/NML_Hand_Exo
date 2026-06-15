# This script connects to the exo and sends an angle command in increments of 5 degrees every second.
# For each motor, in increasing order of the motor id, the angle is starting from home/rest increasing until the maximum joint upper limit is reached for that motor
# and afterwards goes back to home/rest position. Then the motor decreases angle in increments of 5 degrees every second until the minimum joint lower limit is reached for that motor
# and afterwards goes back to home/rest position.

from nml_hand_exo.interface import HandExo, SerialComm
import time

# Serial usage

port = "COM6"
baudrate = 57600

comm = SerialComm(port=port, baudrate=baudrate, verbose=False)
exo = HandExo(comm, verbose=False)

STEP_RATE = 5  # Hz

try:
    exo.connect()
    print(exo.version())
    time.sleep(0.5)

    # Get home states for the motors
    home_states = exo.get_home('all')
    print(f"Home states: {home_states}")
    for motor_id, angle in home_states.items():
        print(f"| Motor {motor_id} home angle: {angle['angle']} degrees")
    time.sleep(0.5)

    # Set home
    print("Setting home positions...")
    exo.home()
    time.sleep(1)

    # Get joint ranges
    data_dir = exo.get_motor_limits('all')
    print("Joint ranges:")
    for motor_id, limits in data_dir.items():
        print(f"| Motor {motor_id}: Lower limit = {limits['lower_limit']}, Upper limit = {limits['upper_limit']}")
    time.sleep(0.5)

    print("Starting joint range test...")
    for motor in sorted(data_dir.keys()):
        motor_data = data_dir[motor]
        motor_id = motor_data['id']
        lower_limit = int(motor_data['lower_limit'])
        upper_limit = int(motor_data['upper_limit'])
        home_state = int(home_states[motor]['angle'])

        # Subtract the home state from limits to get relative angles
        #lower_limit -= home_state
        #upper_limit -= home_state

        print(f"\nTesting Motor {motor_id}")
        time.sleep(0.5)

        # Sweep up to upper limit
        print(f"  Sweeping motor {motor_id} to upper limit ({upper_limit})...")
        for angle in range(home_state, upper_limit + 1, 1):
            exo.set_motor_angle(motor_id, angle)
            print(f"    → Angle: {angle:>3}°", end='\r', flush=True)
            time.sleep(1 / STEP_RATE)
        print(f"    → Reached upper limit {upper_limit}°")

        # Return to home
        for angle in range(upper_limit, home_state - 1, -1):
            exo.set_motor_angle(motor_id, angle)
            print(f"    Returning to home: {angle:>3}°", end='\r', flush=True)
            time.sleep(1 / STEP_RATE)

        # Sweep down to lower limit
        print(f"  Sweeping to lower limit ({lower_limit})...")
        for angle in range(home_state, lower_limit - 1, -1):
            exo.set_motor_angle(motor_id, angle)
            print(f"    ← Angle: {angle:>3}°", end='\r', flush=True)
            time.sleep(1 / STEP_RATE)
        print(f"    ← Reached lower limit {lower_limit}°")

        # Return to home again
        print(f"  Returning to home from lower limit...")
        for angle in range(lower_limit, home_state + 1, 1):
            exo.set_motor_angle(motor_id, angle)
            print(f"    Returning to home: {angle:>3}°", end='\r', flush=True)
            time.sleep(1 / STEP_RATE)

except Exception as e:
    print(f"Error: {e}")

finally:
    exo.close()
