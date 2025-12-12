"""
Example: Batch Operations
Demonstrates using the 'all' keyword for batch motor operations.
"""

from nml_hand_exo.interface import HandExo, SerialComm
import time

# Configuration
port = "COM6"
baudrate = 57600

# Connect to device
comm = SerialComm(port=port, baudrate=baudrate)
exo = HandExo(comm, verbose=True)
exo.connect()

try:
    print("=" * 60)
    print("Batch Operations Example")
    print("=" * 60)
    
    # Get info about all motors
    print("\n1. Getting device information...")
    info = exo.info()
    print(f"Device: {info['name']}, Version: {info['version']}")
    print(f"Number of motors: {info['n_motors']}")
    
    # Get all motor angles at once
    print("\n2. Reading all motor angles...")
    angles = exo.get_motor_angle('all')
    for motor_id, angle in angles.items():
        print(f"  Motor {motor_id}: {angle:.2f}°")
    
    # Get all motor velocities
    print("\n3. Reading all motor velocity profiles...")
    velocities = exo.get_motor_velocity('all')
    for motor_id, vel in velocities.items():
        print(f"  Motor {motor_id}: {vel:.2f} deg/s")
    
    # Get all motor limits
    print("\n4. Reading all motor limits...")
    limits = exo.get_motor_limits('all')
    for motor_id, lim in limits.items():
        print(f"  Motor {motor_id}: {lim}")
    
    # Enable all motors
    print("\n5. Enabling all motors...")
    exo.enable_motor('all')
    time.sleep(1)
    
    # Turn on all LEDs
    print("\n6. Turning on all LEDs...")
    exo.enable_led('all')
    time.sleep(2)
    
    # Move all motors to a synchronized position
    print("\n7. Setting all motors to 20 degrees...")
    for motor_id in angles.keys():
        exo.set_motor_angle(motor_id, 20)
    time.sleep(3)
    
    # Read all angles again
    print("\n8. Verifying positions...")
    new_angles = exo.get_motor_angle('all')
    for motor_id, angle in new_angles.items():
        print(f"  Motor {motor_id}: {angle:.2f}°")
    
    # Return all motors to home
    print("\n9. Returning all motors to home position...")
    exo.home('all')
    time.sleep(3)
    
    # Turn off all LEDs
    print("\n10. Turning off all LEDs...")
    exo.disable_led('all')
    time.sleep(1)
    
    # Get all motor currents
    print("\n11. Reading all motor currents...")
    currents = exo.get_motor_current('all')
    for motor_id, current in currents.items():
        print(f"  Motor {motor_id}: {current:.3f} mA")
    
    print("\n✅ Batch operations example complete!")
    
except Exception as e:
    print(f"❌ Error: {e}")
    
finally:
    # Disable all motors
    print("\n12. Disabling all motors...")
    exo.disable_motor('all')
    exo.close()
    print("\n🔌 Connection closed")
