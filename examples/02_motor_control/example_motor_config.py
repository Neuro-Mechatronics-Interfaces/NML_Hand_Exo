"""
Example: Motor Configuration
Demonstrates setting velocity, acceleration, and motor limits.
"""

from nml_hand_exo.interface import HandExo, SerialComm
import time

# Configuration
port = "COM6"
baudrate = 1000000

# Connect to device
comm = SerialComm(port=port, baudrate=baudrate)
exo = HandExo(comm, verbose=True)
exo.connect()
motor_id = None

try:
    motors = exo.info().get('motors', {})
    if not motors:
        raise RuntimeError("Firmware reported no motors; cannot configure a motor.")
    motor_id = sorted(motors)[0]
    print(f"Using configured motor ID {motor_id} ({motors[motor_id].get('name', 'unnamed')})")

    print("=" * 60)
    print("Motor Configuration Example")
    print("=" * 60)
    
    # Get current configuration
    print(f"\nCurrent Configuration for Motor {motor_id}:")
    velocity = exo.get_motor_velocity(motor_id)
    acceleration = exo.get_motor_acceleration(motor_id)
    limits = exo.get_motor_limits(motor_id)
    
    print(f"  Velocity: {velocity} deg/s")
    print(f"  Acceleration: {acceleration} deg/s²")
    print(f"  Limits: {limits}")
    
    # Set new velocity profile (slower movement)
    print(f"\nSetting velocity to 50 deg/s...")
    exo.set_motor_velocity(motor_id, 50)
    time.sleep(0.5)
    
    # Verify
    new_velocity = exo.get_motor_velocity(motor_id)
    print(f"  New velocity: {new_velocity} deg/s")
    
    # Set new acceleration profile
    print(f"\nSetting acceleration to 30 deg/s²...")
    exo.set_motor_acceleration(motor_id, 30)
    time.sleep(0.5)
    
    # Verify
    new_accel = exo.get_motor_acceleration(motor_id)
    print(f"  New acceleration: {new_accel} deg/s²")
    
    # Set custom motor limits (safety bounds)
    print(f"\nSetting motor limits to [-30, 60] degrees...")
    exo.set_motor_limits(motor_id, lower_limit=-30, upper_limit=60)
    time.sleep(0.5)
    
    # Verify
    new_limits = exo.get_motor_limits(motor_id)
    print(f"  New limits: {new_limits}")
    
    # Test movement with new settings
    print(f"\nTesting movement with new velocity/acceleration settings...")
    exo.enable_motor(motor_id)
    time.sleep(0.5)
    
    print("  Moving to 30 degrees...")
    exo.set_motor_angle(motor_id, 30)
    time.sleep(2)
    
    print("  Moving back to 0 degrees...")
    exo.set_motor_angle(motor_id, 0)
    time.sleep(2)
    
    # Restore original settings (optional)
    print(f"\nRestoring original settings...")
    exo.set_motor_velocity(motor_id, velocity)
    exo.set_motor_acceleration(motor_id, acceleration)
    exo.set_motor_limits(motor_id, limits[0], limits[1])
    
    print("\n✅ Configuration example complete!")

except Exception as e:
    print(f"❌ Error: {e}")
    
finally:
    if motor_id is not None:
        exo.disable_motor(motor_id)
    exo.close()
    print("\n🔌 Connection closed")
