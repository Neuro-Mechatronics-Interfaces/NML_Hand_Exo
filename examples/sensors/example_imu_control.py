"""
Example: IMU-Based Wrist Control
Demonstrates controlling motor angle based on IMU wrist orientation.
"""

from nml_hand_exo.interface import HandExo, SerialComm
import time

# Configuration
port = "COM6"
baudrate = 57600
wrist_motor_id = 5  # Adjust to your wrist motor ID

# Connect to device
comm = SerialComm(port=port, baudrate=baudrate)
exo = HandExo(comm, verbose=True)
exo.connect()

try:
    print("=" * 60)
    print("IMU-Based Wrist Control Example")
    print("=" * 60)
    
    # Enable wrist motor
    print(f"\nEnabling wrist motor (ID: {wrist_motor_id})...")
    exo.enable_motor(wrist_motor_id)
    time.sleep(1)
    
    # Get current IMU orientation
    print("\nCurrent IMU orientation:")
    imu_data = exo.get_imu_data()
    print(f"  Heading (Yaw): {imu_data['heading']:.2f}°")
    print(f"  Pitch: {imu_data['pitch']:.2f}°")
    print(f"  Roll: {imu_data['roll']:.2f}°")
    
    # Set wrist to track specific yaw angles
    print("\nMoving wrist to match target yaw angles...")
    
    # Target angle 1: 45 degrees, flexion direction
    target_yaw_1 = 45
    print(f"\n  Target yaw: {target_yaw_1}° (flex direction)")
    exo.set_yaw_angle(wrist_motor_id, target_yaw_1, direction="flex")
    time.sleep(3)
    
    # Check achieved angle
    current_yaw = exo.get_imu_heading()
    print(f"  Achieved yaw: {current_yaw:.2f}°")
    
    # Target angle 2: 0 degrees, extension direction
    target_yaw_2 = 0
    print(f"\n  Target yaw: {target_yaw_2}° (extend direction)")
    exo.set_yaw_angle(wrist_motor_id, target_yaw_2, direction="extend")
    time.sleep(3)
    
    # Check achieved angle
    current_yaw = exo.get_imu_heading()
    print(f"  Achieved yaw: {current_yaw:.2f}°")
    
    # Target angle 3: -30 degrees, extension direction
    target_yaw_3 = -30
    print(f"\n  Target yaw: {target_yaw_3}° (extend direction)")
    exo.set_yaw_angle(wrist_motor_id, target_yaw_3, direction="e")  # Can use short form
    time.sleep(3)
    
    # Check achieved angle
    current_yaw = exo.get_imu_heading()
    print(f"  Achieved yaw: {current_yaw:.2f}°")
    
    print("\n✅ IMU-based control example complete!")
    
except Exception as e:
    print(f"❌ Error: {e}")
    
finally:
    exo.disable_motor(wrist_motor_id)
    exo.close()
    print("\n🔌 Connection closed")
