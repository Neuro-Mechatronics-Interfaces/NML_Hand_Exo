"""
Example: Advanced Device Configuration
Demonstrates baudrate, debug mode, OLED control, and motor modes.
"""

from nml_hand_exo.interface import HandExo, SerialComm
import time

# Configuration
port = "COM6"
baudrate = 1000000

# Connect to device
comm = SerialComm(port=port, baudrate=baudrate)
exo = HandExo(comm, verbose=False)  # Start with verbose off
exo.connect()

try:
    print("=" * 60)
    print("Advanced Device Configuration Example")
    print("=" * 60)
    
    # 1. Debug Mode Control
    print("\n1. Testing Debug Mode Control")
    print("  Enabling debug mode on Arduino...")
    exo.set_debug(True)
    time.sleep(0.5)
    
    print("  Sending a command to see debug output...")
    exo.version()
    time.sleep(1)
    
    print("  Disabling debug mode...")
    exo.set_debug(False)
    time.sleep(0.5)
    
    # 2. OLED Display Control
    print("\n2. Testing OLED Display Control")
    print("  Getting OLED status...")
    oled_status = exo.get_oled_status()
    print(f"  Current status: {oled_status}")
    
    print("  Enabling OLED...")
    result = exo.enable_oled()
    print(f"  Response: {result}")
    time.sleep(1)
    
    print("  Disabling OLED...")
    result = exo.disable_oled()
    print(f"  Response: {result}")
    time.sleep(1)
    
    # 3. Motor Control Modes
    print("\n3. Testing Motor Control Modes")
    print("  Current motor mode:")
    current_mode = exo.get_motor_mode()
    print(f"    {current_mode}")
    
    print("\n  Switching to velocity mode...")
    exo.set_motor_mode("velocity")
    time.sleep(0.5)
    new_mode = exo.get_motor_mode()
    print(f"    Mode: {new_mode}")
    
    print("\n  Switching to current_position mode...")
    exo.set_motor_mode("current_position")
    time.sleep(0.5)
    new_mode = exo.get_motor_mode()
    print(f"    Mode: {new_mode}")
    
    print("\n  Switching back to position mode...")
    exo.set_motor_mode("position")
    time.sleep(0.5)
    new_mode = exo.get_motor_mode()
    print(f"    Mode: {new_mode}")
    
    # 4. Exo Operating Modes
    print("\n4. Testing Exo Operating Modes")
    print("  Current exo mode:")
    exo_mode = exo.get_exo_mode()
    print(f"    {exo_mode}")
    
    print("\n  Available modes: FREE, GESTURE_FIXED, GESTURE_CONTINUOUS")
    
    print("\n  Switching to GESTURE_FIXED mode...")
    exo.set_exo_mode("GESTURE_FIXED")
    time.sleep(0.5)
    new_exo_mode = exo.get_exo_mode()
    print(f"    Mode: {new_exo_mode}")
    
    print("\n  Switching back to FREE mode...")
    exo.set_exo_mode("FREE")
    time.sleep(0.5)
    new_exo_mode = exo.get_exo_mode()
    print(f"    Mode: {new_exo_mode}")
    
    # 5. Baudrate Information (read-only in this example)
    print("\n5. Motor Baudrate Information")
    motors = exo.info().get('motors', {})
    if not motors:
        raise RuntimeError("Firmware reported no motors; cannot query motor baudrate.")
    motor_id = sorted(motors)[0]
    baudrate_info = exo.get_baudrate(motor_id)
    print(f"  Motor {motor_id} baudrate: {baudrate_info}")
    
    # Note: Changing baudrate requires careful handling and reconnection
    # Uncomment with caution:
    # print("\n  ⚠️  Changing baudrate (requires reconnection)...")
    # exo.set_baudrate(motor_id, 1000000)
    # time.sleep(1)
    
    # 6. Device Information Summary
    print("\n6. Complete Device Information")
    info = exo.info()
    print(f"  Name: {info['name']}")
    print(f"  Version: {info['version']}")
    print(f"  Motors: {info['n_motors']}")
    for motor_id, motor_info in info['motors'].items():
        print(f"    Motor {motor_id} ({motor_info['name']}): "
              f"angle={motor_info['angle']:.1f}°, "
              f"limits={motor_info['limits']}, "
              f"enabled={motor_info['enabled']}")
    
    print("\n✅ Advanced configuration example complete!")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
    
finally:
    exo.close()
    print("\n🔌 Connection closed")
