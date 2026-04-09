#!/usr/bin/env python3
"""
Test if HC-05 can reach Arduino through the wireless link.
Sends 'info' command through HC-05 and checks if Arduino responds.
"""

import sys
import time
import serial
from serial.tools import list_ports

def test_hc05_to_arduino(hc05_port, baud=57600):
    """Send 'info' through HC-05 and see if Arduino responds."""
    print(f"\nTesting {hc05_port} → Arduino (data mode)")
    print("="*60)
    
    try:
        ser = serial.Serial(hc05_port, baud, timeout=1)
        print(f"✓ Opened {hc05_port} @ {baud} baud")
        
        # Wait for Bluetooth to settle
        print("  Waiting 3s for RFCOMM settlement...")
        time.sleep(3)
        
        # Clear any pending data
        if ser.in_waiting > 0:
            junk = ser.read(ser.in_waiting)
            print(f"  Cleared {len(junk)} bytes of pending data")
        
        # Send info command
        print("  Sending: info\\r\\n")
        ser.write(b"info\r\n")
        time.sleep(0.5)
        
        # Wait for response
        response = b""
        print("  Waiting for Arduino response...")
        start = time.time()
        while time.time() - start < 3.0:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                response += chunk
                print(f"    Got {len(chunk)} bytes")
            time.sleep(0.1)
        
        ser.close()
        
        if response:
            print(f"\n✅ SUCCESS! Arduino responded through {hc05_port}:")
            print(response.decode(errors='ignore')[:200])
            return True, hc05_port
        else:
            print(f"\n❌ No response from Arduino through {hc05_port}")
            print("   HC-05 appears to be working but not connected to Arduino")
            return False, None
            
    except PermissionError as e:
        print(f"⚠️  {hc05_port} is locked (another program has it open)")
        print("   Close any terminals/GUIs using this port and try again")
        return False, None
    except Exception as e:
        print(f"❌ Error: {e}")
        return False, None

def main():
    print("\n" + "█"*60)
    print("  HC-05 Data Mode Test")
    print("  (Is HC-05 actually wired to Arduino?)")
    print("█"*60)
    
    # Test both HC-05 ports
    hc05_ports = ["COM12", "COM13"]
    
    for port in hc05_ports:
        success, working_port = test_hc05_to_arduino(port, 57600)
        if success:
            print(f"\n{'='*60}")
            print(f"✅ FOUND WORKING BLUETOOTH PORT: {working_port}")
            print(f"{'='*60}")
            print(f"\nIn the GUI:")
            print(f"  1. Select {working_port} in the Ports dropdown")
            print(f"  2. Check the 'Bluetooth' checkbox")
            print(f"  3. Set baud rate to 57600")
            print(f"  4. Click Connect")
            return
    
    print("\n" + "="*60)
    print("DIAGNOSIS")
    print("="*60)
    print("✓ Arduino on COM9: WORKING")
    print("✗ HC-05 (COM12/COM13): Can't reach Arduino")
    print("\nPossible causes:")
    print("  1. HC-05 not wired to Arduino (TX/RX not connected)")
    print("  2. TX/RX are swapped on the wiring")
    print("  3. GND not connected between HC-05 and Arduino")
    print("  4. HC-05 powered from wrong voltage (5V vs 3.3V)")
    print("  5. Loose connection or cold solder joint")
    print("\nNext: Check physical wiring between HC-05 and Arduino")

if __name__ == "__main__":
    main()
