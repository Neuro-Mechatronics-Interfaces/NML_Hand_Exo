#!/usr/bin/env python3
"""
HC-05 ↔ Arduino Wiring Test
Helps verify the physical connections between HC-05 and Arduino are correct.
"""

import sys
import os
import time
import serial

def test_direct_arduino(com_port="COM9", baud=57600):
    """Test Arduino via direct USB - baseline test."""
    print("\n" + "="*60)
    print("TEST 1: Arduino Direct (USB)")
    print("="*60)

    try:
        ser = serial.Serial(com_port, baud, timeout=2)
        print(f"✓ Connected to {com_port} @ {baud} baud")

        # Send info command
        ser.write(b"info\r\n")
        time.sleep(0.5)

        response = b""
        while ser.in_waiting > 0:
            response += ser.read(ser.in_waiting)
            time.sleep(0.05)

        if response:
            print(f"✓ Got response: {response[:80].decode(errors='ignore')}...")
            ser.close()
            return True
        else:
            print(f"✗ No response from Arduino")
            ser.close()
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def test_hc05_loopback(hc05_port, baud=57600):
    """
    Test if HC-05 echoes back data (loopback test).
    If this works, HC-05 is working but not wired to Arduino.
    If this fails, HC-05 isn't receiving data at all.
    """
    print("\n" + "="*60)
    print(f"TEST 2: HC-05 Loopback ({hc05_port})")
    print("="*60)
    print("This tests if HC-05 is receiving AT commands properly.")

    try:
        ser = serial.Serial(hc05_port, baud, timeout=2)
        print(f"✓ Connected to {hc05_port} @ {baud} baud")

        # Wait for HC-05 to settle
        print("  Waiting 3s for Bluetooth RFCOMM settlement...")
        time.sleep(3)

        # Send AT command to HC-05 directly
        print("  Sending AT command...")
        ser.write(b"AT\r\n")
        time.sleep(0.2)

        response = b""
        start = time.time()
        while time.time() - start < 1.0:
            if ser.in_waiting > 0:
                response += ser.read(ser.in_waiting)
            time.sleep(0.05)

        ser.close()

        if b"OK" in response:
            print(f"✓ HC-05 responded to AT command: {response.decode(errors='ignore').strip()}")
            print("  → HC-05 module is working!")
            return True
        else:
            print(f"✗ No AT response (got: {response})")
            print("  → HC-05 might not be in AT command mode")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def test_hc05_arduino_comm(hc05_port, baud=57600):
    """
    Test if 'info' command sent to HC-05 gets to Arduino.
    If Arduino isn't responding but HC-05 module works, wiring is bad.
    """
    print("\n" + "="*60)
    print(f"TEST 3: HC-05 → Arduino Communication ({hc05_port})")
    print("="*60)
    print("Sending 'info' through HC-05 and checking for Arduino response.")

    try:
        ser = serial.Serial(hc05_port, baud, timeout=2)
        print(f"✓ Connected to {hc05_port} @ {baud} baud")

        # Wait for HC-05 to settle
        print("  Waiting 3s for Bluetooth RFCOMM settlement...")
        time.sleep(3)

        # Clear buffers
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)

        # Send info command
        print("  Sending 'info\\r\\n'...")
        ser.write(b"info\r\n")
        time.sleep(0.2)

        response = b""
        start = time.time()
        while time.time() - start < 3.0:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                response += chunk
                print(f"    Received {len(chunk)} bytes")
            time.sleep(0.05)

        ser.close()

        if response:
            print(f"✓ Got response: {response[:100].decode(errors='ignore')}...")
            return True
        else:
            print(f"✗ No response (Arduino not sending data through HC-05)")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def main():
    print("\n" + "█"*60)
    print("  HC-05 ↔ Arduino Wiring Diagnostic")
    print("█"*60)

    # Test 1: Baseline - Arduino via USB
    arduino_works = test_direct_arduino("COM9", 57600)

    if not arduino_works:
        print("\n⚠️  Arduino doesn't respond on COM9!")
        print("   This shouldn't happen since you said it works...")
        print("   Try a different COM port or baud rate.")
        return

    # Determine which HC-05 port to test (try both)
    hc05_ports = ["COM12", "COM13"]
    print(f"\n➤ Testing HC-05 ports: {hc05_ports}")

    for hc05_port in hc05_ports:
        print(f"\n{'='*60}")
        print(f"Testing {hc05_port}")
        print(f"{'='*60}")

        # Test 2: Can we reach HC-05 module?
        hc05_responds = test_hc05_loopback(hc05_port, 57600)

        if not hc05_responds:
            print(f"✗ HC-05 on {hc05_port} not responding to AT commands")
            print(f"  → Try different baud rate (9600, 38400, 115200)")
            continue

        # Test 3: Can data reach Arduino through HC-05?
        arduino_responds = test_hc05_arduino_comm(hc05_port, 57600)

        if arduino_responds:
            print(f"\n✅ SUCCESS! {hc05_port} is working correctly!")
            print(f"   Use {hc05_port} in the GUI.")
            return
        else:
            print(f"\n⚠️  HC-05 on {hc05_port} works but Arduino isn't responding")
            print(f"   This means: HC-05 ↔ Arduino wiring is broken")
            print(f"\n   Check:")
            print(f"   1. HC-05 TX is connected to Arduino RX")
            print(f"   2. HC-05 RX is connected to Arduino TX (with voltage divider)")
            print(f"   3. HC-05 GND is connected to Arduino GND")
            print(f"   4. Connections are solid (not loose)")

    print("\n" + "="*60)
    print("DIAGNOSIS SUMMARY")
    print("="*60)
    print("✓ Arduino hardware: OK (responds on COM9 via USB)")
    print("? HC-05 module: Depends on tests above")
    print("? HC-05 → Arduino wiring: Likely issue if HC-05 works but Arduino doesn't")
    print("\nNext steps:")
    print("1. Fix any loose wiring connections")
    print("2. Check voltage divider on HC-05 RX line if present")
    print("3. Verify HC-05 TX/RX not swapped")
    print("4. Power-cycle HC-05 and retry")

if __name__ == "__main__":
    main()
