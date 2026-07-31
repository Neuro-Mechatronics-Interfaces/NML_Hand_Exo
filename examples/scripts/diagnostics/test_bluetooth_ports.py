#!/usr/bin/env python3
"""
Bluetooth Port Diagnostic Tool
Tests both COM ports to identify which one is actually receiving data from the Arduino.
"""

import sys
import os
import time
import serial
from serial.tools import list_ports

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def test_port(port, baudrate=57600, timeout=2.0):
    """Test a single port for Bluetooth communication."""
    print(f"\n{'='*60}")
    print(f"Testing {port}")
    print(f"{'='*60}")

    try:
        # Open port
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"✓ Port opened successfully")

        # Wait for HC-05 RFCOMM to settle (Bluetooth only)
        if 'COM' in port.upper():
            print("  Waiting 3s for Bluetooth RFCOMM settlement...")
            time.sleep(3.0)

        # Clear any pending data
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)

        # Send the info command
        cmd = "info\r\n"
        print(f"  Sending: {repr(cmd)}")
        ser.write(cmd.encode())
        time.sleep(0.1)

        # Wait for response
        print(f"  Waiting up to {timeout}s for response...")
        start = time.time()
        response = b""

        while time.time() - start < timeout:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                response += chunk
                print(f"    Received {len(chunk)} bytes: {repr(chunk[:80])}")
            time.sleep(0.05)

        ser.close()

        if response:
            print(f"\n✓ SUCCESS: {port} received {len(response)} bytes")
            print(f"Response:\n{response.decode(errors='ignore')}")
            return True
        else:
            print(f"\n✗ FAILED: {port} received NO data")
            return False

    except Exception as e:
        print(f"\n✗ ERROR: {port} - {e}")
        return False

def main():
    print("Bluetooth Port Diagnostic Tool")
    print("This tests both HC-05 COM ports to find which one is connected.\n")

    # List available ports
    ports = [p.device for p in list_ports.comports()]
    print(f"Available ports: {ports}\n")

    # Filter for Bluetooth ports (usually COM12-COM13 range or marked as Bluetooth in description)
    bt_ports = []
    for p in list_ports.comports():
        if 'bluetooth' in p.description.lower() or 'hc-05' in p.description.lower() or p.device in ['COM12', 'COM13']:
            bt_ports.append(p.device)

    if not bt_ports:
        print("No Bluetooth ports found in device list.")
        print("Available ports:")
        for p in list_ports.comports():
            print(f"  {p.device}: {p.description}")
        print("\nManually test ports (enter comma-separated list, e.g., COM12,COM13):")
        user_input = input("> ").strip()
        bt_ports = [p.strip() for p in user_input.split(',')]

    if not bt_ports:
        print("No ports to test.")
        return

    print(f"\nTesting {len(bt_ports)} port(s):\n")

    results = {}
    for port in bt_ports:
        success = test_port(port)
        results[port] = success

    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    working = [p for p, s in results.items() if s]
    if working:
        print(f"✓ Working port(s): {', '.join(working)}")
        print(f"\nUse the working port in the GUI to connect.")
    else:
        print(f"✗ No working ports found!")
        print("\nTroubleshooting:")
        print("  1. Check HC-05 is powered (red LED blinking)")
        print("  2. Check Arduino is powered and uploading 'info' command")
        print("  3. Verify HC-05 TX/RX connected to Arduino RX/TX (not RX/RX)")
        print("  4. Try different baud rates (57600, 115200) in Settings")
        print("  5. Restart HC-05 by power-cycling")

if __name__ == "__main__":
    main()
