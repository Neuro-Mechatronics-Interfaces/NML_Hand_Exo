#!/usr/bin/env python3
"""
Simple port scanner - list all available COM ports and their status.
"""

import sys
import os
import time
import serial
from serial.tools import list_ports

def list_all_ports():
    """Show all available COM ports."""
    ports = list_ports.comports()
    if not ports:
        print("No COM ports found!")
        return
    
    print("\nAvailable COM Ports:")
    print("="*60)
    for p in ports:
        print(f"  {p.device:8} - {p.description}")

def quick_test(port, baud=57600, timeout=1.0):
    """Quick test to see if a port responds to 'info' command."""
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        ser.write(b"info\r\n")
        time.sleep(0.3)
        
        response = b""
        while ser.in_waiting > 0:
            response += ser.read(ser.in_waiting)
            time.sleep(0.05)
        
        ser.close()
        
        if response:
            return True, response[:50].decode(errors='ignore')
        return False, ""
    except Exception as e:
        return False, str(e)

def main():
    print("\n" + "█"*60)
    print("  COM Port Quick Scan")
    print("█"*60)
    
    # List all ports
    list_all_ports()
    
    # Test different baud rates on likely ports
    print("\n\nTesting Common Baud Rates:")
    print("="*60)
    
    test_ports = ["COM9", "COM12", "COM13"]
    baud_rates = [9600, 38400, 57600, 115200]
    
    found_working = []
    
    for port in test_ports:
        for baud in baud_rates:
            success, response = quick_test(port, baud, timeout=0.5)
            if success:
                found_working.append((port, baud, response))
                print(f"✓ {port} @ {baud:6} baud - GOT RESPONSE: {response}")
            # Don't spam output with failures
    
    if not found_working:
        print("✗ No working ports found with standard baud rates")
        print("\nManual steps:")
        print("1. Check USB cable is plugged into Arduino")
        print("2. Check Arduino LED is blinking/powered")
        print("3. Try connecting to each port manually:")
        print("   python -c \"import serial; s=serial.Serial('COMx', 115200); s.write(b'info\\r\\n'); print(s.read(100))\"")
    else:
        print(f"\n✓ Found {len(found_working)} working port(s)")
        for port, baud, resp in found_working:
            print(f"  → Use {port} @ {baud} baud in GUI")

if __name__ == "__main__":
    main()
