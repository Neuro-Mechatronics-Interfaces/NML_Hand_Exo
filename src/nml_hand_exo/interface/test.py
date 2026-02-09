"""
Gesture control test — direct serial with response draining.

Uses the same send-and-read pattern as calibrate_exo.py to avoid
deadlocking the firmware's USB serial buffer.
"""
import json
import os
import time
import serial

PORT = "COM3"
BAUD = 57600
PROFILE = "zach"

# ── Helpers ──────────────────────────────────────────────────────────

def send(ser: serial.Serial, cmd: str, verbose: bool = True) -> str:
    """Send a command and drain the response (prevents buffer deadlock)."""
    ser.reset_input_buffer()
    ser.write((cmd.strip() + "\n").encode())
    time.sleep(0.15)

    response = b""
    deadline = time.time() + 2.0
    while time.time() < deadline:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting)
            response += chunk
            if b";" in chunk:
                break
        else:
            time.sleep(0.02)

    text = response.decode(errors="ignore").strip()
    if verbose:
        print(f"  > {cmd}")
        if text:
            print(f"  < {text[:120]}")
    return text


def apply_calibration(ser: serial.Serial, profile_name: str):
    """Load a calibration profile JSON and push values to the device."""
    profiles_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__))))),
        "examples", "calibration", "profiles",
    )
    filepath = os.path.join(profiles_dir, f"{profile_name}.json")
    with open(filepath, "r") as f:
        cal = json.load(f)

    for name, vals in cal["motors"].items():
        send(ser, f"set_zero_offset:{name}:{vals['home']}")
        send(ser, f"set_motor_limits:{name}:{vals['limit_min']}:{vals['limit_max']}")
        send(ser, f"set_flip:{name}:{'1' if vals['flip'] else '0'}")
        time.sleep(0.05)

    print(f"  Calibration '{profile_name}' applied ({len(cal['motors'])} motors).")

# ── Main ─────────────────────────────────────────────────────────────

print(f"Connecting to {PORT} at {BAUD} baud...")
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # wait for board to boot after serial-DTR reset
ser.reset_input_buffer()

# Verify firmware is alive
resp = send(ser, "version")
print(f"  Device says: {resp}\n")

# Apply calibration
print("Applying calibration...")
apply_calibration(ser, PROFILE)
time.sleep(0.5)

# Switch to gesture mode
print("\nSetting gesture_fixed mode...")
send(ser, "set_exo_mode:gesture_fixed")
time.sleep(0.5)

# Test gestures
print("\n--- Testing grasp OPEN ---")
send(ser, "set_gesture:grasp:open")
time.sleep(2)

print("--- Testing grasp CLOSE ---")
send(ser, "set_gesture:grasp:close")
time.sleep(2)

print("--- Testing grasp OPEN ---")
send(ser, "set_gesture:grasp:open")
time.sleep(1)

ser.close()
print("\nDone.")
