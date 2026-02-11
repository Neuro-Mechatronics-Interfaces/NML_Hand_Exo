import time, os, json
import socket
from typing import Optional

import serial
from serial.tools import list_ports

# --------- UDP CONFIG ----------
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 10001

# --------- SERIAL CONFIG ----------
BAUD = 115200
SERIAL_SETTLE_S = 1.5  # board may reset when serial opens

# Map UDP messages -> gesture commands to OpenRB
# Your plan: 1 means pinch active, 0 means pinch released
PINCH_ON_CMD = "3\n"   # pinch gesture
PINCH_OFF_CMD = "1\n"  # release/open gesture (change if you want)
PROFILES_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))),
    "examples", "calibration", "profiles",
)


# def guess_openrb_port() -> Optional[str]:
#     ports = list(list_ports.comports())
#     cu_ports = [p for p in ports if "/dev/cu." in p.device]  # macOS preference

#     for p in cu_ports:
#         d = p.device.lower()
#         desc = (p.description or "").lower()
#         if "usbmodem" in d or "openrb" in desc:
#             return p.device
#     for p in cu_ports:
#         if "usbserial" in p.device.lower():
#             return p.device
#     if cu_ports:
#         return cu_ports[0].device

#     return ports[0].device if ports else None

def setup_serial(port: str, baud: int, profile: str, gesture: str) -> serial.Serial:
    """Open serial, apply calibration, set gesture mode. Returns the Serial object."""
    print(f"Connecting to {port} at {baud} baud...")
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)  # wait for board to boot after DTR reset
    ser.reset_input_buffer()

    resp = send(ser, "version")
    print(f"  Device: {resp}\n")

    # print("Applying calibration...")
    # apply_calibration(ser, profile)
    # time.sleep(0.5)

    print("Setting gesture_fixed mode...")
    send(ser, "set_exo_mode:gesture_fixed")
    time.sleep(0.5)

    # Start at open position
    print(f"Setting initial state: {gesture}:open")
    send(ser, f"set_gesture:{gesture}:open")
    time.sleep(1)

    print("Ready.\n")
    return ser

def apply_calibration(ser: serial.Serial, profile_name: str):
    """Load a calibration profile JSON and push values to the device."""
    filepath = os.path.join(PROFILES_DIR, f"{profile_name}.json")
    if not os.path.exists(filepath):
        print(f"  Warning: profile '{profile_name}' not found at {filepath}, skipping calibration.")
        return

    with open(filepath, "r") as f:
        cal = json.load(f)

    for name, vals in cal["motors"].items():
        send(ser, f"set_zero_offset:{name}:{vals['home']}")
        send(ser, f"set_motor_limits:{name}:{vals['limit_min']}:{vals['limit_max']}")
        send(ser, f"set_flip:{name}:{'1' if vals['flip'] else '0'}")
        time.sleep(0.05)

    print(f"  Calibration '{profile_name}' applied ({len(cal['motors'])} motors).")

def send(ser: serial.Serial, cmd: str, verbose: bool = True) -> str:
    """Send a command and drain the response to prevent buffer deadlock."""
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


def main():
    # port = guess_openrb_port()
    port = "COM9" # hardcoded for Windows, change as needed
    if not port:
        print("No serial ports found. Plug in the OpenRB-150 and try again.")
        return

    print(f"Opening serial: {port} @ {BAUD}")
    # ser = serial.Serial(port, baudrate=BAUD, timeout=0.1)
    ser = setup_serial(port, BAUD, "zach", "pinch_index")
    time.sleep(SERIAL_SETTLE_S)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    # sock.settimeout(0.5)  # seconds

    print(f"Listening UDP on {LISTEN_IP}:{LISTEN_PORT}")
    print("UDP '1' -> send PINCH, UDP '0' -> send RELEASE/OPEN")
    print("Press Ctrl+C to quit.\n")

    last_state = None  # debounce duplicate packets

    try:
        while True:
            data, addr = sock.recvfrom(1024)
            msg = data.decode(errors="ignore").strip()

            if msg not in ("0", "1"):
                continue

            state = int(msg)
            if state == last_state:
                continue  # ignore repeats
            last_state = state

            print(state)
            if state == 1:
                # print(f"From {addr}: {msg}  -> serial {PINCH_ON_CMD.strip()}")
                send(ser, "set_gesture:pinch_index:close")
            else:
                # print(f"From {addr}: {msg}  -> serial {PINCH_OFF_CMD.strip()}")
                send(ser, "set_gesture:pinch_index:open")

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        try:
            sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()