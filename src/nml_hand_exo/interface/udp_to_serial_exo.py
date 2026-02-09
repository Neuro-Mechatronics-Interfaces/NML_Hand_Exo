"""
WebSocket-to-Serial bridge for NML Hand Exo gesture control.

Runs a WebSocket server. Any client can connect and send '1' or '0':
  - '1' → pinch close (set_gesture:pinch_index:close)
  - '0' → pinch open  (set_gesture:pinch_index:open)

On startup the script:
  1. Opens the serial port and waits for the board to boot.
  2. Applies a calibration profile (sets zero offsets, limits, flip flags).
  3. Switches the exo to gesture_fixed mode.
  4. Starts the WebSocket server and waits for messages.

Usage:
    pip install pyserial websockets
    python udp_to_serial_exo.py                         # defaults
    python udp_to_serial_exo.py --port COM5             # specify serial port
    python udp_to_serial_exo.py --ws-port 8765          # change WS listen port
    python udp_to_serial_exo.py --gesture grasp         # use grasp instead of pinch_index
    python udp_to_serial_exo.py --profile max           # use a different calibration profile
"""

import argparse
import asyncio
import json
import os
import time

import serial
import serial.tools.list_ports

try:
    import websockets
except ImportError:
    print("ERROR: websockets is required.  Install with:  pip install websockets")
    raise SystemExit(1)


# ── Defaults ────────────────────────────────────────────────────────────
DEFAULT_BAUD = 57600
DEFAULT_WS_HOST = "0.0.0.0"
DEFAULT_WS_PORT = 8765
DEFAULT_GESTURE = "pinch_index"
DEFAULT_PROFILE = "zach"

PROFILES_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))),
    "examples", "calibration", "profiles",
)


# ── Serial helpers (proven send-and-drain pattern) ──────────────────────

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


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("  No serial ports found.")
    for p in ports:
        print(f"  {p.device}  -  {p.description}")
    return ports


# ── Serial setup ────────────────────────────────────────────────────────

def setup_serial(port: str, baud: int, profile: str, gesture: str) -> serial.Serial:
    """Open serial, apply calibration, set gesture mode. Returns the Serial object."""
    print(f"Connecting to {port} at {baud} baud...")
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)  # wait for board to boot after DTR reset
    ser.reset_input_buffer()

    resp = send(ser, "version")
    print(f"  Device: {resp}\n")

    print("Applying calibration...")
    apply_calibration(ser, profile)
    time.sleep(0.5)

    print("Setting gesture_fixed mode...")
    send(ser, "set_exo_mode:gesture_fixed")
    time.sleep(0.5)

    # Start at open position
    print(f"Setting initial state: {gesture}:open")
    send(ser, f"set_gesture:{gesture}:open")
    time.sleep(1)

    print("Ready.\n")
    return ser


# ── WebSocket server ────────────────────────────────────────────────────

def make_handler(ser: serial.Serial, gesture: str):
    """Return an async handler that maps '0'/'1' to gesture open/close."""
    last_state = None

    async def handler(websocket):
        nonlocal last_state
        peer = websocket.remote_address
        print(f"[ws] Client connected: {peer}")

        try:
            async for message in websocket:
                msg = message.strip() if isinstance(message, str) else message.decode(errors="ignore").strip()

                if msg not in ("0", "1"):
                    continue

                state = int(msg)
                if state == last_state:
                    continue  # debounce duplicates
                last_state = state

                if state == 1:
                    cmd = f"set_gesture:{gesture}:close"
                else:
                    cmd = f"set_gesture:{gesture}:open"

                print(f"[ws] {peer} sent {msg}  ->  {cmd}")
                send(ser, cmd, verbose=False)

        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            print(f"[ws] Client disconnected: {peer}")

    return handler


async def run_server(ser: serial.Serial, gesture: str, host: str, port: int):
    handler = make_handler(ser, gesture)
    async with websockets.serve(handler, host, port):
        print(f"WebSocket server listening on ws://{host}:{port}")
        print(f"  Send '1' to close ({gesture}:close)")
        print(f"  Send '0' to open  ({gesture}:open)")
        print("  Press Ctrl+C to quit.\n")
        await asyncio.get_running_loop().create_future()  # run forever


# ── CLI ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="WebSocket-to-Serial bridge for NML Hand Exo gesture control"
    )
    parser.add_argument("--port", type=str, default=None,
                        help="Serial port (e.g. COM3). Prompts if omitted.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default {DEFAULT_BAUD})")
    parser.add_argument("--profile", type=str, default=DEFAULT_PROFILE,
                        help=f"Calibration profile name (default '{DEFAULT_PROFILE}')")
    parser.add_argument("--gesture", type=str, default=DEFAULT_GESTURE,
                        help=f"Gesture to trigger (default '{DEFAULT_GESTURE}')")
    parser.add_argument("--ws-host", type=str, default=DEFAULT_WS_HOST,
                        help=f"WebSocket listen address (default '{DEFAULT_WS_HOST}')")
    parser.add_argument("--ws-port", type=int, default=DEFAULT_WS_PORT,
                        help=f"WebSocket listen port (default {DEFAULT_WS_PORT})")
    parser.add_argument("--list-ports", action="store_true",
                        help="List serial ports and exit")
    args = parser.parse_args()

    if args.list_ports:
        print("Available serial ports:")
        list_serial_ports()
        return

    if args.port is None:
        print("Available serial ports:")
        list_serial_ports()
        args.port = input("\nEnter the COM port to use: ").strip()

    ser = setup_serial(args.port, args.baud, args.profile, args.gesture)

    try:
        asyncio.run(run_server(ser, args.gesture, args.ws_host, args.ws_port))
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        try:
            ser.close()
            print("Serial port closed.")
        except Exception:
            pass


if __name__ == "__main__":
    main()
