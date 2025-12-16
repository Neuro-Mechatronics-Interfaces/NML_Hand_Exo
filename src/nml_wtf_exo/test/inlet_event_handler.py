import signal
import sys
import json
from pylsl import resolve_byprop, StreamInlet
from nml_hand_exo.interface import HandExo, SerialComm

# Most firmwares use 57600 and semicolon-delimited tokens.
comm = SerialComm(port="COM3", baudrate=57600, command_delimiter="\r\n", timeout=1)

# HandExo expects a comm object at construction
exo = HandExo(comm, name="NMLHandExo", verbose=True)

TARGET_TYPE = "NML-CTRL"
TARGET_NAME = "ceres-emg-events"

# ----------------------------------------------------------------
#  Global state
# ----------------------------------------------------------------
running = True

def handle_sigint(sig, frame):
    global running
    print("\n[LSL] SIGINT received, shutting down...")
    running = False

signal.signal(signal.SIGINT, handle_sigint)

# --------------------------------------------------
#  Event mapping helpers
# --------------------------------------------------

def send_start_gesture(exo_client, gesture_name):
    """Tell the exo to activate a gesture."""
    try:
        print(f"[API] set_gesture('{gesture_name}', 'closed')")
        exo_client.set_gesture(gesture_name, 'closed')
    except Exception as e:
        print(f"[API ERROR] set_gesture: {e}")

def send_end_gesture(exo_client, gesture_name):
    """Tell the exo to release/open."""
    try:
        print(f"[API] set_gesture('{gesture_name}','open')")
        exo_client.set_gesture(gesture_name, 'open')
    except Exception as e:
        print(f"[API ERROR] set_gesture_state: {e}")

# --------------------------------------------------
#  Main LSL loop
# --------------------------------------------------

def main():
    global exo

    # Create the exo client object
    print("[API] Initializing NMLHandExo client...")

    # Try connecting (auto finds the device)
    print("[API] Connecting to exo device...")
    exo.connect()
    exo.enable_motor('all')
    exo.set_exo_mode("GESTURE_CONTINUOUS");
    exo.home('all')

    # Wait for stream
    print(f"[LSL] Resolving streams with type='{TARGET_TYPE}'...")

    streams = resolve_byprop("type", TARGET_TYPE, timeout=5.0)

    if not streams:
        print("[LSL] No streams found with matching type.")
        sys.exit(1)

    info = None
    for s in streams:
        if s.name() == TARGET_NAME:
            info = s
            break

    if info is None:
        print(f"[LSL] No stream found with name='{TARGET_NAME}'.")
        print("[LSL] Available streams:")
        for s in streams:
            print(f"  - name='{s.name()}', source_id='{s.source_id()}'")
        sys.exit(1)

    print("[LSL] Connected to stream:")
    print(f"  Name      : {info.name()}")
    print(f"  Type      : {info.type()}")
    print(f"  Channels  : {info.channel_count()}")
    print(f"  Source ID : {info.source_id()}")

    inlet = StreamInlet(info)

    print("[LSL] Listening for events...\n")

    while running:
        sample, ts = inlet.pull_sample(timeout=1.0)
        if not sample:
            continue

        name, timing, evt_type, payload_raw = sample

        # Lowercase timing for matching
        timing_l = timing.lower()

        print("--------------------------------------------------")
        print(f"Timestamp: {ts:.6f}  name={name} timing={timing}")

        # Map timing -> action
        if timing_l == "start":
            send_start_gesture(exo, name)

        elif timing_l == "end":
            send_end_gesture(exo, name)

        elif timing_l == "instant":
            # You can customize this: call exo.set_gesture(name, 'instant'), etc.
            print(f"[API] instant event for {name} (not supported yet)")

        else:
            print(f"[LSL] Unknown timing '{timing}'")

    # Clean exit
    print("[LSL] Closing exo connection.")
    exo.disable_motor('all')
    exo.close()
    print("[LSL] Clean exit.")

if __name__ == "__main__":
    main()
