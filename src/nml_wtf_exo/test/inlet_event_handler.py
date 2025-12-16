import signal
import sys
import json
from pylsl import resolve_byprop, StreamInlet
from nml_hand_exo.interface import HandExo, SerialComm
import re
import time

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
def camel_to_snake(name: str) -> str:
    """
    Convert CamelCase or PascalCase to snake_case.
    Examples:
        PinchIndex   -> pinch_index
        KeyGrip      -> key_grip
        Grasp        -> grasp
    """
    if name == "IndexPinch":
        return "pinch_index"
    elif name == "MiddlePinch":
        return "pinch_middle"
    else:
        s1 = re.sub(r'(.)([A-Z][a-z]+)', r'\1_\2', name)
        s2 = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s1)
        return s2.lower()

def send_start_gesture(exo_client, gesture_name):
    """Tell the exo to activate a gesture."""
    try:
        name = camel_to_snake(gesture_name)
        print(f"[API] set_gesture('{name}', 'close')")
        exo_client.set_gesture(name, 'close')
    except Exception as e:
        print(f"[API ERROR] set_gesture: {e}")

def send_end_gesture(exo_client, gesture_name):
    """Tell the exo to release/open."""
    try:
        name = camel_to_snake(gesture_name)
        print(f"[API] set_gesture('{name}','open')")
        exo_client.set_gesture(name, 'open')
    except Exception as e:
        print(f"[API ERROR] set_gesture_state: {e}")


def resolve_lsl_stream_by_type_and_name(
    stream_type: str,
    stream_name: str,
    timeout: float = 2.0,
    retry_interval: float = 1.0,
    max_retries: int | None = None,
):
    """
    Resolve an LSL stream by (type, name), retrying if not found.

    max_retries = None  -> retry forever
    """
    attempt = 0

    while True:
        attempt += 1
        print(f"[LSL] Resolving '{stream_name}' (attempt {attempt})...")

        streams = resolve_byprop("type", stream_type, timeout=timeout)

        for s in streams:
            if s.name() == stream_name:
                print("[LSL] Found matching stream.")
                return s

        print("[LSL] Stream not found.")
        if streams:
            print("[LSL] Available streams:")
            for s in streams:
                print(f"  - name='{s.name()}', type='{s.type()}', source_id='{s.source_id()}'")

        if max_retries is not None and attempt >= max_retries:
            raise RuntimeError(
                f"Failed to resolve LSL stream '{stream_name}' "
                f"after {max_retries} attempts"
            )

        time.sleep(retry_interval)

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
    exo.set_exo_mode("GESTURE_FIXED");
    exo.home('all')

    # Wait for stream
    print(f"[LSL] Resolving streams with type='{TARGET_TYPE}'...")

    try:
        info = resolve_lsl_stream_by_type_and_name(
            stream_type=TARGET_TYPE,
            stream_name=TARGET_NAME,
            timeout=2.0,
            retry_interval=1.0,
            max_retries=None,   # retry forever
        )
    except RuntimeError as e:
        print(f"[LSL ERROR] {e}")
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
