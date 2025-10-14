import argparse
from nml_hand_exo.interface import (
    HandExo,
    SerialComm,
    LSLMarkerSubscriber,
    GestureController
)


def get_gesture_and_state_from_marker(marker: str) -> tuple[str, str]:
    """
    Helper function to extract gesture and state from a marker string.
    Expected format: KeyGripOpen, HandClose, IndexPinchClose, etc.
    """
    marker = marker.lower()
    parts = marker.split('_')
    gesture, state = None, None

    if len(parts) == 2:
        gesture, state = parts

    if len(parts) == 1:
        # Try to see if the word 'open' or 'close' is at the end of the string without underscore
        if parts[0].endswith('open'):
            state = 'open'
            gesture = parts[0][:-4]
        elif parts[0].endswith('close'):
            state = 'close'
            gesture = parts[0][:-5]

    if gesture == 'indexpinch':
        gesture = 'pinch_index'
    elif gesture == 'middlepinch':
        gesture = 'pinch_middle'
    elif gesture == 'ringpinch':
        gesture = 'pinch_ring'
    elif gesture == 'hand':
        gesture = 'grasp'

    return gesture, state


if __name__ == "__main__":

    ap = argparse.ArgumentParser(description="LSL Gesture Controller for Hand Exoskeleton.")
    ap.add_argument("--type", default="Markers", help="LSL stream type to subscribe to.")
    ap.add_argument("--name", default="EMGGesture", help="LSL stream name to subscribe to.")
    ap.add_argument("--port", type=str, default="COM4", help="Serial port for Hand Exoskeleton.")
    ap.add_argument("--baudrate", type=int, default=115200, help="Baudrate for serial communication.")
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("--verbose", action="store_true", help="Enable verbose output.")
    args = ap.parse_args()

    # Connect to Hand Exoskeleton
    comm = SerialComm(port=args.port, baudrate=args.baudrate)
    exo = HandExo(comm, verbose=args.verbose)
    exo.connect()

    # Create gesture controller
    gc = GestureController(exo=exo, verbose=args.verbose)

    def on_gesture(value: str, ts: float):
        # Pass gesture to gesture controller
        #print(f"[gesture] {value} at {ts:.6f}")
        gesture, state = get_gesture_and_state_from_marker(value)
        if gc.current_gesture != gesture or gc.current_state != state:
            gc.set_gesture(gesture, state)

    # Create an LSL subscriber to listen and receive gesture commands
    with LSLMarkerSubscriber(stream_type=args.type, stream_name=args.name, timeout=args.timeout, verbose=args.verbose) as sub:
        # background callback
        sub.set_callback(on_gesture, poll_hz=20)
        print("Listening… Ctrl+C to quit.")
        try:
            while True:
                # you can also poll explicitly:
                # msg = sub.pull(timeout=0.0)
                # if msg: print("polled:", msg)
                pass
        except KeyboardInterrupt:
            print("\nDone.")

