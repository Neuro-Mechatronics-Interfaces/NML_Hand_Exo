# examples/lsl_quick_listen.py
#!/usr/bin/env python3
import time
import argparse
from nml_hand_exo.interface import LSLMarkerSubscriber
from nml_hand_exo.interface import HandExo, SerialComm, LSLClient

def main():
    ap = argparse.ArgumentParser(description="Quick LSL listener (Markers).")
    ap.add_argument("--name", default=None, help="Stream name to resolve")
    ap.add_argument("--type", default="Markers", help="Stream type to resolve")
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("--drive_exo", action="store_true", help="If set, send markers to the exo")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    # Connect to your exo
    exo = None
    if args.drive_exo:
        comm = SerialComm(port=args.port, baudrate=args.baudrate)
        exo = HandExo(comm, verbose=args.verbose)
        exo.connect()

    # Callback
    def on_marker_cb(value: str, ts: float, exo: HandExo = None):
        print(f"[marker] {value} at {ts:.6f}")
        if exo is not None:
            try:
                exo.set_gesture(value)
            except Exception as e:
                print(f"[exo] error calling set_gesture: {e}")

    # Subscribe & listen
    with LSLMarkerSubscriber(stream_name=args.name,
                             stream_type=args.type,
                             timeout=args.timeout,
                             verbose=args.verbose) as sub:
        # Fire only when the text changes; pass the exo as a positional arg
        sub.set_callback(on_marker_cb, exo, only_on_change=True)

        print("Listening… Ctrl+C to quit.")
        try:
            while True:
                time.sleep(0.25)  # idle; callbacks run in background
        except KeyboardInterrupt:
            print("\nDone.")

if __name__ == "__main__":
    main()