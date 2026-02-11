"""
Range of Motion (ROM) Assessment Script for the NML Hand Exoskeleton.

Walks through a two-phase protocol:
  Phase 1 – Unassisted ROM
      Record open-hand motor positions, then closed-hand motor positions.
  Phase 2 – Active-Assisted ROM
      Same recording procedure with the exoskeleton assisting movement.

For each phase the script continuously samples motor positions until the
operator presses ENTER, then keeps the max (or min) value per motor.

Motor orientation is handled by loading a calibration profile (--profile).
Motors with ``flip: true`` decrease in absolute angle when closing, while
others increase.  Raw absolute angles are converted to *normalized* angles
(0 = fully open / home, positive = closing direction) so that ROM values
are always positive and comparable across motors regardless of encoder
direction.

Results are saved to  output_data/<participant>_rom_<date>_<run>.csv

Usage:
    python rom_assessment.py --port COM5
    python rom_assessment.py --port COM5 --profile zach
    python rom_assessment.py --port COM5 --baud 57600
"""

import argparse
import csv
import json
import os
import re
import sys
import time
import threading
import serial
import serial.tools.list_ports
from datetime import datetime


PROFILES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles")
CONFIG_FILE = os.path.join(PROFILES_DIR, "config.json")


# ── Calibration profile helpers ───────────────────────────────────────────

def load_calibration_profile(name: str | None) -> dict | None:
    """Load a calibration profile by name, falling back to the default.

    Returns the profile dict or None if no profile is available.
    """
    if name is None:
        # Try the default profile
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, "r") as f:
                cfg = json.load(f)
            name = cfg.get("default")
        if name is None:
            return None

    path = os.path.join(PROFILES_DIR, f"{name}.json")
    if not os.path.exists(path):
        print(f"  Warning: calibration profile '{name}' not found at {path}")
        return None

    with open(path, "r") as f:
        return json.load(f)


def build_motor_orientation(cal: dict | None, motor_names: list[str]) -> dict:
    """Build a per-motor orientation dict from a calibration profile.

    Returns {motor_name: {"home": float, "flip": bool}} for every motor.
    Motors not in the profile default to home=0, flip=False.
    """
    orientation = {}
    for name in motor_names:
        if cal and name in cal.get("motors", {}):
            m = cal["motors"][name]
            orientation[name] = {"home": m["home"], "flip": m["flip"]}
        else:
            orientation[name] = {"home": 0.0, "flip": False}
    return orientation


def normalize_angle(absolute: float, home: float, flip: bool) -> float:
    """Convert an absolute encoder angle to a normalized ROM angle.

    Normalized angle = 0 at the home (open) position, positive in the
    closing direction.  This makes ROM values always positive regardless
    of whether the physical encoder counts up or down when closing.
    """
    if flip:
        return home - absolute  # flipped motors decrease when closing
    else:
        return absolute - home  # normal motors increase when closing


# ── Serial helpers (same conventions as calibrate_exo.py) ─────────────────

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("  No serial ports found.")
    for p in ports:
        print(f"  {p.device}  -  {p.description}")


def send(ser: serial.Serial, cmd: str) -> str:
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
    return response.decode(errors="ignore").strip()


def query_motor_names(ser: serial.Serial) -> list[str]:
    raw = send(ser, "info")
    names = []
    for line in raw.splitlines():
        m = re.search(r"name:\s*(\w+)", line)
        if m:
            names.append(m.group(1))
    if not names:
        raise RuntimeError(f"Could not parse motor names from device info:\n{raw}")
    return names


def get_absolute_angle(ser: serial.Serial, motor_name: str) -> float:
    raw = send(ser, f"get_absolute_angle:{motor_name}")
    for line in raw.splitlines():
        if "absolute_angle" in line:
            parts = line.split("absolute_angle:")
            if len(parts) >= 2:
                num_str = parts[1].strip().rstrip("};").strip()
                try:
                    return float(num_str)
                except ValueError:
                    pass
    raise RuntimeError(f"Could not parse absolute angle from response:\n{raw}")


# ── Recording helpers ─────────────────────────────────────────────────────

def record_positions(ser: serial.Serial, motor_names: list[str],
                     sample_interval: float = 0.1) -> dict[str, list[float]]:
    """Continuously sample absolute motor angles until ENTER is pressed.

    Returns a dict mapping each motor name to its list of sampled angles.
    """
    samples: dict[str, list[float]] = {name: [] for name in motor_names}
    stop_event = threading.Event()

    def _wait_for_enter():
        input()
        stop_event.set()

    listener = threading.Thread(target=_wait_for_enter, daemon=True)
    listener.start()

    print("  Recording... press ENTER to stop.")
    while not stop_event.is_set():
        for name in motor_names:
            try:
                angle = get_absolute_angle(ser, name)
                samples[name].append(angle)
            except RuntimeError:
                pass  # skip failed reads
        time.sleep(sample_interval)

    count = min(len(v) for v in samples.values()) if samples else 0
    print(f"  Stopped. Collected ~{count} samples per motor.")
    return samples


# ── Single ROM phase ──────────────────────────────────────────────────────

def run_rom_phase(ser: serial.Serial, motor_names: list[str],
                  orientation: dict, phase_label: str) -> dict:
    """Run one ROM phase (open then closed) and return results.

    Returns dict per motor with keys:
        open_abs_max, open_abs_min   – raw absolute extremes during open
        closed_abs_max, closed_abs_min – raw absolute extremes during closed
        open_norm_max   – max normalized angle during open recording
        closed_norm_max – max normalized angle during closed recording
        rom             – total range of motion (closed_norm_max - open_norm_min)
    """
    print(f"\n{'=' * 60}")
    print(f"  {phase_label}")
    print(f"{'=' * 60}")

    # ── Open hand ──
    print(f"\n  Step 1: OPEN hand position")
    input("  >>> Place the hand in the OPEN position and press ENTER to begin recording. ")
    open_samples = record_positions(ser, motor_names)

    # ── Closed hand ──
    print(f"\n  Step 2: CLOSED hand position")
    input("  >>> Place the hand in the CLOSED position and press ENTER to begin recording. ")
    closed_samples = record_positions(ser, motor_names)

    # ── Compute results per motor ──
    results = {}
    for name in motor_names:
        home = orientation[name]["home"]
        flip = orientation[name]["flip"]

        o_vals = open_samples[name]
        c_vals = closed_samples[name]

        o_abs_max = max(o_vals) if o_vals else float("nan")
        o_abs_min = min(o_vals) if o_vals else float("nan")
        c_abs_max = max(c_vals) if c_vals else float("nan")
        c_abs_min = min(c_vals) if c_vals else float("nan")

        # Normalized angles (0 = home/open, positive = closing direction)
        o_norm = [normalize_angle(v, home, flip) for v in o_vals] if o_vals else [0.0]
        c_norm = [normalize_angle(v, home, flip) for v in c_vals] if c_vals else [0.0]

        o_norm_max = max(o_norm)
        o_norm_min = min(o_norm)
        c_norm_max = max(c_norm)
        c_norm_min = min(c_norm)

        # ROM = furthest closed position minus resting open position
        rom = c_norm_max - o_norm_min

        results[name] = {
            "open_abs_max": o_abs_max,
            "open_abs_min": o_abs_min,
            "closed_abs_max": c_abs_max,
            "closed_abs_min": c_abs_min,
            "open_norm_max": o_norm_max,
            "open_norm_min": o_norm_min,
            "closed_norm_max": c_norm_max,
            "closed_norm_min": c_norm_min,
            "rom": rom,
            "flip": flip,
        }

    # Print summary
    print(f"\n  {phase_label} – Results")
    print(f"  {'Motor':<12} {'Flip':>5} {'Open(norm)':>11} {'Closed(norm)':>13} {'ROM':>8}")
    print(f"  {'-'*12} {'-'*5} {'-'*11} {'-'*13} {'-'*8}")
    for name in motor_names:
        r = results[name]
        flip_str = "yes" if r["flip"] else "no"
        print(f"  {name:<12} {flip_str:>5} {r['open_norm_min']:>11.2f} "
              f"{r['closed_norm_max']:>13.2f} {r['rom']:>8.2f}")

    return results


# ── CSV output ────────────────────────────────────────────────────────────

def determine_run_number(output_dir: str, participant: str, date_str: str) -> int:
    """Find the next available run number for this participant + date."""
    prefix = f"{participant}_rom_{date_str}_"
    run = 1
    if os.path.isdir(output_dir):
        for fname in os.listdir(output_dir):
            if fname.startswith(prefix) and fname.endswith(".csv"):
                try:
                    n = int(fname.removeprefix(prefix).removesuffix(".csv"))
                    run = max(run, n + 1)
                except ValueError:
                    pass
    return run


def save_results(output_dir: str, participant: str, date_str: str,
                 motor_names: list[str],
                 unassisted: dict, assisted: dict):
    os.makedirs(output_dir, exist_ok=True)
    run = determine_run_number(output_dir, participant, date_str)
    filename = f"{participant}_rom_{date_str}_{run}.csv"
    filepath = os.path.join(output_dir, filename)

    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "participant", "date", "run",
            "motor", "flip", "phase",
            "open_abs_max", "open_abs_min",
            "closed_abs_max", "closed_abs_min",
            "open_norm_max", "open_norm_min",
            "closed_norm_max", "closed_norm_min",
            "rom_deg",
        ])
        for phase_label, data in [("unassisted", unassisted),
                                  ("assisted", assisted)]:
            for name in motor_names:
                r = data[name]
                writer.writerow([
                    participant, date_str, run,
                    name, r["flip"], phase_label,
                    f"{r['open_abs_max']:.2f}",
                    f"{r['open_abs_min']:.2f}",
                    f"{r['closed_abs_max']:.2f}",
                    f"{r['closed_abs_min']:.2f}",
                    f"{r['open_norm_max']:.2f}",
                    f"{r['open_norm_min']:.2f}",
                    f"{r['closed_norm_max']:.2f}",
                    f"{r['closed_norm_min']:.2f}",
                    f"{r['rom']:.2f}",
                ])

    print(f"\n  Results saved to: {filepath}")
    return filepath


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="NML Hand Exo – ROM Assessment")
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g. COM5)")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate (default 57600)")
    parser.add_argument("--profile", type=str, default=None,
                        help="Calibration profile name (default: use default profile)")
    parser.add_argument("--list-ports", action="store_true",
                        help="List available serial ports and exit")
    args = parser.parse_args()

    if args.list_ports:
        print("Available serial ports:")
        list_serial_ports()
        return

    # ── Participant name ──
    participant = input("Enter participant name: ").strip().lower()
    if not participant:
        print("Error: participant name cannot be empty.")
        sys.exit(1)

    # ── Load calibration profile for motor orientation ──
    cal = load_calibration_profile(args.profile)
    if cal:
        profile_name = args.profile or "(default)"
        print(f"  Loaded calibration profile: {profile_name}")
    else:
        print("  Warning: no calibration profile loaded.")
        print("  Motor orientation (flip) will not be accounted for.")
        print("  Use --profile <name> to load one.\n")

    # ── Serial port ──
    if args.port is None:
        print("\nAvailable serial ports:")
        list_serial_ports()
        args.port = input("\nEnter the COM port to use: ").strip()

    print(f"\nConnecting to {args.port} at {args.baud} baud...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()
    resp = send(ser, "version")
    print(f"Device response: {resp}")

    motor_names = query_motor_names(ser)
    print(f"\nDetected {len(motor_names)} motors: {', '.join(motor_names)}")

    # Build orientation map from calibration profile
    orientation = build_motor_orientation(cal, motor_names)
    print("\n  Motor orientation:")
    print(f"  {'Motor':<12} {'Home':>8} {'Flip':>5}")
    print(f"  {'-'*12} {'-'*8} {'-'*5}")
    for name in motor_names:
        o = orientation[name]
        flip_str = "yes" if o["flip"] else "no"
        print(f"  {name:<12} {o['home']:>8.2f} {flip_str:>5}")

    # Disable torque so the hand can be moved freely
    send(ser, "disable:all")
    time.sleep(0.5)
    print("\nTorque disabled – you can move fingers freely.\n")

    # ── Phase 1: Unassisted ROM ──
    unassisted = run_rom_phase(ser, motor_names, orientation,
                               "PHASE 1 – UNASSISTED ROM")

    # ── Phase 2: Active-Assisted ROM ──
    input("\n>>> Ready for active-assisted ROM? Press ENTER to continue. ")
    assisted = run_rom_phase(ser, motor_names, orientation,
                             "PHASE 2 – ACTIVE-ASSISTED ROM")

    # ── Save ──
    date_str = datetime.now().strftime("%Y-%m-%d")
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "..", "..", "output_data")
    save_results(output_dir, participant, date_str, motor_names,
                 unassisted, assisted)

    ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
