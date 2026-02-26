"""
Interactive Calibration Script for the NML Hand Exoskeleton.

Calibration profiles are stored by name in the profiles/ directory.
Each person gets their own JSON file.  One profile can be marked as
the default so scripts can load it without specifying a name.

The script auto-detects which motors are active on the device, so it
works regardless of which ENABLE_* flags are set in config.h.

Usage:
    # Run interactive calibration for a person
    python calibrate_exo.py --port COM3 --name zach

    # Apply a saved profile (no interactive prompts)
    python calibrate_exo.py --port COM3 --apply zach

    # Apply the default profile
    python calibrate_exo.py --port COM3 --apply

    # Set a profile as the default
    python calibrate_exo.py --set-default zach

    # List all saved profiles
    python calibrate_exo.py --list-profiles
"""

import argparse
import json
import os
import re
import time
import serial
import serial.tools.list_ports


PROFILES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles")
CONFIG_FILE = os.path.join(PROFILES_DIR, "config.json")


# ---------------------------------------------------------------------------
# Profile helpers
# ---------------------------------------------------------------------------

def get_profile_path(name: str) -> str:
    """Return the full path to a named profile JSON."""
    return os.path.join(PROFILES_DIR, f"{name}.json")


def list_profiles() -> list[str]:
    """Return a sorted list of saved profile names."""
    os.makedirs(PROFILES_DIR, exist_ok=True)
    names = []
    for f in os.listdir(PROFILES_DIR):
        if f.endswith(".json") and f != "config.json":
            names.append(f.removesuffix(".json"))
    return sorted(names)


def get_default_profile() -> str | None:
    """Read the default profile name from config.json, or None."""
    if not os.path.exists(CONFIG_FILE):
        return None
    with open(CONFIG_FILE, "r") as f:
        cfg = json.load(f)
    return cfg.get("default")


def set_default_profile(name: str):
    """Write the given name as the default in config.json."""
    cfg = {}
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r") as f:
            cfg = json.load(f)
    cfg["default"] = name
    os.makedirs(PROFILES_DIR, exist_ok=True)
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=2)


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------

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
    """Ask the device for its active motor list via the 'info' command."""
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


def prompt_enter(msg: str):
    input(f"\n>>> {msg}  [Press ENTER when ready] ")


# ---------------------------------------------------------------------------
# Calibration save / load / apply
# ---------------------------------------------------------------------------

def save_calibration(filepath: str, motor_names: list, home_positions: list,
                     joint_limits: list, flip_flags: list):
    data = {"motors": {}}
    for i, name in enumerate(motor_names):
        lo, hi = joint_limits[i]
        data["motors"][name] = {
            "home": round(home_positions[i], 2),
            "limit_min": round(lo, 2),
            "limit_max": round(hi, 2),
            "flip": flip_flags[i],
        }
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n  Profile saved to: {filepath}")


def load_calibration(filepath: str) -> dict:
    with open(filepath, "r") as f:
        return json.load(f)


def update_config_h(cal: dict):
    """Update HOME_STATES, jointLimits, and DEFAULT_FLIPS in config.h from calibration."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(os.path.dirname(script_dir))
    config_path = os.path.join(repo_root, "src", "cpp", "nml_hand_exo", "config.h")

    if not os.path.exists(config_path):
        print(f"  Warning: config.h not found at {config_path}, skipping.")
        return

    with open(config_path, "r") as f:
        content = f.read()

    def replace_in_array(text, array_marker, motor_upper, new_val_str):
        """Replace a motor's value inside a specific array in config.h."""
        arr_start = text.find(array_marker)
        if arr_start == -1:
            return text
        arr_end = text.find("};", arr_start)
        if arr_end == -1:
            return text
        arr_end += 2

        section = text[arr_start:arr_end]
        new_section = re.sub(
            r'(#if ENABLE_' + motor_upper + r'\s*\n).+(\n#endif)',
            r'\1  ' + new_val_str + r',\2',
            section
        )
        return text[:arr_start] + new_section + text[arr_end:]

    for name, vals in cal["motors"].items():
        upper = name.upper()

        content = replace_in_array(
            content, "HOME_STATES[]",
            upper, str(vals["home"])
        )
        content = replace_in_array(
            content, "jointLimits[][2]",
            upper, '{' + str(vals["limit_min"]) + ', ' + str(vals["limit_max"]) + '}'
        )
        content = replace_in_array(
            content, "DEFAULT_FLIPS[]",
            upper, 'true' if vals['flip'] else 'false'
        )

    with open(config_path, "w") as f:
        f.write(content)

    print(f"  config.h updated: {config_path}")


def apply_calibration(ser: serial.Serial, cal: dict):
    """Push calibration values to the device over serial."""
    motors = cal["motors"]
    for name, vals in motors.items():
        send(ser, f"set_zero_offset:{name}:{vals['home']}")
        send(ser, f"set_motor_limits:{name}:{vals['limit_min']}:{vals['limit_max']}")
        send(ser, f"set_flip:{name}:{'1' if vals['flip'] else '0'}")
        time.sleep(0.05)
    print(f"  Applied calibration for {len(motors)} motors.")


# ---------------------------------------------------------------------------
# Interactive calibration
# ---------------------------------------------------------------------------

def run_interactive_calibration(ser: serial.Serial, motor_names: list) -> tuple:
    send(ser, "disable:all")
    time.sleep(0.5)
    print("\nTorque disabled on all motors - you can move fingers freely.")

    n = len(motor_names)
    print(f"\n  Detected {n} active motors: {', '.join(motor_names)}")

    print("\n" + "=" * 60)
    print("  INTERACTIVE CALIBRATION")
    print("  Step 1: Move ALL fingers to the FULLY OPEN position")
    print("  Step 2: Move ALL fingers to the FULLY CLOSED position")
    print("=" * 60)

    # --- Step 1: record open (home) positions ---
    prompt_enter("Move ALL fingers to the FULLY OPEN (rest) position.")
    open_angles = []
    for name in motor_names:
        angle = get_absolute_angle(ser, name)
        open_angles.append(angle)
        print(f"  {name:<12}  OPEN = {angle:.2f} deg")

    # --- Step 2: record closed positions ---
    prompt_enter("Move ALL fingers to the FULLY CLOSED position.")
    close_angles = []
    for name in motor_names:
        angle = get_absolute_angle(ser, name)
        close_angles.append(angle)
        print(f"  {name:<12}  CLOSE = {angle:.2f} deg")

    # --- Compute home, limits, flip ---
    home_positions = list(open_angles)
    joint_limits = []
    flip_flags = []
    for i in range(n):
        lo = min(open_angles[i], close_angles[i])
        hi = max(open_angles[i], close_angles[i])
        joint_limits.append((lo, hi))
        flip_flags.append(close_angles[i] < open_angles[i])

    return home_positions, joint_limits, flip_flags


def print_summary(motor_names, home_positions, joint_limits, flip_flags):
    print("\n" + "=" * 60)
    print("  CALIBRATION SUMMARY")
    print("=" * 60)
    print(f"  {'Motor':<12} {'Home':>8} {'Min':>8} {'Max':>8} {'Flip':>6}")
    print(f"  {'-'*12} {'-'*8} {'-'*8} {'-'*8} {'-'*6}")
    for i, name in enumerate(motor_names):
        lo, hi = joint_limits[i]
        flip_str = "yes" if flip_flags[i] else "no"
        print(f"  {name:<12} {home_positions[i]:>8.1f} {lo:>8.1f} {hi:>8.1f} {flip_str:>6}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="NML Hand Exo Calibration")
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g. COM3)")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate (default 57600)")
    parser.add_argument("--name", type=str, default=None,
                        help="Profile name for this person (e.g. 'zach', 'max')")
    parser.add_argument("--apply", nargs="?", const="__default__", default=None,
                        metavar="NAME",
                        help="Apply a saved profile by name (no arg = use default)")
    parser.add_argument("--set-default", type=str, default=None, metavar="NAME",
                        help="Set a profile as the default and exit")
    parser.add_argument("--list-profiles", action="store_true",
                        help="List all saved calibration profiles and exit")
    parser.add_argument("--list-ports", action="store_true",
                        help="List available serial ports and exit")
    args = parser.parse_args()

    # --- List profiles ---
    if args.list_profiles:
        profiles = list_profiles()
        default = get_default_profile()
        if not profiles:
            print("No saved profiles. Run calibration with --name <name> to create one.")
        else:
            print("Saved calibration profiles:")
            for p in profiles:
                marker = " (default)" if p == default else ""
                print(f"  {p}{marker}")
        return

    # --- Set default ---
    if args.set_default is not None:
        name = args.set_default
        path = get_profile_path(name)
        if not os.path.exists(path):
            print(f"Error: No profile found for '{name}'. Available: {list_profiles()}")
            return
        set_default_profile(name)
        print(f"Default profile set to: {name}")
        return

    # --- List ports ---
    if args.list_ports:
        print("Available serial ports:")
        list_serial_ports()
        return

    # --- Resolve port ---
    if args.port is None and (args.apply is not None or args.name is not None):
        print("Available serial ports:")
        list_serial_ports()
        args.port = input("\nEnter the COM port to use: ").strip()

    # --- Apply-only mode ---
    if args.apply is not None:
        if args.apply == "__default__":
            profile_name = get_default_profile()
            if profile_name is None:
                print("Error: No default profile set. Use --set-default <name> first,")
                print("       or specify a name: --apply <name>")
                return
        else:
            profile_name = args.apply

        path = get_profile_path(profile_name)
        if not os.path.exists(path):
            print(f"Error: No profile found for '{profile_name}'. Available: {list_profiles()}")
            return

        print(f"\nConnecting to {args.port} at {args.baud} baud...")
        ser = serial.Serial(args.port, args.baud, timeout=1)
        time.sleep(2)
        ser.reset_input_buffer()
        resp = send(ser, "version")
        print(f"Device response: {resp}")

        print(f"\nApplying profile: {profile_name}")
        cal = load_calibration(path)
        apply_calibration(ser, cal)
        update_config_h(cal)

        motor_names = list(cal["motors"].keys())
        home_positions = [cal["motors"][n]["home"] for n in motor_names]
        joint_limits = [(cal["motors"][n]["limit_min"], cal["motors"][n]["limit_max"]) for n in motor_names]
        flip_flags = [cal["motors"][n]["flip"] for n in motor_names]
        print_summary(motor_names, home_positions, joint_limits, flip_flags)

        ser.close()
        print(f"\nProfile '{profile_name}' applied. Done.")
        return

    # --- Interactive calibration ---
    if args.name is None:
        args.name = input("Enter a name for this calibration profile: ").strip().lower()
    if not args.name:
        print("Error: Profile name cannot be empty.")
        return

    if args.port is None:
        print("Available serial ports:")
        list_serial_ports()
        args.port = input("\nEnter the COM port to use: ").strip()

    print(f"\nConnecting to {args.port} at {args.baud} baud...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()
    resp = send(ser, "version")
    print(f"Device response: {resp}")

    # Query active motors from the device
    motor_names = query_motor_names(ser)

    home_positions, joint_limits, flip_flags = run_interactive_calibration(ser, motor_names)

    # Save profile
    profile_path = get_profile_path(args.name)
    save_calibration(profile_path, motor_names, home_positions, joint_limits, flip_flags)

    # Update config.h so firmware defaults match this calibration
    cal = load_calibration(profile_path)
    update_config_h(cal)

    # Set as default if it's the first profile or user wants it
    current_default = get_default_profile()
    if current_default is None:
        set_default_profile(args.name)
        print(f"  Set '{args.name}' as the default profile (first profile created).")
    else:
        make_default = input(f"\n  Set '{args.name}' as the default profile? (y/n): ").strip().lower()
        if make_default == "y":
            set_default_profile(args.name)
            print(f"  Default profile updated to: {args.name}")

    # Apply to device
    print("\n" + "=" * 60)
    print("  APPLYING CALIBRATION TO DEVICE")
    print("=" * 60)
    cal = load_calibration(profile_path)
    apply_calibration(ser, cal)

    print_summary(motor_names, home_positions, joint_limits, flip_flags)

    print()
    reenable = input("Re-enable torque on all motors? (y/n): ").strip().lower()
    if reenable == "y":
        send(ser, "enable:all")
        print("Torque re-enabled.")

    ser.close()
    print(f"\nCalibration complete. Profile '{args.name}' saved and applied.")


if __name__ == "__main__":
    main()
