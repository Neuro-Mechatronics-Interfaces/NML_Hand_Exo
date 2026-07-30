#!/usr/bin/env python3
"""
Read the exo's current joint angles and emit paste-ready config.h values.

Motors are never enabled by this tool -- position the digits by hand with
torque off, then capture.

Typical workflow, one capture per physical pose:

    python scripts/diagnostics/capture_pose.py --save home
    #   ... move the digits to ONE extreme of travel ...
    python scripts/diagnostics/capture_pose.py --save min
    #   ... move the digits to the OTHER extreme ...
    python scripts/diagnostics/capture_pose.py --save max
    python scripts/diagnostics/capture_pose.py --emit

"min" and "max" are just slot names for the two extremes -- it does NOT matter
which one you capture as which.  Each motor's pair is sorted numerically at emit
time, and it has to be: motors differ in direction (DEFAULT_FLIPS), so full
flexion is the LOWER angle on some joints and the HIGHER angle on others.  What
matters is only that the two captures are the two ends of travel, and that the
home pose lies between them.

`--emit` prints HOME_STATES and jointLimits blocks built from the saved poses,
ordered by Dynamixel ID to match MOTOR_IDS, and warns about anything that would
produce a joint with zero travel (home outside its limits, or the two extremes
too close together) -- the exact failure that leaves a digit accepting commands
without moving.

With no arguments it just prints the live angles and does not save anything.
"""

import argparse
import json
import os
import sys
import time

from nml_hand_exo import DualSerialComm, HandExo


CMD_PORT = "COM10"
TELEM_PORT = "COM11"
BAUD = 1000000
LINE_TERMINATOR = "\r\n"
DEFAULT_STORE = "pose_captures.json"
POSE_NAMES = ("home", "min", "max")

# Enabling torque is a physical move, not just a register write: give the
# motors time to reach their goal before reading positions back.
ARM_SETTLE_S = 1.0

# The firmware reports exactly 0.0 for a motor that did not answer on the
# Dynamixel bus -- its own initializeMotors() uses the same test. Capturing
# those would write a wall of zeros into config.h for hardware that is not
# even connected, so they are dropped unless --include-zero is passed.
NONRESPONSIVE_ANGLE = 0.0

# A swept range smaller than this is almost certainly a capture mistake (the
# digit was not actually moved between the two extremes) rather than a real
# mechanical limit.
DEFAULT_MIN_RANGE_DEG = 2.0


def read_angles(comm, exo, timeout, attempts=3, include_zero=False):
    """Return {dxl_id: (name, absolute_angle)} for every motor.

    get_absolute_angle:all does one Dynamixel round trip PER MOTOR before it
    replies, so this is far slower than a single-value query -- hence the
    generous timeout and the retries.
    """
    raw = ""
    for attempt in range(1, attempts + 1):
        comm.flush_input()
        comm.send("get_absolute_angle:all" + LINE_TERMINATOR)
        raw = comm.receive(wait_until_return=True, timeout=timeout)
        if raw:
            break
        if attempt < attempts:
            print(f"  no reply (attempt {attempt}/{attempts}), retrying...",
                  file=sys.stderr)
    if not raw:
        detail = ["No reply to get_absolute_angle:all after "
                  f"{attempts} attempts at {timeout}s."]
        alive = getattr(comm, "reader_alive", lambda: None)()
        detail.append(f"  telemetry reader alive : {alive}")
        lines = getattr(comm, "telemetry_lines", list)()
        detail.append(f"  undelimited lines seen : {len(lines)}")
        for line in lines[-8:]:
            detail.append(f"    | {line}")
        if lines:
            detail.append("  Lines are arriving but none are ';'-terminated -- "
                          "the device is talking, the reply just is not framed.")
        elif alive:
            detail.append("  Nothing at all on the telemetry port. Check the "
                          "port pair, power, and `get_reply_route`.")
        detail.append("  Try a longer --timeout; this read scales with motor "
                      "count.")
        raise RuntimeError("\n".join(detail))
    parsed = exo._parse_motor_data_block(raw)
    out = {}
    silent = []
    for motor_id, data in parsed.items():
        angle = data.get("absolute_angle")
        if angle is None:
            continue
        name = data.get("name", "?")
        if not include_zero and float(angle) == NONRESPONSIVE_ANGLE:
            silent.append((int(motor_id), name))
            continue
        out[int(motor_id)] = (name, float(angle))
    if silent:
        ids = ", ".join(f"{n}(id {i})" for i, n in sorted(silent))
        print(f"  skipping {len(silent)} motor(s) reporting exactly 0.00 -- "
              f"not responding on the bus: {ids}", file=sys.stderr)
        print("  (pass --include-zero to capture them anyway)", file=sys.stderr)
    if not out:
        raise RuntimeError(
            "No responding motors. Every motor reported 0.00, which means none "
            f"answered on the Dynamixel bus.\nReply was:\n{raw}"
        )
    return out


def print_table(angles):
    print(f"\n  {'id':>4}  {'name':<12} {'absolute_angle':>15}")
    print("  " + "-" * 34)
    for motor_id in sorted(angles):
        name, angle = angles[motor_id]
        print(f"  {motor_id:>4}  {name:<12} {angle:>15.2f}")


def load_store(path):
    if not os.path.exists(path):
        return {}
    with open(path) as handle:
        return json.load(handle)


def save_store(path, store):
    with open(path, "w") as handle:
        json.dump(store, handle, indent=2, sort_keys=True)


def emit(store, min_range=DEFAULT_MIN_RANGE_DEG):
    """Print config.h blocks from the saved poses, flagging unusable joints."""
    missing = [p for p in POSE_NAMES if p not in store]
    if missing:
        print(f"[ERROR] Missing capture(s): {', '.join(missing)}. "
              f"Save each with --save <name>.", file=sys.stderr)
        return 1

    ids = sorted({int(i) for pose in POSE_NAMES for i in store[pose]})
    for pose in POSE_NAMES:
        pose_ids = {int(i) for i in store[pose]}
        if pose_ids != set(ids):
            print(f"[WARN] '{pose}' covers {sorted(pose_ids)}, expected {ids}. "
                  f"Captures were taken against different motor sets.",
                  file=sys.stderr)

    def val(pose, motor_id):
        return float(store[pose][str(motor_id)][1])

    names = {i: store["home"][str(i)][0] for i in ids if str(i) in store["home"]}

    print("\n// ---- HOME_STATES (paste into config.h, MOTOR_IDS order) ----")
    print("constexpr float HOME_STATES[] = {")
    print("  " + ", ".join(f"{val('home', i):.2f}" for i in ids))
    print("};")

    print("\n// ---- jointLimits (paste into config.h, MOTOR_IDS order) ----")
    print("constexpr float jointLimits[][2] = {")
    rows = []
    for i in ids:
        lo, hi = sorted((val("min", i), val("max", i)))
        rows.append(f"  {{{lo:.2f}, {hi:.2f}}}")
    print(",\n".join(rows))
    print("};")

    print("\n// ---- sanity check ----")
    print(f"//  {'joint':<22} {'range':>8}  {'home':>8} {'min':>8} {'max':>8}")
    problems = 0
    for i in ids:
        lo, hi = sorted((val("min", i), val("max", i)))
        home = val("home", i)
        span = hi - lo
        flags = []
        if span < min_range:
            flags.append("RANGE_TOO_SMALL")
        if not (lo <= home <= hi):
            flags.append("HOME_OUTSIDE_LIMITS")
        label = f"{names.get(i, '?')} (id {i})"
        if flags:
            problems += 1
            print(f"//  {label:<22} {span:>8.2f}  {home:>8.2f} {lo:>8.2f} "
                  f"{hi:>8.2f}  {' '.join(flags)}")
    if problems:
        print(f"//")
        print(f"//  {problems} joint(s) unusable. What each flag means:")
        print(f"//    RANGE_TOO_SMALL     swept < {min_range:g} deg -- the digit was "
              f"barely moved")
        print(f"//                        between the 'min' and 'max' captures. "
              f"Sweep each")
        print(f"//                        joint through its FULL travel.")
        print(f"//    HOME_OUTSIDE_LIMITS the home pose was not between the two "
              f"extremes.")
        print(f"//                        Capture 'home' somewhere inside the "
              f"swept range.")
        return 2
    print("//  all joints OK: home lies inside limits with usable range")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Read live joint angles and emit config.h values."
    )
    parser.add_argument("--cmd-port", default=CMD_PORT)
    parser.add_argument("--telem-port", default=TELEM_PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    # This reads every motor over the Dynamixel bus before replying, so it
    # needs far more headroom than a single-value query.
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="Seconds to wait for the bulk angle reply "
                             "(default 5.0; scales with motor count)")
    parser.add_argument("--store", default=DEFAULT_STORE,
                        help=f"Capture file (default {DEFAULT_STORE})")
    parser.add_argument("--save", choices=POSE_NAMES, metavar="POSE",
                        help="Save this reading as 'home', 'min', or 'max'. "
                             "min/max are just the two extremes of travel in "
                             "either order -- they are sorted per motor.")
    parser.add_argument("--emit", action="store_true",
                        help="Print config.h blocks from saved captures and exit "
                             "(no device needed).")
    parser.add_argument("--arm", action="store_true",
                        help="Send enable:all before reading. THE HAND WILL "
                             "MOVE: torque-on drives every motor to its last "
                             "goal position, destroying a hand-set pose. Only "
                             "use this to capture a pose the exo is holding.")
    parser.add_argument("--no-disarm", dest="disarm", action="store_false",
                        help="Leave torque as-is on exit instead of sending "
                             "disable:all.")
    parser.add_argument("--include-zero", action="store_true",
                        help="Keep motors reporting exactly 0.00. They are "
                             "dropped by default as not-responding.")
    parser.add_argument("--min-range", type=float,
                        default=DEFAULT_MIN_RANGE_DEG, metavar="DEG",
                        help=f"Flag joints swept less than this many degrees "
                             f"(default {DEFAULT_MIN_RANGE_DEG}).")
    parser.set_defaults(disarm=True)
    args = parser.parse_args(argv)

    if args.emit:
        return emit(load_store(args.store), args.min_range)

    comm = DualSerialComm(
        cmd_port=args.cmd_port, telem_port=args.telem_port, baudrate=args.baud,
        response_timeout=args.timeout, line_terminator=LINE_TERMINATOR,
    )
    print(f"Serial: cmd={args.cmd_port} telem={args.telem_port} @ {args.baud}")
    try:
        comm.connect()
    except Exception as exc:
        print(f"[FATAL] Could not open exo: {exc}", file=sys.stderr)
        return 1

    try:
        # Quiet the firmware so debug lines do not sit in front of the reply.
        comm.send("debug:off" + LINE_TERMINATOR)
        time.sleep(0.15)

        if args.arm:
            print("Arming: enable:all -- THE HAND WILL MOVE to the last "
                  "commanded goal position.")
            comm.send("enable:all" + LINE_TERMINATOR)
            time.sleep(ARM_SETTLE_S)
            comm.flush_input()

        exo = HandExo(comm, command_delimiter=LINE_TERMINATOR)
        angles = read_angles(comm, exo, args.timeout,
                             include_zero=args.include_zero)
        print_table(angles)

        if args.save:
            store = load_store(args.store)
            store[args.save] = {str(i): [n, a] for i, (n, a) in angles.items()}
            save_store(args.store, store)
            have = ", ".join(sorted(p for p in POSE_NAMES if p in store))
            print(f"\nSaved as '{args.save}' in {args.store}  (have: {have})")
            if all(p in store for p in POSE_NAMES):
                print("All three poses captured -- run with --emit.")
        else:
            print("\n(not saved -- pass --save home|min|max to record)")
        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    finally:
        # Always release torque, on every exit path -- including the case where
        # we never armed but a previous session left the motors energized. This
        # also leaves the digits free to be repositioned for the next capture.
        if args.disarm:
            try:
                print("Disarming: disable:all")
                comm.send("disable:all" + LINE_TERMINATOR)
                time.sleep(0.15)
            except Exception as exc:
                print(f"[Warning] Could not disable motors: {exc}",
                      file=sys.stderr)
                print("          Power-cycle if the hand is still holding.",
                      file=sys.stderr)
        comm.close()


if __name__ == "__main__":
    sys.exit(main())
