"""Automatic impedance range-of-motion (ROM) calibration, by gesture.

Calibrates the six functional gesture axes of the NML Hand Exoskeleton --
**wrist, thumb, index, middle, ring, pinky** -- the same axes the
`set_finger_angles` batch positioner drives. For each gesture the *firmware*
decomposes the axis into the motors it actually moves (``thumb`` spans
thumbadd/thumbrot/thumbflex, ``wrist`` spans wrist/wrist2, the fingers are one
motor each) and sweeps every one of them under current control: it ramps
current slowly, watches the joint angle, and marks the angle at which the joint
just resists a gentle push as that direction's endstop. Every joint is returned
to where it started after its sweep.

This is the host-side driver for the firmware `calibrate_rom_gesture` endpoint
(firmware **0.8.0**). Flash 0.8.0+ first (see
``src/cpp/nml_hand_exo/README.md``); the script refuses to run against older
firmware.

    Board flash (default variant is dual-hand, dual-CDC, no Axon):
        arduino-cli compile --upload -p %COM% \
            --fqbn OpenRB-150:samd:OpenRB-150 src/cpp/nml_hand_exo

No extra build flags are needed for autocal -- `calibrate_rom_gesture` is
compiled into every variant. Only the runtime preconditions matter: 0.8.0+
firmware, and the device in `current` mode (this script sets that for you). In a
dual-hand build a gesture that exists on both sides sweeps both.

--------------------------------------------------------------------------------
Usage
--------------------------------------------------------------------------------
The default OpenRB-150 firmware exposes TWO USB-CDC COM ports on one cable
(dual-CDC): commands on one, replies/telemetry on the other. Give this script
either port; it finds and pairs the sibling automatically.

    # Dual-CDC (default firmware): one port is enough, the sibling is auto-found
    python auto_cal_rom.py --port COM21

    # If auto-pairing fails, name both explicitly
    python auto_cal_rom.py --cmd-port COM21 --telem-port COM22

    # Single-CDC firmware (built with -DSINGLE_CDC): one port does both
    python auto_cal_rom.py --port COM21 --single-cdc

    # Calibrate only some gestures, one direction
    python auto_cal_rom.py --port COM21 --gestures index middle --directions flex

    # Print discovered endstops AND write them into the device's joint limits
    python auto_cal_rom.py --port COM21 --apply

Nothing is written to the device's stored limits unless you pass --apply.
"""

from __future__ import annotations

import argparse
import sys
import time

from nml_hand_exo.interface import HandExo, SerialComm, DualSerialComm
from nml_hand_exo.interface._serial_ports import find_cdc_sibling

# Firmware >= 0.8.0 is required: earlier builds have no calibrate_rom_gesture.
REQUIRED_FW = (0, 8, 0)
# USB CDC baud is nominal but must match the firmware's DEBUG_BAUD_RATE.
DEFAULT_BAUD = 1_000_000
# Per-motor wait: the firmware bounds each motor's sweep with its own
# ROM_CAL_TIMEOUT_MS watchdog (15 s default), so allow headroom over that.
RESULT_TIMEOUT_S = 30.0

# The six functional gesture axes, in the order set_finger_angles lists them
# (wrist last, as its optional field). These are gesture NAMES resolved by the
# firmware into their motors -- not motor names.
DEFAULT_GESTURES = ["thumb", "index", "middle", "ring", "pinky", "wrist"]


def build_comm(args):
    """Open the right transport for the firmware's USB layout."""
    if args.single_cdc:
        port = args.port or args.cmd_port
        if not port:
            raise SystemExit("--single-cdc needs --port")
        print(f"Single-CDC on {port} @ {args.baud}")
        return SerialComm(port, baudrate=args.baud, response_timeout=RESULT_TIMEOUT_S)

    cmd_port, telem_port = args.cmd_port, args.telem_port
    if not (cmd_port and telem_port):
        # One port given: pair its CDC sibling. DualSerialComm re-probes and
        # corrects the command/telemetry direction at connect time, so the
        # order returned here is only a hint.
        seed = args.port or cmd_port or telem_port
        if not seed:
            raise SystemExit("give --port (dual-CDC) or both --cmd-port/--telem-port")
        sibling = find_cdc_sibling(seed)
        if sibling is None:
            raise SystemExit(
                f"Could not find the second CDC port paired with {seed}. "
                "Name both with --cmd-port/--telem-port, or use --single-cdc."
            )
        cmd_port, telem_port = sibling
    print(f"Dual-CDC cmd={cmd_port} telem={telem_port} @ {args.baud}")
    return DualSerialComm(
        cmd_port=cmd_port, telem_port=telem_port, baudrate=args.baud,
        response_timeout=RESULT_TIMEOUT_S,
    )


def sweep_gesture(exo: HandExo, gesture: str, direction: str) -> list[dict]:
    """Sweep every motor a gesture drives, in one direction.

    Returns one parsed result dict per motor (the firmware decides how many).
    """
    print(f"  {gesture:<7} {direction:<6} ... ", end="", flush=True)
    try:
        results = exo.calibrate_rom_gesture(
            gesture, direction, wait=True, per_motor_timeout=RESULT_TIMEOUT_S
        )
    except TimeoutError:
        print("no ROM_CAL_RESULT (timeout waiting on device)")
        return [{"gesture": gesture, "id": None, "dir": direction,
                 "status": "no_result", "endstop": None, "home": None,
                 "current_mA": None}]
    for r in results:
        r["gesture"] = gesture  # tag each motor result with its gesture axis
    ok = sum(1 for r in results if r.get("status") == "ok")
    print(f"{len(results)} motor(s), {ok} endstop(s) found")
    return results


def apply_limits(exo: HandExo, results: list[dict]) -> None:
    """Write discovered flex/extend endstops into each joint's stored limits.

    Only joints with BOTH a flex and an extend endstop from an `ok` sweep are
    written; the firmware stores limits as [min, max] in absolute degrees, so
    the two endstops are sorted into that order regardless of flip direction.
    """
    by_joint: dict[int, dict[str, float]] = {}
    for r in results:
        if r.get("status") == "ok" and r.get("endstop") is not None \
                and r.get("id") is not None:
            by_joint.setdefault(r["id"], {})[r["dir"]] = r["endstop"]

    wrote = 0
    for motor_id, ends in sorted(by_joint.items()):
        if "flex" not in ends or "extend" not in ends:
            print(f"  motor {motor_id}: skipped (need both flex and extend endstops)")
            continue
        lo, hi = sorted((ends["flex"], ends["extend"]))
        exo.set_motor_limits(motor_id, lo, hi)
        print(f"  motor {motor_id}: set_motor_limits {lo:.2f} .. {hi:.2f}")
        wrote += 1
    print(f"Applied limits to {wrote} joint(s). "
          "These are runtime-only; persist them with your calibration profile "
          "if you want them to survive a power cycle.")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="One CDC port; the sibling is auto-detected "
                                  "(dual-CDC), or the only port (--single-cdc).")
    p.add_argument("--cmd-port", help="Explicit command CDC port (dual-CDC).")
    p.add_argument("--telem-port", help="Explicit telemetry CDC port (dual-CDC).")
    p.add_argument("--single-cdc", action="store_true",
                   help="Firmware built with -DSINGLE_CDC: one port does both.")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                   help=f"USB CDC baud (default {DEFAULT_BAUD}).")
    p.add_argument("--gestures", nargs="+", default=DEFAULT_GESTURES,
                   metavar="NAME",
                   help="Gesture axes to calibrate (default: "
                        f"{' '.join(DEFAULT_GESTURES)}).")
    p.add_argument("--directions", nargs="+", default=["flex", "extend"],
                   choices=["flex", "extend"],
                   help="Directions to sweep per gesture (default: both).")
    p.add_argument("--apply", action="store_true",
                   help="Write discovered endstops into the device joint limits "
                        "(default: report only).")
    p.add_argument("--yes", action="store_true",
                   help="Skip the safety confirmation prompt.")
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args(argv)

    if not args.yes:
        print("SAFETY: this drives current into each joint on the hand to find "
              "its endstops.\n        Keep the emergency stop within reach. The "
              "firmware caps current\n        (ROM_CAL_MAX_CURRENT_MA in "
              "config.h) and returns each joint home.")
        if input("Proceed? [y/N] ").strip().lower() not in ("y", "yes"):
            print("Aborted.")
            return 1

    comm = build_comm(args)
    exo = HandExo(comm, verbose=args.verbose)
    try:
        comm.connect()
    except Exception as exc:
        print(f"[FATAL] Could not open the device: {exc}", file=sys.stderr)
        return 1

    try:
        fw = exo.firmware_version(refresh=True)
        fw_str = ".".join(str(x) for x in fw) if fw else "unknown"
        print(f"Firmware: {fw_str}")
        if not exo.firmware_at_least(REQUIRED_FW):
            need = ".".join(str(x) for x in REQUIRED_FW)
            print(f"[FATAL] Autocal needs firmware >= {need}; device reports "
                  f"{fw_str}. Reflash src/cpp/nml_hand_exo.", file=sys.stderr)
            return 1

        print(f"Calibrating gestures {args.gestures}, "
              f"directions {args.directions}\n")

        # calibrate_rom_gesture requires the global mode to already be CURRENT. A
        # mode change turns torque off on every motor, so it is set once here.
        exo.set_control_mode("current")

        results: list[dict] = []
        for gesture in args.gestures:
            for direction in args.directions:
                results.extend(sweep_gesture(exo, gesture, direction))
                time.sleep(0.2)  # let the last return-home hold fully release

        print("\nSummary:")
        print(f"  {'gesture':<8} {'id':>4} {'dir':<7} {'endstop':>9} "
              f"{'home':>9} {'mA':>5}  status")
        for r in results:
            mid = r["id"] if r.get("id") is not None else "-"
            es = f"{r['endstop']:.2f}" if r.get("endstop") is not None else "-"
            hm = f"{r['home']:.2f}" if r.get("home") is not None else "-"
            ma = f"{r['current_mA']:.0f}" if r.get("current_mA") is not None else "-"
            print(f"  {r.get('gesture',''):<8} {mid:>4} {r['dir']:<7} {es:>9} "
                  f"{hm:>9} {ma:>5}  {r.get('status')}")

        if args.apply:
            print("\nApplying discovered endstops to device joint limits:")
            apply_limits(exo, results)
        else:
            print("\n(report only; re-run with --apply to write these into the "
                  "device joint limits)")
    finally:
        # Leave motors de-energized no matter how the run ended.
        try:
            exo.cancel_rom()
        except Exception:
            pass
        try:
            exo.send_command("disable:all")
        except Exception:
            pass
        exo.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
