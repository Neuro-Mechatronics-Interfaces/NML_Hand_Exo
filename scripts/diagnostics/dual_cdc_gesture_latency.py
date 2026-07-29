#!/usr/bin/env python3
"""
Dual USB-CDC gesture round-trip latency test.

Opens a DualSerialComm (commands out on one CDC, replies in on the other) and
measures the time from writing a gesture command to reading its delimited
acknowledgement back off the telemetry port.

Runs two blocks by default:

    10 x  pinch_index:close  -> 250 ms  -> pinch_index:open  -> 250 ms
    10 x  pinch_middle:close -> 250 ms  -> pinch_middle:open -> 250 ms

The 250 ms pauses are excluded from the timings; each sample is one
write->ack round trip only.

REQUIRES the firmware build where set_gesture acknowledges with
"OK: gesture <name>:<state>;".  Against older firmware that command is silent,
every sample times out, and the script reports 0 acks rather than pretending
the timeout value is a latency.

Motors are armed (gesture mode + enable:all) before the run and disabled after
it, on every exit path including Ctrl-C and a mid-run exception.  THE HAND WILL
MOVE.  Keep the mechanism clear.  Pass --no-arm to measure latency without
movement: the firmware still parses and acknowledges each command with torque
off, so the round trip is unaffected.

Usage:
    python scripts/diagnostics/dual_cdc_gesture_latency.py
    python scripts/diagnostics/dual_cdc_gesture_latency.py --no-arm
    python scripts/diagnostics/dual_cdc_gesture_latency.py --raw
    python scripts/diagnostics/dual_cdc_gesture_latency.py --cmd-port COM12 --telem-port COM13
"""

import argparse
import statistics
import sys
import time

from nml_hand_exo import DualSerialComm


DEFAULT_CMD_PORT = "COM10"
DEFAULT_TELEM_PORT = "COM11"
DEFAULT_BAUD = 1000000
DEFAULT_CYCLES = 4
DEFAULT_PAUSE_S = 0.25
DEFAULT_TIMEOUT_S = 0.5
LINE_TERMINATOR = "\r\n"
ACK_MARKER = "OK: gesture"


class Sample:
    """One write->ack round trip."""

    __slots__ = ("command", "elapsed_ms", "acked", "reply")

    def __init__(self, command, elapsed_ms, acked, reply):
        self.command = command
        self.elapsed_ms = elapsed_ms
        self.acked = acked
        self.reply = reply


def transact(comm, command, timeout):
    """Send one command and time the wait for its delimited reply."""
    comm.flush_input()
    start = time.perf_counter()
    comm.send(command + LINE_TERMINATOR)
    reply = comm.receive(wait_until_return=True, timeout=timeout)
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    return Sample(command, elapsed_ms, ACK_MARKER in reply, reply.strip())


def run_block(comm, gesture, cycles, pause_s, timeout):
    """Run `cycles` close/open pairs for one gesture, pausing between each."""
    print(f"\n--- {gesture}: {cycles} close/open cycles "
          f"({pause_s * 1000:.0f} ms pause between commands) ---")
    samples = []
    for i in range(1, cycles + 1):
        sample = transact(comm, f"set_gesture:{gesture}", timeout)
        samples.append(sample)
        flag = "" if sample.acked else "   <-- NO ACK"
        print(f"  cycle {i:2d}  {gesture:<5}  {sample.elapsed_ms:7.2f} ms{flag}")
        time.sleep(pause_s)
        sample = transact(comm, f"set_gesture:grasp:open", timeout)
        samples.append(sample)
        flag = "" if sample.acked else "   <-- NO ACK"
        print(f"  cycle {i:2d} open {sample.elapsed_ms:7.2f} ms{flag}")
        time.sleep(pause_s)
    return samples


def arm_motors(comm):
    """Put the exo in gesture mode and enable torque. THE HAND WILL MOVE.

    `enable` and `set_exo_mode` are silent in firmware -- no delimited reply --
    so these are sent with a settle delay rather than waited on.
    """
    print("Arming: set_exo_mode:gesture_fixed + enable:all -- hand will move.")
    for command in ("set_exo_mode:gesture_fixed", "enable:all"):
        comm.send(command + LINE_TERMINATOR)
        time.sleep(0.15)
    comm.flush_input()


def disarm_motors(comm):
    """Release torque. Best effort: never mask the error that got us here."""
    try:
        print("\nDisarming: disable:all")
        comm.send("disable:all" + LINE_TERMINATOR)
        time.sleep(0.15)
        comm.flush_input()
    except Exception as exc:
        print(f"[Warning] Could not disable motors: {exc}", file=sys.stderr)
        print("          Motors may still be holding torque -- "
              "power-cycle if needed.", file=sys.stderr)


def run_raw(comm, gesture, timeout):
    """Dump what actually comes back, to separate transport bugs from silence.

    Distinguishes the three ways a run can report zero acks: the reader thread
    died (transport bug), the port is silent (routing / wrong port), or lines
    arrive but none are delimited (firmware without the gesture ack).
    """
    print("\n" + "=" * 52)
    print("RAW TELEMETRY PROBE")
    print("=" * 52)
    print(f"  cmd={comm.cmd_port}  telem={comm.telem_port}")
    print(f"  reader thread alive: {comm.reader_alive()}")

    for command in ("version", f"set_gesture:{gesture}:close"):
        comm.flush_input()
        before = len(comm.telemetry_lines())
        comm.send(command + LINE_TERMINATOR)
        reply = comm.receive(wait_until_return=True, timeout=timeout)
        new_lines = comm.telemetry_lines()[before:]

        print(f"\n  >>> {command}")
        print(f"      framed reply : {reply!r}")
        print(f"      undelimited  : {len(new_lines)} line(s)")
        for line in new_lines[:12]:
            print(f"        | {line}")
        if len(new_lines) > 12:
            print(f"        | ... {len(new_lines) - 12} more")

    print(f"\n  reader thread alive: {comm.reader_alive()}")
    print("\n  Reading the result:")
    print("    reply present ................ transport is fine")
    print("    no reply, lines present ...... firmware sent no ';'-terminated")
    print("                                   ack -- reflash for gesture acks")
    print("    no reply, no lines ........... nothing on the telemetry port;")
    print("                                   check get_reply_route / port pair")
    print("    reader not alive ............. transport bug, report the error")


def median_ms(samples):
    """Median latency of the acknowledged samples, or None."""
    acked = [s.elapsed_ms for s in samples if s.acked]
    return statistics.median(acked) if acked else None


def run_diagnose(comm, gesture, cycles, timeout):
    """Decompose round-trip latency into link, logging, and motor-work costs.

    Times a trivial command (`version`: parse plus one short reply, no motor
    traffic) against a gesture command, each with firmware VERBOSE on and off.
    The deltas separate the fixed link cost from the per-motor debug logging
    and the actual Dynamixel work.
    """
    def timed(command, marker, n):
        out = []
        for _ in range(n):
            comm.flush_input()
            start = time.perf_counter()
            comm.send(command + LINE_TERMINATOR)
            reply = comm.receive(wait_until_return=True, timeout=timeout)
            out.append(Sample(command, (time.perf_counter() - start) * 1000.0,
                              marker in reply, reply.strip()))
            time.sleep(0.05)
        return out

    def set_debug(state):
        comm.flush_input()
        comm.send(f"debug:{state}" + LINE_TERMINATOR)
        comm.receive(wait_until_return=True, timeout=timeout)
        time.sleep(0.1)
        comm.flush_input()

    results = {}
    for debug_state in ("on", "off"):
        set_debug(debug_state)
        results[("version", debug_state)] = median_ms(
            timed("version", "Version", cycles)
        )
        results[("gesture", debug_state)] = median_ms(
            timed(f"set_gesture:{gesture}:close", ACK_MARKER, cycles)
        )
    set_debug("on")  # leave the board as we found it

    print("\n" + "=" * 52)
    print("LATENCY DECOMPOSITION (median ms)")
    print("=" * 52)
    print(f"  {'':<22}{'debug:on':>12}{'debug:off':>12}")
    for label, key in (("version (link floor)", "version"),
                       (f"{gesture}:close", "gesture")):
        on, off = results[(key, "on")], results[(key, "off")]
        on_s = f"{on:.1f}" if on is not None else "no ack"
        off_s = f"{off:.1f}" if off is not None else "no ack"
        print(f"  {label:<22}{on_s:>12}{off_s:>12}")

    v_on, v_off = results[("version", "on")], results[("version", "off")]
    g_on, g_off = results[("gesture", "on")], results[("gesture", "off")]
    print("\n  Interpretation:")
    if v_off is not None:
        print(f"    link + parse floor .......... {v_off:6.1f} ms")
    if g_off is not None and v_off is not None:
        print(f"    motor work (gesture) ........ {g_off - v_off:6.1f} ms")
    if v_on is not None and v_off is not None:
        print(f"    VERBOSE cost, ~2 log lines .. {v_on - v_off:6.1f} ms")
    if g_on is not None and g_off is not None:
        print(f"    VERBOSE cost, ~1 line/motor . {g_on - g_off:6.1f} ms"
              "   <-- removed by debug:off")
    if None not in (v_on, v_off, g_on, g_off) and (v_on - v_off) > 0.5:
        scale = (g_on - g_off) / (v_on - v_off)
        print(f"\n    Per-gesture logging is {scale:.1f}x the baseline logging cost.")
        print("    That ratio tracks motor count: the debugPrint at")
        print("    gesture_controller.cpp:86 runs once per motor.")
    return results


def summarize(label, samples):
    """Print latency stats for the acknowledged samples in `samples`."""
    acked = [s.elapsed_ms for s in samples if s.acked]
    missing = len(samples) - len(acked)
    print(f"\n{label}: {len(acked)}/{len(samples)} acked", end="")
    if missing:
        print(f"  ({missing} timed out)")
    else:
        print()
    if not acked:
        print("  no round-trip data -- every command went unanswered")
        return
    acked.sort()
    p95 = acked[min(len(acked) - 1, int(round(0.95 * (len(acked) - 1))))]
    print(f"  min {min(acked):7.2f} ms")
    print(f"  med {statistics.median(acked):7.2f} ms")
    print(f"  avg {statistics.fmean(acked):7.2f} ms")
    print(f"  p95 {p95:7.2f} ms")
    print(f"  max {max(acked):7.2f} ms")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Measure dual-CDC gesture command round-trip latency."
    )
    parser.add_argument("--cmd-port", default=DEFAULT_CMD_PORT,
                        help=f"Command CDC (default {DEFAULT_CMD_PORT})")
    parser.add_argument("--telem-port", default=DEFAULT_TELEM_PORT,
                        help=f"Telemetry/reply CDC (default {DEFAULT_TELEM_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Nominal CDC baud (default {DEFAULT_BAUD})")
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES,
                        help=f"Close/open cycles per gesture (default {DEFAULT_CYCLES})")
    parser.add_argument("--pause", type=float, default=DEFAULT_PAUSE_S,
                        help=f"Pause after each command, seconds (default {DEFAULT_PAUSE_S})")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help=f"Per-command ack timeout, seconds (default {DEFAULT_TIMEOUT_S})")
    parser.add_argument("--gestures", nargs="+", default=["thumb:extend", "thumb:flex", "index:flex", "middle:flex", "ring:flex", "pinky:flex"],
                        help="Gestures to cycle (default: thumb:extend index:flex middle:flex pinky:flex)")
    parser.add_argument("--no-arm", dest="arm", action="store_false",
                        help="Skip motor arming. Commands are still parsed and "
                             "acked, so latency is measured without movement.")
    parser.set_defaults(arm=True)
    parser.add_argument("--raw", action="store_true",
                        help="Dump raw telemetry for one command pair to tell a "
                             "transport bug from a silent or unflashed device.")
    parser.add_argument("--diagnose", action="store_true",
                        help="Break the round trip down into link, logging, and "
                             "motor-work costs instead of running the cycle blocks.")
    args = parser.parse_args(argv)

    comm = DualSerialComm(
        cmd_port=args.cmd_port,
        telem_port=args.telem_port,
        baudrate=args.baud,
        response_timeout=args.timeout,
        line_terminator=LINE_TERMINATOR,
        verbose=True,
    )

    print(f"Connecting: cmd={args.cmd_port} telem={args.telem_port} @ {args.baud}")
    try:
        # connect() probes both ports and swaps them if enumeration put the
        # command CDC on the higher COM number, so the defaults above are a
        # starting guess rather than a hard requirement.
        comm.connect()
    except Exception as exc:
        print(f"[FATAL] {exc}", file=sys.stderr)
        return 1

    armed = False
    try:
        if args.arm:
            arm_motors(comm)
            armed = True

        if args.raw:
            run_raw(comm, args.gestures[0], args.timeout)
            return 0

        if args.diagnose:
            run_diagnose(comm, args.gestures[0], args.cycles, args.timeout)
            return 0

        all_samples = []
        for gesture in args.gestures:
            block = run_block(comm, gesture, args.cycles, args.pause, args.timeout)
            summarize(f"{gesture} summary", block)
            all_samples.extend(block)

        print("\n" + "=" * 52)
        summarize("OVERALL", all_samples)

        if not any(s.acked for s in all_samples):
            print("\nNothing was acknowledged -- no delimited reply arrived.\n"
                  "Run with --raw to tell which of these it is: a dead reader\n"
                  "thread, a silent telemetry port, or firmware without the\n"
                  "set_gesture ack.")
            return 2
        return 0
    finally:
        # Disarm before closing, and on every exit path -- including Ctrl-C and
        # an exception mid-run. Leaving the hand under torque is the one
        # outcome this script must never produce.
        if armed:
            disarm_motors(comm)
        comm.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        # main()'s finally has already disarmed and closed the port by now.
        print("\nInterrupted.", file=sys.stderr)
        sys.exit(130)
