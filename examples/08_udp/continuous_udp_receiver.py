#!/usr/bin/env python3
"""Continuous-vector UDP -> exo receiver.

Binds a UDP socket, decodes the versioned ``nml.continuous.v1`` JSON datagrams
emitted by ``continuous_to_udp_bridge.py`` (see ``CONTINUOUS_UDP_SCHEMA.md``),
scales each channel's ``[-1, 1]`` value to a signed integer actuator target, and
drives every joint from ONE ``set_finger_angles`` batch command per frame. An
``NGA3`` binary ack is returned upstream for every accepted datagram. Runs until
SIGINT (Ctrl-C), then returns the hand to rest, disables the motors, and closes
the port.

Difference from ``udp_gesture_receiver.py``
------------------------------------------
That receiver takes discrete integer commands. This one takes a *continuous*
per-channel vector on the same wire and turns each accepted datagram into a
single batched joint move. The decoder decides how many channels it sends and
names them; the fingers it does not name are HELD at rest here, so a
three-channel decoder still leaves the other joints in a known neutral pose
rather than wherever they were last left.

Upstream ack (schema nml.continuous.v2)
--------------------------------------
Schema v1 datagrams are one-way, but this receiver acks every accepted one with
a compact ``NGA3`` binary frame -- the uint32 sequence being acked plus the
signed int8 value dispatched to each joint -- sent back to the datagram's source
address. This is the v2 addition over the unidirectional v1 input contract; a
sender that does not read its source socket simply ignores it. See
``pack_continuous_ack`` in the SDK and ``CONTINUOUS_UDP_SCHEMA.md``.

The ack fires when the DEVICE's reply for the frame arrives on the telemetry
port, not when the host writes the command -- so it attests the device answered,
not merely that the bytes were sent. DualSerialComm drains those replies on a
background thread, so this costs no command latency: each accepted frame queues
one ack, and each solicited reply retires the oldest queued frame (the same
reply-driven scheme as ``udp_gesture_receiver.py``). If the host briefly outruns
the device's reply rate, the oldest un-acked frame is dropped past a bounded
queue rather than growing without limit.

Value convention
----------------
Each ``values[i]`` is finite and clipped to ``[-1, 1]``: positive is Flexion,
negative is Extension, zero is Rest. The receiver scales it to the signed
integer wire range the firmware's ``set_finger_angles`` expects,
``round(v * 100)`` in ``[-100, 100]``:

    v = -1  ->  -100   (the joint's extend posture)
    v =  0  ->     0   (its rest posture)
    v = +1  ->  +100   (its flex posture)

The firmware anchors that signed value at each joint's calibrated rest posture
and interpolates rest<->flex above 0, rest<->extend below 0, per motor, so the
three anchors reproduce those postures exactly even on a multi-motor gesture.

Channel resolution
------------------
``channel_names`` is matched case-insensitively (with a few aliases) against the
exo's joints. A datagram naming a channel this receiver cannot resolve is
rejected wholesale rather than silently applying the rest of it -- the schema
requires the channel layout to stay fixed once streaming, so a changed or
unknown layout is a fault, not a partial update.

Safety policy (from the schema)
-------------------------------
* Stale or duplicate ``sequence`` numbers are dropped (the newest wins; the
  uint32 counter is treated as wrapping).
* Values are clamped once more here before becoming actuator targets.
* If no valid datagram arrives for the watchdog interval, every joint is driven
  back to rest (0), so a dropped source cannot leave the hand holding a flexed
  pose.

Terminal output
---------------
To keep the console readable at 60 ms/frame, accepted datagrams are logged in
batches: every ``PRINT_EVERY`` accepted, one line reports the mean of that
batch's per-joint values. Rejects, stale drops and watchdog trips still print as
they happen. ``--quiet`` silences the batch lines.

``PRINT_EVERY`` throttles ONLY that averaged summary line. The per-frame device
reply and ack echoes fire once each on every accepted frame, so they would flood
at the frame rate; they are OFF by default and gated behind ``--trace``, which
is for debugging one frame at a time, not for a running stream.

Requires firmware >= 0.6.4 for the signed ``set_finger_angles`` batch command.
There is NO per-joint fallback: the receiver dispatches exactly one command per
frame and acks on its reply, so an older device that ignores the command would
never reply and never ack. Startup aborts if the device reports an older (or no)
version.

Usage:
    python examples/08_udp/continuous_udp_receiver.py
    python examples/08_udp/continuous_udp_receiver.py --port 10003 --cmd-port COM10
    python examples/08_udp/continuous_udp_receiver.py --no-arm       # do not enable motors
    python examples/08_udp/continuous_udp_receiver.py --watchdog-ms 500
    python examples/08_udp/continuous_udp_receiver.py --print-every 5
    python examples/08_udp/continuous_udp_receiver.py --mock         # no exo attached

--mock swaps the serial transport for an in-process fake that answers like the
firmware, so the decode, channel resolution, value mapping, batching and the
reply-driven ack loopback can all be exercised with no hardware present.
"""

import argparse
import collections
import json
import math
import signal
import socket
import sys
import time

from nml_hand_exo import DualSerialComm
from nml_hand_exo.interface._gesture_protocol import (
    SET_FINGER_ANGLES_MAX,
    SET_FINGER_ANGLES_ORDER,
    clamp_finger_value,
    format_set_finger_angles,
    pack_continuous_ack,
)


# ======================================================================
#  CONFIGURATION -- edit here
# ======================================================================

UDP_HOST = "0.0.0.0"
UDP_PORT = 10003

CMD_PORT = "COM10"      # commands out
TELEM_PORT = "COM11"    # replies in
BAUD = 1000000

# The INPUT datagrams are schema v1 (unidirectional). The upstream ack this
# receiver adds is the v2 revision of the contract; see CONTINUOUS_UDP_SCHEMA.md.
SCHEMA = "nml.continuous.v1"
SEQUENCE_MODULUS = 2**32

# Log one averaged line per this many accepted datagrams, so a 60 ms/frame stream
# does not flood the console. Rejects/stale/watchdog still print immediately.
PRINT_EVERY = 100

# Cap on frames dispatched but not yet acked (awaiting their device reply). The
# host can briefly outrun the device's reply rate; past this the oldest un-acked
# frame is dropped (counted) rather than letting the queue grow without bound.
MAX_PENDING_ACKS = 64

# Prefix of the firmware's asynchronous move-outcome report. It arrives AFTER
# the command reply, once the motors have settled, so it is UNSOLICITED and must
# not retire a pending frame. Needs firmware >= 0.5.0; older builds never emit
# it. (This receiver requires >= 0.6.4 anyway.)
GESTURE_RESULT_PREFIX = "GESTURE_RESULT:"

# Joints this receiver can drive, in the fixed wire order the firmware's
# set_finger_angles batch command expects. Imported so this receiver and the
# firmware share one contract.
JOINTS = SET_FINGER_ANGLES_ORDER

# Channel-name aliases -> canonical joint name. Matching is case-insensitive.
# The decoder is free to label channels as it likes; these cover the obvious
# spellings so a stream of ["Thumb", "Index", "Pinky"] resolves without a config
# change. A name that resolves to neither a joint nor an alias rejects the whole
# datagram (the schema forbids an unknown or reordered layout mid-stream).
CHANNEL_ALIASES = {
    "thumb": "thumb",
    "thumbflex": "thumb",
    "index": "index",
    "pointer": "index",
    "middle": "middle",
    "ring": "ring",
    "pinky": "pinky",
    "little": "pinky",
    "wrist": "wrist",
}

# The [-1, 1] -> signed [-100, 100] scaling and the rest-anchored interpolation
# live elsewhere now: the host scales with clamp_finger_value() (round(v*100))
# and the FIRMWARE anchors the signed value at each joint's calibrated rest
# posture. There are therefore no per-joint percentage tables to tune here;
# retune the postures in config.h instead.

# Held fingers -- joints no channel drives -- are commanded to this signed value
# every frame so they sit in a known neutral pose rather than wherever they were
# left. 0 is Rest. Set to None to instead leave unaddressed joints untouched.
HELD_JOINT_VALUE = 0

# ======================================================================

LINE_TERMINATOR = "\r\n"
# recvfrom wakes immediately on a datagram, so this only sets how often we fall
# through to service the watchdog and drain device replies when no traffic is
# arriving. Keep it well below the watchdog interval.
SOCKET_POLL_S = 0.02

# Return every joint to rest if no valid datagram arrives within this window.
DEFAULT_WATCHDOG_MS = 500

# Commands sent once at startup when arming, and the release sent on exit.
ARM_COMMANDS = ("reboot:all", "enable:all", "home:all")
DISARM_COMMANDS = ("disable:all",)

# Sent after arming (--no-home to skip). THE HAND MOVES when this runs.
HOME_COMMANDS = (
    "set_gesture:wrist:50", "set_gesture:thumb:35", "set_gesture:index:35",
    "set_gesture:middle:35", "set_gesture:ring:35", "set_gesture:pinky:35",
)
HOME_SETTLE_S = 5

# Firmware VERBOSE emits a blocking USB-CDC write per debug line, which directly
# delays every command. Turned off at startup unless --debug-on.
QUIET_COMMANDS = ("debug:off",)

DEFAULT_CURRENT_MA = 250
DEFAULT_TOTAL_CURRENT_MA = 800

# Firmware version whose set_finger_angles takes the signed [-100, 100] fields
# this receiver sends. Below this the receiver falls back to per-joint
# set_gesture_angle writes.
FW_SET_FINGER_ANGLES = (0, 6, 4)

# Recovery from the device dropping off the USB bus mid-session.
RECONNECT_ATTEMPTS = 5
RECONNECT_DELAY_S = 1.0

MOCK_LATENCY_MS = 8.0


def value_to_signed(value):
    """Scale a schema value in [-1, 1] to a signed [-100, 100] wire integer.

    round(v * 100), clamped. The schema guarantees the [-1, 1] range, but a
    receiver clamps once more before it becomes an actuator target. The firmware
    anchors this signed value at each joint's calibrated rest posture, so 0 is
    rest, +100 is flex and -100 is extend.
    """
    return clamp_finger_value(float(value) * SET_FINGER_ANGLES_MAX)


def resolve_channels(channel_names):
    """Map a datagram's channel names to canonical joints, in order.

    Returns the list of resolved joint names (one per channel), or None if any
    name does not resolve or two channels resolve to the same joint -- either of
    which makes the datagram unusable.
    """
    resolved = []
    seen = set()
    for name in channel_names:
        joint = CHANNEL_ALIASES.get(str(name).strip().lower())
        if joint is None or joint in seen:
            return None
        seen.add(joint)
        resolved.append(joint)
    return resolved


def decode_continuous_packet(data):
    """Strict decoder for one nml.continuous.v1 datagram.

    Mirrors decode_continuous_packet() in continuous_to_udp_bridge.py so the
    receiver enforces the same contract without importing the sender's CTRL-R
    dependencies. Returns the parsed dict, or raises ValueError.
    """
    try:
        payload = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("continuous UDP datagram is not valid UTF-8 JSON") from exc
    if not isinstance(payload, dict) or payload.get("schema") != SCHEMA:
        raise ValueError(f"continuous UDP schema must equal {SCHEMA!r}")
    required = {"schema", "sequence", "source_time_s", "channel_names", "values"}
    if set(payload) != required:
        raise ValueError(
            "continuous UDP fields must be exactly " + ", ".join(sorted(required))
        )
    sequence = payload["sequence"]
    if isinstance(sequence, bool) or not isinstance(sequence, int):
        raise ValueError("continuous UDP sequence must be an integer")
    if not 0 <= sequence < SEQUENCE_MODULUS:
        raise ValueError("continuous UDP sequence must be uint32")
    if isinstance(payload["source_time_s"], bool):
        raise ValueError("continuous UDP source_time_s must be numeric")
    timestamp = float(payload["source_time_s"])
    if not math.isfinite(timestamp):
        raise ValueError("continuous UDP source_time_s must be finite")
    raw_values = payload["values"]
    raw_names = payload["channel_names"]
    if not isinstance(raw_values, list) or not isinstance(raw_names, list):
        raise ValueError("continuous UDP channel_names and values must be arrays")
    if not raw_values:
        raise ValueError("continuous UDP values must be a non-empty vector")
    if any(
        isinstance(value, bool) or not isinstance(value, (int, float))
        for value in raw_values
    ):
        raise ValueError("continuous UDP values must contain only JSON numbers")
    values = [float(v) for v in raw_values]
    if any(not math.isfinite(v) for v in values):
        raise ValueError("continuous UDP values must be finite")
    if any(v < -1.0 or v > 1.0 for v in values):
        raise ValueError("continuous UDP values must lie in [-1, 1]")
    names = [str(name).strip() for name in raw_names]
    if len(names) != len(values):
        raise ValueError("continuous UDP channel_names and values must match in length")
    if any(not name for name in names):
        raise ValueError("continuous UDP channel names must be non-empty")
    if len({name.casefold() for name in names}) != len(names):
        raise ValueError("continuous UDP channel names must be unique")
    return {
        "schema": SCHEMA,
        "sequence": int(sequence),
        "source_time_s": timestamp,
        "channel_names": names,
        "values": values,
    }


def sequence_is_newer(candidate, last):
    """True if `candidate` is a newer uint32 sequence than `last`.

    Treats the counter as wrapping: a value within half the modulus ahead of the
    last accepted one is newer, which tolerates the 4294967295 -> 0 wrap without
    treating an old straggler after the wrap as fresh.
    """
    if last is None:
        return True
    return 0 < ((candidate - last) % SEQUENCE_MODULUS) < (SEQUENCE_MODULUS // 2)


class MockComm:
    """In-process stand-in for DualSerialComm, for running with no exo attached.

    Implements only the surface the receiver uses. Replies mirror the firmware:
    set_finger_angles and set_gesture_angle get an OK frame; the commands that
    are silent in firmware -- enable, disable -- get nothing.

    ``log`` echoes every command sent, which floods at the frame rate on a
    continuous stream, so it is a per-frame trace and tracks --trace rather than
    --quiet. Real hardware has no such echo, so leaving it off keeps mock output
    faithful to a real run.
    """

    def __init__(self, latency_ms=MOCK_LATENCY_MS, log=False):
        self.cmd_port = "MOCK-CMD"
        self.telem_port = "MOCK-TELEM"
        self.latency_s = max(0.0, latency_ms) / 1000.0
        self.log = log
        self.sent = []
        self._pending = collections.deque()   # (ready_at, frame)
        self._open = False

    def connect(self):
        self._open = True

    def close(self):
        self._open = False
        self._pending.clear()

    def is_connected(self):
        return self._open

    def flush_input(self):
        self._pending.clear()

    def send(self, message):
        if not self._open:
            raise OSError("MockComm is not connected")
        command = message.strip()
        self.sent.append(command)
        if self.log:
            print(f"      [mock] <- {command}")
        reply = self._reply_for(command)
        if reply is not None:
            self._pending.append((time.monotonic() + self.latency_s, reply))

    def receive(self, wait_until_return=False, timeout=None):
        deadline = time.monotonic() + (timeout or 0.0)
        while True:
            if self._pending and self._pending[0][0] <= time.monotonic():
                return self._pending.popleft()[1]
            if not wait_until_return or time.monotonic() >= deadline:
                return ""
            time.sleep(0.002)

    @staticmethod
    def _reply_for(command):
        head, _, rest = command.partition(":")
        if head == "set_finger_angles":
            fields = [f for f in rest.split(":") if f.strip()]
            return f"OK: finger_angles commanded={len(fields)} held=0"
        if head == "set_gesture_angle":
            target, _, percent = rest.partition(":")
            try:
                percent = f"{min(100.0, max(0.0, float(percent))):.1f}"
            except ValueError:
                return f"ERROR: set_gesture_angle percent not numeric: {percent}"
            return f"OK: gesture_angle {target}:{percent}"
        if head == "set_gesture":
            return f"OK: gesture {rest}"
        if head == "set_current_lim":
            target, _, value = rest.partition(":")
            return f"OK: set_current_lim {target} {value}"
        if head == "set_total_current_lim":
            return f"OK: total_current_lim {rest}"
        if head == "home":
            return f"OK: home {rest}"
        if head == "debug":
            return f" Debug state: {'true' if rest == 'on' else 'false'}"
        if head == "version":
            return "Exo Device Version: 0.6.4"
        # enable / disable / reboot / set_exo_mode are silent in firmware.
        return None


def rediscover_cdc_pair(cmd_port, telem_port):
    """Re-locate this device's CDC pair after a USB re-enumeration."""
    from serial.tools import list_ports
    from nml_hand_exo.interface._serial_ports import find_cdc_sibling

    ports = list(list_ports.comports())
    present = {p.device for p in ports}
    for candidate in (cmd_port, telem_port):
        if candidate in present:
            pair = find_cdc_sibling(candidate, ports)
            if pair:
                return pair
    for port in ports:
        pair = find_cdc_sibling(port.device, ports)
        if pair:
            return pair
    return None


class Receiver:
    """Decodes continuous datagrams and drives the hand from batched moves.

    Each accepted datagram dispatches exactly ONE ``set_finger_angles`` command
    and, once the device's reply for it lands on the telemetry port, is acked
    upstream. The ack therefore attests that the DEVICE answered the frame, not
    merely that the host wrote it -- and because DualSerialComm drains replies on
    a background thread, that answer never gates a subsequent command write.
    """

    def __init__(self, comm, verbose=True, watchdog_s=None,
                 print_every=PRINT_EVERY, trace=False):
        self.comm = comm
        self.verbose = verbose
        #: Echo every per-frame device reply and every ack as they happen. One
        #: of each fires on EVERY accepted frame, so this floods at the frame
        #: rate and is independent of print_every (which throttles only the
        #: averaged summary line). Default off; --trace turns it on.
        self.trace = trace
        self.watchdog_s = watchdog_s
        self.print_every = max(1, int(print_every))
        self.sock = None
        self._ack_addr = None            # source of the last datagram, for acks
        self.last_sequence = None
        self.last_packet_monotonic = None
        self.channel_layout = None       # first accepted channel_names, locked
        self._resolved_joints = []       # canonical joints for the locked layout
        #: Last signed value commanded per joint, so the watchdog only re-sends
        #: when the pose needs to change and reconnect can restore it.
        self.commanded = {joint: HELD_JOINT_VALUE for joint in JOINTS}
        self.at_rest = True
        #: FIFO of frames awaiting their device reply, as [seq, values, addr].
        #: One entry per dispatched command; drain_replies() retires the oldest
        #: and acks it when a solicited reply arrives. The reply carries no
        #: sequence, so this is the only thing tying a reply back to its frame.
        self._pending = collections.deque()
        #: Rolling buffer of the last `print_every` accepted per-joint value
        #: dicts, averaged and flushed as one console line.
        self._print_buf = collections.deque(maxlen=self.print_every)
        # counters
        self.received = 0
        self.accepted = 0
        self.rejected = 0
        self.stale = 0
        self.dispatched = 0
        self.acked = 0
        self.dropped_acks = 0
        self.watchdog_trips = 0
        self.link_down = False
        self.reconnects = 0
        self.send_failures = 0
        self.on_reconnect = None

    # -- serial link ---------------------------------------------------

    def send(self, command):
        try:
            self.comm.send(command + LINE_TERMINATOR)
            return True
        except Exception as exc:
            self.send_failures += 1
            if not self.link_down:
                print(f"[ERROR] Serial write failed: {exc}", file=sys.stderr)
            self.link_down = True
            return False

    def reconnect(self, attempts=RECONNECT_ATTEMPTS):
        try:
            self.comm.close()
        except Exception:
            pass
        for attempt in range(1, attempts + 1):
            time.sleep(RECONNECT_DELAY_S)
            pair = rediscover_cdc_pair(self.comm.cmd_port, self.comm.telem_port)
            if pair is None:
                print(f"  [reconnect {attempt}/{attempts}] device not present",
                      file=sys.stderr)
                continue
            self.comm.cmd_port, self.comm.telem_port = pair
            try:
                self.comm.connect()
            except Exception as exc:
                print(f"  [reconnect {attempt}/{attempts}] {exc}", file=sys.stderr)
                continue
            self.link_down = False
            self.reconnects += 1
            print(f"  [reconnect] link restored on cmd={self.comm.cmd_port} "
                  f"telem={self.comm.telem_port}")
            if self.on_reconnect is not None:
                try:
                    self.on_reconnect()
                except Exception as exc:
                    print(f"  [reconnect] setup failed: {exc}", file=sys.stderr)
            return True
        print("  [reconnect] giving up", file=sys.stderr)
        return False

    def require_firmware(self, version_tuple):
        """Fail unless the device has the signed set_finger_angles command.

        There is no fallback: this receiver dispatches exactly one
        set_finger_angles per frame and acks on its reply, so a device that does
        not answer that command would never ack and the pending queue would just
        fill and drop. An unknown command is SILENT in firmware, so gate on the
        reported version rather than a round trip.

        Returns True if the firmware is new enough. A None version means the
        device did not answer the version query at all -- also fatal, since we
        cannot confirm the command exists.
        """
        if version_tuple is None:
            raise RuntimeError(
                "device did not report a firmware version; cannot confirm "
                "set_finger_angles support (needs >= 0.6.4)"
            )
        if version_tuple < FW_SET_FINGER_ANGLES:
            need = ".".join(map(str, FW_SET_FINGER_ANGLES))
            have = ".".join(map(str, version_tuple))
            raise RuntimeError(
                f"firmware {have} lacks signed set_finger_angles (needs "
                f">= {need}); this receiver has no per-joint fallback"
            )
        return True

    # -- downstream (UDP -> serial) ------------------------------------

    def handle(self, data, sender, addr=None):
        self.received += 1
        # Ack (and re-ack) go to the source of the most recent datagram. A
        # continuous source streams from one socket, so this tracks it without a
        # separate return-port registration.
        if addr is not None:
            self._ack_addr = addr
        try:
            packet = decode_continuous_packet(data)
        except ValueError as exc:
            self.rejected += 1
            if self.verbose:
                print(f"  [{sender}] rejected: {exc}")
            return

        # Lock the channel layout to the first accepted datagram. The schema
        # requires it to stay fixed once streaming, so a changed layout is a
        # fault -- reject rather than silently re-map.
        names = packet["channel_names"]
        if self.channel_layout is None:
            joints = resolve_channels(names)
            if joints is None:
                self.rejected += 1
                if self.verbose:
                    print(f"  [{sender}] rejected: unresolved/duplicate channel "
                          f"names {names}")
                return
            self.channel_layout = names
            self._resolved_joints = joints
            print(f"  channel layout locked: "
                  + ", ".join(f"{n}->{j}" for n, j in zip(names, joints)))
        elif names != self.channel_layout:
            self.rejected += 1
            if self.verbose:
                print(f"  [{sender}] rejected: channel layout changed "
                      f"{self.channel_layout} -> {names}")
            return

        # Drop stale/duplicate sequence numbers; the newest wins.
        if not sequence_is_newer(packet["sequence"], self.last_sequence):
            self.stale += 1
            if self.verbose:
                print(f"  [{sender}] stale sequence {packet['sequence']} "
                      f"(last {self.last_sequence})")
            return
        self.last_sequence = packet["sequence"]
        self.last_packet_monotonic = time.monotonic()
        self.accepted += 1

        # Build the target pose as SIGNED integers in [-100, 100]: driven joints
        # take their scaled channel value, the rest are held at HELD_JOINT_VALUE
        # (or left at their last value if None).
        targets = dict(self.commanded)
        for joint, value in zip(self._resolved_joints, packet["values"]):
            targets[joint] = value_to_signed(value)
        if HELD_JOINT_VALUE is not None:
            driven = set(self._resolved_joints)
            for joint in JOINTS:
                if joint not in driven:
                    targets[joint] = HELD_JOINT_VALUE

        # Dispatch the command and QUEUE the ack; it is emitted later, once the
        # device's reply for this frame lands on the telemetry port. Acking here
        # would only attest the host wrote the bytes -- reply-time acking attests
        # the DEVICE answered, at no latency cost because the reply is drained on
        # a background thread and never gates the next write.
        self._apply(targets, packet["sequence"])
        self.at_rest = all(v == 0 for v in targets.values())

        # Console output is batched: buffer this frame's driven values and flush
        # one averaged line every print_every accepted frames.
        self._print_buf.append({j: targets[j] for j in self._resolved_joints})
        if self.verbose and len(self._print_buf) >= self.print_every:
            self._flush_print(sender)

    def _apply(self, targets, sequence=None):
        """Dispatch one set_finger_angles command for `targets` (signed values).

        Exactly one command per call, so exactly one solicited reply comes back
        -- which is what lets drain_replies() ack one frame per reply. When
        `sequence` is given the frame is queued for a reply-time ack; the
        watchdog and shutdown rest pass None because a safety move is not acked.
        """
        command = format_set_finger_angles(targets)
        self.send(command)
        self.commanded = dict(targets)
        self.dispatched += 1

        if sequence is not None and self._ack_addr is not None:
            # Bounded FIFO: if the host has outrun the device's reply rate,
            # drop the oldest un-acked frame rather than grow without bound.
            if len(self._pending) >= MAX_PENDING_ACKS:
                self._pending.popleft()
                self.dropped_acks += 1
            self._pending.append([sequence, dict(targets), self._ack_addr])

    # -- upstream (serial -> UDP ack) ----------------------------------

    def _retire_pending(self):
        """Ack the oldest frame awaiting a reply, now that one has arrived.

        Mirrors udp_gesture_receiver.py: a solicited device reply retires the
        oldest outstanding frame. One command was dispatched per frame, so one
        reply retires one frame -- there is no per-frame reply count to track.
        """
        if not self._pending:
            return
        sequence, targets, addr = self._pending.popleft()
        if self.sock is None:
            return
        try:
            self.sock.sendto(pack_continuous_ack(sequence, JOINTS, targets), addr)
            self.acked += 1
            if self.trace:
                print(f"      -> ack seq {sequence} to {addr[0]}:{addr[1]}")
        except OSError as exc:
            # A source that is not listening (ICMP port-unreachable) must not
            # kill the receiver; it is still driving the hand fine.
            if self.verbose:
                print(f"  [ack] send to {addr} failed: {exc}", file=sys.stderr)

    def _flush_print(self, sender):
        """Print one line with the mean of the buffered driven values."""
        if not self._print_buf:
            return
        n = len(self._print_buf)
        sums = {}
        for frame in self._print_buf:
            for joint, value in frame.items():
                sums[joint] = sums.get(joint, 0.0) + value
        means = " ".join(
            f"{joint}={sums[joint] / n:+.1f}"
            for joint in self._resolved_joints if joint in sums
        )
        print(f"  [{sender}] avg of last {n} (seq ~{self.last_sequence}) "
              f"-> {means}")
        self._print_buf.clear()

    def service_watchdog(self):
        """Return every joint to rest if the source has gone quiet."""
        if self.watchdog_s is None or self.last_packet_monotonic is None:
            return
        if self.at_rest:
            return
        if time.monotonic() - self.last_packet_monotonic < self.watchdog_s:
            return
        self.watchdog_trips += 1
        print(f"  [watchdog] no valid datagram for {self.watchdog_s*1000:.0f} ms; "
              f"returning to rest")
        self._apply({joint: 0 for joint in JOINTS})
        self.at_rest = True

    def rest_all(self):
        """Drive every joint to rest, e.g. on shutdown before disarming."""
        self._apply({joint: 0 for joint in JOINTS})
        self.at_rest = True

    def drain_replies(self):
        """Consume device replies, acking the frame each solicited reply retires.

        A `set_finger_angles` reply (`OK: finger_angles ...`) is SOLICITED: it
        answers the one command a frame dispatched, so it retires that frame and
        triggers its ack. The asynchronous `GESTURE_RESULT:` move-outcome line is
        UNSOLICITED -- it arrives later, after the motors settle -- so it is
        logged but never retires a pending frame. This is the same split
        udp_gesture_receiver.py makes.
        """
        if self.link_down:
            return
        while True:
            reply = self.comm.receive()          # non-blocking
            if not reply:
                return
            reply_lines = [line.strip() for line in reply.splitlines()
                           if line.strip()]
            solicited = False
            for line in reply_lines:
                if self.trace:
                    print(f"      <- {line}")
                if not line.startswith(GESTURE_RESULT_PREFIX):
                    solicited = True
            # One frame dispatched one command, so one solicited frame retires
            # one pending frame even if the reply spans several lines.
            if solicited:
                self._retire_pending()

    def print_summary(self):
        print(f"\n  datagrams received : {self.received}")
        print(f"  accepted           : {self.accepted}")
        print(f"  commands dispatched: {self.dispatched}")
        print(f"  acks sent upstream : {self.acked} (on device reply)")
        if self.dropped_acks:
            print(f"  acks dropped       : {self.dropped_acks} "
                  f"(host outran device replies)")
        print(f"  rejected           : {self.rejected}")
        print(f"  stale sequences    : {self.stale}")
        if self.watchdog_trips:
            print(f"  watchdog trips     : {self.watchdog_trips}")
        if self.send_failures or self.reconnects:
            print(f"  serial write fails : {self.send_failures}")
            print(f"  link reconnects    : {self.reconnects}")


def read_firmware_version(comm):
    """Ask the device its version and parse it to a tuple, or None."""
    try:
        comm.flush_input()
    except Exception:
        pass
    try:
        comm.send("version" + LINE_TERMINATOR)
    except Exception:
        return None
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        reply = comm.receive(wait_until_return=True, timeout=1.0)
        if not reply:
            break
        for line in reply.splitlines():
            _, sep, tail = line.partition("Version:")
            if not sep:
                continue
            token = tail.strip().split()[0] if tail.strip() else ""
            parts = token.split(".")
            try:
                return tuple(int(p) for p in parts[:3])
            except ValueError:
                return None
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Drive the exo from continuous nml.continuous.v1 UDP vectors "
                    "until Ctrl-C."
    )
    parser.add_argument("--host", default=UDP_HOST,
                        help=f"Bind address (default {UDP_HOST})")
    parser.add_argument("--port", type=int, default=UDP_PORT,
                        help=f"Bind port (default {UDP_PORT})")
    parser.add_argument("--cmd-port", default=CMD_PORT,
                        help=f"Command CDC (default {CMD_PORT})")
    parser.add_argument("--telem-port", default=TELEM_PORT,
                        help=f"Telemetry CDC (default {TELEM_PORT})")
    parser.add_argument("--baud", type=int, default=BAUD,
                        help=f"Nominal CDC baud (default {BAUD})")
    parser.add_argument("--no-arm", dest="arm", action="store_false",
                        help="Do not enable motors on start. Nothing will move.")
    parser.add_argument("--no-home", dest="home", action="store_false",
                        help="Skip the homing sent after arming.")
    parser.add_argument("--watchdog-ms", type=float, default=DEFAULT_WATCHDOG_MS,
                        metavar="MS",
                        help=f"Return every joint to rest if no valid datagram "
                             f"arrives within this window (default "
                             f"{DEFAULT_WATCHDOG_MS:g}; 0 disables).")
    parser.add_argument("--print-every", type=int, default=PRINT_EVERY,
                        metavar="N",
                        help=f"Log one averaged line per N accepted datagrams "
                             f"(default {PRINT_EVERY}). The line reports the mean "
                             f"of that batch's per-joint values. Rejects, stale "
                             f"drops and watchdog trips still print immediately.")
    parser.add_argument("--current-ma", type=int, default=DEFAULT_CURRENT_MA,
                        metavar="MA",
                        help=f"Per-motor working current in mA "
                             f"(default {DEFAULT_CURRENT_MA}). 0 leaves firmware "
                             f"defaults alone.")
    parser.add_argument("--total-current-ma", type=int,
                        default=DEFAULT_TOTAL_CURRENT_MA, metavar="MA",
                        help=f"Combined current budget across all motors in mA "
                             f"(default {DEFAULT_TOTAL_CURRENT_MA}). Needs "
                             f"firmware >= 0.4.0. 0 leaves the default alone.")
    parser.add_argument("--debug-on", action="store_true",
                        help="Leave firmware VERBOSE enabled (slower commands).")
    parser.add_argument("--quiet", action="store_true",
                        help="Do not log the batched summary lines.")
    parser.add_argument("--trace", action="store_true",
                        help="Echo every per-frame device reply and ack. This "
                             "prints one of each PER accepted frame, so it "
                             "floods at the frame rate independently of "
                             "--print-every; use it only for debugging.")
    parser.add_argument("--mock", action="store_true",
                        help="Run with no exo attached. Uses an in-process fake "
                             "device that replies like the firmware.")
    parser.add_argument("--mock-latency-ms", type=float,
                        default=MOCK_LATENCY_MS, metavar="MS",
                        help=f"Simulated device turnaround for --mock "
                             f"(default {MOCK_LATENCY_MS:g}).")
    parser.set_defaults(arm=True, home=True)
    args = parser.parse_args(argv)

    watchdog_s = None if args.watchdog_ms <= 0 else args.watchdog_ms / 1000.0

    if args.mock:
        comm = MockComm(latency_ms=args.mock_latency_ms, log=args.trace)
    else:
        comm = DualSerialComm(
            cmd_port=args.cmd_port, telem_port=args.telem_port,
            baudrate=args.baud, response_timeout=0.5,
            line_terminator=LINE_TERMINATOR,
        )

    if args.mock:
        print(f"MOCK MODE -- no exo attached. Nothing will physically move. "
              f"({args.mock_latency_ms:g} ms simulated turnaround)")
    else:
        print(f"Serial: cmd={args.cmd_port} telem={args.telem_port} @ {args.baud}")
    try:
        comm.connect()
    except Exception as exc:
        print(f"[FATAL] Could not open exo: {exc}", file=sys.stderr)
        return 1
    print(f"        connected as cmd={comm.cmd_port} telem={comm.telem_port}")

    receiver = Receiver(comm, verbose=not args.quiet,
                        watchdog_s=watchdog_s, print_every=args.print_every,
                        trace=args.trace)

    running = {"go": True}

    def on_sigint(_signum, _frame):
        running["go"] = False
        print("\nSIGINT received, shutting down...")

    try:
        signal.signal(signal.SIGINT, on_sigint)
    except ValueError:
        pass

    sock = None
    armed = False

    def apply_session_setup():
        """Quiet the firmware, choose the command path, and restore arm state."""
        if not args.debug_on:
            for command in QUIET_COMMANDS:
                receiver.send(command)
                time.sleep(0.5)
            comm.flush_input()
        # Hard version gate: signed set_finger_angles (>= 0.6.4) is required --
        # there is no per-joint fallback, and a device that ignores the command
        # would never reply and so never ack. An unknown command is silent, so
        # gate on the reported version rather than a round trip. This raises on
        # an old or unversioned device, which aborts startup in main().
        receiver.require_firmware(read_firmware_version(comm))
        comm.flush_input()
        if args.total_current_ma > 0:
            receiver.send(f"set_total_current_lim:{args.total_current_ma}")
            time.sleep(0.5)
            comm.flush_input()
        if args.current_ma > 0:
            receiver.send(f"set_current_lim:all:{args.current_ma}")
            time.sleep(0.5)
            comm.flush_input()
        if args.arm:
            for command in ARM_COMMANDS:
                receiver.send(command)
                time.sleep(0.5)
            comm.flush_input()
            if args.home:
                for command in HOME_COMMANDS:
                    receiver.send(command)
                    time.sleep(0.5)
                time.sleep(0.05 if args.mock else HOME_SETTLE_S)
                comm.flush_input()

    receiver.on_reconnect = apply_session_setup

    try:
        if args.current_ma > 0:
            print(f"Current: {args.current_ma} mA per motor")
        if args.total_current_ma > 0:
            print(f"         {args.total_current_ma} mA combined budget "
                  f"(needs firmware >= 0.4.0)")
        if args.arm:
            steps = list(ARM_COMMANDS) + (list(HOME_COMMANDS) if args.home else [])
            tail = " -- simulated." if args.mock else " -- hand will move."
            print("Arming: " + ", ".join(steps) + tail)
        apply_session_setup()
        armed = args.arm

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(SOCKET_POLL_S)
        sock.bind((args.host, args.port))
        receiver.sock = sock

        print(f"Listening on {args.host}:{args.port} for {SCHEMA} datagrams  "
              f"-- Ctrl-C to stop")
        if watchdog_s is not None:
            print(f"Watchdog: rest after {args.watchdog_ms:g} ms of silence")
        print()

        while running["go"]:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                receiver.service_watchdog()
                receiver.drain_replies()
                continue
            except OSError:
                break
            if data:
                receiver.handle(data, f"{addr[0]}:{addr[1]}", addr)
            receiver.service_watchdog()
            receiver.drain_replies()

            if receiver.link_down:
                print("[WARN] Serial link lost; attempting to reconnect...",
                      file=sys.stderr)
                if not receiver.reconnect():
                    print("[FATAL] Could not restore the serial link.",
                          file=sys.stderr)
                    break

        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        if armed:
            receiver.on_reconnect = None
            try:
                # Rest before disarming so the hand relaxes to neutral rather
                # than dropping torque wherever it happened to be holding.
                if not receiver.link_down:
                    receiver.rest_all()
                    time.sleep(0.2)
                print("Disarming: " + ", ".join(DISARM_COMMANDS))
                ok = False
                for _ in range(2):
                    if receiver.link_down:
                        print("Serial link is down; reconnecting to disarm...",
                              file=sys.stderr)
                        if not receiver.reconnect(attempts=3):
                            break
                    ok = True
                    for command in DISARM_COMMANDS:
                        ok = receiver.send(command) and ok
                        time.sleep(0.15)
                    if ok:
                        break
                if not ok:
                    raise OSError("disarm write did not reach the device")
            except Exception as exc:
                print(f"[Warning] Could not disable motors: {exc}",
                      file=sys.stderr)
                print("          Power-cycle if the hand is still holding.",
                      file=sys.stderr)
        # Flush any buffered frames that did not reach a full print batch.
        if not args.quiet and receiver._print_buf:
            receiver._flush_print("shutdown")
        receiver.print_summary()
        comm.close()


if __name__ == "__main__":
    sys.exit(main())
