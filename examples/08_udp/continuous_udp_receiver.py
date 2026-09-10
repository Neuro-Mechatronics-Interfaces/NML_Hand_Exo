#!/usr/bin/env python3
"""Continuous-vector UDP -> exo receiver.

Binds a UDP socket, decodes the versioned ``nml.continuous.v1`` JSON datagrams
emitted by ``continuous_to_udp_bridge.py`` (see ``CONTINUOUS_UDP_SCHEMA.md``),
and drives the hand's POSITION from a per-channel velocity integrator rather than
relaying each raw sample. Each channel's ``[-1, 1]`` value is a signed DRIVE that
integrates a persistent float pose toward the endpoints; commands to the exo are
THROTTLED so a holding or jittering hand does not saturate the serial link. An
``NGA3`` binary ack is returned upstream for every accepted frame (flow control
for an ack-gated sender), separately from the throttled commands. Runs until
SIGINT (Ctrl-C), then returns the hand to rest, disables the motors, and closes
the port.

Adaptive integration + throttling
---------------------------------
These are POSITION commands, so rather than sending ``round(value * 100)``
straight through every 60 ms -- which relays the decoder's per-sample noise to
the actuators -- each channel's value is treated as a signed VELOCITY. It
integrates a float position in ``[-1, 1]`` toward +1 (flex) or -1 (extend):
``position += value * dt / RAMP_TIME_S``, clamped. A steady +1 stream walks that
joint from rest to full flex over ``RAMP_TIME_S``; a value near 0 freezes it.

The integrated position, not the raw sample, is what gets scaled and sent -- and
it is sent only when it has EARNED a command. A command goes out when all of:
the rate cap (``MIN_COMMAND_INTERVAL_S``) has elapsed; the pose has moved past
``POSITION_DEADBAND`` versus the last sent command; and the recent drive on some
moved joint has been CONSISTENTLY directional (``CONSISTENCY_MIN``) -- a smooth,
sustained push toward one endpoint rather than sign-flipping jitter. A separate
settle-release lands the final pose once after the drive goes idle, so a joint is
not left one deadband short of where a consistent push was taking it. Frames that
arrive between two dispatches are integrated but not each sent (counted as
"coalesced").

Difference from ``udp_gesture_receiver.py``
------------------------------------------
That receiver takes discrete integer commands. This one takes a *continuous*
per-channel vector on the same wire, integrates it into a persistent pose, and
sends a throttled batched joint move when that pose has earned one -- not one
move per datagram. The decoder decides how many channels it sends and names them;
the fingers it does not name are HELD (their integrated position is left
unchanged) rather than driven, and the watchdog is what restores rest.

Upstream ack (schema nml.continuous.v2) -- and why it acks EVERY accepted frame
------------------------------------------------------------------------------
Schema v1 datagrams are one-way, but this receiver acks accepted ones with a
compact ``NGA3`` binary frame -- the uint32 sequence plus the signed int8 pose
per joint -- to the datagram's source address. See ``pack_continuous_ack`` in the
SDK and ``CONTINUOUS_UDP_SCHEMA.md``.

The ack is FLOW CONTROL. An ack-gated sender (the credit-based scheme the schema
suggests) sends its NEXT frame only once the previous frame's ack returns, so
**every accepted frame must be acked or the stream stalls** -- the receiver goes
quiet, the sender waits forever, and the hand never moves. Throttling the exo
commands must therefore NOT throttle the acks. So by default (``ack_on_accept``)
the receiver acks each accepted frame IMMEDIATELY, reporting the current
integrated pose, decoupled from the sparse device commands. The ack rate tracks
the datagram rate; the command rate is capped separately.

``--ack-on-reply`` selects the alternative: ack only the frames DISPATCHED to the
device, on the device's serial reply, so the ack attests the exo answered.
Because throttling coalesces many frames into one command, most frames then go
un-acked (a gap in the acked sequence -- the schema's "coalesced" signal). That is
correct only for a FREE-RUNNING sender; it STALLS an ack-gated one, which is why
it is not the default.

Value convention
----------------
Each ``values[i]`` is finite and clipped to ``[-1, 1]``, and is a signed
VELOCITY (drive), not a direct target: positive drives toward Flexion, negative
toward Extension, zero holds. It integrates a per-channel position in ``[-1, 1]``
that IS the target; that position is scaled to the signed wire range the
firmware's ``set_finger_angles`` expects, ``round(pos * 100)`` in ``[-100, 100]``:

    pos = -1  ->  -100   (the joint's extend posture)
    pos =  0  ->     0   (its rest posture)
    pos = +1  ->  +100   (its flex posture)

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
* The integrated position is clamped to ``[-1, 1]`` and re-clamped to the signed
  wire range before becoming an actuator target.
* If no valid datagram arrives for the watchdog interval, the integrator is reset
  and every joint is driven back to rest (0), so a dropped source cannot leave
  the hand holding a flexed pose. The watchdog rest bypasses the throttle.

Terminal output
---------------
To keep the console readable at 60 ms/frame, accepted datagrams are logged in
batches: every ``PRINT_EVERY`` accepted, one line reports the mean of that
batch's INTEGRATED pose (scaled to the signed wire value) and the running
command count. Rejects, stale drops and watchdog trips still print as they
happen. ``--quiet`` silences the batch lines.

``PRINT_EVERY`` throttles ONLY that averaged summary line. The per-frame device
reply and ack echoes fire once each on every accepted frame, so they would flood
at the frame rate; they are OFF by default and gated behind ``--trace``, which
is for debugging one frame at a time, not for a running stream.

Requires firmware >= 0.6.4 for the signed ``set_finger_angles`` batch command.
There is NO per-joint fallback: the receiver dispatches exactly one command per
DISPATCH and acks on its reply, so an older device that ignores the command would
never reply and never ack. Startup aborts if the device reports an older (or no)
version.

Usage:
    python examples/08_udp/continuous_udp_receiver.py
    python examples/08_udp/continuous_udp_receiver.py --port 10003 --cmd-port COM10
    python examples/08_udp/continuous_udp_receiver.py --no-arm       # do not enable motors
    python examples/08_udp/continuous_udp_receiver.py --watchdog-ms 500
    python examples/08_udp/continuous_udp_receiver.py --print-every 5
    python examples/08_udp/continuous_udp_receiver.py --ramp-time 0.8 --min-interval-ms 40
    python examples/08_udp/continuous_udp_receiver.py --deadband 0.05 --consistency-min 0.6
    python examples/08_udp/continuous_udp_receiver.py --mock         # no exo attached

--mock swaps the serial transport for an in-process fake that answers like the
firmware, so the decode, channel resolution, value mapping, the integrate/throttle
loop and the reply-driven ack loopback can all be exercised with no hardware
present.
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
PRINT_EVERY = 1

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

# The five finger joints, in wire order, driven together in --grasp mode from a
# single scalar (the first received value). Excludes the wrist, which grasp pins
# at rest. Derived from JOINTS so it stays in sync with the shared contract.
GRASP_FINGERS = tuple(j for j in JOINTS if j != "wrist")

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

# ----------------------------------------------------------------------
#  Adaptive integration + command throttling
# ----------------------------------------------------------------------
# These commands DRIVE POSITION. Rather than sending round(value*100) straight
# through every 60 ms frame -- which relays the decoder's per-sample noise to the
# actuators -- each channel's [-1, 1] value is treated as a signed VELOCITY that
# integrates a per-channel float position toward +1 (flex) or -1 (extend). A
# steady +1 stream walks that joint from rest to full flex over RAMP_TIME_S; a
# value near 0 freezes it. The integrated position, not the raw sample, is what
# gets scaled to the signed wire value and dispatched.
#
# On top of that, commands to the exo are THROTTLED so a holding or jittering
# hand does not saturate the serial link with near-identical "noise" frames. A
# command is sent only when all of: the rate cap has elapsed, the integrated pose
# has moved past a deadband since the last SENT command, and the recent drive has
# been CONSISTENTLY directional (a smooth push toward one endpoint, not sign-
# flipping jitter). A separate settle-release commit lands the final pose once
# after the drive stops, so a joint is not left one deadband short of its target.

# Seconds of sustained full-scale (|value| == 1) drive to travel a channel's full
# rest->endpoint span. Larger = more deliberate/slower; the integrator ramps at
# value*(dt/RAMP_TIME_S) per frame. At 1.2 s a steady +1 reaches full flex in
# ~1.2 s; a steady +0.5 takes ~2.4 s.
RAMP_TIME_S = 0.030

# Upper bound on the per-frame integration dt. Frames drained back-to-back from
# the socket buffer carry dt~0 (no travel, correct); a scheduling stall or a gap
# between bursts is capped here so a single long gap cannot jump the pose. Should
# be a few normal frame periods; well below the watchdog, which handles true
# silence separately.
MAX_INTEGRATION_DT = 0.030

# Minimum wall-clock spacing between commands actually sent to the exo. Caps the
# command rate regardless of the ~16.7 Hz datagram rate; frames arriving inside
# the window integrate but do not each dispatch.
MIN_COMMAND_INTERVAL_S = 0.01

# The integrated position (in [-1, 1]) must move at least this much on some joint,
# versus the last SENT command, before a new command is worth sending. Kills the
# stream of near-identical holds. 0.03 ~= 3 % of full travel ~= 3 signed wire
# counts.
POSITION_DEADBAND = 0.002

# Directional-consistency gate. Per channel we track an EMA of the drive's sign
# (CONSISTENCY_BETA sets its memory). A motion is only committed when some joint
# that moved past the deadband has |consistency| >= CONSISTENCY_MIN -- i.e. the
# recent drive pushed that joint the SAME way for several frames, rather than
# oscillating. This is what makes the receiver send on smooth, consistent motion
# toward an endpoint and stay quiet on noise.
CONSISTENCY_BETA = 0.35
CONSISTENCY_MIN = 0.5

# Drives with |value| below this count as "no push" for the consistency EMA and
# the integrator, so decoder noise around rest neither ramps the position nor
# builds false directional consistency.
DRIVE_EPS = 0.02

# After the drive falls idle (all channels below DRIVE_EPS) the integrator stops
# moving; if the settled pose still differs from the last sent command by the
# deadband, ONE release command lands it (subject to the rate cap) so the joint
# is not left short of where the last consistent push was taking it.

# ======================================================================

LINE_TERMINATOR = "\r\n"
# recvfrom wakes immediately on a datagram, so this only sets how often we fall
# through to service the watchdog and drain device replies when no traffic is
# arriving. Keep it well below the watchdog interval.
SOCKET_POLL_S = 0.02

# Return every joint to rest if no valid datagram arrives within this window.
# This is a SAFETY net (a dead source must not leave the hand holding a grip),
# NOT a stream-pacing knob. It must sit ABOVE the worst-case gap between valid
# datagrams, or it fights the stream -- snapping the hand to rest and back
# between bursts (the classic "herky-jerky" with an ack-gated sender whose
# round-trip occasionally exceeds the window). 1500 ms clears normal ack-round-
# trip variance while still catching a genuine dropout within ~1.5 s. Set 0 to
# disable entirely (only when something else guarantees a safe stop).
DEFAULT_WATCHDOG_MS = 1500

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


class PoseIntegrator:
    """Velocity-integrates each channel's drive into a throttled position command.

    Each channel holds a float position in ``[-1, 1]``. An incoming per-frame
    value is a signed DRIVE (velocity): ``position += value * dt / ramp_time_s``,
    clamped, so a steady ``+1`` walks the joint from rest to full flex over
    ``ramp_time_s`` and a value near 0 freezes it. A per-channel EMA of the drive
    sign tracks how CONSISTENTLY the recent drive pushed one way, which gates
    whether accumulated motion is worth committing.

    The integrator is the single source of truth for the target pose. Accepted
    frames only ``drive()`` it; ``due()`` decides when the integrated pose has
    earned a command, per the rate cap + deadband + consistency policy. It does
    not itself talk to the device -- the receiver dispatches when ``due()`` says
    so and calls ``mark_sent()`` with what it committed.
    """

    def __init__(self, joints, ramp_time_s=RAMP_TIME_S,
                 min_interval_s=MIN_COMMAND_INTERVAL_S,
                 deadband=POSITION_DEADBAND, consistency_beta=CONSISTENCY_BETA,
                 consistency_min=CONSISTENCY_MIN, drive_eps=DRIVE_EPS):
        self.joints = list(joints)
        self.ramp_time_s = max(1e-3, float(ramp_time_s))
        self.min_interval_s = max(0.0, float(min_interval_s))
        self.deadband = max(0.0, float(deadband))
        self.consistency_beta = min(1.0, max(0.0, float(consistency_beta)))
        self.consistency_min = max(0.0, float(consistency_min))
        self.drive_eps = max(0.0, float(drive_eps))
        #: Integrated float position per joint, in [-1, 1]. 0 is rest.
        self.position = {j: 0.0 for j in self.joints}
        #: EMA of drive sign per joint, in [-1, 1]; |.| near 1 == consistent push.
        self.consistency = {j: 0.0 for j in self.joints}
        #: Position last actually SENT to the device (float, [-1, 1]).
        self.last_sent = {j: 0.0 for j in self.joints}
        self._last_send_monotonic = None
        #: True while NO channel is being actively driven (all below drive_eps).
        #: Updated every drive() call; the settle-release keys off it.
        self._drive_idle = True
        #: Set once the settle-release has landed the residual for the current
        #: idle episode, so it does not re-fire every idle frame. Cleared when
        #: the drive becomes active again.
        self._settled = True

    def drive(self, joint_values, dt):
        """Integrate one frame of per-joint drive over elapsed ``dt`` seconds.

        ``joint_values`` maps joint -> signed drive in [-1, 1]; joints absent
        from it get zero drive (they hold position). Only drives past
        ``drive_eps`` move the position or build consistency, so near-rest decoder
        noise neither ramps a joint nor fakes a direction.
        """
        if dt <= 0.0:
            return
        step_scale = dt / self.ramp_time_s
        any_active = False
        for joint in self.joints:
            drive = float(joint_values.get(joint, 0.0))
            if abs(drive) < self.drive_eps:
                # No meaningful push: hold position, let consistency decay toward 0.
                self.consistency[joint] += self.consistency_beta * (0.0 - self.consistency[joint])
                continue
            any_active = True
            self.position[joint] = _clip_unit(self.position[joint] + drive * step_scale)
            target_sign = 1.0 if drive > 0 else -1.0
            self.consistency[joint] += self.consistency_beta * (target_sign - self.consistency[joint])
        # Track the drive-idle edge: an active frame re-arms the settle-release so
        # the NEXT idle episode gets one landing commit.
        self._drive_idle = not any_active
        if any_active:
            self._settled = False

    def moved_joints(self):
        """Joints whose integrated position differs from the last sent by >= deadband."""
        return [j for j in self.joints
                if abs(self.position[j] - self.last_sent[j]) >= self.deadband]

    def due(self, now):
        """Should a command be dispatched now? Returns True when the pose has earned one.

        The rate cap is the ONE spacing rule that always applies -- it gives a
        regular command cadence, which is what makes motion look smooth. Past the
        cap:

        * Under active, consistent directional drive we dispatch EVERY tick,
          deadband or not. The deadband must NOT gate active motion: at a high
          datagram rate each frame integrates a tiny step, so requiring a whole
          deadband of accumulation before sending makes commands fire in
          irregular bursts (accumulate-then-flush) instead of at the steady rate
          cap -- which reads as herky-jerky. During a smooth push the rate cap
          alone paces it; the pose has advanced by exactly one cap-interval's
          worth of travel, which is the smooth step we want.
        * When the drive is NOT consistent (idle/holding/jittering) the deadband
          applies: only re-send if the pose actually moved, so a still or noisy
          hand does not stream near-identical commands. The settle-release is the
          one-shot landing of residual motion after the drive stops.
        """
        if self._last_send_monotonic is not None and \
                now - self._last_send_monotonic < self.min_interval_s:
            return False
        # Active consistent drive: pace on the rate cap alone (smooth cadence).
        if not self._drive_idle and \
                any(abs(c) >= self.consistency_min for c in self.consistency.values()):
            return True
        # Otherwise gate on real movement past the deadband.
        moved = self.moved_joints()
        if not moved:
            return False
        if self._drive_idle and not self._settled:
            # Settle-release: the drive stopped with residual uncommitted motion.
            return True
        # A moved-but-not-idle joint under consistent drive (handled above) won't
        # reach here; this catches a moved joint whose own channel is consistent.
        return any(abs(self.consistency[j]) >= self.consistency_min for j in moved)

    def signed_targets(self):
        """The current integrated pose as signed [-100, 100] wire ints per joint."""
        return {j: clamp_finger_value(self.position[j] * SET_FINGER_ANGLES_MAX)
                for j in self.joints}

    def mark_sent(self, now):
        """Record that the current pose was just dispatched.

        If the drive is currently idle, this commit is the settle-release (or a
        rate-limited landing) for this idle episode, so mark the episode settled
        to keep it from re-firing; an active drive re-arms it in drive().
        """
        self.last_sent = dict(self.position)
        self._last_send_monotonic = now
        if self._drive_idle:
            self._settled = True

    def reset_to_rest(self):
        """Force the integrated pose (and last-sent baseline) to rest."""
        for joint in self.joints:
            self.position[joint] = 0.0
            self.consistency[joint] = 0.0
            self.last_sent[joint] = 0.0
        self._drive_idle = True
        self._settled = True


def _clip_unit(x):
    """Clamp to [-1, 1]."""
    return -1.0 if x < -1.0 else (1.0 if x > 1.0 else x)


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
    """Decodes continuous datagrams and drives the hand from throttled moves.

    Each accepted datagram is a signed per-channel DRIVE that velocity-integrates
    a persistent float pose (``PoseIntegrator``) toward the endpoints; the raw
    sample is not relayed straight through. Commands to the exo are THROTTLED: one
    goes out only when the integrated pose has earned it (rate cap + deadband +
    directional consistency), so a holding or jittering hand does not saturate the
    serial link. Frames arriving between two dispatches integrate but do not each
    command.

    The ack is decoupled from the throttled commands so it can serve flow
    control. By default (``ack_on_accept``) every accepted frame is acked
    immediately with the current integrated pose, which is what an ack-gated
    sender needs to release its next frame -- throttling the commands must not
    throttle the acks or the stream stalls. ``ack_on_accept=False`` restores the
    reply-driven ack (only dispatched frames, acked on the device's reply,
    attesting the exo answered), which suits a free-running sender but stalls an
    ack-gated one.
    """

    def __init__(self, comm, verbose=True, watchdog_s=None,
                 print_every=PRINT_EVERY, trace=False,
                 ramp_time_s=RAMP_TIME_S,
                 min_interval_s=MIN_COMMAND_INTERVAL_S,
                 deadband=POSITION_DEADBAND,
                 consistency_beta=CONSISTENCY_BETA,
                 consistency_min=CONSISTENCY_MIN,
                 drive_eps=DRIVE_EPS,
                 ack_on_accept=True, grasp=False):
        self.comm = comm
        self.verbose = verbose
        #: Grasp mode: collapse the incoming vector to its FIRST value and drive
        #: all five fingers from it, pinning the wrist at rest. Off by default; the
        #: full per-channel layout is used otherwise.
        self.grasp = grasp
        #: Ack policy. True: ack every accepted frame IMMEDIATELY (flow-control
        #: credit for an ack-gated sender that only sends the next frame once the
        #: previous is acked -- so every accepted frame MUST be acked or the
        #: stream stalls). False: reply-driven ack, only for dispatched frames,
        #: attesting the device answered (breaks an ack-gated sender). Command
        #: throttling to the exo is independent of this either way.
        self.ack_on_accept = ack_on_accept
        #: Echo every per-frame device reply and every ack as they happen. One
        #: of each fires on EVERY accepted frame, so this floods at the frame
        #: rate and is independent of print_every (which throttles only the
        #: averaged summary line). Default off; --trace turns it on.
        self.trace = trace
        self.watchdog_s = watchdog_s
        self.print_every = max(1, int(print_every))
        self.sock = None
        #: Monotonic time of the last heartbeat status line, and the dispatched
        #: count then, so the heartbeat can flag a stream that is arriving but
        #: producing no commands (the usual "no movement" cause).
        self._last_heartbeat_monotonic = None
        self._heartbeat_dispatched = 0
        self._ack_addr = None            # source of the last datagram, for acks
        self.last_sequence = None
        self.last_packet_monotonic = None
        self.channel_layout = None       # first accepted channel_names, locked
        self._resolved_joints = []       # canonical joints for the locked layout
        #: Velocity integrator: the single source of truth for the target pose.
        #: Accepted frames drive() it; it decides when a command is due.
        self.integrator = PoseIntegrator(
            JOINTS, ramp_time_s=ramp_time_s, min_interval_s=min_interval_s,
            deadband=deadband, consistency_beta=consistency_beta,
            consistency_min=consistency_min, drive_eps=drive_eps)
        #: Monotonic time of the last frame fed to the integrator, for its dt.
        self._last_drive_monotonic = None
        #: Sequence of the newest accepted frame not yet folded into a dispatched
        #: command. Carried into the command so its reply acks that sequence; the
        #: frames it coalesced show as an ack-sequence gap.
        self._pending_ack_sequence = None
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
        #: Accepted frames integrated but NOT dispatched (throttled/coalesced).
        self.coalesced = 0
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
        now = time.monotonic()
        self.last_sequence = packet["sequence"]
        self.last_packet_monotonic = now
        self.accepted += 1

        # This frame's per-joint DRIVE (signed velocity in [-1, 1]). Driven joints
        # take their channel value; joints no channel names get zero drive, so the
        # integrator holds them (undriven != commanded-to-rest here -- rest is what
        # the pose starts at and the watchdog restores).
        drive = {joint: 0.0 for joint in JOINTS}
        if self.grasp:
            # Grasp mode: ignore the channel layout and drive all five fingers
            # from the FIRST received value alone (a single grasp scalar), pinning
            # the wrist at 0. This collapses a multi-channel stream into one
            # whole-hand open/close.
            grip = float(packet["values"][0])
            for joint in GRASP_FINGERS:
                drive[joint] = grip
            # Hold the wrist hard at rest so a residual pose from before --grasp
            # was engaged (or any drift) cannot leave it flexed.
            self.integrator.position["wrist"] = 0.0
        else:
            for joint, value in zip(self._resolved_joints, packet["values"]):
                drive[joint] = float(value)

        # Integrate this frame over the real elapsed time since the last one.
        # Clamp dt so one scheduling hiccup or a burst drained from the socket
        # buffer cannot slam the integrator: dt==0 for buffered back-to-back
        # frames adds no travel (correct), and a long stall is capped at
        # MAX_INTEGRATION_DT rather than jumping the pose. The ramp speed thus
        # tracks real seconds of drive, independent of frame-rate jitter.
        dt = 0.0 if self._last_drive_monotonic is None \
            else min(now - self._last_drive_monotonic, MAX_INTEGRATION_DT)
        self._last_drive_monotonic = now
        self.integrator.drive(drive, dt)

        # Ack policy. An ack-gated sender only emits its NEXT frame once this one
        # is acked, so under ack_on_accept we ack EVERY accepted frame right here
        # -- reporting the current integrated pose -- to keep the stream flowing.
        # The command to the exo is still throttled below; the ack (flow-control
        # credit) is deliberately decoupled from the sparse device commands. Under
        # the reply-driven policy instead, only dispatched frames are acked and
        # the sequence rides on the next command (a coalesced frame shows as a
        # gap -- which STALLS an ack-gated sender, hence it is not the default).
        if self.ack_on_accept:
            self._ack_now(packet["sequence"], self.integrator.signed_targets())
            self._pending_ack_sequence = None
        else:
            self._pending_ack_sequence = packet["sequence"]

        # Throttle: dispatch a command to the exo only if the integrated pose has
        # earned one. Independent of the ack above.
        self._maybe_dispatch(now)

        # Console output is batched: buffer this frame's INTEGRATED pose (the
        # thing being driven toward), not the raw sample, and flush one averaged
        # line every print_every accepted frames.
        pose = self.integrator.position
        self._print_buf.append({j: pose[j] for j in self._reported_joints()})
        if self.verbose and len(self._print_buf) >= self.print_every:
            self._flush_print(sender)

    def _maybe_dispatch(self, now):
        """Dispatch the integrated pose iff the throttle policy says it is due.

        Called after every integrated frame AND from the main loop's idle path,
        so a rate-limited or settle-release command still goes out when no new
        datagram is arriving. When it dispatches, the newest un-acked sequence is
        folded into the command's ack entry; when it does not, the frame is
        counted as coalesced.
        """
        if not self.integrator.due(now):
            return
        # A coalesced frame is one that was ACCEPTED and integrated but is being
        # folded into a later command instead of dispatched on its own. Count it
        # only when there is a genuine un-dispatched accepted frame pending -- not
        # on every idle service_throttle() tick, which carries no new frame.
        if self._pending_ack_sequence is not None:
            self.coalesced += 1
        targets = self.integrator.signed_targets()
        self._apply(targets, self._pending_ack_sequence)
        self.integrator.mark_sent(now)
        self._pending_ack_sequence = None
        self.at_rest = all(v == 0 for v in targets.values())

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

    def _send_ack(self, sequence, targets, addr):
        """Send one NGA3 ack for `sequence` carrying `targets` to `addr`."""
        if self.sock is None or addr is None:
            return
        try:
            self.sock.sendto(pack_continuous_ack(sequence, JOINTS, targets), addr)
            self.acked += 1
            if self.trace:
                print(f"  [{time.monotonic():.3f}] -> ack seq {sequence} to "
                      f"{addr[0]}:{addr[1]}")
        except OSError as exc:
            # A source that is not listening (ICMP port-unreachable) must not
            # kill the receiver; it is still driving the hand fine.
            if self.verbose:
                print(f"  [ack] send to {addr} failed: {exc}", file=sys.stderr)

    def _ack_now(self, sequence, targets):
        """Ack an accepted frame immediately (ack_on_accept flow-control credit).

        Acks at ACCEPTANCE, not on a device reply, so an ack-gated sender always
        gets the credit to send its next frame -- the exo command is throttled
        separately and must not gate the stream. `targets` is the integrated pose
        reported to the sender (what the receiver is driving toward), which is
        honest even when no command was dispatched this frame.
        """
        self._send_ack(sequence, targets, self._ack_addr)

    def _retire_pending(self):
        """Ack the oldest frame awaiting a reply, now that one has arrived.

        Reply-driven path only (ack_on_accept is False): a solicited device reply
        retires the oldest outstanding dispatched frame. One command was
        dispatched per call, so one reply retires one frame.
        """
        if not self._pending:
            return
        sequence, targets, addr = self._pending.popleft()
        self._send_ack(sequence, targets, addr)

    def _reported_joints(self):
        """Joints the console lines report the integrated pose for.

        Normally the locked channel layout's resolved joints; in grasp mode the
        five fingers actually being driven, since the incoming layout no longer
        describes what moves.
        """
        return GRASP_FINGERS if self.grasp else self._resolved_joints

    def _flush_print(self, sender):
        """Print one line with the mean of the buffered INTEGRATED pose.

        The buffer holds float positions in [-1, 1]; they are shown scaled to the
        signed [-100, 100] wire value that actually reaches the exo, so the line
        reads in the same units as the command and the ack.
        """
        if not self._print_buf:
            return
        n = len(self._print_buf)
        sums = {}
        for frame in self._print_buf:
            for joint, value in frame.items():
                sums[joint] = sums.get(joint, 0.0) + value
        means = " ".join(
            f"{joint}={sums[joint] / n * SET_FINGER_ANGLES_MAX:+.0f}"
            for joint in self._reported_joints() if joint in sums
        )
        print(f"  [{time.monotonic():.3f}] [{sender}] avg pose of last {n} "
              f"(seq ~{self.last_sequence}, {self.dispatched} sent) -> {means}")
        self._print_buf.clear()

    def service_throttle(self):
        """Emit a due command when no new datagram is arriving.

        The integrator can become due between datagrams -- a rate-limited move
        whose interval has now elapsed, or the settle-release once the drive has
        gone idle. Called from the main loop's idle path so those still go out
        rather than waiting for the next frame that may never come.
        """
        self._maybe_dispatch(time.monotonic())

    def _rest_now(self):
        """Reset the integrator to rest and dispatch it as an unacked safety move.

        Routes through the integrator so its position and last-sent baseline stay
        consistent with the device; a bare _apply would leave the integrator
        believing the hand is still where it was, corrupting the next deadband and
        settle decision. The rest move is not acked (sequence=None).
        """
        self.integrator.reset_to_rest()
        self._pending_ack_sequence = None
        self._apply({joint: 0 for joint in JOINTS})
        self.at_rest = True

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
        self._rest_now()

    def rest_all(self):
        """Drive every joint to rest, e.g. on shutdown before disarming."""
        self._rest_now()

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

    def heartbeat(self, now, interval_s=1.0):
        """Print a periodic status line so a silent stream is diagnosable.

        The common "no movement" causes are distinct and this tells them apart at
        a glance: nothing arriving (received flat), arriving-but-rejected
        (rejected climbing), or accepted-but-throttled (accepted climbing while
        dispatched is flat -- drive too small/noisy for the deadband/consistency
        gate). Only prints while a source is streaming and not at rest, and stays
        quiet once things are moving normally, so it does not add console noise.
        """
        if not self.verbose:
            return
        if self._last_heartbeat_monotonic is None:
            self._last_heartbeat_monotonic = now
            self._heartbeat_dispatched = self.dispatched
            return
        if now - self._last_heartbeat_monotonic < interval_s:
            return
        sent_since = self.dispatched - self._heartbeat_dispatched
        self._last_heartbeat_monotonic = now
        self._heartbeat_dispatched = self.dispatched
        # Only speak up when a stream is live (something arrived recently) but no
        # command went out this interval -- the "accepted but nothing moved" case.
        recent = (self.last_packet_monotonic is not None
                  and now - self.last_packet_monotonic < interval_s)
        if not recent or sent_since > 0:
            return
        pose = " ".join(
            f"{j}={self.integrator.position[j] * SET_FINGER_ANGLES_MAX:+.0f}"
            for j in (self._resolved_joints or JOINTS)
        )
        print(f"  [status] no exo command in {interval_s:g}s: "
              f"recv={self.received} accepted={self.accepted} "
              f"dispatched={self.dispatched} acked={self.acked} "
              f"rejected={self.rejected} stale={self.stale} | pose {pose}")
        if self.accepted > 0 and self.dispatched == 0:
            # Held back by the throttle. Whether that is a problem depends on the
            # drive: a real push should clear the gate within a few frames.
            print("           (frames accepted but throttled -- drive may be too "
                  "small/noisy for --deadband/--consistency-min, or values ~0)")
        if not self.ack_on_accept and self.accepted > self.acked + 2:
            # Reply-driven acks with an ack-gated sender: this is the stall path.
            print("           (reply-driven acks are lagging accepts -- an "
                  "ack-gated sender will stall; consider ack-on-accept, the "
                  "default)")

    def print_summary(self):
        print(f"\n  datagrams received : {self.received}")
        print(f"  accepted           : {self.accepted}")
        print(f"  commands dispatched: {self.dispatched}")
        if self.coalesced:
            ratio = (self.accepted / self.dispatched) if self.dispatched else 0.0
            print(f"  frames coalesced   : {self.coalesced} "
                  f"(integrated, not sent; {ratio:.1f} frames/command)")
        ack_when = "on accept" if self.ack_on_accept else "on device reply"
        print(f"  acks sent upstream : {self.acked} ({ack_when})")
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
                             f"of that batch's integrated pose. Rejects, stale "
                             f"drops and watchdog trips still print immediately.")
    parser.add_argument("--ramp-time", type=float, default=RAMP_TIME_S,
                        metavar="S",
                        help=f"Seconds of full-scale drive to travel a channel's "
                             f"rest->endpoint span (default {RAMP_TIME_S:g}). The "
                             f"incoming value is a velocity; larger is slower/more "
                             f"deliberate. 0 or less means near-instant (relay).")
    parser.add_argument("--min-interval-ms", type=float,
                        default=MIN_COMMAND_INTERVAL_S * 1000.0, metavar="MS",
                        help=f"Minimum spacing between commands actually sent to "
                             f"the exo (default {MIN_COMMAND_INTERVAL_S*1000:g}). "
                             f"Caps the command rate below the datagram rate.")
    parser.add_argument("--deadband", type=float, default=POSITION_DEADBAND,
                        metavar="FRAC",
                        help=f"Min integrated-pose move (fraction of full travel, "
                             f"[0,1]) versus the last sent command before a new "
                             f"one is sent (default {POSITION_DEADBAND:g}).")
    parser.add_argument("--consistency-min", type=float, default=CONSISTENCY_MIN,
                        metavar="C",
                        help=f"Directional-consistency threshold in [0,1] a moved "
                             f"joint must reach before its motion is committed "
                             f"(default {CONSISTENCY_MIN:g}). Higher = only very "
                             f"smooth, sustained pushes command the exo.")
    parser.add_argument("--ack-on-reply", dest="ack_on_accept",
                        action="store_false",
                        help="Ack a frame only when the DEVICE replies to a "
                             "command dispatched for it (attests the exo "
                             "answered), instead of the default ack-on-accept. "
                             "Fewer acks, but this STALLS an ack-gated sender "
                             "that waits for an ack before sending the next "
                             "frame -- use only with a free-running sender.")
    parser.set_defaults(ack_on_accept=True)
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
    parser.add_argument("--grasp", action="store_true",
                        help="Whole-hand grasp mode: ignore the channel layout and "
                             "drive all five fingers from ONLY the first received "
                             "value, fixing the wrist at 0. Off by default.")
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
                        trace=args.trace, ramp_time_s=args.ramp_time,
                        min_interval_s=args.min_interval_ms / 1000.0,
                        deadband=args.deadband,
                        consistency_min=args.consistency_min,
                        ack_on_accept=args.ack_on_accept, grasp=args.grasp)

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
                # No datagram: a rate-limited or settle-release command may now
                # be due, so pump the throttle before the watchdog and replies.
                receiver.service_throttle()
                receiver.service_watchdog()
                receiver.drain_replies()
                receiver.heartbeat(time.monotonic())
                continue
            except OSError:
                break
            if data:
                receiver.handle(data, f"{addr[0]}:{addr[1]}", addr)
            # handle() already pumped the throttle for this frame; pump again in
            # case its dispatch was gated only by the rate cap and the interval
            # has since elapsed.
            receiver.service_throttle()
            receiver.service_watchdog()
            receiver.drain_replies()
            receiver.heartbeat(time.monotonic())

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
