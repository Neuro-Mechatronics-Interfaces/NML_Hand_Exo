#!/usr/bin/env python3
"""
Standalone UDP -> exo gesture receiver.

Binds a UDP socket, maps each received integer to one or more serial commands,
and writes them straight to the exo over the dual USB-CDC link.  Runs until
SIGINT (Ctrl-C), then disables the motors and closes the port.

Deliberately minimal compared to the GUI: no Qt, no telemetry polling, no
per-command round-trip wait.  A datagram is turned into a serial write with
nothing in between, which is the lowest-latency path available to the host.
Replies land on the telemetry CDC and are drained by DualSerialComm's reader
thread, so they never gate the command path.

Edit build_command_map() below to change the integer -> command associations.

Each acked command is followed by a packed binary frame carrying both the
gesture percentage/status code and the rest-zeroed signed angle for all six
joints, so a consumer learns the resulting pose without polling for it. The
ASCII integer ack is unchanged and still goes out first; the pose is an
additional datagram that only readers who know its magic need to look at. See
pack_pose_ack().

Requires firmware >= 0.6.1: per-joint moves use set_gesture_angle, whose
percentages interpolate each gesture's extend and flex postures from that
version on, and `wrist` drives both dorsal wrist motors together. The pose ack
uses get_gesture_angles, added in 0.6.1, and switches itself off against
anything older. Use examples/08_udp/udp_gesture_gui.py to drive this receiver
by hand.

Usage:
    python examples/08_udp/udp_gesture_receiver.py
    python examples/08_udp/udp_gesture_receiver.py --port 10003 --cmd-port COM10
    python examples/08_udp/udp_gesture_receiver.py --no-arm         # do not enable motors
    python examples/08_udp/udp_gesture_receiver.py --no-pose-ack    # skip pose queries
    python examples/08_udp/udp_gesture_receiver.py --ignore-replies # hide device replies
    python examples/08_udp/udp_gesture_receiver.py --mock           # no exo attached

--mock swaps the serial transport for an in-process fake that answers like the
firmware, so the UDP mapping, the return-port registration and the ack loopback
can all be exercised with no hardware present.  Commands that are silent in
firmware stay silent in the mock, so it cannot make the host look healthier
than the real device.
"""

import argparse
import collections
import signal
import socket
import sys
import time

from nml_hand_exo import DualSerialComm
from nml_hand_exo.interface._gesture_protocol import (
    COMMAND_PASSTHROUGH_ACK,
    POSE_ACK_HEADER,
    POSE_ACK_MAGIC,
    POSE_ACK_RECORD,
    POSE_QUERY,
    POSE_UNAVAILABLE,
    UDP_GESTURE_JOINTS as JOINTS,
    pack_pose_ack,
    unpack_pose_ack,
)
from nml_hand_exo.interface._hand_exo import (
    GESTURE_ANGLES_PREFIX,
    parse_gesture_angle_pairs,
)
from nml_hand_exo.interface._udp_command_bindings import (
    UDP_CONNECTION_PORT_MAX,
    UDP_CONNECTION_PORT_THRESHOLD,
    UDP_HEARTBEAT_REQUEST_VALUE,
    parse_udp_integer,
)


# ======================================================================
#  CONFIGURATION -- edit here
# ======================================================================

UDP_HOST = "0.0.0.0"
UDP_PORT = 10003

CMD_PORT = "COM10"      # commands out
TELEM_PORT = "COM11"    # replies in
BAUD = 1000000

# Per-joint UDP value assignments.  Order fixes the integer for each joint, and
# the layout of the pose frame sent back with each ack.
#
# "wrist" drives BOTH dorsal wrist motors (wrist and wrist2) together: they are
# mounted on the back of the arm and pull on the same structure, so commanding
# one alone leaves the other holding position against it. The joint order and
# pose-frame layout are imported from the SDK so this receiver and the GUI use
# one wire contract.

# Offset added to a joint's value to address its REST position, e.g. index is
# +2 flex / -2 extend / +12 rest.  Must keep every value below
# UDP_CONNECTION_PORT_THRESHOLD (64), above which an integer is read as a
# return-port announcement rather than a command.
REST_VALUE_OFFSET = 10

# Position of each state as a percentage between the joint's two end postures:
# 0 is exactly `set_gesture:<joint>:extend` and 100 is exactly
# `set_gesture:<joint>:flex`, as defined by the EXTEND_*/FLEX_* constants in
# src/cpp/nml_hand_exo/config.h.  So the percentages below are relative to those
# postures, not to raw motor travel -- retuning the constants moves both ends
# and these keep meaning the same thing.  Change them here to retune the UDP
# path alone, or in config.h to retune the postures themselves.
FLEX_PERCENT = {
    "thumb": 85, "index": 85, "middle": 85, "ring": 85, "pinky": 85,
    "wrist": 75,
}
REST_PERCENT = {
    "thumb": 15, "index": 15, "middle": 15, "ring": 15, "pinky": 15,
    "wrist": 25,
}
EXTEND_PERCENT = 0   # 0% == the extend posture (NOT home, unless EXTEND_* is 0)


def build_command_map():
    """Integer payload -> serial command(s).

    Value scheme (a value may map to one command or a list sent in order):

        0         whole-hand release
        +1..+6    flex   joint N        -1..-6    extend joint N
        +11..+16  rest   joint N

    Per-joint moves go through `set_gesture_angle:<joint>:<percent>` rather than
    `set_gesture:<joint>:<state>` so the position is set here in the host map
    instead of being fixed by the firmware constants -- the percentages still
    interpolate the firmware's own extend and flex postures, so 0 and 100
    reproduce those states exactly.  0 stays a `set_gesture` posture on purpose:
    grasp is multi-joint and the firmware rejects postures for
    set_gesture_angle, since one percentage cannot describe a whole posture.

    Requires firmware >= 0.6.0.
    """
    command_map = {0: "set_gesture:grasp:open"}
    for index, joint in enumerate(JOINTS, start=1):
        # Calibrated positions: each integer code maps to a set_gesture_angle at
        # this joint's FLEX_PERCENT / EXTEND_PERCENT / REST_PERCENT below, so the
        # discrete FLEX/EXTEND/REST endstops are set HERE (tune the percent dicts)
        # rather than by the firmware's compile-time postures. 0 and 100 still
        # reproduce the firmware's extend/flex exactly, so these interpolate them.
        command_map[index] = f"set_gesture_angle:{joint}:{FLEX_PERCENT[joint]}"
        command_map[-index] = f"set_gesture_angle:{joint}:{EXTEND_PERCENT}"
        command_map[index + REST_VALUE_OFFSET] = (
            f"set_gesture_angle:{joint}:{REST_PERCENT[joint]}"
        )
        # Fixed-posture form (uncomment, and comment the three above, to drive
        # the firmware's compile-time extend/flex/rest instead of the percents):
        # command_map[index] = f"set_gesture:{joint}:flex"
        # command_map[-index] = f"set_gesture:{joint}:extend"
        # command_map[index + REST_VALUE_OFFSET] = f"set_gesture:{joint}:rest"
    return command_map


COMMAND_MAP = build_command_map()

# Prefix of the firmware's asynchronous move-outcome report. It arrives AFTER
# the command reply, once the motors have finished (or failed to finish), so it
# cannot ride on the command ack -- it is forwarded upstream as its own
# datagram. Needs firmware >= 0.5.0; older builds simply never emit it.
GESTURE_RESULT_PREFIX = "GESTURE_RESULT:"

# Pose-frame constants and pack/unpack helpers come from
# nml_hand_exo.interface._gesture_protocol.


def parse_passthrough_command(payload):
    """Validate a `set_gesture_angle:<joint>:<percent>` datagram.

    The integer protocol can only address positions the map was built with,
    which is fine for a decoder driving fixed postures but not for tuning a
    joint by hand -- there is no integer that carries an arbitrary percentage.
    This accepts the command form directly for that case.

    Deliberately NOT a general command passthrough. The receiver binds
    0.0.0.0 by default, so anything it forwards is reachable from the network;
    only this one command shape, with a known joint and an in-range percentage,
    is allowed through. Everything else stays unreachable over UDP.

    Args:
        payload (str): Raw datagram text.

    Returns:
        str or None: The command to forward, or None if it does not qualify.
    """
    if not payload:
        return None
    parts = payload.strip().split(":")
    if len(parts) != 3:
        return None
    head, joint, percent = (part.strip() for part in parts)
    if head.lower() != "set_gesture_angle":
        return None
    if joint.lower() not in JOINTS:
        return None
    try:
        value = float(percent)
    except ValueError:
        return None
    if not 0.0 <= value <= 100.0:
        return None
    return f"set_gesture_angle:{joint.lower()}:{value:g}"

# Working effort per motor, in mA (--current-ma). This is the NOMINAL effort:
# how hard ONE motor may push while holding or chasing a position.
#
# Before firmware 0.4.0 this had to be detuned down to whatever the supply could
# survive with every joint pushing at once, which is why it kept drifting lower.
# From 0.4.0 the combined budget below bounds the fleet, so this can be set to
# what a single joint actually needs to move reliably.
DEFAULT_CURRENT_MA = 150

# Combined budget across ALL motors, in mA (--total-current-ma). This is the
# number the power supply cares about: per-motor limits do not bound what the
# motors draw together, which is what browns out the board when a posture such
# as `grasp:open` commands every joint at once. Sized for the supply, not the
# motors. 0 leaves whatever the firmware booted with alone. Requires >= 0.4.0.
DEFAULT_TOTAL_CURRENT_MA = 800

# Commands sent once at startup when arming, and the release sent on exit.
# ARM_COMMANDS = ("reboot:all", "set_exo_mode:gesture_fixed", "enable:all")
ARM_COMMANDS = ("reboot:all", "enable:all", "home:all")
DISARM_COMMANDS = ("disable:all",)

# Sent after arming (--no-home to skip). Homing drives every motor to its zero
# offset, so gestures start from a known pose instead of wherever the hand was
# left. THE HAND MOVES when this runs.
# HOME_COMMANDS = ("home:1","home:2","home:3","home:4","home:5","home:6","home:7","home:8","home:9", 
#                  "home:11","home:12","home:13","home:14","home:15","home:16", "home:17", "home:18", "home:19")
# HOME_COMMANDS = ("home:all")
HOME_COMMANDS = ("home:11", "home:12")

# Homing is a physical move, not just a register write, so give it time to
# finish before commands start arriving.
HOME_SETTLE_S = 5

# Firmware VERBOSE emits a blocking USB-CDC write per debug line, which
# directly delays every command. Turned off at startup unless --debug-on.
QUIET_COMMANDS = ("debug:off",)

# ======================================================================

LINE_TERMINATOR = "\r\n"
# recvfrom wakes immediately on a datagram, so this only sets how often we fall
# through to drain device replies when no traffic is arriving. It must stay
# small: at 0.25 s a reply could sit undisplayed for a quarter second and look
# like device latency that isn't there.
SOCKET_POLL_S = 0.005


#: Cap on outstanding un-acked commands. Firmware without gesture acks never
#: replies, so without this the pending queue would grow forever.
MAX_PENDING_ACKS = 64

#: Recovery from the device dropping off the USB bus mid-session (a reset or a
#: brownout re-enumerates it and invalidates the COM handle).
RECONNECT_ATTEMPTS = 5
RECONNECT_DELAY_S = 1.0


def rediscover_cdc_pair(cmd_port, telem_port):
    """Re-locate this device's CDC pair after a USB re-enumeration.

    The COM numbers usually survive a reset, but not always, so fall back to
    scanning for any dual-CDC pair currently present.
    """
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


#: Simulated device turnaround for --mock, in milliseconds. Non-zero on
#: purpose: acks must arrive AFTER the send so the pending-ack correlation is
#: exercised the same way it is against real hardware.
MOCK_LATENCY_MS = 8.0


class MockComm:
    """In-process stand-in for DualSerialComm, for running with no exo attached.

    Implements only the surface the receiver uses (connect/close/send/receive/
    flush_input plus the port-name attributes). Replies mirror the firmware:
    commands that call commandPrint get a ';'-terminated frame, and the ones
    that are silent in firmware -- enable, disable, set_exo_mode -- get nothing,
    so the mock cannot make the host look healthier than the real device.
    """

    def __init__(self, latency_ms=MOCK_LATENCY_MS, log=True):
        self.cmd_port = "MOCK-CMD"
        self.telem_port = "MOCK-TELEM"
        self.latency_s = max(0.0, latency_ms) / 1000.0
        self.log = log
        self.sent = []
        self._pending = collections.deque()   # (ready_at, frame)
        self._open = False

    # -- transport surface --------------------------------------------

    def connect(self):
        self._open = True

    def close(self):
        self._open = False
        self._pending.clear()

    def is_connected(self):
        return self._open

    def reader_alive(self):
        return self._open

    def telemetry_lines(self):
        return []

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
        outcome = self._outcome_for(command)
        if outcome is not None:
            # Well after the command reply: a real move concludes over hundreds
            # of ms, and the ordering is what the receiver has to cope with.
            self._pending.append((time.monotonic() + self.latency_s + 0.25, outcome))

    def receive(self, wait_until_return=False, timeout=None):
        deadline = time.monotonic() + (timeout or 0.0)
        while True:
            if self._pending and self._pending[0][0] <= time.monotonic():
                return self._pending.popleft()[1]
            if not wait_until_return or time.monotonic() >= deadline:
                return ""
            time.sleep(0.002)

    #: Outcome the mock reports for a move. Override in tests to simulate a
    #: hand that accepts commands but does not actually move.
    mock_outcome = ("GESTURE_RESULT: reached=1 stalled=0 short=0 starved=0 "
                    "budget_scale=1.00")

    def _outcome_for(self, command):
        """The asynchronous verdict this command would eventually produce."""
        head = command.partition(":")[0]
        if head in ("set_gesture", "set_gesture_angle", "home"):
            return self.mock_outcome
        return None

    # -- firmware-shaped replies --------------------------------------

    @staticmethod
    def _reply_for(command):
        """Return the frame this command would produce, or None if silent."""
        head, _, rest = command.partition(":")
        if head == "set_gesture":
            return f"OK: gesture {rest}"
        if head == "set_gesture_state":
            return f"OK: gesture_state {rest}"
        if head == "set_gesture_angle":
            target, _, percent = rest.partition(":")
            try:
                percent = f"{min(100.0, max(0.0, float(percent))):.1f}"
            except ValueError:
                return f"ERROR: set_gesture_angle percent not numeric: {percent}"
            return f"OK: gesture_angle {target}:{percent}"
        if head == "set_current_lim":
            target, _, value = rest.partition(":")
            return f"OK: set_current_lim {target} {value}"
        if head == "set_total_current_lim":
            return f"OK: total_current_lim {rest}"
        if head == "get_total_current_lim":
            return "Total current limit: 800 mA"
        if head == "set_hold_current":
            return f"OK: hold_current {rest}"
        if head == "get_hold_current":
            return "Hold current: 25 mA"
        if head == "set_current_governor":
            return f"OK: current_governor {rest}"
        if head == "get_gesture_angles":
            # Fixed pose: the mock has no motors, so there is nothing to move
            # and nothing to read. The point is that the query ANSWERS, which
            # is what the ack correlation and the pose frame depend on.
            codes = (0, 25, 50, 75, 100, 101, 102, 255)
            angles = (-12.5, -4.0, 0.0, 8.25, 16.5, 24.0)
            return GESTURE_ANGLES_PREFIX + " " + " ".join(
                f"{joint}={code},{angle:.2f}"
                for joint, code, angle in zip(JOINTS, codes, angles)
            )
        if head == "home":
            return f"OK: home {rest}"
        if head == "debug":
            return f" Debug state: {'true' if rest == 'on' else 'false'}"
        if head == "version":
            return "Exo Device Version: mock"
        if head == "set_reply_route":
            return f"OK: reply_route {rest}"
        # enable / disable / set_exo_mode are silent in firmware.
        return None


class Receiver:
    """Maps UDP integers to serial commands and acks back to the sender.

    Ack protocol (matches the GUI's UDP binding convention):
      * An integer above UDP_CONNECTION_PORT_THRESHOLD is not a command -- it
        is the sender announcing the port it wants replies on. The port number
        is echoed straight back as a wake-up ACK.
      * After that, each handled command is acked by echoing its own integer,
        but only once the DEVICE has actually replied. Acking on dispatch would
        claim success for a command the exo never executed.
      * On shutdown the negated port is sent as a close notice.
    """

    def __init__(self, comm, command_map, echo_replies=True, verbose=True,
                 pose_ack=True):
        self.comm = comm
        self.command_map = command_map
        self.echo_replies = echo_replies
        self.verbose = verbose
        #: Whether each command is followed by a pose query. Cleared by
        #: probe_pose_support() against firmware that does not answer it -- an
        #: unknown command is SILENT in firmware, so leaving it on would mean
        #: waiting forever for a reply that never comes and stalling every ack.
        self.pose_ack = pose_ack
        self.sock = None
        self.return_addr = None          # (ip, port) once registered
        self._pending = collections.deque()   # [value, replies_still_expected]
        self.pose = {}                   # last reported {gesture: code}
        self.received = 0
        self.dispatched = 0
        self.unmapped = 0
        self.malformed = 0
        self.acked = 0
        self.pose_acks = 0
        self.dropped_acks = 0
        self.outcomes = 0
        self.link_down = False
        self.reconnects = 0
        self.send_failures = 0
        self.on_reconnect = None   # callback to re-apply session setup

    # -- serial link ---------------------------------------------------

    def send(self, command):
        """Write a command. Returns False and flags the link on failure.

        A dead USB handle must not abort the session: the device may be
        re-enumerating after a reset, and we still need a working link later to
        disarm the motors.
        """
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
        """Re-open the serial link after the device dropped off the bus."""
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
            # A reset restores firmware defaults, so re-apply session setup.
            if self.on_reconnect is not None:
                try:
                    self.on_reconnect()
                except Exception as exc:
                    print(f"  [reconnect] setup failed: {exc}", file=sys.stderr)
            return True
        print("  [reconnect] giving up", file=sys.stderr)
        return False

    def probe_pose_support(self, timeout=0.6):
        """Check the device answers the pose query, and disable it if not.

        Firmware ignores an unknown command SILENTLY, so an unsupported query
        would leave one reply outstanding on every command and stall the acks
        behind it forever. Testing the feature beats testing the version: it is
        the same round trip either way, and it also catches a device that is on
        a new enough build but has the query routed to a port we do not read.
        """
        if not self.pose_ack:
            return False
        try:
            self.comm.flush_input()
        except Exception:
            pass
        if not self.send(POSE_QUERY):
            return False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            reply = self.comm.receive(wait_until_return=True, timeout=timeout)
            if not reply:
                break
            pose = parse_gesture_angle_pairs(reply)
            if pose:
                self.pose = pose
                print(f"  pose acks enabled: {self._format_pose(pose)}")
                return True
        self.pose_ack = False
        print("  [warn] device did not answer "
              f"{POSE_QUERY!r}; pose acks disabled "
              "(needs firmware >= 0.6.1)", file=sys.stderr)
        return False

    @staticmethod
    def _format_pose(pose):
        rendered = []
        for joint in JOINTS:
            record = pose.get(joint) or {}
            fraction = record.get("fraction", POSE_UNAVAILABLE)
            angle = record.get("angle_delta_deg")
            angle_text = "nan" if angle is None else f"{angle:+.2f}deg"
            rendered.append(f"{joint}={fraction}%/{angle_text}")
        return " ".join(rendered)

    # -- upstream (UDP) ------------------------------------------------

    def send_upstream(self, value):
        """Echo an integer back to the registered sender."""
        if self.sock is None or self.return_addr is None:
            return False
        try:
            self.sock.sendto(str(int(value)).encode("ascii"), self.return_addr)
            return True
        except OSError as exc:
            print(f"  [ack] send to {self.return_addr} failed: {exc}",
                  file=sys.stderr)
            return False

    def send_pose_upstream(self, value):
        """Send the packed pose frame that accompanies an ack.

        Skipped when no pose has been reported yet, so a consumer never sees a
        frame of 255s that it cannot tell apart from a hand with no
        calibration.
        """
        if self.sock is None or self.return_addr is None or not self.pose:
            return False
        try:
            self.sock.sendto(pack_pose_ack(value, JOINTS, self.pose),
                             self.return_addr)
            self.pose_acks += 1
            return True
        except OSError as exc:
            print(f"  [pose] send to {self.return_addr} failed: {exc}",
                  file=sys.stderr)
            return False

    def send_text_upstream(self, text):
        """Forward a device line to the registered sender verbatim.

        Move outcomes carry more than an integer can, and the sender is a
        debugging GUI, so they travel as text. Consumers that only parse
        integers ignore them, which is why this is additive rather than a
        change to the ack encoding.
        """
        if self.sock is None or self.return_addr is None:
            return False
        try:
            self.sock.sendto(text.encode("utf-8", errors="ignore"), self.return_addr)
            return True
        except OSError as exc:
            print(f"  [outcome] send to {self.return_addr} failed: {exc}",
                  file=sys.stderr)
            return False

    def register_return_port(self, sender_ip, port):
        addr = (sender_ip, port)
        if self.return_addr == addr:
            print(f"  [{sender_ip}] return port {port} still live")
            return
        self.return_addr = addr
        print(f"  [{sender_ip}] registered return port {port} "
              f"-- acks will be sent to {sender_ip}:{port}")
        # Wake-up ACK: echo the port itself, as the GUI receiver does.
        self.send_upstream(port)

    def send_close_notice(self):
        """Tell the sender this receiver is going away (negated port)."""
        if self.return_addr is None:
            return
        print(f"Notifying {self.return_addr[0]}:{self.return_addr[1]} of close")
        self.send_upstream(-self.return_addr[1])

    # -- downstream (UDP -> serial) ------------------------------------

    def handle(self, payload, sender):
        self.received += 1
        sender_ip = sender.rsplit(":", 1)[0]
        value = parse_udp_integer(payload)
        if value is None:
            # Not an integer: it may still be the one command form accepted
            # directly, `set_gesture_angle:<joint>:<percent>`, which exists so a
            # joint can be positioned at an arbitrary percentage that no mapped
            # integer covers.
            command = parse_passthrough_command(payload)
            if command is None:
                self.malformed += 1
                if self.verbose:
                    print(f"  [{sender}] ignored non-integer payload: {payload!r}")
                return
            self._dispatch([command], COMMAND_PASSTHROUGH_ACK)
            if self.verbose:
                print(f"  [{sender}] passthrough -> {command}")
            return

        # Heartbeat is outbound-only; ignore it if it arrives inbound.
        if value == UDP_HEARTBEAT_REQUEST_VALUE:
            if self.verbose:
                print(f"  [{sender}] heartbeat {value} is outbound-only, ignored")
            return

        # Above the threshold the integer is a return-port announcement.
        if value > UDP_CONNECTION_PORT_THRESHOLD:
            if value > UDP_CONNECTION_PORT_MAX:
                print(f"  [{sender}] rejected: port must be "
                      f"<= {UDP_CONNECTION_PORT_MAX}")
                return
            self.register_return_port(sender_ip, value)
            return

        commands = self.command_map.get(value)
        if commands is None:
            self.unmapped += 1
            if self.verbose:
                print(f"  [{sender}] {value:+d} -> (unmapped)")
            return

        if isinstance(commands, str):
            commands = [commands]
        self._dispatch(commands, value)

        if self.verbose:
            print(f"  [{sender}] {value:+d} -> {' | '.join(commands)}")

    def _dispatch(self, commands, ack_value):
        """Write a command batch and queue the ack it will retire.

        The pose query rides at the END of the batch on purpose: it must not
        delay the move, and its reply is then the last one in, so the pose is
        current by the time the ack goes out.
        """
        commands = list(commands)
        if self.pose_ack:
            commands.append(POSE_QUERY)
        for command in commands:
            self.send(command)
        self.dispatched += 1

        # Queue the ack; it is emitted once the device answers. Each command
        # produces one reply, so a multi-command mapping acks on the last.
        if self.return_addr is not None:
            if len(self._pending) >= MAX_PENDING_ACKS:
                self._pending.popleft()
                self.dropped_acks += 1
            self._pending.append([ack_value, len(commands)])

    # -- upstream (serial -> UDP) --------------------------------------

    def drain_replies(self):
        """Consume device replies, echoing an ack upstream for each command."""
        if self.link_down:
            return                               # nothing to drain, avoid spam
        while True:
            reply = self.comm.receive()          # non-blocking
            if not reply:
                return
            reply_lines = [line.strip() for line in reply.splitlines()
                           if line.strip()]
            outcome_lines = []
            solicited_lines = []
            for line in reply_lines:
                if self.echo_replies:
                    print(f"      <- {line}")
                if line.startswith(GESTURE_RESULT_PREFIX):
                    outcome_lines.append(line)
                else:
                    solicited_lines.append(line)

            # A pose reply is SOLICITED -- it answers the query appended to the
            # batch -- so it is recorded and then falls through to retire its
            # slot like any other reply. A standalone unsolicited outcome skips
            # retirement; an outcome coalesced with this reply does not hide it.
            if GESTURE_ANGLES_PREFIX in reply:
                pose = parse_gesture_angle_pairs(reply)
                if pose:
                    self.pose = pose

            # GESTURE_RESULT has no command delimiter. DualSerialComm therefore
            # retains it as pending telemetry until the NEXT delimited command
            # reply arrives, and publishes both lines in one frame. Forward the
            # outcome, but do not discard the solicited reply sharing its frame.
            # A standalone outcome (the mock transport and simple test stubs can
            # produce one) still must not retire a pending command.
            for line in outcome_lines:
                self.outcomes += 1
                self.send_text_upstream(line)
            if not solicited_lines:
                continue
            self._retire_pending()

    def _retire_pending(self):
        """Account one device reply against the oldest outstanding command."""
        if not self._pending:
            return
        entry = self._pending[0]
        entry[1] -= 1
        if entry[1] > 0:
            return
        self._pending.popleft()
        if self.send_upstream(entry[0]):
            self.acked += 1
            # The pose follows the integer, so a consumer that only parses
            # integers sees exactly the traffic it always did.
            sent_pose = self.send_pose_upstream(entry[0])
            if self.verbose:
                detail = f" [{self._format_pose(self.pose)}]" if sent_pose else ""
                print(f"      -> ack {entry[0]:+d} to "
                      f"{self.return_addr[0]}:{self.return_addr[1]}{detail}")

    def print_summary(self):
        print(f"\n  datagrams received : {self.received}")
        print(f"  commands dispatched: {self.dispatched}")
        print(f"  acks sent upstream : {self.acked}")
        print(f"  pose frames sent   : {self.pose_acks}"
              + ("" if self.pose_ack else "  (disabled: firmware < 0.6.1)"))
        print(f"  move outcomes      : {self.outcomes}")
        print(f"  unmapped values    : {self.unmapped}")
        print(f"  malformed payloads : {self.malformed}")
        if self.dropped_acks:
            print(f"  acks dropped       : {self.dropped_acks} "
                  f"(device replies never arrived)")
        if self.send_failures or self.reconnects:
            print(f"  serial write fails : {self.send_failures}")
            print(f"  link reconnects    : {self.reconnects}")
        if self.return_addr is None:
            print("  return port        : never registered "
                  f"(send a value > {UDP_CONNECTION_PORT_THRESHOLD} to register)")
        else:
            print(f"  return port        : "
                  f"{self.return_addr[0]}:{self.return_addr[1]}")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Map UDP integers to exo gesture commands until Ctrl-C."
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
                        help="Skip the home:all sent after arming. By default "
                             "the hand is driven to its zero pose first so "
                             "gestures start from a known position.")
    parser.add_argument("--current-ma", type=int, default=DEFAULT_CURRENT_MA,
                        metavar="MA",
                        help=f"Per-motor working current in mA "
                             f"(default {DEFAULT_CURRENT_MA}). What ONE joint "
                             f"may push with; the combined budget bounds the "
                             f"fleet. 0 leaves firmware defaults alone.")
    parser.add_argument("--total-current-ma", type=int,
                        default=DEFAULT_TOTAL_CURRENT_MA, metavar="MA",
                        help=f"Combined current budget across all motors in mA "
                             f"(default {DEFAULT_TOTAL_CURRENT_MA}). Size this "
                             f"for the SUPPLY -- it is what stops a whole-hand "
                             f"posture browning out the board. Needs firmware "
                             f">= 0.4.0. 0 leaves the firmware default alone.")
    parser.add_argument("--debug-on", action="store_true",
                        help="Leave firmware VERBOSE enabled (slower commands).")
    parser.add_argument("--ignore-replies", dest="echo_replies",
                        action="store_false",
                        help="Do not print replies coming back on the telemetry "
                             "port. They are still drained from the queue.")
    parser.add_argument("--no-pose-ack", dest="pose_ack", action="store_false",
                        help="Do not query the joint positions after each "
                             "command. By default every ack is followed by a "
                             "packed frame carrying each joint's percentage "
                             "code and rest-zeroed signed degree angle, "
                             "which costs one extra device round trip per "
                             "command and needs firmware >= 0.6.1.")
    parser.add_argument("--quiet", action="store_true",
                        help="Do not log each datagram.")
    parser.add_argument("--mock", action="store_true",
                        help="Run with no exo attached. Uses an in-process fake "
                             "device that replies like the firmware, so the UDP "
                             "mapping and ack loopback can be exercised offline.")
    parser.add_argument("--mock-latency-ms", type=float,
                        default=MOCK_LATENCY_MS, metavar="MS",
                        help=f"Simulated device turnaround for --mock "
                             f"(default {MOCK_LATENCY_MS:g}).")
    parser.set_defaults(arm=True, home=True, pose_ack=True)
    args = parser.parse_args(argv)

    if args.mock:
        comm = MockComm(latency_ms=args.mock_latency_ms, log=not args.quiet)
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

    receiver = Receiver(comm, COMMAND_MAP, args.echo_replies, not args.quiet,
                        pose_ack=args.pose_ack)

    # SIGINT flips a flag rather than raising, so the socket timeout lets the
    # loop unwind through its normal cleanup path.
    running = {"go": True}

    def on_sigint(_signum, _frame):
        running["go"] = False
        print("\nSIGINT received, shutting down...")

    try:
        signal.signal(signal.SIGINT, on_sigint)
    except ValueError:
        # Only the main thread may install handlers. Running embedded is fine:
        # KeyboardInterrupt still unwinds through the cleanup in `finally`.
        pass

    sock = None
    armed = False

    def apply_session_setup():
        """Quiet the firmware and restore arm state.

        Also used after a reconnect: a device reset restores firmware defaults
        (VERBOSE back on, reply route back to BOTH), so the session setup has
        to be re-sent rather than assumed.
        """
        if not args.debug_on:
            for command in QUIET_COMMANDS:
                receiver.send(command)
                time.sleep(0.5)
            comm.flush_input()
        # Before anything is armed: a reset restores firmware defaults, so
        # re-probe rather than assume the capability survived the reconnect.
        receiver.pose_ack = args.pose_ack
        receiver.probe_pose_support()
        # Both current settings go out BEFORE enabling torque, so the motors
        # never energize at whatever the firmware happened to boot with. The
        # combined budget goes first: it constrains the per-motor value, so
        # setting it second would leave a window at the wider allocation.
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
            # Home only makes sense with torque on, so it follows the enable.
            if args.home:
                for command in HOME_COMMANDS:
                    receiver.send(command)
                    time.sleep(0.5)
                # The settle exists to let real motors finish travelling; with
                # no hardware there is nothing to wait for.
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
        receiver.sock = sock   # same socket carries acks back upstream

        print(f"Listening on {args.host}:{args.port}  "
              f"({len(COMMAND_MAP)} mapped values)  -- Ctrl-C to stop")
        print(f"Send an integer > {UDP_CONNECTION_PORT_THRESHOLD} first to "
              f"register the port acks should return on.\n")

        while running["go"]:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                receiver.drain_replies()
                continue
            except ConnectionResetError:
                # Windows surfaces an ICMP port-unreachable from a PREVIOUS
                # sendto as WSAECONNRESET on the next recv. That means the
                # registered return port is not listening -- it must not kill
                # the receiver, which is still perfectly able to take commands.
                if receiver.return_addr is not None:
                    print(f"  [ack] {receiver.return_addr[0]}:"
                          f"{receiver.return_addr[1]} is not listening",
                          file=sys.stderr)
                receiver.drain_replies()
                continue
            except OSError:
                break
            payload = data.decode("utf-8", errors="ignore").strip()
            if payload:
                receiver.handle(payload, f"{addr[0]}:{addr[1]}")
            receiver.drain_replies()

            if receiver.link_down:
                # The device dropped off the bus -- commonly a reset or a
                # brownout under motor load, which re-enumerates the CDC and
                # invalidates the COM handle. Recover rather than exit: the
                # session is still useful, and we need a live link to disarm.
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
            # Close notice goes out BEFORE the socket closes, so the sender
            # learns this endpoint is gone instead of timing out.
            try:
                receiver.send_close_notice()
            except Exception:
                pass
            try:
                sock.close()
            except OSError:
                pass
        if armed:
            # Release torque on every exit path, including an exception.
            # Disarming is the one thing worth fighting for: a dead handle must
            # not leave the hand energized. Try, and if the link is (or goes)
            # down mid-write, restore it and try again -- the link can die ON
            # the disarm write itself, not just before it.
            receiver.on_reconnect = None   # never re-arm on the way out
            try:
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
        receiver.print_summary()
        comm.close()


if __name__ == "__main__":
    sys.exit(main())
