#!/usr/bin/env python3
"""
Manual UDP sender for examples/scripts/udp_gesture_receiver.py.

A small tkinter panel that speaks the receiver's integer protocol so gestures
can be triggered by hand -- useful for checking wiring, calibration and travel
without a decoder, an EMG rig or the full Qt GUI in the loop.

Defaults assume the receiver is running on this machine with its own default
bind port, so `python examples/scripts/udp_gesture_gui.py` next to a default
`python examples/scripts/udp_gesture_receiver.py` needs no arguments.
Destination host and port are editable in the window for a receiver running
elsewhere.

Protocol (see Receiver in udp_gesture_receiver.py):
  * Connecting first sends this GUI's own listen port as an integer. Any value
    above UDP_CONNECTION_PORT_THRESHOLD is read as a return-port announcement
    rather than a command, and the receiver echoes it back as a wake-up ACK.
  * Command integers are then acked by echoing them back, but only once the
    DEVICE has actually replied -- an ack here means the exo executed it, not
    merely that the datagram arrived.
  * A packed binary pose frame follows each ack, carrying where all seven
    joints now sit as percentages of their calibrated travel. A joint whose
    percentage never changes -- or that reports no position at all -- is one
    the firmware accepted a command for and could not actually move.
  * The negated port arriving means the receiver is shutting down.

The command values come from the receiver module itself, so the two cannot
drift apart: edit build_command_map() there and this panel follows.

Usage:
    python examples/scripts/udp_gesture_gui.py
    python examples/scripts/udp_gesture_gui.py --host 192.168.1.50 --port 10003
    python examples/scripts/udp_gesture_gui.py --local-port 10005
"""

import argparse
import os
import queue
import socket
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

# The receiver lives beside this file and is the single source of truth for the
# integer -> command mapping; import it rather than restating the values.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from udp_gesture_receiver import (  # noqa: E402
    COMMAND_MAP,
    COMMAND_PASSTHROUGH_ACK,
    GESTURE_RESULT_PREFIX,
    FLEX_PERCENT,
    JOINTS,
    POSE_ACK_MAGIC,
    POSE_UNAVAILABLE,
    REST_VALUE_OFFSET,
    UDP_PORT as RECEIVER_DEFAULT_PORT,
    unpack_pose_ack,
)
from nml_hand_exo.interface._udp_command_bindings import (  # noqa: E402
    UDP_CONNECTION_PORT_MAX,
    UDP_CONNECTION_PORT_THRESHOLD,
)

DEFAULT_DEST_HOST = "127.0.0.1"

#: Port this GUI listens on for acks. Must exceed UDP_CONNECTION_PORT_THRESHOLD
#: or the receiver would read the announcement as a command value.
DEFAULT_LOCAL_PORT = 10004

#: How often the socket thread checks the stop flag, and how often the Tk main
#: loop drains the inbound queue. Tk is not thread-safe, so the listener never
#: touches a widget: it queues, and the main thread renders.
SOCKET_TIMEOUT_S = 0.25
QUEUE_POLL_MS = 50

LOG_LINE_LIMIT = 500


class UdpGestureGui:
    """Tkinter panel that sends receiver command integers and shows acks."""

    def __init__(self, root, dest_host, dest_port, local_port):
        self.root = root
        self.sock = None
        self.listener = None
        self.stop_flag = threading.Event()
        self.inbox = queue.Queue()
        self.sent_count = 0
        self.ack_count = 0

        root.title("NML Hand Exo -- UDP Gesture Sender")
        root.minsize(560, 520)

        self.dest_host_var = tk.StringVar(value=dest_host)
        self.dest_port_var = tk.StringVar(value=str(dest_port))
        self.local_port_var = tk.StringVar(value=str(local_port))
        self.status_var = tk.StringVar(value="Not connected")

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(QUEUE_POLL_MS, self._drain_inbox)

    # -- UI ------------------------------------------------------------

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        conn = ttk.LabelFrame(outer, text="Receiver", padding=8)
        conn.pack(fill="x")

        ttk.Label(conn, text="Host").grid(row=0, column=0, sticky="w")
        ttk.Entry(conn, textvariable=self.dest_host_var, width=16).grid(
            row=0, column=1, padx=(4, 12))
        ttk.Label(conn, text="Port").grid(row=0, column=2, sticky="w")
        ttk.Entry(conn, textvariable=self.dest_port_var, width=8).grid(
            row=0, column=3, padx=(4, 12))
        ttk.Label(conn, text="Listen on").grid(row=0, column=4, sticky="w")
        ttk.Entry(conn, textvariable=self.local_port_var, width=8).grid(
            row=0, column=5, padx=(4, 12))

        self.connect_btn = ttk.Button(conn, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=6)

        ttk.Label(conn, textvariable=self.status_var, foreground="#555").grid(
            row=1, column=0, columnspan=7, sticky="w", pady=(6, 0))

        joints = ttk.LabelFrame(outer, text="Per-joint position", padding=8)
        joints.pack(fill="x", pady=(10, 0))

        for col, heading in enumerate(("Joint", "Extend", "Rest", "Flex")):
            ttk.Label(joints, text=heading, font=("", 9, "bold")).grid(
                row=0, column=col, padx=4, pady=(0, 4), sticky="w")
        # Spans the entry and its button.
        ttk.Label(joints, text="Manual %", font=("", 9, "bold")).grid(
            row=0, column=4, columnspan=2, padx=4, pady=(0, 4), sticky="w")

        #: Per-joint percent entry backing the manual set_gesture_angle column.
        self.angle_vars = {}

        for row, joint in enumerate(JOINTS, start=1):
            value = row  # JOINTS order defines the integer, same as the receiver
            ttk.Label(joints, text=joint).grid(row=row, column=0, sticky="w", padx=4)
            for col, val in enumerate(
                (-value, value + REST_VALUE_OFFSET, value), start=1
            ):
                ttk.Button(
                    joints,
                    text=f"{val:+d}",
                    width=8,
                    command=lambda v=val: self.send_value(v),
                ).grid(row=row, column=col, padx=4, pady=1)

            # Manual percentage: 0 = extend/home, 100 = the flexion endstop.
            # Prefilled with this joint's flex percent so the field starts on a
            # value that is known to be safe for it rather than empty or at 100.
            var = tk.StringVar(value=str(FLEX_PERCENT[joint]))
            self.angle_vars[joint] = var
            entry = ttk.Entry(joints, textvariable=var, width=6, justify="right")
            entry.grid(row=row, column=4, padx=(12, 2), pady=1)
            entry.bind("<Return>", lambda _e, j=joint: self.send_joint_angle(j))
            ttk.Button(
                joints,
                text="Send",
                width=6,
                command=lambda j=joint: self.send_joint_angle(j),
            ).grid(row=row, column=5, padx=(2, 4), pady=1)

        joints.grid_columnconfigure(0, minsize=70)

        manual = ttk.LabelFrame(outer, text="Manual", padding=8)
        manual.pack(fill="x", pady=(10, 0))

        ttk.Button(manual, text="Open hand (0)", width=16,
                   command=lambda: self.send_value(0)).grid(row=0, column=0, padx=(0, 12))

        ttk.Label(manual, text="Value").grid(row=0, column=1)
        self.manual_var = tk.StringVar()
        manual_entry = ttk.Entry(manual, textvariable=self.manual_var, width=10)
        manual_entry.grid(row=0, column=2, padx=4)
        manual_entry.bind("<Return>", lambda _event: self.send_manual())
        ttk.Button(manual, text="Send", command=self.send_manual).grid(row=0, column=3)

        ttk.Label(
            manual,
            text=(f"Values above {UDP_CONNECTION_PORT_THRESHOLD} register a "
                  f"return port, they are not commands."),
            foreground="#777",
        ).grid(row=1, column=0, columnspan=4, sticky="w", pady=(6, 0))

        log_frame = ttk.LabelFrame(outer, text="Log", padding=8)
        log_frame.pack(fill="both", expand=True, pady=(10, 0))

        self.log_text = tk.Text(log_frame, height=12, wrap="none", state="disabled")
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

    def log(self, message):
        stamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{stamp}] {message}\n")
        # Trim from the top so a long session cannot grow the widget without
        # bound; Text keeps every line in memory otherwise.
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > LOG_LINE_LIMIT:
            self.log_text.delete("1.0", f"{line_count - LOG_LINE_LIMIT}.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _set_status(self, text):
        self.status_var.set(text)

    # -- connection ----------------------------------------------------

    def toggle_connection(self):
        if self.sock is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        try:
            local_port = self._validated_port(self.local_port_var.get(), "Listen port")
            dest_port = self._validated_port(self.dest_port_var.get(), "Receiver port")
        except ValueError as exc:
            self.log(f"[error] {exc}")
            return
        if local_port <= UDP_CONNECTION_PORT_THRESHOLD:
            self.log(f"[error] Listen port must exceed "
                     f"{UDP_CONNECTION_PORT_THRESHOLD}, or the receiver reads "
                     f"the announcement as a command value.")
            return

        host = self.dest_host_var.get().strip() or DEFAULT_DEST_HOST
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(SOCKET_TIMEOUT_S)
            # One socket for both directions: the port announced to the receiver
            # is then necessarily the port acks arrive on.
            sock.bind(("", local_port))
        except OSError as exc:
            self.log(f"[error] Could not bind local port {local_port}: {exc}")
            return

        self.sock = sock
        self.stop_flag.clear()
        self.listener = threading.Thread(target=self._listen, daemon=True)
        self.listener.start()

        self.connect_btn.configure(text="Disconnect")
        self._set_status(f"Listening on :{local_port}, sending to {host}:{dest_port}")
        self.log(f"Bound :{local_port}; target {host}:{dest_port}")

        # Announce the return port. The receiver echoes it back as a wake-up ACK.
        self.send_value(local_port, announce=True)

    def disconnect(self):
        self.stop_flag.set()
        if self.listener is not None:
            self.listener.join(timeout=1.0)
            self.listener = None
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        self.connect_btn.configure(text="Connect")
        self._set_status("Not connected")
        self.log("Disconnected")

    @staticmethod
    def _validated_port(text, label):
        try:
            port = int(str(text).strip())
        except (TypeError, ValueError):
            raise ValueError(f"{label} must be an integer, got {text!r}")
        if not 1 <= port <= UDP_CONNECTION_PORT_MAX:
            raise ValueError(f"{label} must be 1..{UDP_CONNECTION_PORT_MAX}")
        return port

    # -- sending -------------------------------------------------------

    def _sendto(self, payload, description):
        """Send one datagram. Returns True if it went out."""
        if self.sock is None:
            self.log("[error] Not connected -- press Connect first")
            return False
        try:
            dest_port = self._validated_port(self.dest_port_var.get(), "Receiver port")
        except ValueError as exc:
            self.log(f"[error] {exc}")
            return False
        host = self.dest_host_var.get().strip() or DEFAULT_DEST_HOST

        try:
            self.sock.sendto(str(payload).encode("ascii"), (host, dest_port))
        except OSError as exc:
            self.log(f"[error] send {payload!r} failed: {exc}")
            return False

        self.sent_count += 1
        self.log(f"-> {description}")
        return True

    def send_value(self, value, announce=False):
        value = int(value)
        if announce:
            description = f"{value} (return-port announcement)"
        else:
            mapped = COMMAND_MAP.get(value)
            detail = mapped if mapped else "(unmapped by the receiver)"
            description = f"{value:+d}  {detail}"
        self._sendto(value, description)

    def send_joint_angle(self, joint):
        """Send `set_gesture_angle:<joint>:<percent>` for one joint.

        The integer protocol can only reach the positions the receiver's map was
        built with, so an arbitrary percentage travels as the command form,
        which the receiver validates and forwards.
        """
        text = self.angle_vars[joint].get().strip()
        if not text:
            self.log(f"[error] Enter a percent for {joint} first")
            return
        try:
            percent = float(text)
        except ValueError:
            self.log(f"[error] {joint}: percent must be a number, got {text!r}")
            return
        if not 0.0 <= percent <= 100.0:
            # Checked here as well as in the receiver: rejecting it locally says
            # so in this window, where the operator is looking.
            self.log(f"[error] {joint}: percent must be 0-100, got {percent:g}")
            return
        command = f"set_gesture_angle:{joint}:{percent:g}"
        self._sendto(command, command)

    def send_manual(self):
        text = self.manual_var.get().strip()
        if not text:
            return
        try:
            value = int(text)
        except ValueError:
            self.log(f"[error] Not an integer: {text!r}")
            return
        self.send_value(value)

    # -- receiving -----------------------------------------------------

    def _listen(self):
        """Socket thread: queue inbound datagrams for the Tk main loop."""
        while not self.stop_flag.is_set():
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except ConnectionResetError:
                # Windows reports an ICMP port-unreachable from a PREVIOUS
                # sendto on the next recv. It means the receiver is not
                # listening; it must not kill this thread.
                self.inbox.put(("error", "receiver is not listening at that address"))
                continue
            except OSError:
                break     # socket closed by disconnect()
            # Pose frames are binary, so they are dispatched on the raw bytes:
            # decoding them as text first would mangle the payload.
            if data.startswith(POSE_ACK_MAGIC):
                self.inbox.put(("pose", data))
                continue
            payload = data.decode("utf-8", errors="ignore").strip()
            if payload:
                self.inbox.put(("rx", (payload, f"{addr[0]}:{addr[1]}")))

    def _drain_inbox(self):
        """Tk main loop: render whatever the socket thread queued."""
        try:
            while True:
                kind, item = self.inbox.get_nowait()
                if kind == "error":
                    self.log(f"[error] {item}")
                    continue
                if kind == "pose":
                    self._handle_pose(item)
                    continue
                payload, sender = item
                self._handle_reply(payload, sender)
        except queue.Empty:
            pass
        self.root.after(QUEUE_POLL_MS, self._drain_inbox)

    def _handle_reply(self, payload, sender):
        if payload.startswith(GESTURE_RESULT_PREFIX):
            # The move verdict, not a command ack. This is what distinguishes
            # "the firmware accepted it" from "the joint actually got there".
            self._handle_outcome(payload)
            return
        try:
            value = int(payload)
        except ValueError:
            self.log(f"<- {payload!r} from {sender}")
            return

        try:
            local_port = int(self.local_port_var.get().strip())
        except (TypeError, ValueError):
            local_port = None

        if local_port is not None and value == -local_port:
            self.log(f"<- {value} -- receiver is shutting down")
            self._set_status("Receiver closed the session")
            return
        if local_port is not None and value == local_port:
            self.log(f"<- {value} -- registered, receiver is awake")
            self._set_status(f"Connected; acks arriving on :{local_port}")
            return

        self.ack_count += 1
        if value == COMMAND_PASSTHROUGH_ACK:
            # A passthrough has no integer of its own to echo, so the receiver
            # acks it with this sentinel.
            self.log("<- ack set_gesture_angle (device executed it)")
            return
        self.log(f"<- ack {value:+d} (device executed it)")

    def _handle_pose(self, data):
        """Render the packed pose frame that follows a command ack.

        This is the panel's answer to "did anything move?": a joint whose
        percentage does not change between commands, or that reports 255, is
        one the exo accepted a goal for and could not travel.
        """
        parsed = unpack_pose_ack(data)
        if parsed is None:
            self.log(f"<- unparseable pose frame ({len(data)} bytes)")
            return
        _, pose = parsed
        stuck = [joint for joint, code in pose.items() if code == POSE_UNAVAILABLE]
        rendered = " ".join(
            f"{joint}={'--' if pose[joint] == POSE_UNAVAILABLE else pose[joint]}"
            for joint in JOINTS
        )
        self.log(f"   pose: {rendered}")
        if stuck:
            self.log(f"   !! no calibrated travel: {', '.join(stuck)}")

    def _handle_outcome(self, payload):
        """Render a GESTURE_RESULT line and flag anything that did not move."""
        body = payload[len(GESTURE_RESULT_PREFIX):].strip()
        fields = {}
        for token in body.split():
            key, sep, value = token.partition("=")
            if sep:
                fields[key] = value

        def count(name):
            try:
                return int(fields.get(name, 0))
            except ValueError:
                return 0

        failed = count("stalled") + count("short") + count("starved")
        self.log(f"<= outcome: {body}")
        if failed:
            detail = fields.get("detail", "")
            self.log(f"   !! {failed} joint(s) did not reach target"
                     + (f" -- {detail}" if detail else ""))
            self._set_status(f"Last gesture: {failed} joint(s) did not reach target")
        else:
            self._set_status(f"Last gesture: {count('reached')} joint(s) reached target")

    # -- lifecycle -----------------------------------------------------

    def on_close(self):
        if self.sock is not None:
            self.disconnect()
        self.root.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Tkinter panel that sends gesture integers to "
                    "examples/scripts/udp_gesture_receiver.py."
    )
    parser.add_argument("--host", default=DEFAULT_DEST_HOST,
                        help=f"Receiver address (default {DEFAULT_DEST_HOST})")
    parser.add_argument("--port", type=int, default=RECEIVER_DEFAULT_PORT,
                        help=f"Receiver port (default {RECEIVER_DEFAULT_PORT}, "
                             f"the receiver's own default)")
    parser.add_argument("--local-port", type=int, default=DEFAULT_LOCAL_PORT,
                        help=f"Port this GUI listens on for acks "
                             f"(default {DEFAULT_LOCAL_PORT})")
    args = parser.parse_args(argv)

    root = tk.Tk()
    UdpGestureGui(root, args.host, args.port, args.local_port)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
