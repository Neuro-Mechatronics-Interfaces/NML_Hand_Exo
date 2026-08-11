import collections
import queue
import socket
import serial
import asyncio
import threading
import time


def _read_framed(dev, delimiter: bytes, timeout: float) -> bytes:
    """Read from ``dev`` until ``delimiter`` arrives or ``timeout`` elapses.

    Uses pyserial's C-level ``read_until`` rather than a byte-at-a-time Python
    loop.  With verbose firmware the telemetry stream carries unterminated
    debug lines, and a per-byte loop cannot drain that backlog fast enough to
    reach the reply's delimiter before the timeout expires.
    """
    original = dev.timeout
    try:
        dev.timeout = timeout
        return dev.read_until(delimiter)
    finally:
        dev.timeout = original


class BaseComm:
    def connect(self): pass
    def disconnect(self): pass
    def send(self, message: str): pass
    def receive(self) -> str: pass
    def is_connected(self) -> bool: pass

    def flush_input(self):
        """Discard buffered inbound bytes so the next read starts clean."""
        pass

    def fast_telemetry_device(self):
        """Return the raw byte stream carrying fast telemetry, if supported."""
        return None


class TCPComm(BaseComm):
    def __init__(self, ip, port=5001, timeout=5, verbose=False):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.verbose = verbose

    def connect(self):
        try:
            if self.verbose:
                print(f"Attempting to connect to {self.ip}:{self.port} with timeout {self.timeout} seconds")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.ip, self.port))
            if self.verbose:
                print("Connection established")
        except socket.error as e:
            raise ConnectionError(f"Failed to connect to {self.ip}:{self.port} - {e}")


    def close(self):
        if self.sock:
            self.sock.close()

    def send(self, message: str):
        self.sock.sendall(message.encode())

    def receive(self) -> str:
        return self.sock.recv(1024).decode().strip()

    def is_connected(self) -> bool:
        return self.sock is not None


class SerialComm(BaseComm):
    def __init__(
        self, port, baudrate, command_delimiter=';', timeout=1,
        response_timeout=2.0, verbose=False,
    ):
        self.port = port
        self.baudrate = baudrate
        self.command_delimiter = command_delimiter
        self.timeout = timeout
        self.response_timeout = response_timeout
        self.verbose = verbose
        self.device = None

    def connect(self):
        self.device = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def close(self):
        if self.device and self.device.is_open:
            self.device.close()

    def send(self, message: str):
        self.device.write(message.encode())

    def receive(self, wait_until_return=False, timeout=None) -> str:
        """
            Reads data from the serial device. If `wait_until_return` is True,
            waits for a command delimiter until the timeout is reached.

            Args:
                wait_until_return (bool): Whether to wait for a full response ending with delimiter.
                timeout (float): Maximum time to wait (in seconds) for complete response.

            Returns:
                str: Decoded and cleaned response string.
            """
        try:
            if not self.device or not self.device.is_open:
                raise ConnectionError("Serial device is not connected")

            if wait_until_return:
                timeout = self.response_timeout if timeout is None else timeout
                if self.verbose:
                    print("Waiting for complete response from serial device...")

                response = _read_framed(
                    self.device, self.command_delimiter.encode(), timeout
                )

                if not response.endswith(self.command_delimiter.encode()):
                    print(f"[Warning] Incomplete response or timeout after {timeout} seconds")

                # Decode and clean up
                return response.decode(errors="ignore").replace(self.command_delimiter, '\n').strip()

            else:
                # Non-blocking mode: read everything currently available
                if self.verbose:
                    print("Reading available data from serial device (non-blocking)")

                if self.device.in_waiting > 0:
                    response = self.device.read(self.device.in_waiting)
                    return response.decode(errors="ignore").replace(self.command_delimiter, '\n').strip()

                return ""  # Nothing available

        except Exception as e:
            raise ConnectionError(
                f"Failed to read from serial device: {e}"
            ) from e


    def is_connected(self) -> bool:
        return self.device and self.device.is_open

    def flush_input(self):
        if self.device and self.device.is_open:
            self.device.reset_input_buffer()

    def fast_telemetry_device(self):
        return self.device


class DualSerialComm(BaseComm):
    """Two USB-CDC (ACM) ports from one device on a single cable.

    Commands are written on the *command* port and replies / telemetry are read
    from the *telemetry* port, so command writes never wait behind telemetry
    reads (the device-side fix for host head-of-line blocking).  The ASCII line
    protocol and framing are identical to :class:`SerialComm`; this only splits
    the transport across two COM ports.  Requires the dual-CDC OpenRB-150
    firmware (see the ``set_reply_route`` command).

    ``connect()`` probes the two ports to determine direction, so it is robust
    to COM-number / USB-interface ordering: whichever port answers is used as
    the command port.  It then switches the firmware to ``reply_route:telem`` so
    the command port carries no return traffic (full decoupling).

    Reads are decoupled from writes by a background reader thread.  The thread
    drains the telemetry port continuously and splits the stream into frames:
    a delimited *reply* goes on a queue, and any unsolicited telemetry / debug
    line is kept separately.  Nothing on the command path ever performs an
    inline blocking read of the port, so a chatty device cannot stall a write
    and a backlog cannot accumulate.  :meth:`receive` waits on the queue, which
    returns immediately when a reply has already landed.
    """

    #: Cap on retained unsolicited lines, so a chatty device cannot grow memory
    #: without bound when nobody is consuming telemetry.
    MAX_TELEMETRY_LINES = 1000

    #: Cap on queued reply frames. A long-running process that sends commands
    #: without reading replies would otherwise grow this queue forever; past
    #: the cap the oldest frame is dropped, since the newest is what a caller
    #: waiting on a fresh transaction actually wants.
    MAX_QUEUED_REPLIES = 256

    def __init__(
        self, cmd_port, telem_port, baudrate, command_delimiter=';', timeout=1,
        response_timeout=2.0, verbose=False, line_terminator='\r\n',
    ):
        self.cmd_port = cmd_port
        self.telem_port = telem_port
        self.baudrate = baudrate
        self.command_delimiter = command_delimiter
        self.timeout = timeout
        self.response_timeout = response_timeout
        self.verbose = verbose
        self.line_terminator = line_terminator
        self._cmd = None
        self._telem = None
        self._replies = queue.Queue(maxsize=self.MAX_QUEUED_REPLIES)
        self._telemetry = collections.deque(maxlen=self.MAX_TELEMETRY_LINES)
        self._reader = None
        self._reader_error = None
        self._stop = threading.Event()

    def connect(self):
        self._cmd = serial.Serial(self.cmd_port, self.baudrate, timeout=self.timeout)
        self._telem = serial.Serial(self.telem_port, self.baudrate, timeout=self.timeout)
        # Let the CDC endpoints settle after the DTR assert that open() triggers.
        time.sleep(0.2)
        self._cmd.reset_input_buffer()
        self._telem.reset_input_buffer()

        # Decouple first: put the device in split mode so replies go ONLY to its
        # telemetry CDC.  Both CDCs accept commands, so this works regardless of
        # which physical port we happened to open as "command".
        self._write_line(self._cmd, "set_reply_route:telem")
        time.sleep(0.1)
        self._cmd.reset_input_buffer()
        self._telem.reset_input_buffer()

        # With replies now bound to the device's telemetry CDC, discover which
        # physical port that is: send a query on one port and see which port the
        # reply lands on.  Exactly one orientation answers; swap if reversed.
        if self._probe(self._cmd, self._telem):
            pass
        elif self._probe(self._telem, self._cmd):
            self._cmd, self._telem = self._telem, self._cmd
            self.cmd_port, self.telem_port = self.telem_port, self.cmd_port
        else:
            self.close()
            raise ConnectionError(
                f"Dual-CDC probe failed: no reply on either of {self.cmd_port} / "
                f"{self.telem_port}. Confirm both COM ports belong to the same "
                "device and the dual-CDC firmware is flashed."
            )

        self._cmd.reset_input_buffer()
        self._telem.reset_input_buffer()

        # Probing is done, so hand the telemetry port to the reader thread. From
        # here nothing else may read it directly.
        self._start_reader()

        if self.verbose:
            print(f"[DualSerialComm] cmd={self.cmd_port} telem={self.telem_port}")

    def _start_reader(self):
        """Take ownership of the telemetry port with the background reader."""
        self._stop.clear()
        self._reader = threading.Thread(
            target=self._drain_telemetry,
            name=f"DualSerialComm-reader-{self.telem_port}",
            daemon=True,
        )
        self._reader.start()

    def _drain_telemetry(self):
        """Continuously read the telemetry port and split it into frames.

        Runs for the life of the connection.  A line ending in the command
        delimiter closes the current reply frame and publishes it; every other
        line is unsolicited telemetry / debug output and is retained separately
        so it can never delay or be mistaken for a reply.
        """
        delimiter = self.command_delimiter
        pending = []
        buffer = b""
        while not self._stop.is_set():
            try:
                chunk = self._telem.read(max(1, self._telem.in_waiting))
            except Exception as exc:
                # Only a close() is expected to land here. Anything else means
                # the reply path is dead, and a silent exit would be
                # indistinguishable from a device that simply never answers --
                # so record it and let receive() surface it.
                if not self._stop.is_set():
                    self._reader_error = exc
                break
            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                line = raw.decode(errors="ignore").strip("\r\n")
                if not line.strip():
                    continue
                if delimiter in line:
                    # Terminator seen: this line closes the reply frame.
                    head, _, tail = line.partition(delimiter)
                    pending.append(head)
                    self._publish_reply(
                        "\n".join(part for part in pending if part.strip()).strip()
                    )
                    pending = []
                    if tail.strip():
                        pending.append(tail)
                else:
                    # Either a continuation of a multi-line reply or unsolicited
                    # output. Keep it in both places: the frame under
                    # construction, and the telemetry log for observability.
                    pending.append(line)
                    self._telemetry.append(line)

    def _publish_reply(self, frame: str):
        """Queue a reply frame, discarding the oldest if the cap is reached."""
        try:
            self._replies.put_nowait(frame)
            return
        except queue.Full:
            pass
        try:
            self._replies.get_nowait()
        except queue.Empty:
            pass
        try:
            self._replies.put_nowait(frame)
        except queue.Full:
            pass

    def telemetry_lines(self):
        """Snapshot of recent unsolicited telemetry / debug lines."""
        return list(self._telemetry)

    def reader_alive(self) -> bool:
        """True while the background telemetry reader is running."""
        return bool(self._reader and self._reader.is_alive())

    def _write_line(self, dev, text: str):
        if not text.endswith(self.line_terminator):
            text = text + self.line_terminator
        dev.write(text.encode())

    def _probe(self, cmd_dev, telem_dev, timeout: float = 1.0) -> bool:
        """Send a benign query on ``cmd_dev`` and check for any reply on
        ``telem_dev`` within ``timeout`` seconds."""
        cmd_dev.reset_input_buffer()
        telem_dev.reset_input_buffer()
        self._write_line(cmd_dev, "version")
        start = time.time()
        while time.time() - start < timeout:
            if telem_dev.in_waiting > 0:
                return True
            time.sleep(0.01)
        return False

    def close(self):
        self._stop.set()
        reader, self._reader = self._reader, None
        if reader is not None and reader.is_alive():
            # The read() call honours the port timeout, so the thread observes
            # the stop flag within one timeout period.
            reader.join(timeout=max(1.0, (self.timeout or 1.0) * 2))
        for dev in (self._cmd, self._telem):
            try:
                if dev and dev.is_open:
                    dev.close()
            except Exception:
                pass

    def send(self, message: str):
        # Write and return. The reply lands on the telemetry port and is picked
        # up by the reader thread, so this never waits on return traffic.
        self._cmd.write(message.encode())

    def receive(self, wait_until_return=False, timeout=None) -> str:
        """Return the next reply frame collected by the reader thread.

        This never touches the serial port: the reader thread owns it. Waiting
        here blocks only this caller, and only until a frame is queued -- the
        telemetry port keeps draining regardless, so a slow or absent reply can
        never back up the link or stall a subsequent write.
        """
        try:
            if not self.is_connected():
                raise ConnectionError("Telemetry serial port is not connected")

            if wait_until_return:
                timeout = self.response_timeout if timeout is None else timeout
                try:
                    return self._replies.get(timeout=timeout)
                except queue.Empty:
                    if self._reader_error is not None:
                        print(f"[Error] Telemetry reader stopped: "
                              f"{self._reader_error}")
                    elif not self.reader_alive():
                        print("[Error] Telemetry reader is not running; no "
                              "replies can be received.")
                    else:
                        print(f"[Warning] No delimited reply within "
                              f"{timeout} seconds")
                    return ""

            # Non-blocking: hand back a queued frame if one is already waiting.
            try:
                return self._replies.get_nowait()
            except queue.Empty:
                return ""
        except Exception as e:
            raise ConnectionError(
                f"Failed to read from telemetry serial port: {e}"
            ) from e

    def is_connected(self) -> bool:
        return bool(
            self._cmd and self._cmd.is_open and self._telem and self._telem.is_open
        )

    def flush_input(self):
        """Drop replies queued before this point.

        Called before a send so a transaction cannot be handed a stale frame
        left over from an earlier timed-out request.  The port itself is not
        reset -- the reader thread owns it, and resetting underneath it would
        truncate a frame mid-parse.
        """
        while True:
            try:
                self._replies.get_nowait()
            except queue.Empty:
                break

    def fast_telemetry_device(self):
        """Return the CDC stream used by the current binary-frame firmware.

        Text replies follow ``reply_route:telem`` and are drained from
        ``_telem`` by the background reader.  The current firmware writes the
        compact ``NX`` frame directly to the primary command CDC, which has no
        competing reader and can therefore be consumed synchronously here.
        """
        return self._cmd

