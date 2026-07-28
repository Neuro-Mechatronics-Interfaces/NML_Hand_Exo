import socket
import serial
import asyncio
import time


class BaseComm:
    def connect(self): pass
    def disconnect(self): pass
    def send(self, message: str): pass
    def receive(self) -> str: pass
    def is_connected(self) -> bool: pass


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

                response = b""
                start_time = time.time()

                while time.time() - start_time < timeout:
                    if self.device.in_waiting > 0:
                        byte = self.device.read(1)
                        response += byte
                        if byte == self.command_delimiter.encode():
                            break
                    else:
                        time.sleep(0.01)  # avoid tight loop

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
            print(f"[Error] Failed to read from serial device: {e}")
            return ""


    def is_connected(self) -> bool:
        return self.device and self.device.is_open


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
    """

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
        if self.verbose:
            print(f"[DualSerialComm] cmd={self.cmd_port} telem={self.telem_port}")

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
        for dev in (self._cmd, self._telem):
            try:
                if dev and dev.is_open:
                    dev.close()
            except Exception:
                pass

    def send(self, message: str):
        # HandExo already appended its command delimiter; write bytes verbatim.
        self._cmd.write(message.encode())

    def receive(self, wait_until_return=False, timeout=None) -> str:
        """Read a response from the telemetry port. Same framing as
        :meth:`SerialComm.receive`."""
        try:
            if not self._telem or not self._telem.is_open:
                raise ConnectionError("Telemetry serial port is not connected")

            if wait_until_return:
                timeout = self.response_timeout if timeout is None else timeout
                response = b""
                start_time = time.time()
                while time.time() - start_time < timeout:
                    if self._telem.in_waiting > 0:
                        byte = self._telem.read(1)
                        response += byte
                        if byte == self.command_delimiter.encode():
                            break
                    else:
                        time.sleep(0.01)
                if not response.endswith(self.command_delimiter.encode()):
                    print(f"[Warning] Incomplete response or timeout after {timeout} seconds")
                return response.decode(errors="ignore").replace(self.command_delimiter, '\n').strip()

            # Non-blocking: return whatever is currently available.
            if self._telem.in_waiting > 0:
                response = self._telem.read(self._telem.in_waiting)
                return response.decode(errors="ignore").replace(self.command_delimiter, '\n').strip()
            return ""
        except Exception as e:
            print(f"[Error] Failed to read from telemetry serial port: {e}")
            return ""

    def is_connected(self) -> bool:
        return bool(
            self._cmd and self._cmd.is_open and self._telem and self._telem.is_open
        )

