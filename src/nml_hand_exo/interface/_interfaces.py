import socket
import serial
import asyncio
import time


class BaseComm:
    def connect(self):
        raise NotImplementedError()

    def close(self):
        raise NotImplementedError()

    def disconnect(self):
        self.close()

    def send(self, message: str):
        raise NotImplementedError()

    def receive(self, wait_until_return: bool = False, timeout: float = 2.0) -> str:
        raise NotImplementedError()

    def is_connected(self) -> bool:
        raise NotImplementedError()


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
            self.sock = None

    def send(self, message: str):
        self.sock.sendall(message.encode())

    def receive(self, wait_until_return: bool = False, timeout: float = 2.0) -> str:
        if not self.sock:
            raise ConnectionError("TCP socket is not connected")

        if not wait_until_return:
            self.sock.settimeout(timeout)
            return self.sock.recv(4096).decode(errors="ignore").strip()

        chunks = []
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            self.sock.settimeout(min(0.1, max(remaining, 0.01)))
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                if chunks:
                    break
                continue

            if not chunk:
                break

            chunks.append(chunk)

        return b"".join(chunks).decode(errors="ignore").strip()

    def is_connected(self) -> bool:
        return self.sock is not None and self.sock.fileno() != -1


class SerialComm(BaseComm):
    def __init__(self, port, baudrate, command_delimiter=';', timeout=1, verbose=False):
        self.port = port
        self.baudrate = baudrate
        self.command_delimiter = command_delimiter
        self.timeout = timeout
        self.verbose = verbose
        self.device = None

    def connect(self):
        self.device = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def close(self):
        if self.device and self.device.is_open:
            self.device.close()

    def send(self, message: str):
        self.device.write(message.encode())

    def receive(self, wait_until_return=False, timeout=2.0) -> str:
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

