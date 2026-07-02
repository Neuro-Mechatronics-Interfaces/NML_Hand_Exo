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
        self.device = serial.serial_for_url(self.port, self.baudrate, timeout=self.timeout)

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

