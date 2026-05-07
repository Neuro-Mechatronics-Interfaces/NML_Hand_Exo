import re
import socket
import serial
import asyncio
import time
import threading
import logging
import platform

# Optional BLE support via bleak (pip install bleak).
# Only needed for BLE modules (e.g. HM-10); HC-05 Bluetooth Classic uses SerialComm.
try:
    import bleak
    _BLEAK_AVAILABLE = True
except ImportError:
    bleak = None
    _BLEAK_AVAILABLE = False

# Optional PyBluez2 for live Bluetooth Classic discovery and direct RFCOMM connections.
# pip install pybluez2
# Needed for RFCOMMComm and BluetoothScanner.
# Not needed for SerialComm/BluetoothComm (which use COM ports assigned by Windows).
try:
    import bluetooth as _pybluez
    _PYBLUEZ_AVAILABLE = True
except ImportError:
    _pybluez = None
    _PYBLUEZ_AVAILABLE = False

_log = logging.getLogger(__name__)


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
    def __init__(self, port, baudrate, command_delimiter=';', timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.command_delimiter = command_delimiter
        self.timeout = timeout
        self.device = None
        # RLock serializes all writes across threads.  Reentrant so that callers
        # holding the lock can still call send() without deadlocking.
        self._lock = threading.RLock()

    def connect(self):
        self.device = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def close(self):
        if self.device and self.device.is_open:
            self.device.close()

    def send(self, message: str):
        with self._lock:
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


def is_bluetooth_port(port_info) -> bool:
    """Return True if a ListPortInfo entry looks like a Bluetooth virtual COM port."""
    desc = (port_info.description or "").lower()
    hwid = (port_info.hwid or "").lower()
    keywords = ("bluetooth", "bthenum", "rfcomm", "hc-05", "hc05", "hc-06", "hc06")
    return any(kw in desc or kw in hwid for kw in keywords)


def bt_port_direction(port_info) -> str:
    """Return the direction of a Windows Bluetooth SPP COM port.

    When Windows pairs a Bluetooth Classic device it creates two COM ports:
    - **outgoing** (client): your PC initiates the connection — use this one.
    - **incoming** (server): the device would connect to you — not useful here.

    Windows encodes direction in the hardware ID suffix:
      ``_C00000000``  →  outgoing (client)
      ``_00000000``   →  incoming (server)

    Returns:
        ``"outgoing"``, ``"incoming"``, or ``"unknown"`` if the pattern is not
        present (e.g. on Linux/macOS or non-Windows BT stacks).
    """
    hwid = (port_info.hwid or "").upper()
    if hwid.endswith("_C00000000"):
        return "outgoing"
    if hwid.endswith("_00000000"):
        return "incoming"
    return "unknown"


def _bt_mac_from_hwid(hwid: str) -> str | None:
    """Extract the Bluetooth MAC address (12 lowercase hex chars) from a Windows BTHENUM HWID.

    BTHENUM HWIDs look like:
      ``BTHENUM\\{...}_LOCALMFG&...\...&000BE3149A6C_C00000000``
    The MAC sits between the last ``&`` and the ``_[C]00000000`` suffix.
    """
    m = re.search(r'&([0-9A-Fa-f]{12})_', hwid)
    return m.group(1).lower() if m else None


def _bt_device_name_windows(mac_hex: str) -> str | None:
    """Read a paired Bluetooth device's friendly name from the Windows registry.

    Windows stores the name under:
    ``HKLM\\SYSTEM\\CurrentControlSet\\Services\\BTHPORT\\Parameters\\Devices\\<mac>``

    The ``FriendlyName`` value is REG_BINARY encoded as UTF-16 LE.

    Returns the name string, or None on any failure (non-Windows, unpaired, registry miss).
    """
    if platform.system() != "Windows":
        return None
    try:
        import winreg
        key_path = (
            r"SYSTEM\CurrentControlSet\Services\BTHPORT\Parameters\Devices\\"
            + mac_hex.lower()
        )
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, key_path) as k:
            val, _ = winreg.QueryValueEx(k, "FriendlyName")
            if isinstance(val, bytes):
                return val.decode("utf-16-le").rstrip("\x00")
            return str(val)
    except Exception:
        return None


def _scan_bthenum_registry() -> list[dict]:
    """Enumerate paired BT Classic COM ports directly from the Windows BTHENUM registry.

    Supplements ``list_ports.comports()`` which can miss ports that are registered
    in Device Manager but not currently active.  Reads:

    * ``HKLM\\...\\BTHPORT\\Parameters\\Devices`` for MAC → friendly-name mapping
    * ``HKLM\\...\\Enum\\BTHENUM\\{svc}\\{instance}\\Device Parameters\\PortName``
      for the actual COM port assignment

    Returns the same dict schema as ``scan_paired_bluetooth_devices``.
    """
    if platform.system() != "Windows":
        return []
    results: list[dict] = []
    try:
        import winreg
        mac_to_name: dict[str, str] = {}
        try:
            root_path = r"SYSTEM\CurrentControlSet\Services\BTHPORT\Parameters\Devices"
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, root_path) as root:
                n = winreg.QueryInfoKey(root)[0]
                for i in range(n):
                    mac = winreg.EnumKey(root, i).lower()
                    try:
                        with winreg.OpenKey(root, mac) as dev:
                            val, _ = winreg.QueryValueEx(dev, "FriendlyName")
                            name = (val.decode("utf-16-le").rstrip("\x00")
                                    if isinstance(val, bytes) else str(val))
                            mac_to_name[mac] = name
                    except OSError:
                        pass
        except OSError:
            pass

        bthenum_path = r"SYSTEM\CurrentControlSet\Enum\BTHENUM"
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, bthenum_path) as root:
            n_svc = winreg.QueryInfoKey(root)[0]
            for i in range(n_svc):
                svc = winreg.EnumKey(root, i)
                try:
                    with winreg.OpenKey(root, svc) as svc_key:
                        n_inst = winreg.QueryInfoKey(svc_key)[0]
                        for j in range(n_inst):
                            inst = winreg.EnumKey(svc_key, j)
                            try:
                                with winreg.OpenKey(svc_key,
                                                    inst + r"\Device Parameters") as dp:
                                    port_name, _ = winreg.QueryValueEx(dp, "PortName")
                            except OSError:
                                continue
                            inst_upper = inst.upper()
                            if inst_upper.endswith("_C00000000"):
                                direction = "outgoing"
                            elif "_C00000000" in inst_upper:
                                direction = "outgoing"
                            elif inst_upper.endswith("_00000000"):
                                direction = "incoming"
                            else:
                                direction = "unknown"
                            m = re.search(r'&([0-9A-Fa-f]{12})_', inst)
                            mac = m.group(1).lower() if m else ""
                            results.append({
                                "name":      mac_to_name.get(mac, ""),
                                "port":      str(port_name),
                                "direction": direction,
                                "mac":       mac,
                            })
                except OSError:
                    pass
    except Exception:
        pass
    return results


def scan_paired_bluetooth_devices() -> list[dict]:
    """Return all paired Bluetooth Classic devices visible as COM ports on this machine.

    Each entry is a dict with:
      - ``name``      — device's Bluetooth friendly name (e.g. ``"NML_EXO"``), or ``""``
      - ``port``      — COM port string (e.g. ``"COM5"``)
      - ``direction`` — ``"outgoing"``, ``"incoming"``, or ``"unknown"``
      - ``mac``       — 12-char lowercase hex MAC (e.g. ``"000be3149a6c"``), or ``""``

    Tries two methods and merges results:
    1. ``list_ports.comports()`` filtered to BT ports (fast, misses offline ports).
    2. Direct BTHENUM registry scan (works even when the device is off).
    """
    from serial.tools import list_ports as _lp
    results: list[dict] = []
    seen_ports: set[str] = set()

    for p in _lp.comports():
        if not is_bluetooth_port(p):
            continue
        mac = _bt_mac_from_hwid(p.hwid or "")
        name = _bt_device_name_windows(mac) if mac else ""
        entry = {
            "name":      name or "",
            "port":      p.device,
            "direction": bt_port_direction(p),
            "mac":       mac or "",
        }
        results.append(entry)
        seen_ports.add(p.device)

    for entry in _scan_bthenum_registry():
        if entry["port"] not in seen_ports:
            results.append(entry)
            seen_ports.add(entry["port"])

    return results


def find_bluetooth_port_by_name(device_name: str,
                                 direction: str = "outgoing") -> str | None:
    """Return the COM port for a paired BT device whose name contains *device_name*.

    Case-insensitive, partial match.  When ``direction="outgoing"`` (the default),
    ports with ``direction="unknown"`` are also accepted as a fallback — Windows 11
    does not always encode direction in the HWID suffix.

    Args:
        device_name: Substring to look for in the BT device's friendly name.
        direction:   ``"outgoing"`` (default), ``"incoming"``, or ``"any"``.

    Returns:
        COM port string (e.g. ``"COM5"``) or ``None`` if not found.
    """
    entries = scan_paired_bluetooth_devices()
    name_lower = device_name.lower()

    # Preferred pass: exact direction match
    for entry in entries:
        if direction != "any" and entry["direction"] != direction:
            continue
        if name_lower in entry["name"].lower():
            return entry["port"]

    # Fallback pass: accept "unknown" direction when "outgoing" was requested
    # (Windows 11 often omits the _C00000000 suffix so direction is undetectable)
    if direction == "outgoing":
        for entry in entries:
            if entry["direction"] == "unknown" and name_lower in entry["name"].lower():
                return entry["port"]

    return None


class BluetoothScanner:
    """Live Bluetooth Classic device scanner using PyBluez2.

    Scans the air for nearby advertising devices — useful for discovery before
    pairing, or on platforms where the registry lookup is unavailable.

    For already-paired devices on Windows, ``scan_paired_bluetooth_devices()``
    is faster and does not require PyBluez2.

    Requires: ``pip install pybluez2``
    """

    @staticmethod
    def is_available() -> bool:
        """Return True if pybluez2 is installed and functional."""
        return _PYBLUEZ_AVAILABLE

    @staticmethod
    def scan(duration: int = 8) -> list[dict]:
        """Scan for nearby BT Classic devices.

        Args:
            duration: Inquiry duration in seconds (default 8 s).  Bluetooth
                      inquiry takes time — shorter values may miss devices.

        Returns:
            List of ``{'name': str, 'address': str}`` dicts.

        Raises:
            ImportError: if pybluez2 is not installed.
        """
        if not _PYBLUEZ_AVAILABLE:
            raise ImportError("pybluez2 is required for live scanning: pip install pybluez2")
        devices = _pybluez.discover_devices(duration=duration, lookup_names=True,
                                             flush_cache=True)
        return [{"name": name, "address": addr} for addr, name in devices]

    @staticmethod
    def find_address(device_name: str, duration: int = 8) -> str | None:
        """Scan and return the MAC address of the first device matching *device_name*.

        Case-insensitive, partial match.  Returns None if not found within
        the inquiry window.
        """
        for d in BluetoothScanner.scan(duration):
            if device_name.lower() in (d["name"] or "").lower():
                return d["address"]
        return None


class RFCOMMComm(BaseComm):
    """Direct Bluetooth Classic RFCOMM connection — no COM port required.

    Bypasses Windows COM port assignment entirely.  The HC-05 SPP profile always
    listens on RFCOMM channel 1.

    Two ways to create an instance::

        # By MAC address (fastest — no scan needed)
        comm = RFCOMMComm("AA:BB:CC:DD:EE:FF")

        # By device name — scans for the device live (~8 s)
        comm = RFCOMMComm.from_name("NML_EXO")

        # From Windows registry (instant, no scan, already-paired devices only)
        comm = RFCOMMComm.from_paired_name("NML_EXO")

        exo = HandExo(comm, auto_connect=True)

    Requires: ``pip install pybluez2``
    """

    SPP_CHANNEL = 1  # HC-05 SPP always uses RFCOMM channel 1

    def __init__(self, address: str, channel: int = SPP_CHANNEL,
                 recv_timeout: float = 1.0, command_delimiter: str = ";"):
        if not _PYBLUEZ_AVAILABLE:
            raise ImportError("pybluez2 is required for RFCOMMComm: pip install pybluez2")
        self.address           = address
        self.channel           = channel
        self._recv_timeout     = recv_timeout
        self.command_delimiter = command_delimiter
        self._sock             = None
        self._lock             = threading.RLock()

    # -- Factory methods -------------------------------------------------------

    @classmethod
    def from_name(cls, device_name: str, scan_duration: int = 8,
                  **kwargs) -> "RFCOMMComm":
        """Create by scanning for a nearby device with the given name (~8 s)."""
        addr = BluetoothScanner.find_address(device_name, scan_duration)
        if addr is None:
            raise ConnectionError(
                f"Bluetooth device '{device_name}' not found nearby "
                f"(scanned {scan_duration} s).  Is it powered on and in range?"
            )
        _log.info("[RFCOMMComm] Found '%s' at %s", device_name, addr)
        return cls(addr, **kwargs)

    @classmethod
    def from_paired_name(cls, device_name: str, **kwargs) -> "RFCOMMComm":
        """Create from the Windows registry — instant, no scan, already-paired only."""
        for entry in scan_paired_bluetooth_devices():
            if device_name.lower() in entry["name"].lower() and entry["mac"]:
                # Format MAC as AA:BB:CC:DD:EE:FF
                m = entry["mac"]
                addr = ":".join(m[i:i+2] for i in range(0, 12, 2)).upper()
                _log.info("[RFCOMMComm] Resolved '%s' → %s via registry", device_name, addr)
                return cls(addr, **kwargs)
        raise ConnectionError(
            f"Paired Bluetooth device '{device_name}' not found in registry.  "
            f"Pair the device in Windows Bluetooth Settings first, or use from_name()."
        )

    # -- BaseComm interface ----------------------------------------------------

    def connect(self) -> None:
        sock = _pybluez.BluetoothSocket(_pybluez.RFCOMM)
        sock.settimeout(10.0)           # connection timeout
        sock.connect((self.address, self.channel))
        sock.settimeout(self._recv_timeout)
        self._sock = sock
        _log.info("[RFCOMMComm] Connected to %s ch%d", self.address, self.channel)

    def close(self) -> None:
        with self._lock:
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

    def send(self, message: str) -> None:
        with self._lock:
            if not self._sock:
                raise ConnectionError("RFCOMMComm not connected")
            self._sock.sendall(message.encode())

    def receive(self, wait_until_return: bool = False,
                timeout: float = 2.0) -> str:
        if not self._sock:
            return ""
        try:
            if wait_until_return:
                self._sock.settimeout(timeout)
                data = b""
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    try:
                        chunk = self._sock.recv(256)
                        if not chunk:
                            break
                        data += chunk
                        if self.command_delimiter.encode() in data:
                            break
                    except socket.timeout:
                        break
                self._sock.settimeout(self._recv_timeout)
            else:
                self._sock.settimeout(0.05)
                try:
                    data = self._sock.recv(1024)
                except socket.timeout:
                    data = b""
                self._sock.settimeout(self._recv_timeout)
            return data.decode(errors="ignore").replace(
                self.command_delimiter, "\n").strip()
        except Exception as e:
            _log.warning("[RFCOMMComm] receive error: %s", e)
            return ""

    def is_connected(self) -> bool:
        return self._sock is not None


class BluetoothComm(SerialComm):
    """SerialComm variant for Bluetooth Classic (SPP) virtual COM ports.

    Adds automatic reconnection when the serial link drops — common with BT
    connections that time out or are temporarily disrupted.  Everything else
    (send, receive, locking) is inherited from SerialComm unchanged.

    Usage::

        comm = BluetoothComm(port="COM6", baudrate=57600)
        exo  = HandExo(comm, auto_connect=True)

    Args:
        port: COM port assigned to the paired HC-05 (e.g. ``"COM6"`` on Windows,
              ``"/dev/rfcomm0"`` on Linux).
        baudrate: Must match the HC-05 data-mode baud rate (default 57600).
        reconnect_attempts: How many times to retry before raising.
        reconnect_delay: Seconds to wait between reconnect attempts.
    """

    def __init__(self, port: str, baudrate: int = 57600,
                 command_delimiter: str = ';', timeout: float = 1.0,
                 reconnect_attempts: int = 5, reconnect_delay: float = 2.0):
        super().__init__(port, baudrate, command_delimiter, timeout)
        self._reconnect_attempts = reconnect_attempts
        self._reconnect_delay    = reconnect_delay

    def send(self, message: str) -> None:
        try:
            super().send(message)
        except (serial.SerialException, OSError) as exc:
            _log.warning("[BluetoothComm] Serial error (%s) — attempting reconnect", exc)
            self._reconnect()
            super().send(message)  # single retry after reconnect

    def _reconnect(self) -> None:
        self.close()
        for attempt in range(1, self._reconnect_attempts + 1):
            _log.info("[BluetoothComm] Reconnect attempt %d/%d …",
                      attempt, self._reconnect_attempts)
            time.sleep(self._reconnect_delay)
            try:
                self.connect()
                _log.info("[BluetoothComm] Reconnected on attempt %d", attempt)
                return
            except Exception as exc:
                _log.warning("[BluetoothComm] Attempt %d failed: %s", attempt, exc)
        raise ConnectionError(
            f"BluetoothComm: could not reconnect to {self.port} "
            f"after {self._reconnect_attempts} attempt(s)"
        )


class BLEComm(BaseComm):
    """Stub for future BLE support using bleak.

    Not yet implemented.  Install bleak (``pip install bleak``) and replace
    this stub once you have a BLE-capable module (e.g. HM-10 or nRF52840).
    """

    def __init__(self, address: str, service_uuid: str = "", char_uuid: str = ""):
        if not _BLEAK_AVAILABLE:
            raise ImportError(
                "bleak is required for BLEComm. Install it with: pip install bleak"
            )
        self.address      = address
        self.service_uuid = service_uuid
        self.char_uuid    = char_uuid
        self._client      = None

    def connect(self):
        raise NotImplementedError("BLEComm is not yet implemented")

    def disconnect(self):
        raise NotImplementedError("BLEComm is not yet implemented")

    def send(self, message: str):
        raise NotImplementedError("BLEComm is not yet implemented")

    def receive(self) -> str:
        raise NotImplementedError("BLEComm is not yet implemented")

    def is_connected(self) -> bool:
        return self._client is not None

