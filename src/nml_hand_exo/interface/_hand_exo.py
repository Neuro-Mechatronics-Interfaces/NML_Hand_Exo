import re
import struct
import time
import math
import numpy as np

from ._interfaces import BaseComm


ANGLE_ADDRESSABLE_GESTURES = frozenset(
    {"thumb", "thumbadd", "thumbrot", "thumbflex", "index", "middle", "ring", "pinky", "wrist"}
)
MIN_GESTURE_ANGLE_FIRMWARE = (0, 2, 16)


class HandExo(object):
    """
    Class to control the NML Hand Exoskeleton via serial communication.

    Features:
      - Enable/disable motors
      - Move motors to specific angles
      - Query status (angle, torque, current)
      - Configure velocity and acceleration
      - Retrieve device information
      - Send low-level serial commands

    """

    def __init__(self, comm: BaseComm, name='NMLHandExo', command_delimiter: str = '\n', send_delay: float = 0.01,
                 auto_connect=False, verbose: bool = False, side: str | None = None):
        """
        Initializes the HandExo interface.

        Args:
            name (str): Name of the exoskeleton instance.
            command_delimiter (str): Delimiter used to separate commands (default is '\n').
            send_delay (float): Delay in seconds after sending a command to allow processing (default is 0.01).
            verbose (bool): If True, enables verbose logging of commands and responses (default is False).
            side (str or None): Hand side this exo is for — ``'right'``, ``'left'``,
                or ``None`` to auto-detect from the firmware 'info' response.

        """
        self.name = name
        self.device = comm
        self.command_delimiter = command_delimiter
        self.send_delay = send_delay
        self.verbose = verbose
        self.device.verbose = verbose
        # Side is set explicitly or detected via detect_side() / info().
        self.side: str | None = side
        self._firmware_version: tuple[int, ...] | None = None

        if auto_connect:
            self.device.connect()

    def logger(self, *argv, warning: bool = False):
        """ 
        Robust debugging print function
        
        Args:
            *argv             : (str) Messages to log.
            warning           : (bool) If True, prints the message in yellow.

        """
        if self.verbose:
            msg = ''.join(argv)
            msg = f"[{time.monotonic():.3f}][{self.name}] {msg}"

            # If a warning, print the text in yellow
            msg = f"\033[93m{msg}\033[0m" if warning else msg
            print(msg)

    def detect_side(self) -> str:
        """
        Query the firmware's 'info' response to determine the hand side.

        Updates ``self.side`` and returns it.  Falls back to ``'right'`` if the
        firmware does not include a Side field (pre-handedness firmware).

        Returns:
            str: ``'left'`` or ``'right'``.
        """
        try:
            d = self.info()
            self.side = d.get('side', self.side or 'right')
        except Exception:
            self.side = self.side or 'right'
        return self.side

    def close(self):
        """Close the underlying communication interface."""
        try:
            self.device.close()
        except Exception:
            pass

    def set_comm(self, comm: BaseComm):
        """
        Sets the communication interface for the exoskeleton.

        Args:
            comm (BaseComm): The communication interface to use.

        """
        self.device = comm
        if self.verbose:
            self.logger(f"Communication interface set to {comm.__class__.__name__}")

    def connect(self):
        """
        Establishes a connection to the exoskeleton device.
        """
        self.device.connect()

    def send_command(self, cmd: str):
        """
        Sends a command to the exoskeleton over the serial connection.

        Args:
            cmd (str): Command to send to the exoskeleton.

        """
        if not cmd.endswith(self.command_delimiter):
            cmd += self.command_delimiter
        try:
            self.device.send(cmd)
            self.logger(f"Sent: {cmd.strip()}")
            time.sleep(self.send_delay)  # Allow time for the command to be processed
        except Exception as e:
            print(f"[ERROR] Failed to send command: {e}")

    def _receive(
        self, wait_until_return: bool = False, timeout: float | None = None
    ) -> str:
        """
        Reads a response from the exoskeleton over the serial connection.
        
        Returns:
            str: The response from the exoskeleton, or an empty string if no response.

        """
        if timeout is None:
            return self.device.receive(wait_until_return=wait_until_return)
        return self.device.receive(
            wait_until_return=wait_until_return, timeout=timeout
        )

    def get_fast_telemetry(
        self,
        timeout: float = 0.5,
        motor_ids: list[int] | tuple[int, ...] | None = None,
    ) -> dict[int, dict[str, float | int | bool | None]]:
        """Read one version-1 compact binary telemetry frame from SerialComm.

        This method must be called only by the serial owner (the GUI worker),
        because it consumes the raw serial stream.  Firmware fallback frames
        contain position only; current and velocity are represented as ``None``
        rather than misleading zero measurements.
        """
        serial_dev = getattr(self.device, "device", None)
        if serial_dev is None:
            raise RuntimeError("get_fast_telemetry requires a SerialComm device")

        header_fmt = "<2sBBBHIH"
        record_fmt = "<BBhiiii"
        header_len = struct.calcsize(header_fmt)
        record_len = struct.calcsize(record_fmt)
        ids = "all" if not motor_ids else ":".join(str(int(mid)) for mid in motor_ids)

        try:
            serial_dev.reset_input_buffer()
        except Exception:
            pass
        self.send_command(f"get_telemetry_fast:{ids}")

        deadline = time.monotonic() + timeout

        def read_exact(n_bytes: int) -> bytes:
            chunks = bytearray()
            while len(chunks) < n_bytes and time.monotonic() < deadline:
                chunk = serial_dev.read(n_bytes - len(chunks))
                if chunk:
                    chunks.extend(chunk)
                else:
                    time.sleep(0.001)
            if len(chunks) != n_bytes:
                raise TimeoutError("Timed out reading fast telemetry frame")
            return bytes(chunks)

        prefix = bytearray()
        while time.monotonic() < deadline:
            byte = serial_dev.read(1)
            if not byte:
                time.sleep(0.001)
                continue
            prefix.extend(byte)
            prefix = prefix[-2:]
            if prefix == b"NX":
                break
        else:
            raise TimeoutError("Timed out waiting for fast telemetry frame")

        header = b"NX" + read_exact(header_len - 2)
        magic, version, flags, count, payload_len, timestamp_ms, checksum = struct.unpack(
            header_fmt, header
        )
        if magic != b"NX" or version != 1:
            raise ValueError("Unsupported fast telemetry frame")
        if count > 32:
            raise ValueError("Fast telemetry frame exceeds the supported motor count")
        if payload_len != count * record_len:
            raise ValueError("Malformed fast telemetry payload length")

        payload = read_exact(payload_len)
        calculated_checksum = (sum(header[:-2]) + sum(payload)) & 0xFFFF
        if calculated_checksum != checksum:
            raise ValueError("Fast telemetry checksum mismatch")

        position_only = flags == 1  # FAST_TELEM_METHOD_FALLBACK_READ
        records: dict[int, dict[str, float | int | bool | None]] = {}
        for offset in range(0, payload_len, record_len):
            mid, error, current_mA, velocity_raw, position_ticks, absolute_cdeg, relative_cdeg = (
                struct.unpack_from(record_fmt, payload, offset)
            )
            records[mid] = {
                "id": mid,
                "error": bool(error),
                "current": None if position_only else current_mA,
                "velocity_raw": None if position_only else velocity_raw,
                "position_ticks": position_ticks,
                "absolute_angle": absolute_cdeg / 100.0,
                "angle": relative_cdeg / 100.0,
                "timestamp_ms": timestamp_ms,
                "flags": flags,
            }
        return records

    def _get_motor_attribute(
        self,
        attr: str,
        motor_id: (int or str) = 'all',
        wait_until_return: bool = False,
        command: str | None = None,
    ) -> float or list or bool or dict:
        """
        Generic method to retrieve a specified attribute from the motor(s).

        Args:
            attr (str): Attribute to extract ('angle', 'torque', 'limits', 'enabled', etc.).
            motor_id (int or str): Motor ID to query, or 'all' for all motors.

        Returns:
            Single value if a motor ID is given, or a dict of {motor_id: attr_value} if 'all'.
        """
        self.send_command(f"{command or f'get_{attr}'}:{motor_id}")
        raw = self._receive(wait_until_return=wait_until_return)
        if self.verbose:
            print(f"Raw return: {raw}")
        raw = raw.strip()

        parsed = self._parse_motor_data_block(raw)

        if motor_id == 'all':
            #return parsed
            return {mid: m.get(attr) for mid, m in parsed.items()}
        elif isinstance(motor_id, int):
            if self.verbose:
                print(f"Returning motor {motor_id}'s {attr} value")
            if motor_id not in parsed:
                raise ValueError(f"Motor ID {motor_id} not found in response.")
            return parsed[motor_id].get(attr)
        else:
            raise TypeError(f"motor_id must be 'all' or int, got {type(motor_id)}")

    def _parse_motor_data_block(self, raw: str) -> dict:
        """
        Parses a raw motor data string and returns a dictionary of motor data.
        Handles both single and multi-motor formats.

        Args:
            raw (str): Raw string from the serial device.

        Returns:
            dict: Dictionary where keys are motor IDs (as int), and values are dicts of parsed motor attributes.
        """
        motor_data = {}
        lines = [line.strip() for line in raw.strip().splitlines() if line.strip()]

        for line in lines:
            # Match either "Motor 0: { ... }" or "Motor: { ... }"
            match = re.match(r"Motor(?:\s+(\d+))?:\s*\{(.+?)\}", line)
            if not match:
                continue

            motor_id_str, data_block = match.groups()

            # Fallback if no ID in prefix: look inside the block for id
            motor_info = {}
            for part in data_block.split(","):
                key_val = part.strip().split(":", 1)
                if len(key_val) != 2:
                    continue
                key, val = key_val[0].strip(), key_val[1].strip()

                if key == "id":
                    motor_info["id"] = int(val)
                elif key == "angle":
                    motor_info["angle"] = float(val)
                elif key == "limits":
                    motor_info["limits"] = [float(x) for x in re.findall(r"[-+]?[0-9]*\.?[0-9]+", val)]
                elif key == "torque":
                    motor_info["torque"] = float(val)
                elif key == "enabled":
                    motor_info["enabled"] = val.lower() == "true"
                elif key == "velocity":
                    _m = re.match(r'[-+]?[\d.]+', val.strip())
                    motor_info["velocity"] = float(_m.group()) if _m else float(val)
                elif key == "acceleration":
                    motor_info["acceleration"] = float(val)
                elif key == "baudrate":
                    motor_info["baudrate"] = int(val)
                elif key == "home":
                    motor_info["home"] = float(val)
                elif key == "absolute_angle":
                    motor_info["absolute_angle"] = float(val)
                elif key == "current":
                    _m = re.match(r'[-+]?[\d.]+', val.strip())
                    motor_info["current"] = float(_m.group()) if _m else float(val)
                elif key == "current_limit":
                    _m = re.match(r'[-+]?[\d.]+', val.strip())
                    motor_info["current_limit"] = float(_m.group()) if _m else float(val)
                elif key == "goal_current":
                    _m = re.match(r'[-+]?[\d.]+', val.strip())
                    motor_info["goal_current"] = float(_m.group()) if _m else float(val)
                else:
                    motor_info[key] = val

            # Prefer the actual Dynamixel ID from the id: field in the blob.
            # The Motor X: prefix uses a loop index (0..N-1), NOT the hardware ID.
            # Firmware embeds the real ID as "id: <N>" inside the braces.
            actual_id = motor_info.get("id")
            if actual_id is None:
                actual_id = int(motor_id_str) if motor_id_str else None
            motor_id = actual_id
            if motor_id is not None:
                motor_data[motor_id] = motor_info

        return motor_data

    def enable_motor(self, motor_id: (int or str) = 'all'):
        """
        Enables the torque output for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to enable.

        Returns:
            None

        """
        self.send_command(f"enable:{motor_id}")

    def enable_motors_by_id(self, motor_ids):
        """Enable torque for a list of explicit Dynamixel IDs.

        Uses per-ID legacy commands for firmware compatibility.
        """
        ids = sorted({int(mid) for mid in motor_ids if int(mid) > 0})
        if not ids:
            return
        for mid in ids:
            self.send_command(f"enable:{mid}")

    def is_enabled(self, motor_id: (int or str) = 'all') -> bool:
        """
        Checks if the specified motor is enabled.

        Args:
            motor_id (int or str): ID of the motor to check.

        Returns:
            bool: True if the motor is enabled, False otherwise.

        """
        return self._get_motor_attribute('enabled', motor_id, wait_until_return=True)

    def disable_motor(self, motor_id: (int or str) = 'all'):
        """
        Disables the torque output for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to disable.

        Returns:
            None

        """
        self.send_command(f"disable:{motor_id}")

    def disable_motors_by_id(self, motor_ids):
        """Disable torque for a list of explicit Dynamixel IDs.

        Uses per-ID legacy commands for firmware compatibility.
        """
        ids = sorted({int(mid) for mid in motor_ids if int(mid) > 0})
        if not ids:
            return
        for mid in ids:
            self.send_command(f"disable:{mid}")

    def enable_led(self, motor_id: (int or str) = 'all'):
        """
        Enables the LED for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to enable the LED for.

        Returns:
            None

        """
        self.send_command(f"led:{motor_id}:on")

    def disable_led(self, motor_id: (int or str) = 'all'):
        """
        Disables the LED for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to disable the LED for.

        Returns:
            None

        """
        self.send_command(f"led:{motor_id}:off")

    def help(self) -> str:
        """
        Sends a help command to the exoskeleton to retrieve available commands.

        Returns:
            str: A string containing the help information from the exoskeleton.

        """
        self.send_command("help")
        return self._receive(wait_until_return=True)

    def set_debug(self, enable: bool):
        """
        Enables or disables verbose debug output from the Arduino.

        Args:
            enable (bool): True to enable debug output, False to disable.

        Returns:
            None

        """
        state = "on" if enable else "off"
        self.send_command(f"debug:{state}")

    def version(self) -> str:
        """
        Gets the version of the exo
        """
        self.send_command("version")
        response = self._receive(wait_until_return=True, timeout=1.0)
        match = re.search(r"Exo Device Version:\s*([0-9]+(?:\.[0-9]+)*)", response)
        if match:
            return match.group(1)
        return ""

    def firmware_version(self) -> tuple[int, ...]:
        if self._firmware_version is None:
            text = self.version()
            if not re.fullmatch(r"\d+(?:\.\d+)*", text):
                raise RuntimeError("could not determine firmware version")
            self._firmware_version = tuple(int(part) for part in text.split("."))
        return self._firmware_version

    def require_gesture_angle_support(self):
        version = self.firmware_version()
        if version < MIN_GESTURE_ANGLE_FIRMWARE:
            raise RuntimeError(
                "set_gesture_angle requires firmware 0.2.16 or newer; device has "
                + ".".join(map(str, version))
            )

    def home(self, motor_id: (int or str) = 'all'):
        """
        Sends a home command to all motors, unless a specific motor ID is provided.

        Args:
            motor_id (int or str): ID of the motor to home, or 'all' to home all motors."

        Returns:
            None

        """
        self.send_command(f"home:{motor_id}")

    def info(self, timeout: float = 5.0) -> dict:
        """
        Request and parse exoskeleton info into a structured dictionary.

        Returns
        -------
        info : dict
            Keys:
            - name : str
            - version : str
            - n_motors : int
            - motors : {motor_id: {...}}
            - motor_<id> : per-motor dicts (back-compat)
        """
        import re

        self.send_command("info")
        raw = self._receive(wait_until_return=True, timeout=timeout)
        if self.verbose:
            print(f"Raw return: {raw}")

        info: dict = {}
        if not raw:
            return info
        info["_raw"] = raw

        # Normalize lines and drop empties
        lines = [ln.strip() for ln in raw.splitlines() if ln.strip()]

        # --- Header lines appear one-per-line in your sample ---
        name_pat    = re.compile(r'^Name:\s*(\S+)')
        ver_pat     = re.compile(r'^Version:\s*(\S+)')
        side_pat    = re.compile(r'^Side:\s*(\S+)')
        nmot_pat    = re.compile(r'^Number of Motors:\s*(\d+)')
        motor_pat   = re.compile(r'^Motor\s+(\d+):\s*\{(.*)\}\s*$')

        motors = {}

        for ln in lines:
            m = name_pat.search(ln)
            if m:
                info['name'] = m.group(1)
                continue
            m = ver_pat.search(ln)
            if m:
                info['version'] = m.group(1)
                continue
            m = side_pat.search(ln)
            if m:
                info['side'] = m.group(1).lower()
                # Update self.side if not explicitly set by the caller
                if self.side is None:
                    self.side = info['side']
                continue
            m = nmot_pat.search(ln)
            if m:
                info['n_motors'] = int(m.group(1))
                continue

        # --- Motor detail lines ---
        for ln in lines:
            mm = motor_pat.match(ln)
            if not mm:
                continue

            motor_id = int(mm.group(1))
            blob = mm.group(2)  # inside {...}

            # Pull fields robustly (values may include parentheses/brackets/commas)
            get_str = lambda key: (re.search(rf'\b{key}:\s*([^,}}]+)', blob) or [None, None])[1]
            get_num = lambda key: (
                (m := re.search(rf'\b{key}:\s*([-+]?\d+(?:\.\d+)?)', blob)) and float(m.group(1))
            )

            m_info = {}

            # name
            nm = get_str('name')
            if nm is not None:
                m_info['name'] = nm.strip()

            # id (should equal motor_id, but we parse for completeness)
            mid = get_num('id')
            if mid is not None:
                m_info['id'] = int(mid)
            else:
                m_info['id'] = motor_id

            # angle: first float only; also capture optional "(abs: ...)" as angle_abs
            ang_m = re.search(r'angle:\s*([-+]?\d+(?:\.\d+)?)', blob)
            if ang_m:
                m_info['angle'] = float(ang_m.group(1))
            abs_m = re.search(r'\(abs:\s*([-+]?\d+(?:\.\d+)?)\)', blob)
            if abs_m:
                m_info['angle_abs'] = float(abs_m.group(1))

            # limits: two floats inside [...]
            lim_m = re.search(r'limits:\s*\[([^\]]+)\]', blob)
            if lim_m:
                lim_vals = re.findall(r'[-+]?\d+(?:\.\d+)?', lim_m.group(1))
                if len(lim_vals) >= 2:
                    m_info['limits'] = [float(lim_vals[0]), float(lim_vals[1])]

            # torque
            tq = get_num('torque')
            if tq is not None:
                m_info['torque'] = float(tq)

            # enabled (true/false)
            en_m = re.search(r'enabled:\s*(true|false)', blob, re.IGNORECASE)
            if en_m:
                m_info['enabled'] = (en_m.group(1).lower() == 'true')

            # Stash — use actual Dynamixel ID (from id: field), not loop index
            actual_id = m_info.get('id', motor_id)
            motors[actual_id] = m_info
            info[f'motor_{actual_id}'] = m_info  # back-compat

        if 'n_motors' not in info:
            info['n_motors'] = len(motors)

        info['motors'] = motors
        return info


    def get_baudrate(self, motor_id: (int or str) = 'all') -> int:
        """
        Retrieves the current baud rate of the serial connection.

        Returns:
            int: The current baud rate.

        """
        return self._get_motor_attribute('baudrate', motor_id, wait_until_return=True)

    def set_baudrate(self, motor_id: (int or str), baudrate: int):
        """
        Sets the baud rate for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the baud rate for.
            baudrate (int): Desired baud rate (e.g., 57600, 115200).

        Returns:
            None

        """
        self.send_command(f"set_baud:{motor_id}:{baudrate}")

    def get_motor_velocity(self, motor_id: (int or str) = 'all') -> float:
        """
        Retrieves the current velocity of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current velocity of the motor in degrees per second.

        """
        return self._get_motor_attribute('velocity', motor_id, True)

    def set_motor_velocity(self, motor_id: (int or str), velocity: float):
        """
        Sets the velocity for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the velocity for.
            velocity (float): Desired velocity in degrees per second.

        Returns:
            None

        """
        self.send_command(f"set_goal_velocity:{motor_id}:{velocity}")

    def get_present_velocity(self, motor_id: (int or str) = 'all'):
        """Read signed present velocity in rpm."""
        return self._get_motor_attribute('velocity', motor_id, True, command='get_velocity')

    def set_direct_velocity(self, motor_id: int, velocity_rpm: float):
        """Command signed velocity in rpm while firmware is in velocity mode."""
        self.send_command(f"set_velocity:{int(motor_id)}:{float(velocity_rpm)}")

    def get_motor_acceleration(self, motor_id: (int or str) = 'all') -> float:
        """
        Retrieves the current acceleration of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current acceleration of the motor in degrees per second squared.

        """
        return self._get_motor_attribute('acceleration', motor_id, True)

    def set_motor_acceleration(self, motor_id: (int or str), acceleration: float):
        """
        Sets the acceleration for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the acceleration for.
            acceleration (float): Desired acceleration in degrees per second squared.

        Returns:
            None

        """
        self.send_command(f"set_goal_acceleration:{motor_id}:{acceleration}")

    def get_motor_angle(self, motor_id: (int or str) = 'all') -> float:
        """
        Retrieves the current relative angle of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current angle of the motor in degrees.

        """
        return self._get_motor_attribute('angle', motor_id, True)

    def set_motor_angle(self, motor_id: (int or str), angle: float):
        """
        Sets the angle for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the angle for.
            angle (float): Desired angle in degrees.

        Returns:
            None

        """
        if isinstance(motor_id, str):
            cmd = f"set_angle:{motor_id}:{angle}"
        else:
            cmd = f"set_angle:{int(motor_id)}:{angle}"
        self.send_command(cmd)

    def get_absolute_motor_angle(self, motor_id: (int or str) = 'all') -> float:
        """
        Retrieves the absolute angle of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Absolute angle of the motor in degrees.

        """
        return self._get_motor_attribute('absolute_angle', motor_id, True)

    def set_absolute_motor_angle(self, motor_id: (int or str), angle: float):
        """
        Sets the absolute angle for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the absolute angle for.
            angle (float): Desired absolute angle in degrees.

        Returns:
            None

        """
        if isinstance(motor_id, str):
            cmd = f"set_absolute_angle:{motor_id}:{angle}"
        else:
            cmd = f"set_absolute_angle:{int(motor_id)}:{angle}"
        self.send_command(cmd)

    def get_home(self, motor_id: (int or str) = 'all') -> float:
        """
        Retrieves the home angle of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Home angle of the motor in degrees.

        """
        return self._get_motor_attribute('home', motor_id, True)

    def set_home(self, motor_id: (int or str)):
        """
        Sets the current position as the new home/zero position for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the home position for.

        Returns:
            None

        """
        self.send_command(f"set_home:{motor_id}")

    def get_motor_torque(self, motor_id: (int or str) = 'all') -> float:
        """
        Retrieves the current torque of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current torque of the motor in Newton-meters.

        """
        return self._get_motor_attribute('torque', motor_id, True)

    def get_motor_current(self, motor_id: (int or str) = 'all') -> float:
        """
        Retrieves the current draw of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current draw of the motor in Amperes.

        """
        return self._get_motor_attribute('current', motor_id, True)

    def get_motor_current_limit(self, motor_id: (int or str) = 'all') -> float:
        """
        Retrieves the current limit of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current limit of the motor in Amperes.

        """
        return self._get_motor_attribute('current_limit', motor_id, True)

    def set_current_limit(self, motor_id: (int or str), current_limit: float):
        """
        Sets the current limit for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the current limit for.
            current_limit (float): Desired current limit in Amperes.

        Returns:
            None

        """
        self.send_command(f"set_current_lim:{motor_id}:{current_limit}")

    def get_goal_current(self, motor_id: (int or str) = 'all'):
        """Read signed direct-current goal in mA."""
        return self._get_motor_attribute(
            'goal_current', motor_id, True, command='get_goal_current'
        )

    def set_direct_current(self, motor_id: int, current_mA: float):
        """Command signed current in mA while firmware is in current mode."""
        self.send_command(f"set_current:{int(motor_id)}:{float(current_mA)}")

    def stop_direct_control(self, motor_id: (int or str) = 'all'):
        """Immediately zero direct velocity and current goals."""
        target = motor_id if isinstance(motor_id, str) else int(motor_id)
        self.send_command(f"stop:{target}")

    def set_direct_command_timeout(self, timeout_ms: int):
        """Set the firmware direct-control watchdog timeout."""
        self.send_command(f"set_command_timeout:{int(timeout_ms)}")

    def set_control_mode(self, mode: str):
        """Set the global motor mode; firmware leaves torque disabled."""
        normalized = mode.strip().lower()
        allowed = {"position", "current_position", "velocity", "current"}
        if normalized not in allowed:
            raise ValueError(f"Unsupported control mode: {mode}")
        self.send_command(f"set_control_mode:all:{normalized}")

    def get_motor_limits(self, motor_id: (int or str) = 'all') -> tuple:
        """
        Retrieves the limits for the specified motor, including minimum and maximum angles.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            tuple: A tuple containing the minimum and maximum angles of the motor.

        """
        return self._get_motor_attribute('limits', motor_id, True)

    def set_motor_upper_limit(self, motor_id: (int or str), upper_limit: float):
        """
        Sets the upper limit for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the upper limit for.
            upper_limit (float): Desired upper limit in degrees.

        Returns:
            None

        """
        self.send_command(f"set_upper_limit:{motor_id}:{upper_limit}")

    def set_motor_lower_limit(self, motor_id: (int or str), lower_limit: float):
        """
        Sets the lower limit for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the lower limit for.
            lower_limit (float): Desired lower limit in degrees.

        Returns:
            None

        """
        self.send_command(f"set_lower_limit:{motor_id}:{lower_limit}")

    def set_motor_limits(self, motor_id: (int or str), lower_limit: float, upper_limit: float):
        """
        Sets both the lower and upper limits for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the limits for.
            lower_limit (float): Desired lower limit in degrees.
            upper_limit (float): Desired upper limit in degrees.

        Returns:
            None

        """
        self.send_command(f"set_motor_limits:{motor_id}:{lower_limit}:{upper_limit}")

    def reboot_motor(self, motor_id: (int or str) = 'all'):
        """
        Reboots the specified motor.

        Args:
            motor_id (int or str): ID of the motor to reboot.

        Returns:
            None

        """
        self.send_command(f"reboot:{motor_id}")

    def get_motor_mode(self) -> str:
        """
        Retrieves the current control mode of the specified motor.

        Returns:
            str: Current mode of the motor (e.g., "position", "velocity", "current_position").

        """
        self.send_command(f"get_motor_mode")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip()
            except IndexError:
                print(f"[ERROR] Invalid response")
        return ""

    def set_motor_mode(self, mode: str):
        """
        Sets the control mode for all motors (global setting).

        Args:
            mode (str): Desired control mode (e.g., "position", "velocity", "current_position").

        Returns:
            None

        """
        self.send_command(f"set_motor_mode:{mode}")

    def get_exo_mode(self) -> str:
        """
        Retrieves the current operating mode of the exoskeleton.

        Returns:
            str: Current mode of the exoskeleton (e.g., "manual", "autonomous").

        """
        self.send_command("get_exo_mode")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip()
            except IndexError:
                print(f"[ERROR] Invalid response")
        return ""

    def set_exo_mode(self, mode: str):
        """
        Sets the operating mode for the exoskeleton.

        Args:
            mode (str): Desired operating mode (e.g., "manual", "autonomous").

        Returns:
            None

        """
        self.send_command(f"set_exo_mode:{mode}")

    def get_gesture(self) -> str:
        """
        Retrieves the current gesture recognized by the exoskeleton.

        Returns:
            str: Current gesture (e.g., "open", "close", "pinch").

        """
        self.send_command("get_gesture")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip()
            except IndexError:
                print(f"[ERROR] Invalid response")
        return ""

    def set_gesture(self, gesture: str, state: str = "default"):
        """
        Sets the gesture for the exoskeleton.

        Args:
            gesture (str): Desired gesture (e.g., "open", "close", "pinch").

        Returns:
            None

        """
        self.send_command(f"set_gesture:{gesture}:{state}")

    def set_gesture_angle(self, gesture: str, percent: float):
        """Set an angle-addressable joint gesture between extend (0) and flex (100)."""
        name = str(gesture).strip().lower()
        if name not in ANGLE_ADDRESSABLE_GESTURES:
            supported = ", ".join(sorted(ANGLE_ADDRESSABLE_GESTURES))
            raise ValueError(
                f"gesture must be one of {supported}, got {gesture!r}"
            )
        try:
            value = float(percent)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"percent must be numeric, got {percent!r}") from exc
        if not math.isfinite(value):
            raise ValueError(f"percent must be finite, got {percent!r}")
        self.require_gesture_angle_support()
        self.send_command(f"set_gesture_angle:{name}:{value:g}")

    def set_gesture_state(self, state: str):
        """
        Sets the state of the current gesture for the exoskeleton.

        Args:
            state (str): Desired state of the gesture (e.g., "default", "active").

        Returns:
            None

        """
        self.send_command(f"set_gesture_state:{state}")

    def get_gesture_list(self) -> list:
        """
        Retrieves the list of available gestures for the exoskeleton.

        Returns:
            list: A list of available gestures.

        """
        self.send_command("gesture_list")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip().split(',')
            except IndexError:
                print(f"[ERROR] Invalid response")
        return []

    def cycle_gesture(self):
        """
        Cycles through the available gestures for the exoskeleton.

        Returns:
            None

        """
        self.send_command("cycle_gesture")

    def cycle_gesture_state(self):
        """
        Cycles through the states of the current gesture for the exoskeleton.

        Returns:
            None

        """
        self.send_command("cycle_gesture_state")

    def set_zero_offset(self, motor_id: (int or str), offset: float):
        """
        Sets the zero offset for the specified motor to an arbitrary value.

        Args:
            motor_id (int or str): ID or name of the motor.
            offset (float): Zero offset in degrees (absolute angle of the open/home position).

        """
        self.send_command(f"set_zero_offset:{motor_id}:{offset}")

    def set_flip(self, motor_id: (int or str), flip: bool):
        """
        Sets the direction flip flag for a motor.

        Args:
            motor_id (int or str): ID or name of the motor.
            flip (bool): True to invert direction, False for normal.

        """
        self.send_command(f"set_flip:{motor_id}:{'1' if flip else '0'}")

    def apply_calibration(self, profile_or_path: str = None, profiles_dir: str = None,
                          name_to_id: dict = None):
        """
        Loads a calibration profile and pushes all values to the device.

        Can be called with a profile name (e.g. "zach"), a full file path,
        or with no arguments to load the default profile.

        Args:
            profile_or_path (str or None): One of:
                - A profile name (e.g. "zach") → loads profiles/zach.json
                - A full file path to a calibration JSON
                - None → loads the default profile from profiles/config.json
            profiles_dir (str or None): Directory containing profile JSONs.
                Defaults to examples/calibration/profiles/ relative to
                the repo root.
            name_to_id (dict[str, int] or None): Optional mapping of bare motor
                name → Dynamixel ID.  When provided, calibration commands use
                the explicit integer ID instead of the bare name string so that
                duplicate motor names in dual firmware (e.g. two "wrist" motors
                on left ID 1 and right ID 11) are resolved to the correct side
                without ambiguity.  When None, bare names are used (legacy
                behaviour, safe only in single-exo firmware builds).

        """
        import json
        import os

        if profiles_dir is None:
            repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.abspath(__file__)))))
            profiles_dir = os.path.join(repo_root, "examples", "calibration", "profiles")

        if profile_or_path is None:
            config_path = os.path.join(profiles_dir, "config.json")
            if not os.path.exists(config_path):
                raise FileNotFoundError(f"No profiles config found at {config_path}")
            with open(config_path, "r") as f:
                cfg = json.load(f)
            # Prefer side-specific default (e.g. "default_right"), fall back to
            # legacy "default" key so old config.json files still work.
            my_side = self.side or "right"
            default_name = cfg.get(f"default_{my_side}") or cfg.get("default")
            if not default_name:
                raise ValueError(
                    f"No default profile set for side='{my_side}'. "
                    "Pass a profile name or run calibrate_exo.py --set-default."
                )
            filepath = os.path.join(profiles_dir, f"{default_name}.json")
        elif os.path.isfile(profile_or_path):
            filepath = profile_or_path
        else:
            filepath = os.path.join(profiles_dir, f"{profile_or_path}.json")

        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Calibration profile not found: {filepath}")

        with open(filepath, "r") as f:
            cal = json.load(f)

        # Warn if the profile's declared side does not match this exo's side.
        profile_side = cal.get("side")
        if profile_side and self.side and profile_side != self.side:
            self.logger(
                f"[apply_calibration] WARNING: profile side='{profile_side}' "
                f"but this exo is side='{self.side}'. Applying anyway.",
                warning=True,
            )

        # --- Epoch alignment for multi-turn motors ---
        # In OP_CURRENT_BASED_POSITION the Dynamixel resets its multi-turn counter
        # at power-on.  The same physical joint angle may be reported as N*360° away
        # from the value recorded during calibration.  Query actual positions first
        # and snap every profile value to the motor's current epoch before pushing to
        # firmware; otherwise getRelativeAngle() subtracts a home that is ~360° off.
        self.send_command("get_absolute_angle:all")
        _raw = self._receive(wait_until_return=True)
        _parsed = self._parse_motor_data_block(_raw)

        # Key by integer DXL ID for unambiguous lookup in dual firmware where
        # multiple motors share the same bare name (e.g. two "wrist" motors).
        # The name-keyed fallback is kept for callers that do not supply name_to_id.
        abs_by_id = {
            motor_id: info["absolute_angle"]
            for motor_id, info in _parsed.items()
            if "absolute_angle" in info
        }
        abs_by_name = {
            info["name"]: info["absolute_angle"]
            for info in _parsed.values()
            if "name" in info and "absolute_angle" in info
        }

        for name, vals in cal["motors"].items():
            # Resolve motor reference: prefer explicit DXL ID when map is provided
            # so the firmware command targets the correct side unambiguously.
            dxl_id    = name_to_id.get(name) if name_to_id else None
            motor_ref = dxl_id if dxl_id is not None else name

            profile_home = vals["home"]
            current_abs  = abs_by_id.get(dxl_id) if dxl_id is not None else abs_by_name.get(name)

            if current_abs is not None:
                epoch_shift = round((current_abs - profile_home) / 360.0) * 360.0
            else:
                epoch_shift = 0.0
                self.logger(f"[apply_calibration] {name}: position unreadable, no epoch correction", warning=True)

            adj_home      = profile_home       + epoch_shift
            adj_limit_min = vals["limit_min"]  + epoch_shift
            adj_limit_max = vals["limit_max"]  + epoch_shift

            self.logger(
                f"[cal] {name} (id={motor_ref}): profile_home={profile_home:.2f} cur_abs="
                + (f"{current_abs:.2f}" if current_abs is not None else "N/A")
                + f" shift={epoch_shift:+.0f} adj_home={adj_home:.2f}"
            )

            self.set_zero_offset(motor_ref, adj_home)
            self.set_motor_limits(motor_ref, adj_limit_min, adj_limit_max)
            self.set_flip(motor_ref, vals["flip"])

        profile_name = os.path.basename(filepath).removesuffix(".json")
        self.logger(f"Calibration profile '{profile_name}' applied from {filepath}")

    def calibrate_exo(self, mode: str = "timed", duration: float = 10.0):
        """
        Starts the calibration routine for the exoskeleton.
        Note: This feature may not be fully implemented in the firmware.

        Args:
            mode (str): Calibration mode (default: "timed").
            duration (float): Duration in seconds for timed calibration (default: 10.0).

        Returns:
            None

        """
        self.send_command(f"calibrate_exo:{mode}:{duration}")

    def enable_oled(self) -> str:
        """
        Enables the OLED display on the exoskeleton.

        Returns:
            str: Response from the device.

        """
        self.send_command("oled:on")
        return self._receive()

    def disable_oled(self) -> str:
        """
        Disables the OLED display on the exoskeleton.

        Returns:
            str: Response from the device.

        """
        self.send_command("oled:off")
        return self._receive()

    def get_oled_status(self) -> str:
        """
        Gets the current status of the OLED display.

        Returns:
            str: OLED status ("OLED ENABLED" or "OLED DISABLED").

        """
        self.send_command("oled:status")
        return self._receive()

    def close(self):
        """
        Closes the serial connection to the exoskeleton.

        Returns:
            None

        """
        if self.device and self.device.is_connected():
            self.device.close()
            self.logger("Device connection closed.")

    def get_imu_data(self) -> dict:
        """
        Retrieves the IMU data from the exoskeleton.

        Receives a serial message with contents, as an example:
            "Received: get_imu
            Heading: 0.00, Pitch: 0.00, Roll: 0.00"

        Returns:
            dict: A dictionary containing IMU data (e.g., accelerometer, gyroscope, magnetometer).

        """
        self.send_command("get_imu")
        full_response = self._receive()
        # print(full_response)
        while (full_response is None) or ("Heading" not in full_response):
            self.send_command("get_imu")
            full_response = self._receive()

        lines = full_response.strip().splitlines()  #added to handle additional response info from arduino 
        response = lines[-1]

        # print("response:", response)
        imu_data = {}
        if response:
            try:
                lines = [line.strip() for line in response.splitlines() if line.strip()]
                for part in lines:
                    if part.startswith("Temp:"):
                        imu_data['temperature'] = float(part.split(':')[-1].strip().replace('C', ''))
                    elif part.startswith("Accel:"):
                        accel_str = part.split(':')[-1].strip().replace(']', '').replace('[', '')
                        accel_str = accel_str.replace('m/s^2', '')
                        imu_data['acceleration'] = list(map(float, accel_str.split(',')))
                    elif part.startswith("Gyro:"):
                        gyro_str = part.split(':')[-1].strip().replace(']', '').replace('[', '')
                        gyro_str = gyro_str.replace('rad/s', '')
                        imu_data['gyroscope'] = list(map(float, gyro_str.split(',')))

                new_msgs = response.split(",")
                for part in new_msgs:
                    if part.startswith("Heading:"):
                        heading_str = part.split(':')[-1].strip()
                        imu_data['heading'] = float(heading_str)
                    elif part.startswith(" Pitch:"):                
                        pitch_str = part.split(':')[-1].strip()
                        imu_data['pitch'] = float(pitch_str)
                    elif part.startswith(" Roll:"):
                        roll_str = part.split(':')[-1].strip()
                        imu_data['roll'] = float(roll_str)
                    elif part.startswith(" Positionx"):
                        posx_str = part.split(":")[-1].strip()
                        imu_data['positionx'] = float(posx_str)
                    elif part.startswith(" Positiony:"):
                        posy_str = part.split(":")[-1].strip()
                        imu_data['positiony'] = float(posy_str)
                    elif part.startswith(" Speed:"):
                        speed_str = part.split(':')[-1].strip()
                        imu_data['speed'] = float(speed_str)
                        # Add more parts as needed (e.g., magnetometer)

                return imu_data
            except (ValueError, IndexError):
                print(f"[ERROR] Invalid IMU data response: {response}")
        return {}

   
    def get_imu_angles(self) -> list:       #TODO: add radians conversion option
        """
        Retrieves the current roll, pitch, and yaw of the IMU.

        Returns:
            list: [Roll, Pitch, Yaw]

        """
        data = self.get_imu_data()

        if data: 
            try:
                heading = data['heading']
                roll = data['roll']
                pitch = data['pitch']
                return [roll, pitch, heading]

            except Exception as e:
                print(f"[ERROR] Failed to parse an angle from IMU angles: {e}")

    def get_imu_heading(self) -> float:    
        """
        Retrieves the current yaw of the IMU.

        Returns:
            float: Current yaw in degrees.

        """
        data = self.get_imu_data()

        if data: 
            try:
                heading = data['heading']
                return heading
            except Exception as e:
                print(f"[ERROR] Failed to parse heading from IMU angles: {e}")

    def get_imu_roll(self) -> float:  
        """
        Retrieves the current roll of the IMU.

        Returns:
            float: Current roll in degrees.

        """  
        data = self.get_imu_data()

        if data: 
            try:
                roll = data['roll']
                return roll
            except Exception as e:
                print(f"[ERROR] Failed to parse roll from IMU angles: {e}")

    def get_imu_pitch(self) -> float:   
        """
        Retrieves the current pitch of the IMU.

        Returns:
            float: Current pitch in degrees.

        """   
        data = self.get_imu_data()

        if data: 
            try:
                pitch = data['pitch']
                return pitch
            except Exception as e:
                print(f"[ERROR] Failed to parse pitch from IMU angles: {e}")

    def set_yaw_angle(self, motor_id: (int or str), target_angle: float, direction: str):
        """
        Sets the wrist angle (IMU yaw) for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the angle for.
            target_angle (float): Desired angle in degrees.
            direction (str): Desired direction of motion ("flex" or "extend", or single char 'f'/'e')

        Returns:
            None

        """
        # Map full direction strings to single characters expected by Arduino
        direction_map = {"flex": "f", "extend": "e", "f": "f", "e": "e"}
        direction_char = direction_map.get(direction.lower())
        
        if direction_char is None:
            print("Invalid function call. direction must be either 'flex', 'extend', 'f', or 'e'")
            return
        
        if isinstance(motor_id, str):
            cmd = f"set_yaw_angle:{motor_id}:{target_angle}:{direction_char}"
        else:
            cmd = f"set_yaw_angle:{int(motor_id)}:{target_angle}:{direction_char}"
        self.send_command(cmd)
        

    def get_gesture_state(self):
        """
        Retrieves the current state of the gesture.

        Returns:
            str: Current gesture state (e.g., "default", "active").

        """
        self.send_command("get_gesture_state")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip()
            except IndexError:
                print(f"[ERROR] Invalid response")
        return ""
