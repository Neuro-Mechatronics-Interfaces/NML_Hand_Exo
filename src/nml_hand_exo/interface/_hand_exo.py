import re
import time
import numpy as np

from ._interfaces import BaseComm


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

    def _receive(self, wait_until_return: bool = False) -> str:
        """
        Reads a response from the exoskeleton over the serial connection.
        
        Returns:
            str: The response from the exoskeleton, or an empty string if no response.

        """
        return self.device.receive(wait_until_return=wait_until_return)

    def _get_motor_attribute(self, attr: str, motor_id: (int or str) = 'all', wait_until_return: bool = False) -> float or list or bool or dict:
        """
        Generic method to retrieve a specified attribute from the motor(s).

        Args:
            attr (str): Attribute to extract ('angle', 'torque', 'limits', 'enabled', etc.).
            motor_id (int or str): Motor ID to query, or 'all' for all motors.

        Returns:
            Single value if a motor ID is given, or a dict of {motor_id: attr_value} if 'all'.
        """
        self.send_command(f"get_{attr}:{motor_id}")
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
        """Parse a raw multi-motor response into {dxl_id: {field: value}} dicts.

        Uses re.finditer on the full raw string rather than splitlines + re.match
        so that records are found even when BT timing glitches deliver multiple
        ';'-terminated chunks concatenated without intervening newlines.

        All int/float conversions are wrapped in try/except so a single corrupted
        byte in one field does not discard the entire motor record.
        """
        motor_data = {}

        for match in re.finditer(r"Motor(?:\s+(\d+))?:\s*\{(.+?)\}", raw, re.DOTALL):
            motor_id_str, data_block = match.groups()

            motor_info: dict = {}
            for part in data_block.split(","):
                kv = part.strip().split(":", 1)
                if len(kv) != 2:
                    continue
                key, val = kv[0].strip(), kv[1].strip()
                try:
                    if key == "id":
                        motor_info["id"] = int(val)
                    elif key in ("angle", "absolute_angle", "torque",
                                 "velocity", "acceleration", "home"):
                        motor_info[key] = float(val)
                    elif key in ("current", "current_limit"):
                        _m = re.match(r'[-+]?[\d.]+', val)
                        motor_info[key] = float(_m.group()) if _m else float(val)
                    elif key == "limits":
                        motor_info["limits"] = [
                            float(x) for x in re.findall(r"[-+]?[0-9]*\.?[0-9]+", val)
                        ]
                    elif key == "enabled":
                        motor_info["enabled"] = val.lower() == "true"
                    elif key == "baudrate":
                        motor_info["baudrate"] = int(val)
                    else:
                        motor_info[key] = val
                except (ValueError, TypeError):
                    pass  # corrupted field — skip it, keep parsing remaining fields

            # Motor X: prefix is a 0-based loop index, not the DXL hardware ID.
            # The real ID is inside the block as "id: <N>".
            actual_id = motor_info.get("id")
            if actual_id is None and motor_id_str:
                try:
                    actual_id = int(motor_id_str)
                except ValueError:
                    pass
            if actual_id is not None:
                motor_data[actual_id] = motor_info

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

    def is_enabled(self, motor_id: (int or str) = 'all') -> bool:
        """
        Checks if the specified motor is enabled.

        Args:
            motor_id (int or str): ID of the motor to check.

        Returns:
            bool: True if the motor is enabled, False otherwise.

        """
        self._get_motor_attribute('enabled', motor_id, wait_until_return=True)

    def disable_motor(self, motor_id: (int or str) = 'all'):
        """
        Disables the torque output for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to disable.

        Returns:
            None

        """
        self.send_command(f"disable:{motor_id}")

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
        response = self._receive()

        if response:
            return response.strip().split(':')[1]
        return ""

    def home(self, motor_id: (int or str) = 'all'):
        """
        Sends a home command to all motors, unless a specific motor ID is provided.

        Args:
            motor_id (int or str): ID of the motor to home, or 'all' to home all motors."

        Returns:
            None

        """
        self.send_command(f"home:{motor_id}")

    def info(self) -> dict:
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
        raw = self._receive(wait_until_return=True)
        if self.verbose:
            print(f"Raw return: {raw}")

        info: dict = {}
        if not raw:
            return info

        # Normalize lines and drop empties
        lines = [ln.strip() for ln in raw.splitlines() if ln.strip()]

        # --- Header lines appear one-per-line in your sample ---
        name_pat    = re.compile(r'^Name:\s*(\S+)')
        ver_pat     = re.compile(r'^Version:\s*(\S+)')
        side_pat    = re.compile(r'^Side:\s*(\S+)')
        calname_pat = re.compile(r'^CalibrationName:\s*(.*)')
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
            m = calname_pat.search(ln)
            if m:
                info['calibration_name'] = m.group(1).strip()
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

    def set_gesture(self, gesture: str, state: str = "default",
                    side: str | None = None):
        """
        Sets the gesture for the exoskeleton.

        Args:
            gesture (str): Desired gesture (e.g., "grasp", "pinch_index").
            state (str): Gesture state (e.g., "open", "close").
            side (str or None): "left", "right", or None (default) to command both sides.

        Returns:
            None

        """
        if side:
            self.send_command(f"set_gesture:{gesture}:{state}:{side}")
        else:
            self.send_command(f"set_gesture:{gesture}:{state}")

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

    # -------------------------------------------------------------------------
    # Gesture calibration EEPROM commands
    # -------------------------------------------------------------------------

    # Default gesture fractions — Python mirror of gestureLibrary in gesture_library.cpp.
    # Layout: gesture → state → joint → fraction [0.0=open/home, 1.0=fully closed/flexed].
    # Joint order matches EEPROM canonical order (eeprom_schema.md): wrist, wrist2,
    # thumbadd, thumbflex, thumbrot, index, middle, ring, pinky.
    # Verify against gesture_library.cpp before bumping GESTURE_EEPROM_MAGIC.
    _DEFAULT_GESTURE_FRACTIONS: dict = {
        # ── grasp ─────────────────────────────────────────────────────────────
        # All fingers + thumb close fully; wrists stay at home.
        "grasp": {
            "open":  {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 0.0, "thumbflex": 0.0, "thumbrot": 0.0,
                      "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0},
            "close": {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 1.0, "thumbflex": 1.0, "thumbrot": 1.0,
                      "index": 1.0, "middle": 1.0, "ring": 1.0, "pinky": 1.0},
        },
        # ── keygrip ───────────────────────────────────────────────────────────
        # Fingers held extended (1.0 open); thumb closes on open, full close = all thumb+fingers.
        # thumbrot stays at 0 (side pinch orientation, not opposition).
        "keygrip": {
            "open":  {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 0.0, "thumbflex": 0.0, "thumbrot": 0.0,
                      "index": 1.0, "middle": 1.0, "ring": 1.0, "pinky": 1.0},
            "close": {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 1.0, "thumbflex": 1.0, "thumbrot": 0.0,
                      "index": 1.0, "middle": 1.0, "ring": 1.0, "pinky": 1.0},
        },
        # ── pinch_index ───────────────────────────────────────────────────────
        # Index + full thumb opposition close; other fingers stay open.
        "pinch_index": {
            "open":  {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 0.0, "thumbflex": 0.0, "thumbrot": 0.0,
                      "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0},
            "close": {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 1.0, "thumbflex": 1.0, "thumbrot": 1.0,
                      "index": 1.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0},
        },
        # ── pinch_middle ──────────────────────────────────────────────────────
        # Middle + full thumb opposition close; index, ring, pinky stay open.
        "pinch_middle": {
            "open":  {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 0.0, "thumbflex": 0.0, "thumbrot": 0.0,
                      "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0},
            "close": {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 1.0, "thumbflex": 1.0, "thumbrot": 1.0,
                      "index": 0.0, "middle": 1.0, "ring": 0.0, "pinky": 0.0},
        },
        # ── pinch_ring ────────────────────────────────────────────────────────
        # Ring + full thumb opposition close; index, middle, pinky stay open.
        "pinch_ring": {
            "open":  {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 0.0, "thumbflex": 0.0, "thumbrot": 0.0,
                      "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0},
            "close": {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 1.0, "thumbflex": 1.0, "thumbrot": 1.0,
                      "index": 0.0, "middle": 0.0, "ring": 1.0, "pinky": 0.0},
        },
        # ── peace ─────────────────────────────────────────────────────────────
        # Index + middle extended; ring, pinky + thumb close.
        "peace": {
            "open":  {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 0.0, "thumbflex": 0.0, "thumbrot": 0.0,
                      "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0},
            "close": {"wrist": 0.0, "wrist2": 0.0, "thumbadd": 1.0, "thumbflex": 1.0, "thumbrot": 1.0,
                      "index": 0.0, "middle": 0.0, "ring": 1.0, "pinky": 1.0},
        },
    }

    def set_gesture_cal_value(self, gesture: str, state: str, joint: str,
                               value: float, side: str | None = None) -> None:
        """Update one [0-1] fraction in the firmware's live gestureLibrary.

        Args:
            side: "left", "right", or None (default) to update both sides.

        Does NOT write to EEPROM; call save_gesture_cal() afterward to persist.
        """
        if side:
            self.send_command(f"set_gesture_cal:{gesture}:{state}:{joint}:{value:.4f}:{side}")
        else:
            self.send_command(f"set_gesture_cal:{gesture}:{state}:{joint}:{value:.4f}")

    def save_gesture_cal(self) -> None:
        """Persist the current gestureLibrary fractions and cal name to EEPROM."""
        self.send_command("save_gesture_cal")

    def load_gesture_cal(self) -> None:
        """Reload gestureLibrary fractions and cal name from EEPROM."""
        self.send_command("load_gesture_cal")

    def set_cal_name(self, name: str) -> None:
        """Set the active calibration profile name in firmware memory.

        Does NOT write to EEPROM; call save_gesture_cal() afterward to persist.
        """
        self.send_command(f"set_cal_name:{name}")

    def get_cal_name(self) -> str:
        """Return the calibration name currently stored in firmware memory."""
        self.send_command("get_cal_name")
        raw = self._receive(wait_until_return=True)
        import re
        m = re.search(r'CalibrationName:\s*(.*)', raw or "")
        return m.group(1).strip() if m else ""

    # -------------------------------------------------------------------------
    # Bluetooth management commands
    # -------------------------------------------------------------------------

    def bt_status(self) -> str:
        """Query the firmware BluetoothManager for the HC-05 connection state.

        Requires the HC-05 STATE pin to be wired and declared as BT_STATE_PIN
        in firmware config.h.  When the STATE pin is -1 (not wired), the firmware
        always returns ``'disconnected'``.

        Returns:
            ``'connected'``, ``'disconnected'``, or ``'unknown'`` if the firmware
            does not support the ``bt_status`` command.
        """
        self.send_command("bt_status")
        raw = self._receive(wait_until_return=True) or ""
        if "bt_status:connected" in raw.lower():
            return "connected"
        if "bt_status:disconnected" in raw.lower():
            return "disconnected"
        return "unknown"

    def bt_configure(self, device_name: str, pin: str = "1234") -> bool:
        """Send AT configuration commands to the HC-05 module.

        The module must be idle (not paired to a host) for mini-AT mode to work.
        For full AT mode, pull EN HIGH before powering the module and set
        BT_EN_PIN in firmware config.h.

        Args:
            device_name: Bluetooth device name the HC-05 will advertise.
            pin: 4-digit pairing PIN (default ``"1234"``).

        Returns:
            True if firmware reports success.
        """
        self.send_command(f"bt_configure:{device_name}:{pin}")
        raw = self._receive(wait_until_return=True) or ""
        return "bt_configure:ok" in raw.lower()

    def flash_calibration_to_firmware(self, profile_name: str,
                                       profiles_dir: str | None = None,
                                       name_to_id: dict | None = None,
                                       side: str | None = None) -> None:
        """Apply calibration profile to firmware and persist to EEPROM.

        Args:
            profile_name: Name of the calibration profile (JSON file stem).
            profiles_dir: Directory containing profile JSONs (default: repo profiles/).
            name_to_id: Motor name → Dynamixel ID mapping for dual firmware.
            side: "left", "right", or None (default) to flash both sides.
                  Use when left and right profiles have different gesture fractions.

        Steps:
          1. apply_calibration() — pushes per-motor home / limits / flip
          2. set_gesture_cal for each gesture/state/joint using values from
             the profile's "gestures" key, or built-in defaults if absent
          3. set_cal_name(profile_name)
          4. save_gesture_cal() — writes fractions + name to EEPROM
        """
        import json, os

        self.apply_calibration(profile_name, profiles_dir=profiles_dir,
                               name_to_id=name_to_id)

        # Resolve gesture fractions: prefer profile "gestures" section, else defaults.
        if profiles_dir is None:
            repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.abspath(__file__)))))
            profiles_dir = os.path.join(repo_root, "examples", "calibration", "profiles")
        filepath = os.path.join(profiles_dir, f"{profile_name}.json")
        gestures_data: dict = {}
        if os.path.exists(filepath):
            with open(filepath) as f:
                gestures_data = json.load(f).get("gestures", {})

        fractions = gestures_data if gestures_data else self._DEFAULT_GESTURE_FRACTIONS

        for gesture, states in fractions.items():
            for state, joints in states.items():
                for joint, value in joints.items():
                    self.set_gesture_cal_value(gesture, state, joint, float(value),
                                               side=side)

        self.set_cal_name(profile_name)
        self.save_gesture_cal()
        self.logger(f"Calibration '{profile_name}' flashed to firmware and saved to EEPROM.")

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