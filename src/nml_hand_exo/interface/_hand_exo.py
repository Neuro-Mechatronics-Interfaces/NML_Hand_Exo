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
                 auto_connect=False, verbose: bool = False):
        """ 
        Initializes the HandExo interface.
        
        Args:
            name (str): Name of the exoskeleton instance.
            command_delimiter (str): Delimiter used to separate commands (default is '\n').
            send_delay (float): Delay in seconds after sending a command to allow processing (default is 0.01).
            verbose (bool): If True, enables verbose logging of commands and responses (default is False).

        """
        self.name = name
        self.device = comm
        self.command_delimiter = command_delimiter
        self.send_delay = send_delay
        self.verbose = verbose
        self.device.verbose = verbose

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
                    motor_info["velocity"] = float(val)
                elif key == "acceleration":
                    motor_info["acceleration"] = float(val)
                elif key == "baudrate":
                    motor_info["baudrate"] = int(val)
                elif key == "home":
                    motor_info["home"] = float(val)
                elif key == "absolute_angle":
                    motor_info["absolute_angle"] = float(val)
                elif key == "current":
                    motor_info["current"] = float(val)
                elif key == "current_limit":
                    motor_info["current_limit"] = float(val)
                else:
                    motor_info[key] = val

            motor_id = int(motor_id_str) if motor_id_str else motor_info.get("id")
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
        Parses the exoskeleton info response into a structured dictionary.
        """
        self.send_command("info")
        raw = self._receive(wait_until_return=True)
        if self.verbose:
            print(f"Raw return: {raw}")

        info = {}
        if not raw:
            return info

        try:
            lines = [line.strip() for line in raw.splitlines() if line.strip()]

            # First line should have name, version, and motor count
            header = lines[0]
            name_match = re.search(r"Name:\s*([^\s]+)", header)
            version_match = re.search(r"Version:\s*([^\s]+)", header)
            motor_count_match = re.search(r"Number of Motors:\s*(\d+)", header)

            if name_match:
                info['name'] = name_match.group(1)
            if version_match:
                info['version'] = version_match.group(1)
            if motor_count_match:
                info['n_motors'] = int(motor_count_match.group(1))

            # Parse each motor line
            for line in lines[1:]:
                motor_match = re.match(r"Motor\s+(\d+):\s*\{(.+?)\}", line)
                if not motor_match:
                    continue

                motor_id = int(motor_match.group(1))
                motor_data = motor_match.group(2)

                # Parse the individual fields inside the { ... }
                motor_info = {}
                for part in motor_data.split(','):
                    key_val = part.strip().split(":", 1)
                    if len(key_val) != 2:
                        continue
                    key, val = key_val[0].strip(), key_val[1].strip()
                    if key == "id":
                        motor_info["id"] = int(val)
                    elif key == "angle":
                        motor_info["angle"] = float(val)
                    elif key == "limits":
                        limits_match = re.findall(r"[-+]?[0-9]*\.?[0-9]+", val)
                        motor_info["limits"] = [float(l) for l in limits_match]
                    elif key == "torque":
                        motor_info["torque"] = float(val)
                    elif key == "enabled":
                        motor_info["enabled"] = val.lower() == "true"
                    else:
                        motor_info[key] = val

                info[f"motor_{motor_id}"] = motor_info

            return info

        except Exception as e:
            print(f"[ERROR] Failed to parse info: {e}")
            return {}

    def get_baudrate(self, motor_id: (int or str) = 'all') -> int:
        """
        Retrieves the current baud rate of the serial connection.

        Returns:
            int: The current baud rate.

        """
        return self._get_motor_attribute('baudrate', motor_id, wait_until_return=True)

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
        self.send_command(f"set_accel:{motor_id}:{acceleration}")

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
            cmd = f"set_absangle:{motor_id}:{angle}"
        else:
            cmd = f"set_absangle:{int(motor_id)}:{angle}"
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

    def set_home(self, motor_id: (int or str), home_angle: float):
        """
        Sets the home angle for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the home angle for.
            home_angle (float): Desired home angle in degrees.

        Returns:
            None

        """
        self.send_command(f"set_home:{motor_id}:{home_angle}")

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

    def set_motor_mode(self, motor_id: (int or str), mode: str):
        """
        Sets the control mode for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the mode for.
            mode (str): Desired control mode (e.g., "position", "velocity", "current_position").

        Returns:
            None

        """
        self.send_command(f"set_motor_mode:{motor_id}:{mode}")

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
        print(full_response)
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