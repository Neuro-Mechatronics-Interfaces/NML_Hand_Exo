import math
import re
import struct
import time
import numpy as np

from ._interfaces import BaseComm
from ._gesture_protocol import ANGLE_ADDRESSABLE_GESTURES


#: Firmware version that introduced the per-joint "rest" state, anchored
#: ``extend`` (EXTEND_* = 0.0 == home), the ``wrist`` gesture, and the
#: ``set_gesture_angle`` command.  Older firmware silently ACKs an unknown
#: gesture state -- ``set_gesture`` replies ``OK:`` whether or not the state
#: resolved -- so the host cannot detect the failure from the reply and must
#: gate on the version instead.
FW_PER_JOINT_REST = (0, 3, 0)

#: The integration line briefly shipped ``set_gesture_angle`` as 0.2.16
#: before the feature branch established the 0.3.x-0.6.x firmware sequence.
#: Keep that backport usable while treating all earlier 0.2.x builds as old.
FW_GESTURE_ANGLE_BACKPORT = (0, 2, 16)

#: Firmware version that added the ``rad`` gesture on the wrist2 motor.
FW_RAD_GESTURE = (0, 3, 1)

#: Firmware version that added the combined-motor current budget.  Before this,
#: ``set_current_lim`` wrote GOAL_CURRENT directly and nothing bounded what the
#: motors drew together; from 0.4.0 it sets the per-motor nominal and the
#: firmware allocator owns GOAL_CURRENT under a fleet-wide cap.
FW_CURRENT_BUDGET = (0, 4, 0)

#: Firmware version that added an atomic per-ID current-position hold while
#: the remaining motors stay in global velocity/current direct control.
FW_AUX_POSITION_HOLD = (0, 6, 2)

#: Firmware version that added an optional per-hold current request and
#: reports the safety-clamped current in the hold acknowledgement.
FW_AUX_POSITION_HOLD_CURRENT = (0, 6, 2)

#: Firmware version that added ``get_gesture_angle`` and re-anchored every
#: gesture percentage on a home -> flexion-endstop axis resolved per motor.
#: Before this, joints whose home sat mid-window (the wrist and wrist2 axes)
#: had every state clamp onto the same limit and never moved, while the command
#: still ACKed -- so there is no reply to detect it from, only the version.
FW_GESTURE_ANGLE_READBACK = (0, 6, 0)

#: Firmware version that added rest-zeroed signed gesture angles and the
#: combined percentage/signed-degree query.
FW_GESTURE_SIGNED_ANGLE = (0, 6, 1)

#: Reported instead of a percentage when a joint sits outside its gesture
#: endpoints: below the 0% extend posture or above the 100% flex posture.
#: Reachable by moving the hand by hand, or by a ``set_angle`` off the axis.
GESTURE_ANGLE_BELOW_RANGE = 101
GESTURE_ANGLE_ABOVE_RANGE = 102

#: Reported when no position is available: every motor the gesture names has
#: less than the firmware's minimum calibrated travel, or the read failed.
#: ``check_limits`` distinguishes the two.
GESTURE_ANGLE_UNAVAILABLE = 255

#: Per-joint gesture name -> minimum firmware that defines it.  Gestures absent
#: from this map (the five digits) exist on every firmware, though they only
#: gained their ``rest`` state at :data:`FW_PER_JOINT_REST`.
GESTURE_MIN_FIRMWARE = {
    "wrist": FW_PER_JOINT_REST,
    "rad": FW_RAD_GESTURE,
}

#: Per-joint gesture name -> first firmware that no longer has it (exclusive).
#: ``rad`` drove the wrist2 motor on its own; from 0.6.0 wrist2 moves with the
#: ``wrist`` gesture, because both motors pull on the same dorsal structure and
#: commanding one alone left the other holding position against it.  Firmware
#: ignores an unknown gesture silently and ``set_gesture`` still replies ``OK:``,
#: so without this the call would look like it worked.
GESTURE_MAX_FIRMWARE = {
    "rad": FW_GESTURE_ANGLE_READBACK,
}

def parse_firmware_version(text: str) -> tuple[int, ...]:
    """Parse a firmware version string into a comparable tuple.

    Accepts the bare ``"0.3.0"`` form returned by :meth:`HandExo.version` as
    well as decorated forms such as ``"Version: 0.3.0;"``.  Returns ``()`` when
    no version-looking token is present, which compares less than every real
    version so unknown firmware is treated as "too old" by feature gates.

    Args:
        text (str): Raw version text from the device.

    Returns:
        tuple[int, ...]: e.g. ``(0, 3, 0)``, or ``()`` if unparseable.

    """
    if not text:
        return ()
    m = re.search(r"(\d+(?:\.\d+)*)", str(text))
    if not m:
        return ()
    return tuple(int(part) for part in m.group(1).split("."))


#: Prefix of the firmware's ``get_gesture_angle`` reply.
GESTURE_ANGLE_PREFIX = "GESTURE_ANGLE:"

#: Prefixes of the firmware's signed-only and combined gesture-angle replies.
GESTURE_SANG_PREFIX = "GESTURE_SANG:"
GESTURE_ANGLES_PREFIX = "GESTURE_ANGLES:"


def parse_gesture_angles(text: str) -> dict[str, int]:
    """Parse a ``GESTURE_ANGLE:`` reply into ``{gesture: code}``.

    The reply is one line of ``name=code`` pairs, e.g.::

        GESTURE_ANGLE: thumb=12 index=0 middle=45 ring=101 pinky=100 wrist=33;

    Deliberately tolerant: a device line can arrive with the command delimiter
    attached, wrapped in surrounding telemetry, or truncated by a dropped USB
    frame.  Whatever pairs are present are returned; anything else is skipped
    rather than raising, since a partial pose is still useful to a poller and a
    missing key is easier to handle than an exception on the hot path.

    Args:
        text (str): Raw reply text, possibly multi-line.

    Returns:
        dict[str, int]: Gesture name -> code, in the order the device sent them.

    """
    if not text:
        return {}
    for line in str(text).splitlines():
        line = line.strip()
        if not line.startswith(GESTURE_ANGLE_PREFIX):
            continue
        body = line[len(GESTURE_ANGLE_PREFIX):].strip().rstrip(";")
        angles: dict[str, int] = {}
        for token in body.split():
            name, sep, value = token.partition("=")
            name = name.strip().lower()
            if not sep or not name:
                continue
            try:
                angles[name] = int(value)
            except ValueError:
                continue
        return angles
    return {}


def parse_gesture_signed_angles(text: str) -> dict[str, float | None]:
    """Parse a ``GESTURE_SANG:`` reply into signed degree deltas.

    ``rest`` is 0 degrees by convention, motion toward ``flex`` is positive,
    and motion toward ``extend`` is negative. ``nan`` is returned as ``None``
    so callers do not need floating-point special-value checks.

    Args:
        text (str): Raw reply text, possibly multi-line.

    Returns:
        dict[str, float | None]: Gesture name -> signed angle in degrees, or
        ``None`` when the firmware could not produce a physical angle.

    """
    if not text:
        return {}
    for line in str(text).splitlines():
        line = line.strip()
        if not line.startswith(GESTURE_SANG_PREFIX):
            continue
        body = line[len(GESTURE_SANG_PREFIX):].strip().rstrip(";")
        angles: dict[str, float | None] = {}
        for token in body.split():
            name, sep, value = token.partition("=")
            name = name.strip().lower()
            if not sep or not name:
                continue
            if value.strip().lower() == "nan":
                angles[name] = None
                continue
            try:
                angle = float(value)
            except ValueError:
                continue
            if math.isfinite(angle):
                angles[name] = angle
        return angles
    return {}


def parse_gesture_angle_pairs(
    text: str,
) -> dict[str, dict[str, int | float | None]]:
    """Parse a combined ``GESTURE_ANGLES:`` reply.

    Each wire token is ``name=<percentage-code>,<signed-degrees>``. The first
    field keeps the exact ``get_gesture_angle`` encoding, including status
    codes 101, 102, and 255; the second uses the rest-zeroed signed convention.

    Args:
        text (str): Raw reply text, possibly multi-line.

    Returns:
        dict: Gesture names mapped to ``fraction`` and ``angle_delta_deg``.
        An unavailable signed angle is represented by ``None``.

    """
    if not text:
        return {}
    for line in str(text).splitlines():
        line = line.strip()
        if not line.startswith(GESTURE_ANGLES_PREFIX):
            continue
        body = line[len(GESTURE_ANGLES_PREFIX):].strip().rstrip(";")
        angles: dict[str, dict[str, int | float | None]] = {}
        for token in body.split():
            name, sep, value = token.partition("=")
            fraction_text, comma, signed_text = value.partition(",")
            name = name.strip().lower()
            if not sep or not comma or not name:
                continue
            try:
                fraction = int(fraction_text)
            except ValueError:
                continue
            signed_angle: float | None
            if signed_text.strip().lower() == "nan":
                signed_angle = None
            else:
                try:
                    signed_angle = float(signed_text)
                except ValueError:
                    continue
                if not math.isfinite(signed_angle):
                    continue
            angles[name] = {
                "fraction": fraction,
                "angle_delta_deg": signed_angle,
            }
        return angles
    return {}


class ProtocolResponseError(RuntimeError):
    """A device reply did not match the response required by a command."""

    def __init__(self, *, command: str, expected: str, raw_response: str):
        self.command = str(command)
        self.expected = str(expected)
        self.raw_response = str(raw_response)
        rendered = self.raw_response.strip() or "<empty response>"
        if len(rendered) > 500:
            rendered = rendered[:497] + "..."
        super().__init__(
            f"Command: {self.command}\n"
            f"Expected: {self.expected}\n"
            f"Received: {rendered}"
        )


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
        # Populated lazily by firmware_version(); None means "not yet queried".
        self._firmware_version: tuple[int, ...] | None = None
        # Optional instrumentation callbacks. They are observational only:
        # callback failures are swallowed so recording can never alter control.
        self._command_observers = []

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
            # Setters frequently emit acknowledgements that legacy callers do
            # not consume. Start every new command at a clean frame boundary so
            # the next query cannot parse an older command's ``OK:`` reply.
            try:
                self.device.flush_input()
            except Exception:
                pass
            self.device.send(cmd)
            self._notify_command_observers(
                command=cmd,
                status="sent",
                source="unknown",
            )
            self.logger(f"Sent: {cmd.strip()}")
            time.sleep(self.send_delay)  # Allow time for the command to be processed
        except Exception as e:
            self._notify_command_observers(
                command=cmd,
                status="failed",
                source="unknown",
                error=str(e),
            )
            raise ConnectionError(
                f"Failed to send command {cmd.strip()!r}: {e}"
            ) from e

    def add_command_observer(self, callback) -> None:
        """Register a non-invasive callback for host command observations.

        The callback receives a new dictionary for each event. Observers are
        instrumentation only and cannot prevent or modify a command.
        """

        if not callable(callback):
            raise TypeError("command observer must be callable")
        observers = getattr(self, "_command_observers", None)
        if observers is None:
            observers = []
            self._command_observers = observers
        if callback not in observers:
            observers.append(callback)

    def remove_command_observer(self, callback) -> None:
        """Remove a previously registered command observer if present."""

        try:
            getattr(self, "_command_observers", []).remove(callback)
        except ValueError:
            pass

    def _notify_command_observers(
        self,
        *,
        command: str,
        status: str,
        source: str = "unknown",
        response: str = "",
        error: str = "",
    ) -> None:
        observers = tuple(getattr(self, "_command_observers", ()))
        if not observers:
            return
        event = {
            "command": str(command).strip().rstrip(";\r\n").strip(),
            "status": str(status),
            "source": str(source),
            "host_wall_time_s": time.time(),
            "host_monotonic_s": time.monotonic(),
        }
        if response:
            event["response"] = str(response).strip()[:500]
        if error:
            event["error"] = str(error).strip()[:500]
        for observer in observers:
            try:
                observer(dict(event))
            except Exception:
                # Instrumentation must never affect a device transaction.
                continue

    def _command_transaction(
        self,
        command: str,
        *,
        expected: str,
        timeout: float = 0.75,
    ) -> str:
        """Send a low-rate safety command and validate its acknowledgement."""
        self.send_command(command)
        raw = self._receive(
            wait_until_return=True,
            timeout=timeout,
            warn_on_timeout=False,
        )
        normalized = raw.strip()
        if (
            not normalized
            or "ERROR:" in normalized.upper()
            or expected.lower() not in normalized.lower()
        ):
            self._notify_command_observers(
                command=command,
                status="rejected",
                source="unknown",
                response=raw,
            )
            raise ProtocolResponseError(
                command=command,
                expected=expected,
                raw_response=raw,
            )
        self._notify_command_observers(
            command=command,
            status="acknowledged",
            source="unknown",
            response=normalized,
        )
        return normalized

    def _receive(
        self,
        wait_until_return: bool = False,
        timeout: float | None = None,
        *,
        warn_on_timeout: bool = True,
    ) -> str:
        """
        Reads a response from the exoskeleton over the serial connection.
        
        Returns:
            str: The response from the exoskeleton, or an empty string if no response.

        """
        kwargs = {"wait_until_return": wait_until_return}
        if timeout is not None:
            kwargs["timeout"] = timeout
        try:
            return self.device.receive(
                **kwargs, warn_on_timeout=warn_on_timeout
            )
        except TypeError:
            # Preserve compatibility with custom BaseComm implementations.
            return self.device.receive(**kwargs)

    def get_fast_telemetry(
        self,
        timeout: float = 0.5,
        motor_ids: list[int] | tuple[int, ...] | None = None,
    ) -> dict[int, dict[str, float | int | bool]]:
        """Read the firmware's compact binary telemetry frame.

        The firmware emits an ``NX`` frame with one fixed-size record per motor.
        Records are keyed by Dynamixel ID and include relative/absolute angles,
        present current, raw velocity, and raw position ticks.
        """
        stream_getter = getattr(self.device, "fast_telemetry_device", None)
        serial_dev = stream_getter() if callable(stream_getter) else None
        if serial_dev is None:
            raise RuntimeError(
                "get_fast_telemetry requires a serial transport with raw-byte support"
            )

        ids = "all" if not motor_ids else ":".join(str(int(mid)) for mid in motor_ids)
        try:
            serial_dev.reset_input_buffer()
        except Exception:
            pass
        self.send_command(f"get_telemetry_fast:{ids}")

        header_fmt = "<2sBBBHIH"
        record_fmt = "<BBhiiii"
        header_len = struct.calcsize(header_fmt)
        record_len = struct.calcsize(record_fmt)

        deadline = time.monotonic() + timeout
        prefix = bytearray()
        while time.monotonic() < deadline:
            byte = serial_dev.read(1)
            if not byte:
                time.sleep(0.001)
                continue
            prefix.extend(byte)
            if len(prefix) > 2:
                prefix = prefix[-2:]
            if bytes(prefix) == b"NX":
                break
        else:
            raise TimeoutError("Timed out waiting for fast telemetry frame")

        remaining_header = serial_dev.read(header_len - 2)
        if len(remaining_header) != header_len - 2:
            raise TimeoutError("Timed out reading fast telemetry header")
        header = b"NX" + remaining_header
        magic, version, flags, count, payload_len, timestamp_ms, checksum = struct.unpack(
            header_fmt, header
        )
        if magic != b"NX" or version != 1:
            raise ValueError("Unsupported fast telemetry frame")

        payload = serial_dev.read(payload_len)
        if len(payload) != payload_len:
            raise TimeoutError("Timed out reading fast telemetry payload")
        calc = (sum(header[: header_len - 2]) + sum(payload)) & 0xFFFF
        if calc != checksum:
            raise ValueError("Fast telemetry checksum mismatch")

        records: dict[int, dict[str, float | int | bool]] = {}
        offset = 0
        for _ in range(count):
            if offset + record_len > len(payload):
                break
            mid, error, current_mA, velocity_raw, position_ticks, absolute_cdeg, relative_cdeg = (
                struct.unpack_from(record_fmt, payload, offset)
            )
            records[mid] = {
                "id": mid,
                "error": bool(error),
                "current": current_mA,
                "velocity_raw": velocity_raw,
                "position_ticks": position_ticks,
                "absolute_angle": absolute_cdeg / 100.0,
                "angle": relative_cdeg / 100.0,
                "timestamp_ms": timestamp_ms,
                "flags": flags,
            }
            offset += record_len
        return records

    def configure_shadow_telemetry(
        self,
        motor_ids: list[int] | tuple[int, ...],
        *,
        interval_ms: int = 2,
    ) -> str:
        """Configure read-only firmware sampling for explicit Dynamixel IDs.

        This command does not enable torque, change a mode, or write a motor
        register. Sampling remains stopped until :meth:`start_shadow_telemetry`.
        """
        ids = [int(mid) for mid in motor_ids]
        if not ids or any(mid <= 0 or mid > 253 for mid in ids):
            raise ValueError("motor_ids must contain positive explicit DXL IDs")
        if len(ids) != len(set(ids)):
            raise ValueError("motor_ids must be unique")
        if len(ids) > 9:
            raise ValueError("shadow telemetry supports at most 9 motor IDs")
        interval = int(interval_ms)
        if interval <= 0:
            raise ValueError("interval_ms must be positive")
        command = "shadow_config:" + ":".join(
            [str(interval), *(str(mid) for mid in ids)]
        )
        return self._command_transaction(
            command, expected="OK: shadow_config", timeout=1.0
        )

    def start_shadow_telemetry(self) -> str:
        """Start read-only sampling; firmware accepts this in VELOCITY mode only."""
        return self._command_transaction(
            "shadow_start", expected="OK: shadow_start", timeout=1.0
        )

    def stop_shadow_telemetry(self) -> str:
        """Stop read-only sampling without changing any motor state."""
        return self._command_transaction(
            "shadow_stop", expected="OK: shadow_stop", timeout=1.0
        )

    def get_shadow_telemetry(self, timeout: float = 0.5) -> dict:
        """Return the firmware's buffered Phase-1 shadow evidence snapshot."""
        command = "shadow_status"
        self.send_command(command)
        raw = self._receive(
            wait_until_return=True,
            timeout=timeout,
            warn_on_timeout=False,
        )
        if not raw.strip() or re.search(r"(?:^|\n)\s*ERROR:", raw, re.IGNORECASE):
            raise ProtocolResponseError(
                command=command,
                expected="SHADOW header and per-motor records",
                raw_response=raw,
            )

        header_match = re.search(r"SHADOW:\s*\{([^}]*)\}", raw, re.IGNORECASE)
        if header_match is None:
            raise ProtocolResponseError(
                command=command,
                expected="SHADOW header",
                raw_response=raw,
            )

        header: dict[str, object] = {}
        for item in header_match.group(1).split(","):
            key, sep, value = item.partition(":")
            if not sep:
                continue
            key = key.strip()
            value = value.strip()
            if value.lower() in {"true", "false"}:
                header[key] = value.lower() == "true"
            else:
                try:
                    header[key] = int(value)
                except ValueError:
                    header[key] = value

        parsed = self._parse_motor_data_block(raw)
        records: dict[int, dict[str, object]] = {}
        numeric_fields = {
            "current",
            "position_ticks",
            "absolute_angle",
            "angle",
            "velocity_deg_s",
            "current_sample_ms",
            "position_sample_ms",
            "error",
        }
        for mid, values in parsed.items():
            record = {"id": int(mid)}
            for field in numeric_fields:
                if field in values and values[field] is not None:
                    value = values[field]
                    if field in {
                        "position_ticks",
                        "current_sample_ms",
                        "position_sample_ms",
                        "error",
                    }:
                        record[field] = int(float(value))
                    else:
                        record[field] = float(value)
            records[int(mid)] = record
        if int(header.get("count", 0)) and not records:
            raise ProtocolResponseError(
                command=command,
                expected="per-motor shadow records",
                raw_response=raw,
            )
        return {"meta": header, "records": records, "raw": raw}

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
        command_name = command or f"get_{attr}"
        command_text = f"{command_name}:{motor_id}"
        self.send_command(command_text)
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
                raise ProtocolResponseError(
                    command=command_text,
                    expected=f"motor data containing ID {motor_id} and {attr!r}",
                    raw_response=raw,
                )
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
            # Values such as ``limits: [-30, 45]`` contain commas, so a plain
            # split would truncate the value. Match bracketed values atomically.
            fields = re.finditer(
                r"(?:^|,)\s*([A-Za-z_]+)\s*:\s*(\[[^\]]*\]|[^,]*)",
                data_block,
            )
            for field in fields:
                key, val = field.group(1).strip().lower(), field.group(2).strip()

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
        # A feature gate must not consume a delayed reply from an earlier
        # fire-and-forget command. Both SerialComm and DualSerialComm expose
        # this at the communication-layer boundary.
        try:
            self.device.flush_input()
        except Exception:
            pass
        self.send_command("version")
        response = self._receive(wait_until_return=True)

        if response and ':' in response:
            return response.strip().split(':', 1)[1].strip()
        return ""

    def firmware_version(self, refresh: bool = False) -> tuple[int, ...]:
        """
        Return the device firmware version as a comparable tuple, e.g. (0, 3, 0).

        The result is cached because feature gates call this on every guarded
        command and a serial round-trip per call would dominate their cost.
        Returns ``()`` if the device did not answer or the reply was unparseable;
        that value compares less than any real version, so gates fail closed.

        Args:
            refresh (bool): Re-query the device instead of using the cache.

        Returns:
            tuple[int, ...]: Parsed version, or ``()`` if unknown.

        """
        if refresh or self._firmware_version is None:
            self._firmware_version = parse_firmware_version(self.version())
        return self._firmware_version

    def firmware_at_least(self, minimum: tuple[int, ...]) -> bool:
        """
        Check whether the connected firmware is at least ``minimum``.

        Args:
            minimum (tuple[int, ...]): Version to compare against, e.g. ``(0, 3, 0)``.

        Returns:
            bool: True if the device reports a version >= ``minimum``.

        """
        return self.firmware_version() >= minimum

    def _require_firmware(self, minimum: tuple[int, ...], feature: str) -> None:
        """
        Raise if the connected firmware predates ``feature``.

        Guards commands whose absence the firmware does not report: ``set_gesture``
        ACKs with ``OK:`` even for a state it could not resolve, so an ungated
        call to old firmware looks successful while the hand never moves.

        Args:
            minimum (tuple[int, ...]): Required firmware version.
            feature (str): Human-readable feature name for the error message.

        Raises:
            RuntimeError: If the device firmware is older than ``minimum``.

        """
        actual = self.firmware_version()
        if actual >= minimum:
            return
        want = ".".join(str(p) for p in minimum)
        have = ".".join(str(p) for p in actual) if actual else "unknown"
        raise RuntimeError(
            f"{feature} requires firmware >= {want}, but the device reports {have}. "
            "Reflash src/cpp/nml_hand_exo, or use the flex/extend states only."
        )

    def require_gesture_angle_support(self) -> None:
        """Raise unless the device supports continuous gesture positioning."""
        actual = self.firmware_version()
        if actual == FW_GESTURE_ANGLE_BACKPORT:
            return
        self._require_firmware(FW_PER_JOINT_REST, "set_gesture_angle")

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

        # `info` is the authoritative connect-time handshake. Cache its
        # version so subsequent guarded commands (combined current budget,
        # gesture readback, etc.) do not perform a second serial transaction
        # or incorrectly report the already-seen firmware as unknown.
        parsed_version = parse_firmware_version(info.get('version', ''))
        if parsed_version:
            self._firmware_version = parsed_version

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
        return self._get_motor_attribute(
            'baudrate', motor_id, wait_until_return=True, command="get_baud"
        )

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

    def get_motor_velocity_limit(self, motor_id: (int or str) = 'all'):
        """Read the position-profile velocity limit in rpm."""
        raw = self._get_motor_attribute(
            'velocity', motor_id, True, command='get_goal_velocity'
        )
        if isinstance(raw, dict):
            return {
                int(dxl_id): float(value) * 0.229
                for dxl_id, value in raw.items()
                if value is not None
            }
        return float(raw) * 0.229

    def set_motor_velocity_limit(self, motor_id: int, velocity_rpm: float):
        """Set one motor's position-profile limit from a value expressed in rpm."""
        rpm = float(velocity_rpm)
        if not math.isfinite(rpm) or rpm <= 0:
            raise ValueError("velocity_rpm must be positive and finite")
        raw = max(1, int(round(rpm / 0.229)))
        self.send_command(f"set_goal_velocity:{int(motor_id)}:{raw}")

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
        return self._get_motor_attribute(
            'acceleration', motor_id, True, command="get_goal_acceleration"
        )

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
        return self._get_motor_attribute(
            'current_limit', motor_id, True, command="get_current_lim"
        )

    def set_current_limit(self, motor_id: (int or str), current_limit: float):
        """
        Sets the per-motor current limit for the specified motor.

        This is a PER-MOTOR knob and does not bound what the motors draw
        together.  From firmware 0.4.0 it sets the motor's *nominal* effort and
        the firmware's budget allocator owns GOAL_CURRENT, so the value actually
        applied may be lower while the fleet is near its combined cap.  See
        :meth:`set_total_current_limit`.

        Args:
            motor_id (int or str): ID of the motor to set the current limit for.
            current_limit (float): Desired current limit in mA.

        Returns:
            None

        """
        self.send_command(f"set_current_lim:{motor_id}:{current_limit}")

    def set_total_current_limit(self, budget_mA: float):
        """
        Set the COMBINED current budget across all motors, in mA.

        Per-motor limits cannot protect the supply: N motors each honouring a
        200 mA limit still draw up to N*200 mA together, which is what browns
        out the board when a posture commands every joint at once.  This caps
        the aggregate.  The firmware clamps a new budget to the range it can
        actually satisfy, so read it back with :meth:`get_total_current_limit`.

        Args:
            budget_mA (float): Combined budget in mA.

        Returns:
            None

        Raises:
            RuntimeError: If the device firmware is older than 0.4.0.
            ValueError: If ``budget_mA`` is not a positive number.

        """
        self._require_firmware(FW_CURRENT_BUDGET, "The combined current budget")
        try:
            budget = float(budget_mA)
        except (TypeError, ValueError):
            raise ValueError(f"budget_mA must be numeric, got {budget_mA!r}")
        if budget <= 0:
            raise ValueError(f"budget_mA must be positive, got {budget:g}")
        self.send_command(f"set_total_current_lim:{budget:g}")

    def get_total_current_limit(self) -> float:
        """
        Read the combined current budget across all motors, in mA.

        Returns:
            float: Budget in mA, or ``float('nan')`` if the device did not answer.

        Raises:
            RuntimeError: If the device firmware is older than 0.4.0.

        """
        self._require_firmware(FW_CURRENT_BUDGET, "The combined current budget")
        self.send_command("get_total_current_lim")
        response = self._receive(wait_until_return=True)
        match = re.search(r"([-+]?\d*\.?\d+)", response or "")
        return float(match.group(1)) if match else float("nan")

    def set_hold_current(self, hold_mA: float):
        """
        Set the current a settled or load-shed motor is allowed, in mA.

        Every motor may sit at this value simultaneously, so the firmware caps
        it at ``budget / n_motors``.

        Args:
            hold_mA (float): Hold current in mA.

        Returns:
            None

        Raises:
            RuntimeError: If the device firmware is older than 0.4.0.
            ValueError: If ``hold_mA`` is negative or not a number.

        """
        self._require_firmware(FW_CURRENT_BUDGET, "The combined current budget")
        try:
            hold = float(hold_mA)
        except (TypeError, ValueError):
            raise ValueError(f"hold_mA must be numeric, got {hold_mA!r}")
        if hold < 0:
            raise ValueError(f"hold_mA must be non-negative, got {hold:g}")
        self.send_command(f"set_hold_current:{hold:g}")

    def set_current_governor(self, enabled: bool):
        """
        Enable or disable the closed-loop half of budget enforcement.

        Disabling stops all current sampling and leaves the conservative
        feed-forward clamp in charge: strictly safer for the supply, but every
        motor is allocated its static worst-case share whether or not the fleet
        is actually drawing that much.  It does NOT remove the budget.

        Args:
            enabled (bool): True to run the governor, False for static clamping.

        Returns:
            None

        Raises:
            RuntimeError: If the device firmware is older than 0.4.0.

        """
        self._require_firmware(FW_CURRENT_BUDGET, "The combined current budget")
        self.send_command(f"set_current_governor:{'on' if enabled else 'off'}")

    def current_status(self, timeout: float = 2.0) -> dict:
        """
        Read the budget state: cap, measured aggregate draw and per-motor allocation.

        Returns
        -------
        dict
            Keys ``total_budget_mA``, ``hold_current_mA``, ``governor`` (bool),
            ``measured_total_mA`` (None until the firmware has sampled),
            ``scale`` (fraction of nominal effort currently allowed),
            ``measurement_trusted`` (bool) and ``motors`` keyed by DXL ID with
            ``nominal_mA`` / ``applied_mA`` / ``state``.  ``applied`` below
            ``nominal`` means the budget is actively clamping.

        Raises:
            RuntimeError: If the device firmware is older than 0.4.0.

        """
        self._require_firmware(FW_CURRENT_BUDGET, "The combined current budget")
        self.send_command("current_status")
        raw = self._receive(wait_until_return=True, timeout=timeout) or ""

        status: dict = {"_raw": raw, "motors": {}}
        scalars = {
            "total_budget_mA": (r"total_budget_mA:\s*(\d+)", int),
            "hold_current_mA": (r"hold_current_mA:\s*(\d+)", int),
            "scale": (r"scale:\s*([\d.]+)", float),
        }
        for key, (pattern, cast) in scalars.items():
            match = re.search(pattern, raw)
            if match:
                status[key] = cast(match.group(1))

        match = re.search(r"governor:\s*(on|off)", raw)
        if match:
            status["governor"] = match.group(1) == "on"
        match = re.search(r"measurement_trusted:\s*(true|false)", raw)
        if match:
            status["measurement_trusted"] = match.group(1) == "true"

        # "n/a" until the firmware has taken its first sample; None is a clearer
        # signal to a caller than 0, which would look like "drawing nothing".
        match = re.search(r"measured_total_mA:\s*(n/a|\d+)", raw)
        if match:
            token = match.group(1)
            status["measured_total_mA"] = None if token == "n/a" else int(token)

        for line in raw.splitlines():
            match = re.search(
                r"id:\s*(\d+),\s*nominal_mA:\s*(\d+),\s*applied_mA:\s*(\d+),"
                r"\s*state:\s*(\w+)",
                line,
            )
            if match:
                status["motors"][int(match.group(1))] = {
                    "nominal_mA": int(match.group(2)),
                    "applied_mA": int(match.group(3)),
                    "state": match.group(4),
                }
        return status

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

    def hold_motor_position(
        self,
        motor_id: int,
        relative_angle: float,
        hold_current_mA: float | None = None,
    ):
        """Hold one explicit DXL ID at a limit-clamped relative angle."""
        self._require_firmware(FW_AUX_POSITION_HOLD, "Auxiliary position hold")
        dxl_id = int(motor_id)
        angle = float(relative_angle)
        if dxl_id <= 0:
            raise ValueError("motor_id must be a positive explicit DXL ID")
        if not math.isfinite(angle):
            raise ValueError("relative_angle must be finite")
        command = f"hold_position:{dxl_id}:{angle}"
        if hold_current_mA is not None:
            current = float(hold_current_mA)
            if not math.isfinite(current) or current <= 0:
                raise ValueError("hold_current_mA must be positive and finite")
            self._require_firmware(
                FW_AUX_POSITION_HOLD_CURRENT,
                "Per-hold current control",
            )
            command += f":{current:g}"
        response = self._command_transaction(
            command, expected="OK: hold_position"
        )
        if hold_current_mA is not None and "current_mA=" not in response:
            # An earlier 0.6.2 build accepts and ignores the extra argument.
            # Release immediately rather than claim the requested effort was
            # applied when the acknowledgement cannot prove it.
            try:
                self.release_motor_hold(dxl_id)
            except Exception:
                pass
            raise ProtocolResponseError(
                command=command,
                expected="hold acknowledgement containing current_mA=<applied>",
                raw_response=response,
            )
        return response

    def release_motor_hold(self, motor_id: int):
        """Disable one held DXL ID and restore the current global mode."""
        self._require_firmware(FW_AUX_POSITION_HOLD, "Auxiliary position hold")
        dxl_id = int(motor_id)
        if dxl_id <= 0:
            raise ValueError("motor_id must be a positive explicit DXL ID")
        command = f"release_hold:{dxl_id}"
        return self._command_transaction(command, expected="OK: release_hold")

    def set_direct_command_timeout(self, timeout_ms: int):
        """Set the firmware direct-control watchdog timeout."""
        command = f"set_command_timeout:{int(timeout_ms)}"
        return self._command_transaction(
            command, expected="Direct command timeout:"
        )

    def set_control_mode(self, mode: str):
        """Set the global motor mode; firmware leaves torque disabled."""
        normalized = mode.strip().lower()
        allowed = {"position", "current_position", "velocity", "current"}
        if normalized not in allowed:
            raise ValueError(f"Unsupported control mode: {mode}")
        command = f"set_control_mode:all:{normalized}"
        return self._command_transaction(
            command, expected=f"Motor control mode: {normalized}"
        )

    def get_motor_limits(self, motor_id: (int or str) = 'all') -> tuple:
        """
        Retrieves the limits for the specified motor, including minimum and maximum angles.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            tuple: A tuple containing the minimum and maximum angles of the motor.

        """
        return self._get_motor_attribute(
            'limits', motor_id, True, command="get_motor_limits"
        )

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

        Per-joint gestures (thumb/index/middle/ring/pinky/wrist) accept
        ``extend``, ``rest`` and ``flex``.  ``rest`` needs firmware >= 0.3.0;
        ``rad`` existed only between 0.3.1 and 0.6.0, where its motor was folded
        into ``wrist``.  Firmware ACKs a gesture or state it cannot resolve, so
        this raises on either side of that window instead of letting the caller
        believe a move happened.

        Args:
            gesture (str): Desired gesture (e.g., "grasp", "index", "wrist").
            state (str): Desired state (e.g., "open", "close", "extend", "rest", "flex").

        Returns:
            None

        Raises:
            RuntimeError: If the gesture/state pair needs newer firmware.

        """
        if str(state).strip().lower() == "rest":
            self._require_firmware(FW_PER_JOINT_REST, "The per-joint 'rest' state")
        self._require_gesture_firmware(gesture)
        self.send_command(f"set_gesture:{gesture}:{state}")

    def _require_gesture_firmware(self, gesture: str) -> None:
        """
        Raise if this gesture postdates -- or has been removed from -- the
        connected firmware.

        Both directions matter for the same reason: firmware ACKs a gesture it
        cannot resolve, so neither "too old" nor "no longer exists" is visible
        from the reply.

        Args:
            gesture (str): Gesture name, checked against
                :data:`GESTURE_MIN_FIRMWARE` and :data:`GESTURE_MAX_FIRMWARE`.

        Raises:
            RuntimeError: If the device firmware does not define the gesture.

        """
        name = str(gesture).strip().lower()
        minimum = GESTURE_MIN_FIRMWARE.get(name)
        if minimum is not None:
            self._require_firmware(minimum, f"The {name!r} gesture")
        removed = GESTURE_MAX_FIRMWARE.get(name)
        if removed is not None and self.firmware_version() >= removed:
            gone = ".".join(str(p) for p in removed)
            raise RuntimeError(
                f"The {name!r} gesture was removed in firmware {gone}; the "
                "device would ACK it and do nothing. Use 'wrist', which drives "
                "both dorsal wrist motors together."
            )

    def set_gesture_angle(self, gesture: str, percent: float):
        """
        Position a per-joint gesture anywhere between its two end postures.

        Continuous generalization of the extend/rest/flex states: ``percent``
        interpolates the gesture between its own endpoints, so

        - ``0``   -> exactly ``set_gesture(gesture, 'extend')``
        - ``50``  -> halfway between the extend and flex postures
        - ``100`` -> exactly ``set_gesture(gesture, 'flex')``

        Each motor the gesture names travels its own share, so a gesture driving
        several motors -- the thumb, or the coupled ``wrist`` pair -- keeps the
        ratio between them at every percentage.  Retuning ``EXTEND_*``/``FLEX_*``
        in ``config.h`` moves the axis with them, so a host's percentages keep
        meaning the same postures across a retune.

        Note that ``0`` is the extend posture, **not** home: with a non-zero
        ``EXTEND_*`` a hand parked at home sits below 0% and
        :meth:`get_gesture_angle` reports it as out of range.

        :meth:`get_gesture_angle` is the exact inverse (firmware >= 0.6.0).

        Args:
            gesture (str): A per-joint gesture name; see
                :data:`ANGLE_ADDRESSABLE_GESTURES`.  Multi-joint postures
                (grasp, keygrip, pinch_*) are not addressable this way.
            percent (float): Position in [0, 100].  The firmware clamps values
                outside that range.

        Returns:
            None

        Raises:
            RuntimeError: If the device firmware is older than 0.3.0, or does
                not define the gesture (``rad`` exists only in 0.3.1 - 0.5.x).
            ValueError: If ``percent`` is not a number.

        """
        name = str(gesture).strip().lower()
        if name not in ANGLE_ADDRESSABLE_GESTURES and name != "rad":
            supported = ", ".join(sorted(ANGLE_ADDRESSABLE_GESTURES))
            raise ValueError(
                f"gesture must be one of {supported}, got {gesture!r}"
            )
        try:
            pct = float(percent)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"percent must be numeric, got {percent!r}") from exc
        if not math.isfinite(pct):
            raise ValueError(f"percent must be finite, got {percent!r}")
        self.require_gesture_angle_support()
        if self.firmware_version() != FW_GESTURE_ANGLE_BACKPORT or name == "rad":
            self._require_gesture_firmware(name)
        if not 0.0 <= pct <= 100.0:
            self.logger(
                f"[set_gesture_angle] {pct:g}% is outside [0, 100]; "
                "firmware will clamp it.",
                warning=True,
            )
        self.send_command(f"set_gesture_angle:{name}:{pct:g}")

    def get_gesture_angle(
        self, gesture: str = "all", timeout: float = 1.0
    ) -> dict[str, int]:
        """
        Read where each per-joint gesture currently sits on its 0-100 axis.

        The read-back half of :meth:`set_gesture_angle`, on the same axis: a
        gesture commanded to 40 reads back 40 once it arrives, ``extend`` reads
        back as 0 and ``flex`` as 100.  ``rest`` reads back wherever its
        ``REST_*`` constants place it between the two.

        Values are integers so the out-of-range signals share the encoding:

        - ``0`` to ``100`` -- position between the extend and flex postures
        - :data:`GESTURE_ANGLE_BELOW_RANGE` (101) -- past the extend end
        - :data:`GESTURE_ANGLE_ABOVE_RANGE` (102) -- past the flex end
        - :data:`GESTURE_ANGLE_UNAVAILABLE` (255) -- no position available

        A hand parked at home reads 101 whenever ``EXTEND_*`` is non-zero, since
        home sits below the extend posture.

        A gesture spanning several motors (the thumb, the coupled ``wrist``
        pair, or any joint on a dual build) reports the mean of the per-motor
        percentages, skipping joints that cannot carry one.

        Args:
            gesture (str): A single gesture name, or ``"all"``.
            timeout (float): Seconds to wait for the reply.

        Returns:
            dict[str, int]: Gesture name -> code, in firmware order.  Empty if
            the device did not answer.

        Raises:
            RuntimeError: If the device firmware is older than 0.6.0.

        """
        self._require_firmware(FW_GESTURE_ANGLE_READBACK, "get_gesture_angle")
        name = str(gesture).strip().lower() or "all"
        if name != "all":
            self._require_gesture_firmware(name)
        self.send_command(f"get_gesture_angle:{name}")
        return parse_gesture_angles(
            self._receive(wait_until_return=True, timeout=timeout)
        )

    def get_gesture_sang(
        self, gesture: str = "all", timeout: float = 1.0
    ) -> dict[str, float | None]:
        """Read OpenSim-style signed gesture angles in degrees.

        The first motor named by each gesture supplies the calibrated physical
        degree scale. Its ``rest`` state is 0 degrees; positions toward
        ``flex`` are positive and positions toward ``extend`` are negative.
        Multi-motor gesture percentages are still aggregated as documented by
        :meth:`get_gesture_angle`, then mapped to that first-motor scale.

        Args:
            gesture (str): A single angle-addressable gesture, or ``"all"``.
            timeout (float): Seconds to wait for the reply.

        Returns:
            dict[str, float | None]: Gesture name -> signed degree delta from
            rest. ``None`` means no signed angle was available.

        Raises:
            RuntimeError: If the device firmware is older than 0.6.1.

        """
        self._require_firmware(FW_GESTURE_SIGNED_ANGLE, "get_gesture_sang")
        name = str(gesture).strip().lower() or "all"
        if name != "all":
            self._require_gesture_firmware(name)
        self.send_command(f"get_gesture_sang:{name}")
        return parse_gesture_signed_angles(
            self._receive(wait_until_return=True, timeout=timeout)
        )

    def get_gesture_angles(
        self, gesture: str = "all", timeout: float = 1.0
    ) -> dict[str, dict[str, int | float | None]]:
        """Read percentage codes and signed degree deltas together.

        This is the combined form of :meth:`get_gesture_angle` and
        :meth:`get_gesture_sang`, produced from one batched motor read. Each
        result has ``fraction`` (the legacy 0-100/status code) and
        ``angle_delta_deg`` (rest-zeroed signed degrees, or ``None``).

        Args:
            gesture (str): A single angle-addressable gesture, or ``"all"``.
            timeout (float): Seconds to wait for the reply.

        Returns:
            dict: Gesture names mapped to ``fraction`` and
            ``angle_delta_deg`` fields.

        Raises:
            RuntimeError: If the device firmware is older than 0.6.1.

        """
        self._require_firmware(FW_GESTURE_SIGNED_ANGLE, "get_gesture_angles")
        name = str(gesture).strip().lower() or "all"
        if name != "all":
            self._require_gesture_firmware(name)
        self.send_command(f"get_gesture_angles:{name}")
        return parse_gesture_angle_pairs(
            self._receive(wait_until_return=True, timeout=timeout)
        )

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
