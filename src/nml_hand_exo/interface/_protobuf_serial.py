"""Opt-in USB Protobuf transports.

Never retries a motion RPC: a missing acknowledgement does not prove that the
device failed to act. Schema and engineering units live in protocol/exo_usb.proto.
"""
from __future__ import annotations
import binascii
import math
import re
import secrets
import struct
import threading
import time
import serial

from ._interfaces import DualSerialComm
from ._protocol_routes import (
    PROTOBUF_ONLY_COMMANDS,
    PROTOBUF_PRIMARY_TEXT_COMMANDS,
    PROTOBUF_RPC_COMMANDS,
    protobuf_command_route,
)

MAX_RESPONSE = 2048

_DISCOVERY_FEATURES = (
    ("joint_model_write", "DISCOVERY_CAP_JOINT_MODEL_WRITE"),
    ("batch_motion", "DISCOVERY_CAP_BATCH_MOTION"),
    ("current_limit_batch", "DISCOVERY_CAP_CURRENT_LIMIT_BATCH"),
    ("assist_v2", "DISCOVERY_CAP_ASSIST_V2"),
    ("assist_binary", "DISCOVERY_CAP_ASSIST_BINARY"),
    ("assist_enable", "DISCOVERY_CAP_ASSIST_ENABLE"),
    ("assist_gains", "DISCOVERY_CAP_ASSIST_GAINS"),
)


def discovery_to_info(discovery):
    """Adapt typed HELLO metadata to the established ``HandExo.info`` shape."""
    from . import exo_usb_pb2
    if not discovery.IsInitialized() or discovery.protocol_version != 2:
        raise ValueError('Invalid protobuf-v2 discovery response')
    if discovery.cdc_count not in (1, 2):
        raise ValueError('Invalid protobuf CDC count in discovery response')
    if discovery.cdc_role not in (exo_usb_pb2.CDC_ROLE_PRIMARY_TEXT,
                                  exo_usb_pb2.CDC_ROLE_SECONDARY_BINARY):
        raise ValueError('Invalid protobuf CDC role in discovery response')
    motor_ids = list(discovery.motor_ids)
    if not motor_ids or len(motor_ids) > 18 or any(
        not 1 <= motor_id <= 253 for motor_id in motor_ids
    ) or len(set(motor_ids)) != len(motor_ids):
        raise ValueError('Invalid or duplicate motor ID in discovery response')
    features = [name for name, enum_name in _DISCOVERY_FEATURES
                if discovery.capability_flags & getattr(exo_usb_pb2, enum_name)]
    motors = {motor_id: {'id': motor_id} for motor_id in motor_ids}
    info = {
        'name': 'NMLHandExo',
        'version': discovery.firmware_version,
        'n_motors': len(motor_ids),
        'motors': motors,
        'motor_ids': motor_ids,
        'usb_protocol': 'protobuf-v2',
        'usb_cdc_count': discovery.cdc_count,
        'usb_text_port': 'primary' if discovery.cdc_role == exo_usb_pb2.CDC_ROLE_PRIMARY_TEXT else 'secondary',
        'usb_binary_port': 'secondary' if discovery.cdc_role == exo_usb_pb2.CDC_ROLE_PRIMARY_TEXT else 'primary',
        'usb_features': features,
        'usb_capability_flags': discovery.capability_flags,
        'protocol_schema_version': discovery.schema_version,
        'protocol_version': discovery.protocol_version,
        'side': discovery.side,
    }
    for motor_id, motor in motors.items():
        info[f'motor_{motor_id}'] = motor
    return info


class AssistRejected(RuntimeError):
    """A rejected binary assist operation, including its numeric snapshot."""

    def __init__(self, message, assist_status):
        super().__init__(message)
        self.assist_status = assist_status


class ProtobufFrameReader:
    """Keep partial replies across RPC deadlines and resynchronize by CRC."""

    def __init__(self, expected_version=1):
        self.buffer = bytearray()
        self.expected_version = int(expected_version)

    def read(self, stream, timeout, *, poll_available=False):
        deadline = time.monotonic() + timeout
        discarded = 0
        while True:
            start = self.buffer.find(b'PB')
            if start < 0:
                keep = int(self.buffer.endswith(b'P'))
                discarded += len(self.buffer) - keep
                del self.buffer[:len(self.buffer) - keep]
            elif start:
                discarded += start
                del self.buffer[:start]
            needed = 1
            if self.buffer.startswith(b'PB'):
                needed = 7 - len(self.buffer)
                if len(self.buffer) >= 7:
                    version, size, crc = struct.unpack_from('<BHH', self.buffer, 2)
                    if version != self.expected_version or not 0 < size <= MAX_RESPONSE:
                        discarded += 1
                        del self.buffer[0]
                        continue
                    needed = size + 7 - len(self.buffer)
                    if needed <= 0:
                        payload = bytes(self.buffer[7:7 + size])
                        if binascii.crc_hqx(self.buffer[2:5] + payload, 0xffff) != crc:
                            discarded += 1
                            del self.buffer[0]
                            continue
                        del self.buffer[:7 + size]
                        return payload
            if time.monotonic() >= deadline:
                if not self.buffer:
                    detail = 'no reply bytes received'
                elif len(self.buffer) < 7:
                    detail = f'header {len(self.buffer)}/7 bytes'
                else:
                    detail = f'payload {len(self.buffer) - 7}/{size} bytes'
                if discarded:
                    detail += f'; discarded {discarded} invalid/noise bytes'
                raise TimeoutError(f'Incomplete Protobuf frame: {detail}')
            if poll_available:
                # On Windows, an overlapped pyserial read can still wait
                # inside the OS when the requested size exceeds the bytes
                # currently available. Protobuf-v2 has no competing reader,
                # so poll the port first and only request bytes known to be
                # ready. This keeps the absolute RPC deadline authoritative
                # when the device never answers GET_TELEMETRY_FAST.
                available = getattr(stream, 'in_waiting', None)
                if isinstance(available, int):
                    if available <= 0:
                        time.sleep(min(.001, max(0.0, deadline - time.monotonic())))
                        continue
                    chunk = stream.read(min(needed, available))
                else:
                    chunk = stream.read(needed)
            else:
                chunk = stream.read(needed)
            if chunk:
                self.buffer.extend(chunk)


def encode_frame(payload: bytes, version=1) -> bytes:
    if not 0 < len(payload) <= MAX_RESPONSE:
        raise ValueError("Invalid Protobuf frame length")
    prefix = struct.pack('<BH', int(version), len(payload))
    crc = binascii.crc_hqx(prefix + payload, 0xffff)
    return b'PB' + prefix + struct.pack('<H', crc) + payload


def read_frame(stream, timeout: float, version=1) -> bytes:
    deadline = time.monotonic() + timeout
    def exact(size):
        result = bytearray()
        while len(result) < size and time.monotonic() < deadline:
            result.extend(stream.read(size - len(result)))
        if len(result) != size:
            raise TimeoutError('Incomplete Protobuf frame')
        return bytes(result)
    prefix = b''
    while prefix != b'PB':
        prefix = (prefix + exact(1))[-2:]
    header = exact(5)
    frame_version, size, crc = struct.unpack('<BHH', header)
    if frame_version != int(version) or not 0 < size <= MAX_RESPONSE:
        raise ValueError('Unsupported Protobuf envelope')
    payload = exact(size)
    if binascii.crc_hqx(header[:3] + payload, 0xffff) != crc:
        raise ValueError('Protobuf frame CRC mismatch')
    return payload


class ProtobufSerialComm(DualSerialComm):
    usb_protocol = 'protobuf-v1'
    frame_version = 1
    max_request_frame = 135
    is_split = True
    supports_legacy_fast_telemetry = False
    # Shadow configuration/status remain valid maintenance-plane commands on
    # the primary text CDC. They never share or corrupt the binary endpoint.
    supports_shadow_telemetry = True
    poll_available_before_binary_read = False
    MOTOR_COMMANDS = {'set_current', 'set_velocity', 'set_absolute_angle', 'set_angle', 'stop'}

    def __init__(self, *args, motor_ids=None, model_write_supported=False,
                 batch_motion_supported=False, assist_binary_supported=False,
                 assist_enable_supported=False, assist_gains_supported=False,
                 current_limit_batch_supported=False, **kwargs):
        super().__init__(*args, **kwargs)
        # Loaded only after discovery identifies a Protobuf build.
        try:
            from . import exo_usb_pb2
        except Exception as exc:  # Missing or incompatible generated-code runtime.
            raise RuntimeError('This firmware needs the Protobuf extra: pip install -e ".[protobuf]"') from exc
        self.pb = exo_usb_pb2
        self.motor_ids = list(motor_ids or [])
        self.model_write_supported = bool(model_write_supported)
        self.batch_motion_supported = bool(batch_motion_supported)
        self.assist_binary_supported = bool(assist_binary_supported)
        self.assist_enable_supported = bool(assist_enable_supported)
        self.assist_gains_supported = bool(assist_gains_supported)
        self.current_limit_batch_supported = bool(current_limit_batch_supported)
        self._assist_reasons = {number: name.removeprefix('ASSIST_REASON_').lower()
                               for name, number in self.pb.AssistReason.items()}
        self._rpc_lock = threading.Lock()
        self._sequence = secrets.randbits(32)
        self._frame_reader = ProtobufFrameReader(self.frame_version)
        self._async_event_callback = None

    def set_async_event_callback(self, callback):
        """Receive v2 sequence-zero text events as soon as the RPC reader sees them."""
        self._async_event_callback = callback if callable(callback) else None

    def assist_command(self, command, ids=(), values=(), *, timeout=1.0):
        if command == 'assist_enable' and not self.assist_enable_supported:
            raise RuntimeError('Enable Torque-Assist requires assist-v3 firmware; recompile/reflash the current build.')
        if command == 'assist_gains' and not self.assist_gains_supported:
            raise RuntimeError('Assist P/D gains require the updated firmware; recompile/reflash the current build.')
        if not self.assist_binary_supported:
            raise RuntimeError('This Protobuf firmware lacks assist_binary; recompile/reflash '
                               'the current Protobuf build to use binary assist status.')
        names = {"assist_config": self.pb.ASSIST_CONFIG,
                 "assist_enable": self.pb.ASSIST_ENABLE,
                 "assist_observer": self.pb.ASSIST_OBSERVER,
                 "assist_gains": self.pb.ASSIST_GAINS,
                 "assist_settle_config": self.pb.ASSIST_SETTLE_CONFIG,
                 "assist_calibrate": self.pb.ASSIST_CALIBRATE,
                 "assist_start": self.pb.ASSIST_START,
                 "assist_heartbeat": self.pb.ASSIST_HEARTBEAT,
                 "assist_stop": self.pb.ASSIST_STOP,
                 "assist_status": self.pb.ASSIST_STATUS}
        if command not in names:
            raise ValueError("Unknown assist command")
        response = self._rpc(names[command], ids, values=values, timeout=timeout)
        return self._decode_assist(response)

    def get_assist_status(self, *, timeout=.4):
        return self.assist_command('assist_status', timeout=timeout)

    def _decode_assist(self, response):
        if not response.HasField('assist'):
            raise ValueError('Protobuf assist response has no status snapshot')
        status = response.assist
        if status.state not in range(5) or status.reason not in self._assist_reasons:
            raise ValueError('Invalid binary assist state or reason')
        result = dict(state=status.state, reason=self._assist_reasons[status.reason],
                      reason_code=status.reason, joints={})
        for key in ('reject_id', 'reject_angle_deg', 'reject_velocity_deg_s',
                    'total_current_budget_mA', 'settle_velocity_limit_deg_s',
                    'settle_pose_drift_deg', 'intent_detect_delay_s'):
            if status.HasField(key):
                result[key] = getattr(status, key)
        for joint in status.joints:
            if not 1 <= joint.id <= 253 or joint.id in result['joints'] or len(result['joints']) >= 18:
                raise ValueError('Invalid or duplicate binary assist motor ID')
            result['joints'][joint.id] = {field.name: value for field, value in joint.ListFields()
                                          if field.name != 'id'}
        return result

    def _open_and_probe(self):
        # AutoSerialComm has identified the primary text port via info already.
        self._cmd = serial.Serial(self.cmd_port, self.baudrate, timeout=.02,
                                  write_timeout=self.write_timeout)
        self._telem = serial.Serial(self.telem_port, self.baudrate, timeout=.02,
                                    write_timeout=self.write_timeout)
        self._cmd.reset_input_buffer()
        self._telem.reset_input_buffer()
        self._frame_reader = ProtobufFrameReader(self.frame_version)
        self._reader_error = None
        self._start_reader()

    def text_reply_device(self):
        return self._cmd

    def fast_telemetry_device(self):
        raise RuntimeError('Legacy NX serial polling is disabled for Protobuf firmware')

    def _request_device(self):
        return self._telem

    def _response_device(self):
        return self._telem

    def _exchange(self, request, label, timeout):
        frame = encode_frame(request.SerializeToString(), self.frame_version)
        if len(frame) > self.max_request_frame:
            raise ValueError('Request exceeds firmware frame limit')
        request_device = self._request_device()
        response_device = self._response_device()
        if request_device.write(frame) != len(frame):
            raise ConnectionError('Incomplete Protobuf request write')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                payload = self._frame_reader.read(
                    response_device,
                    deadline-time.monotonic(),
                    poll_available=self.poll_available_before_binary_read,
                )
            except TimeoutError as exc:
                raise TimeoutError(f'{label} on {self.telem_port} '
                                   f'(sequence={request.sequence}): {exc}') from exc
            response = self.pb.Response.FromString(payload)
            if not response.IsInitialized():
                raise ValueError('Incomplete Protobuf response')
            # Sequence zero is reserved for unsolicited v2 events. It must not
            # steal the response belonging to the transaction being awaited.
            if response.sequence == 0:
                if response.HasField('text_reply'):
                    self._publish_reply(response.text_reply)
                    callback = self._async_event_callback
                    if callback is not None:
                        try:
                            callback(response.text_reply)
                        except Exception:
                            # Diagnostic delivery must never break the RPC
                            # that is still waiting for its matching sequence.
                            pass
                continue
            if response.sequence != request.sequence:
                continue # delayed response to an earlier timed-out request
            if response.status != self.pb.OK:
                message = (f'Firmware rejected {label}: '
                           f'{self.pb.Status.Name(response.status)}')
                if response.HasField('assist'):
                    raise AssistRejected(message, self._decode_assist(response))
                raise RuntimeError(message)
            return response
        raise TimeoutError('No matching Protobuf response; command was not retried')

    def _rpc(self, command, ids, value=None, *, timeout=.5, joint_model=None,
             values=None, finger_angles=None, gesture=None):
        ids = list(ids)
        if len(ids) > 18 or len(set(ids)) != len(ids) or any(
            isinstance(mid, bool) or not isinstance(mid, int) or not 1 <= mid <= 253 for mid in ids
        ):
            raise ValueError('Use at most 18 unique explicit integer motor IDs')
        if value is not None and (isinstance(value, bool) or not math.isfinite(value)):
            raise ValueError('Motor command value must be finite')
        # A stale transaction must not make the next telemetry request wait
        # forever.  This is especially important for Assist startup, where a
        # blocked lock acquisition otherwise looks identical to missing motor
        # position/velocity feedback in the GUI.
        lock_timeout = max(0.0, float(timeout))
        if not self._rpc_lock.acquire(timeout=lock_timeout):
            raise TimeoutError(
                f'{self.pb.Command.Name(command)} transaction lock timed out '
                f'after {lock_timeout:.3f} s'
            )
        try:
            self._sequence = (self._sequence + 1) & 0xffffffff
            request = self.pb.Request(sequence=self._sequence, command=command, motor_ids=ids)
            if value is not None:
                request.value = value
            if joint_model is not None:
                request.joint_model.CopyFrom(self.pb.JointModel(**joint_model))
            if values is not None:
                request.values.extend(values)
            if finger_angles is not None:
                request.finger_angles.CopyFrom(self.pb.FingerAngles(**finger_angles))
            if gesture is not None:
                request.gesture.CopyFrom(self.pb.Gesture(**gesture))
            return self._exchange(request, self.pb.Command.Name(command), timeout)
        finally:
            self._rpc_lock.release()

    def get_fast_frame(self, motor_ids=None, *, timeout=.5):
        response = self._rpc(self.pb.GET_TELEMETRY_FAST,
                             self.motor_ids if motor_ids is None else motor_ids, timeout=timeout)
        if not response.HasField('nx_frame'):
            raise ValueError('Protobuf telemetry response has no NX frame')
        return response.nx_frame

    def send_motor_command(self, command: str, motor_id, value=None):
        if command not in self.MOTOR_COMMANDS:
            raise ValueError('Unsupported binary motor command')
        ids = [] if command == 'stop' and motor_id == 'all' else [motor_id]
        if command != 'stop' and value is None:
            raise ValueError('Motor command requires a value')
        self._rpc(self.pb.Command.Value(command.upper()), ids, value)
        # Existing transaction consumers may still expect the familiar ACK text.
        self._publish_reply('OK: ' + command)

    def send_stop(self, motor_ids=()):
        """Stop several explicit IDs in one existing STOP RPC."""
        ids = list(motor_ids)
        if not ids:
            raise ValueError('send_stop requires one or more explicit motor IDs')
        self._rpc(self.pb.STOP, ids)
        self._publish_reply('OK: stop')

    def write_joint_model(self, motor_id, parameters, *, timeout=1.0):
        if not self.model_write_supported:
            raise RuntimeError('This Protobuf firmware lacks joint_model_write; '
                               'recompile/reflash the current Protobuf build. No ASCII fallback was sent.')
        from ._hand_exo import HandExo
        mid = HandExo._joint_model_id(motor_id)
        params = HandExo._joint_model_params(**parameters)
        self._rpc(self.pb.SET_JOINT_MODEL, [mid], timeout=timeout, joint_model=params)

    def _require_batch_motion(self):
        if not self.batch_motion_supported:
            raise RuntimeError('This Protobuf firmware lacks batch_motion; recompile/reflash '
                               'the current Protobuf build. No ASCII fallback was sent.')

    def send_batch_command(self, command, targets):
        self._require_batch_motion()
        if command not in {'set_angles', 'set_absolute_angles', 'set_currents'}:
            raise ValueError('Unsupported batch command')
        from ._hand_exo import HandExo
        targets = HandExo._motor_targets(targets)
        response = self._rpc(self.pb.Command.Value(command.upper()), list(targets), values=list(targets.values()))
        self._publish_reply(self._batch_reply('OK: ' + command, response))

    def send_current_limit_batch(self, targets, *, timeout=2.0):
        """Set explicit per-motor nominal current limits in one RPC.

        Protobuf-v1 firmware does not advertise this opcode, so callers should
        use their per-motor command fallback unless this capability is present.
        """
        if not getattr(self, 'current_limit_batch_supported', False):
            raise RuntimeError('This Protobuf firmware lacks current_limit_batch')
        from ._hand_exo import HandExo
        targets = HandExo._motor_targets(targets)
        values = list(targets.values())
        if any(isinstance(value, bool) or not math.isfinite(float(value)) or not 0 <= float(value) <= 910
               for value in values):
            raise ValueError('Per-motor current limits must be finite values in [0, 910] mA')
        response = self._rpc(self.pb.SET_CURRENT_LIMITS, list(targets), values=values, timeout=timeout)
        self._publish_reply(self._batch_reply('OK: set_current_lim batch', response))
        return response

    @staticmethod
    def _batch_reply(prefix, response):
        if response.HasField('batch'):
            prefix += ''.join(f' {field.name}={value}' for field, value in response.batch.ListFields())
        return prefix

    def send_finger_angles(self, values):
        self._require_batch_motion()
        from ._gesture_protocol import normalize_finger_angles
        fields = normalize_finger_angles(values)
        response = self._rpc(self.pb.SET_FINGER_ANGLES, [], finger_angles=fields)
        self._publish_reply(self._batch_reply('OK: finger_angles transport=sync_write', response))

    def send_gesture(self, name, pose=None, *, percent=None):
        self._require_batch_motion()
        name = self._gesture_token(name)
        fields = dict(name=name)
        if percent is None:
            fields['pose'] = self._gesture_token(pose)
            self._rpc(self.pb.SET_GESTURE, [], gesture=fields)
            self._publish_reply(f'OK: gesture {name}:{fields["pose"]}')
        else:
            if isinstance(percent, bool) or not math.isfinite(float(percent)):
                raise ValueError('Gesture percentage must be finite')
            self._rpc(self.pb.SET_GESTURE_ANGLE, [], float(percent), gesture=fields)
            self._publish_reply(f'OK: gesture_angle {name}:{max(0, min(100, float(percent))):g}')

    @staticmethod
    def _gesture_token(value):
        if not isinstance(value, str):
            raise ValueError('Gesture name and pose must be strings')
        value = value.strip().lower()
        if not value or len(value) > 31 or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789_' for c in value):
            raise ValueError('Gesture name and pose require 1..31 letters, digits or underscores')
        return value

    @staticmethod
    def command_route(command: str) -> str:
        """Expose the audited route matrix to diagnostics and tests."""
        return protobuf_command_route(command)

    @staticmethod
    def _serialized_commands(message: str) -> list[str]:
        """Split the same newline/semicolon batches accepted by ASCII CDC."""
        if not isinstance(message, str):
            raise TypeError('Serialized command payload must be text')
        return [part.strip() for part in re.split(r'[;\r\n]+', message) if part.strip()]

    def send(self, message: str):
        commands = self._serialized_commands(message)
        if not commands:
            raise ValueError('Serialized command payload is empty')
        # The GUI emits one transport write containing a complete real-time
        # action set. Preserve that behavior on Protobuf and use the existing
        # current batch opcode when the complete set is homogeneous.
        heads = [command.split(':', 1)[0].lower() for command in commands]
        if len(commands) > 1 and len(set(heads)) == 1 and heads[0] == 'set_current':
            targets = {}
            for command in commands:
                parts = command.split(':')
                if len(parts) != 3:
                    raise ValueError('set_current requires an ID and value')
                mid = int(parts[1])
                if mid in targets:
                    raise ValueError('Duplicate motor ID')
                targets[mid] = float(parts[2])
            if self.batch_motion_supported:
                self.send_batch_command('set_currents', targets)
                return
        for command in commands:
            self._send_one(command)

    def _send_one(self, message: str):
        parts = message.strip().split(':')
        command = parts[0].lower()
        if command in {'set_angles', 'set_absolute_angles', 'set_currents'}:
            # New explicit-ID form: set_angles:11=5:16=-10 (relative degrees).
            targets = {}
            for part in parts[1:]:
                fields = part.split('=')
                if len(fields) != 2:
                    raise ValueError('Batch fields must be ID=value')
                mid = int(fields[0])
                if mid in targets:
                    raise ValueError('Duplicate motor ID')
                targets[mid] = float(fields[1])
            self.send_batch_command(command, targets)
        elif command == 'set_finger_angles':
            from ._gesture_protocol import SET_FINGER_ANGLES_ORDER
            if not 2 <= len(parts) <= 7:
                raise ValueError('set_finger_angles requires up to six signed integer fields')
            fields = {name: int(value) for name, value in zip(SET_FINGER_ANGLES_ORDER, parts[1:]) if value.strip()}
            self.send_finger_angles(fields)
        elif command in {'set_gesture', 'set_gesture_angle'}:
            if len(parts) != 3:
                raise ValueError('Gesture command requires a name and pose or percentage')
            if command == 'set_gesture':
                self.send_gesture(parts[1], parts[2])
            else:
                self.send_gesture(parts[1], percent=float(parts[2]))
        elif command == 'set_joint_model':
            if len(parts) != 7:
                raise ValueError('set_joint_model requires an ID and five parameters')
            mid = int(parts[1])
            params = dict(zip(('gain', 'time_constant', 'max_velocity', 'stiffness', 'moment'),
                              map(float, parts[2:])))
            self.write_joint_model(mid, params)
            self._publish_reply(f'OK: set_joint_model id={mid}')
        elif command in self.MOTOR_COMMANDS:
            expected = 2 if command == 'stop' else 3
            if len(parts) != expected:
                raise ValueError('Invalid motor command arguments')
            mid = 'all' if command == 'stop' and parts[1].lower() == 'all' else int(parts[1])
            self.send_motor_command(command, mid, None if expected == 2 else float(parts[2]))
        elif command == 'get_telemetry_fast':
            raise RuntimeError('Use the typed get_fast_frame/get_fast_telemetry route on Protobuf firmware')
        else:
            # The explicit contract list is audited against parseMessage().
            # Unknown commands still go to firmware so it can produce its
            # canonical error and forward-compatible additions can be probed.
            self._write_line(self._cmd, message)


class ProtobufV2SerialComm(ProtobufSerialComm):
    """Fully framed split-CDC transport.

    The command CDC accepts version-2 Request frames and the response CDC is
    the only source of Response frames. There is no ASCII reader or writer in
    this backend; maintenance commands are carried in ``TEXT_COMMAND`` so the
    existing firmware command surface remains available without duplicating
    all 80+ legacy parser commands in the schema.
    """

    usb_protocol = 'protobuf-v2'
    frame_version = 2
    max_request_frame = 512
    poll_available_before_binary_read = True

    def __init__(self, *args, **kwargs):
        kwargs.setdefault('current_limit_batch_supported', True)
        super().__init__(*args, **kwargs)

    def _open_and_probe(self):
        self._cmd = serial.Serial(self.cmd_port, self.baudrate, timeout=.02,
                                  write_timeout=self.write_timeout)
        self._telem = serial.Serial(self.telem_port, self.baudrate, timeout=.02,
                                    write_timeout=self.write_timeout)
        self._cmd.reset_input_buffer()
        self._telem.reset_input_buffer()
        self._frame_reader = ProtobufFrameReader(self.frame_version)
        self._reader_error = None
        # Unlike the ASCII/v1 backend, the transaction owns response reads.
        # This keeps request/response pairing deterministic while retaining
        # the response CDC as the sole inbound result path.

    def _request_device(self):
        return self._cmd

    def _response_device(self):
        return self._telem

    def _text_command(self, message, *, timeout=2.0):
        message = str(message).strip()
        if not message or len(message.encode('utf-8')) > 192:
            raise ValueError('protobuf-v2 text command must contain 1..192 bytes')
        with self._rpc_lock:
            self._sequence = (self._sequence + 1) & 0xffffffff
            request = self.pb.Request(sequence=self._sequence,
                                      command=self.pb.TEXT_COMMAND,
                                      text_command=message)
            response = self._exchange(request, 'TEXT_COMMAND', timeout)
        # Some legacy firmware commands intentionally perform an action
        # without calling commandPrint().  Protobuf-v2 still returns the
        # request status, but the optional text field is absent.  Preserve
        # that as an empty reply so the caller can apply the command-specific
        # silent-command contract instead of reporting a transport failure.
        return response.text_reply if response.HasField('text_reply') else ''

    def send_text_command(self, message, *, timeout=2.0):
        """Execute one legacy firmware command through TEXT_COMMAND.

        Keep this as a public transport operation so callers that need a
        longer diagnostic deadline do not have to know about the v2 backend's
        internal RPC helper.
        """
        return self._text_command(message, timeout=timeout)

    def _send_one(self, message: str):
        command = message.strip().split(':', 1)[0].lower()
        if command not in {
            'set_angles', 'set_absolute_angles', 'set_currents',
            'set_finger_angles', 'set_gesture', 'set_gesture_angle',
            'set_joint_model', *self.MOTOR_COMMANDS, 'get_telemetry_fast',
        }:
            reply = self.send_text_command(message)
            self._publish_reply(reply)
            return
        super()._send_one(message)


def probe_protobuf_v2_ports(port_a, port_b, baudrate, *, timeout=.4, return_info=False):
    """Probe both CDC orientations with HELLO.

    Return ``(command_port, response_port)`` normally. With ``return_info``
    enabled, return ``((command_port, response_port), info_dict)`` where the
    dictionary is populated from typed discovery or the legacy text fallback.

    USB CDC enumeration order is not stable, so both orientations are tested.
    A trailing newline is intentional: it lets a v1 line parser discard the
    probe cleanly if discovery is run against an older image.
    """
    from . import exo_usb_pb2
    for command_port, response_port in ((port_a, port_b), (port_b, port_a)):
        command = response = None
        try:
            command = serial.Serial(command_port, baudrate, timeout=.02, write_timeout=1.0)
            response = serial.Serial(response_port, baudrate, timeout=.02, write_timeout=1.0)
            command.reset_input_buffer(); response.reset_input_buffer()
            request = exo_usb_pb2.Request(sequence=1, command=exo_usb_pb2.HELLO)
            command.write(encode_frame(request.SerializeToString(), version=2) + b'\n')
            payload = ProtobufFrameReader(2).read(response, timeout)
            result = exo_usb_pb2.Response.FromString(payload)
            if result.IsInitialized() and result.sequence == 1 and result.status == exo_usb_pb2.OK:
                if result.HasField('discovery'):
                    info = discovery_to_info(result.discovery)
                    return ((command_port, response_port), info) if return_info else (command_port, response_port)
                # Compatibility with pre-discovery protobuf-v2 firmware.
                if result.HasField('text_reply') and result.text_reply == 'protobuf-v2':
                    info = {
                        'name': 'NMLHandExo', 'usb_protocol': 'protobuf-v2',
                        'usb_cdc_count': 2, 'usb_text_port': 'primary',
                        'usb_binary_port': 'secondary',
                        'usb_features': ['joint_model_write', 'batch_motion',
                                         'current_limit_batch', 'assist_binary',
                                         'assist_enable'],
                    }
                    return ((command_port, response_port), info) if return_info else (command_port, response_port)
        except (OSError, serial.SerialException, TimeoutError, ValueError):
            pass
        finally:
            for device in (command, response):
                if device is not None:
                    try:
                        device.close()
                    except Exception:
                        pass
    return (None, None) if return_info else None
