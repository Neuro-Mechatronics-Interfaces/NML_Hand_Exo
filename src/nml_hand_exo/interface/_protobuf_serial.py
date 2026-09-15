"""Opt-in USB Protobuf transport. Primary CDC is text; secondary is binary.

Never retries a motion RPC: a missing acknowledgement does not prove that the
device failed to act. Schema and engineering units live in protocol/exo_usb.proto.
"""
from __future__ import annotations
import binascii
import math
import secrets
import struct
import threading
import time
import serial

from ._interfaces import DualSerialComm


def encode_frame(payload: bytes) -> bytes:
    if not 0 < len(payload) <= 640:
        raise ValueError("Invalid Protobuf frame length")
    prefix = struct.pack('<BH', 1, len(payload))
    crc = binascii.crc_hqx(prefix + payload, 0xffff)
    return b'PB' + prefix + struct.pack('<H', crc) + payload


def read_frame(stream, timeout: float) -> bytes:
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
    version, size, crc = struct.unpack('<BHH', header)
    if version != 1 or not 0 < size <= 640:
        raise ValueError('Unsupported Protobuf envelope')
    payload = exact(size)
    if binascii.crc_hqx(header[:3] + payload, 0xffff) != crc:
        raise ValueError('Protobuf frame CRC mismatch')
    return payload


class ProtobufSerialComm(DualSerialComm):
    usb_protocol = 'protobuf-v1'
    is_split = True
    supports_legacy_fast_telemetry = False
    supports_shadow_telemetry = False
    MOTOR_COMMANDS = {'set_current', 'set_velocity', 'set_absolute_angle', 'set_angle', 'stop'}

    def __init__(self, *args, motor_ids=None, model_write_supported=False, batch_motion_supported=False, **kwargs):
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
        self._rpc_lock = threading.Lock()
        self._sequence = secrets.randbits(32)

    def _open_and_probe(self):
        # AutoSerialComm has identified the primary text port via info already.
        self._cmd = serial.Serial(self.cmd_port, self.baudrate, timeout=.02,
                                  write_timeout=self.write_timeout)
        self._telem = serial.Serial(self.telem_port, self.baudrate, timeout=.02,
                                    write_timeout=self.write_timeout)
        self._cmd.reset_input_buffer()
        self._telem.reset_input_buffer()
        self._reader_error = None
        self._start_reader()

    def text_reply_device(self):
        return self._cmd

    def fast_telemetry_device(self):
        raise RuntimeError('Legacy NX serial polling is disabled for Protobuf firmware')

    def _rpc(self, command, ids, value=None, *, timeout=.5, joint_model=None,
             values=None, finger_angles=None, gesture=None):
        ids = list(ids)
        if len(ids) > 18 or len(set(ids)) != len(ids) or any(
            isinstance(mid, bool) or not isinstance(mid, int) or not 1 <= mid <= 253 for mid in ids
        ):
            raise ValueError('Use at most 18 unique explicit integer motor IDs')
        if value is not None and (isinstance(value, bool) or not math.isfinite(value)):
            raise ValueError('Motor command value must be finite')
        with self._rpc_lock:
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
            frame = encode_frame(request.SerializeToString())
            if len(frame) > 135:
                raise ValueError('Request exceeds firmware frame limit')
            self._telem.reset_input_buffer()
            if self._telem.write(frame) != len(frame):
                raise ConnectionError('Incomplete Protobuf request write')
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                response = self.pb.Response.FromString(read_frame(self._telem, deadline-time.monotonic()))
                if not response.IsInitialized():
                    raise ValueError('Incomplete Protobuf response')
                if response.sequence != request.sequence:
                    continue # delayed response to an earlier timed-out request
                if response.status != self.pb.OK:
                    raise RuntimeError(f'Firmware rejected {self.pb.Command.Name(command)}: '
                                       f'{self.pb.Status.Name(response.status)}')
                return response
            raise TimeoutError('No matching Protobuf response; command was not retried')

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

    def send(self, message: str):
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
        elif command == 'get_telemetry_fast' or command.startswith('shadow_'):
            raise RuntimeError('Legacy telemetry/shadow serialization is disabled for Protobuf firmware')
        else:
            # Slow settings, gestures, calibration, info/help and emergency text
            # diagnostics retain the existing maintenance-plane contract.
            super().send(message)
