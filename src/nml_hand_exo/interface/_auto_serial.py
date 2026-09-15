"""Discover USB capabilities with info before choosing the serial backend."""
from __future__ import annotations
import time
from ._interfaces import BaseComm, SerialComm, DualSerialComm
from ._serial_ports import find_cdc_sibling


class AutoSerialComm(BaseComm):
    def __init__(self, port, baudrate=1_000_000, *, sibling=None, progress=None, **kwargs):
        self.port, self.baudrate, self.sibling = port, baudrate, sibling
        self.progress = progress or (lambda message: None)
        self.kwargs = kwargs
        self.verbose = False
        self.backend = None
        self.info = {}
        self.is_split = False

    def connect(self):
        from ._hand_exo import HandExo
        pair = (self.port, self.sibling) if self.sibling else find_cdc_sibling(self.port)
        candidates = list(pair) if pair else [self.port]
        failures = []
        for port in candidates:
            probe = SerialComm(port, self.baudrate, timeout=.02, response_timeout=2)
            exo = HandExo(probe, send_delay=0)
            try:
                self.progress(f'Reading USB capabilities from {port}')
                exo.connect()
                # Restore text discovery for a legacy device left in split mode.
                # A Protobuf secondary ignores these bytes; only primary replies.
                exo.send_command('set_reply_route:both')
                time.sleep(.1)
                exo.send_command('debug:off')
                time.sleep(.1)
                info = exo.info(timeout=2)
                if not info.get('version') or not info.get('motors'):
                    failures.append(f'{port}: no complete info reply')
                    continue
                self.info = info
                primary = port
                break
            except (OSError, RuntimeError, ValueError) as exc:
                failures.append(f'{port}: {exc}')
            finally:
                exo.close()
        else:
            raise ConnectionError('USB discovery failed: ' + '; '.join(failures))
        protocol = self.info.get('usb_protocol', 'ascii-nx')
        count = self.info.get('usb_cdc_count', 2 if pair else 1)
        if protocol not in ('ascii-nx', 'axon-ascii', 'protobuf-v1') or count not in (1, 2):
            raise ConnectionError(f'Unsupported USB capabilities: {protocol}, CDC count {count}')
        if count == 2 and not pair:
            raise ConnectionError('Firmware reports two CDC ports but its sibling is unavailable')
        if protocol == 'protobuf-v1':
            if count != 2 or self.info.get('usb_text_port') != 'primary' or self.info.get('usb_binary_port') != 'secondary':
                raise ConnectionError('Unsupported Protobuf USB layout')
            from ._protobuf_serial import ProtobufSerialComm
            secondary = next(p for p in pair if p != primary)
            backend = ProtobufSerialComm(primary, secondary, self.baudrate,
                                         motor_ids=sorted(self.info['motors']),
                                         model_write_supported='joint_model_write' in self.info.get('usb_features', []),
                                         batch_motion_supported='batch_motion' in self.info.get('usb_features', []),
                                         **self.kwargs)
        elif count == 2:
            backend = DualSerialComm(*pair, self.baudrate, **self.kwargs)
        else:
            backend = SerialComm(primary, self.baudrate, **self.kwargs)
        self.backend = backend
        self.is_split = count == 2
        self.usb_protocol = protocol
        self.progress(f'USB protocol: {protocol}; CDC ports: {count}')
        try:
            backend.connect()
        except Exception:
            backend.close()
            self.backend = None
            raise

    # BaseComm defines these methods, so they need explicit forwarding.
    def send(self, message): return self.backend.send(message)
    def receive(self, *args, **kwargs): return self.backend.receive(*args, **kwargs)
    def flush_input(self): return self.backend.flush_input()
    def is_connected(self): return bool(self.backend and self.backend.is_connected())
    def fast_telemetry_device(self): return self.backend.fast_telemetry_device()
    def drain_async_lines(self): return self.backend.drain_async_lines()
    def close(self):
        if self.backend: self.backend.close()
    def disconnect(self): self.close()
    def __getattr__(self, name):
        backend = self.__dict__.get('backend')
        if backend is not None: return getattr(backend, name)
        raise AttributeError(name)
