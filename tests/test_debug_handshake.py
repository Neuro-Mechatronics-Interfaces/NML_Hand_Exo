"""Reply-order regressions using the real Protobuf text-reader thread."""
import io
import struct
import threading
from unittest.mock import Mock

import pytest

from nml_hand_exo.interface import HandExo, ProtocolResponseError
from nml_hand_exo.interface._protobuf_serial import ProtobufSerialComm, encode_frame, read_frame

INFO = (b'Name: NMLHandExo\nVersion: 0.9.0\nUSB Protocol: protobuf-v1\n'
        b'USB CDC Count: 2\nNumber of Motors: 1\n'
        b'Motor 0: {name: index, id: 16, limits: [100, 200]}\n;\r\n')


class Pipe:
    def __init__(self):
        self.condition = threading.Condition()
        self.buffer = bytearray()
        self.pending = []
        self.is_open = True
        self.read_threads = set()
        self.on_write = lambda data: None

    @property
    def in_waiting(self):
        with self.condition:
            return len(self.buffer)

    def read(self, size):
        self.read_threads.add(threading.current_thread().name)
        with self.condition:
            self.condition.wait_for(lambda: self.buffer or not self.is_open, timeout=.02)
            result = bytes(self.buffer[:size])
            del self.buffer[:size]
            return result

    def write(self, data):
        self.on_write(data)
        return len(data)

    def feed(self, data):
        with self.condition:
            self.buffer.extend(data)
            self.condition.notify_all()

    def reset_input_buffer(self):
        with self.condition:
            self.buffer.clear() # does not discard a reply still in flight

    def close(self):
        with self.condition:
            self.is_open = False
            self.condition.notify_all()


@pytest.fixture
def connection(monkeypatch):
    pytest.importorskip('nml_hand_exo.interface.exo_usb_pb2')
    text, binary = Pipe(), Pipe()
    # Reproduce the user's actual port orientation: text COM11, binary COM10.
    comm = ProtobufSerialComm('COM11', 'COM10', 1000000)
    def send_text(data):
        if data.strip() == b'debug:off': text.pending.append(b'Debug state: false;\r\n')
        elif data.strip() == b'info': text.pending.append(INFO)
        else: raise AssertionError(data)
    text.on_write = send_text
    def send_binary(data):
        request = comm.pb.Request.FromString(read_frame(io.BytesIO(data), .1))
        assert request.command == comm.pb.GET_TELEMETRY_FAST and list(request.motor_ids) == [16]
        payload = struct.pack('<BBhiiiiBQI', 16, 0, -80, 0, 2048, 18000, 0, 0x15, 0, 123)
        header = struct.pack('<2sBBBHI', b'NX', 2, 3, 1, len(payload), 123)
        nx = header + struct.pack('<H', (sum(header)+sum(payload)) & 0xffff) + payload
        response = comm.pb.Response(sequence=request.sequence, status=comm.pb.OK, nx_frame=nx)
        binary.feed(encode_frame(response.SerializeToString()))
    binary.on_write = send_binary
    monkeypatch.setattr('nml_hand_exo.interface._protobuf_serial.serial.Serial',
                        lambda port, *args, **kwargs: text if port == 'COM11' else binary)
    receive = comm.receive
    def deliver_in_flight_replies(*args, **kwargs):
        # Hold replies until the caller waits, forcing ACK delivery to occur
        # after any pre-send queue flush. No scheduling sleeps are needed.
        text.feed(b''.join(text.pending))
        text.pending.clear()
        return receive(*args, **kwargs)
    monkeypatch.setattr(comm, 'receive', deliver_in_flight_replies)
    exo = HandExo(comm, send_delay=0)
    exo.connect()
    try:
        yield exo, text, binary
    finally:
        exo.close()


def test_old_fire_and_forget_sequence_reproduces_missing_version(connection):
    exo, _, _ = connection
    exo.send_command('debug:off')
    info = exo.info(timeout=.5)
    assert 'version' not in info
    assert info['_raw'] == 'Debug state: false'


def test_debug_ack_is_consumed_before_info_then_binary_telemetry(connection):
    exo, text, binary = connection
    assert exo.set_debug(False, timeout=.5) is None
    info = exo.info(timeout=.5)
    assert info['version'] == '0.9.0' and info['usb_protocol'] == 'protobuf-v1'
    samples = exo.get_fast_telemetry(motor_ids=[16])
    assert samples[16]['current'] == -80 and samples[16]['absolute_angle'] == 180
    assert len(text.read_threads) == 1
    assert binary.read_threads == {threading.current_thread().name}
    assert text.read_threads.isdisjoint(binary.read_threads)


@pytest.mark.parametrize('reply', ['', 'Debug state: true', 'ERROR: debug failed'])
def test_missing_or_wrong_debug_ack_fails_at_debug_stage(reply):
    comm = Mock()
    comm.receive.return_value = reply
    with pytest.raises(ProtocolResponseError):
        HandExo(comm, send_delay=0).set_debug(False, timeout=.01)
