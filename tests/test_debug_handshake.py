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
        elif data.strip() == b'set_total_current_lim:900':
            text.pending.append(b'OK: total_current_lim 900;\r\n')
        elif data.strip() == b'get_total_current_lim':
            text.pending.append(b'Total current limit: 900 mA;\r\n')
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


def test_budget_ack_is_consumed_before_subsequent_readback(connection):
    exo, text, _ = connection
    exo._firmware_version = (0, 9, 0)
    assert exo.set_total_current_limit(900, timeout=.5) == 900
    assert not text.pending
    assert exo.get_total_current_limit(timeout=.5) == 900
    assert exo.info(timeout=.5)['version'] == '0.9.0'


def test_unconsumed_budget_setter_ack_is_not_a_query_response(connection):
    exo, _, _ = connection
    exo._firmware_version = (0, 9, 0)
    exo.send_command('set_total_current_lim:900')
    with pytest.raises(ProtocolResponseError, match='get_total_current_lim') as caught:
        exo.get_total_current_limit(timeout=.5)
    assert 'OK: total_current_lim 900' in str(caught.value)


@pytest.mark.parametrize('operation,reply', [
    ('set', ''), ('set', 'ERROR: assist owns bus'),
    ('set', 'OK: total_current_lim nan'), ('set', 'OK: total_current_lim 0'),
    ('get', ''), ('get', 'Total current limit: nan mA'),
    ('get', 'Total current limit: 0 mA'),
])
def test_budget_failures_report_the_command_and_actual_response(operation, reply):
    comm = Mock()
    comm.receive.return_value = reply
    exo = HandExo(comm, send_delay=0)
    exo._firmware_version = (0, 9, 0)
    with pytest.raises(ProtocolResponseError) as caught:
        if operation == 'set':
            exo.set_total_current_limit(900, timeout=.01)
        else:
            exo.get_total_current_limit(timeout=.01)
    assert f'{operation}_total_current_lim' in str(caught.value)
    assert caught.value.raw_response == reply


def test_budget_ack_returns_firmware_clamping_instead_of_requested_value():
    comm = Mock()
    comm.receive.return_value = 'OK: total_current_lim 450'
    exo = HandExo(comm, send_delay=0)
    exo._firmware_version = (0, 9, 0)
    assert exo.set_total_current_limit(100, timeout=.01) == 450
    comm.send.assert_called_once_with('set_total_current_lim:100\n')


@pytest.mark.parametrize('timeout', [None, .75])
def test_global_mode_change_waits_for_slow_protobuf_text_ack(connection, timeout):
    exo, text, binary = connection
    writes = []
    reply = (b'Motor control mode: current_position '
             b'(torque remains off until explicitly enabled);\r\n')
    timer = threading.Timer(.9, text.feed, args=(reply,))

    def send_mode(data):
        writes.append(data)
        assert data.strip() == b'set_control_mode:all:current_position'
        # A global bus operation can finish after the former 0.75 s timeout.
        timer.start()

    text.on_write = send_mode
    binary.on_write = lambda data: pytest.fail('Mode changes must use the text CDC')
    try:
        if timeout is None:
            assert exo.set_control_mode('current_position') == reply.decode().strip().rstrip(';')
        else:
            with pytest.raises(ProtocolResponseError) as caught:
                exo.set_control_mode('current_position', timeout=timeout)
            assert caught.value.raw_response == ''
        assert writes == [b'set_control_mode:all:current_position\r\n']
    finally:
        timer.cancel()
        timer.join(timeout=1)


@pytest.mark.parametrize('reply', ['', 'ERROR: motor mode change failed safety verification',
                                 'OK: disable 11', 'Motor control mode: current'])
def test_global_mode_change_still_requires_matching_success_ack(reply):
    comm = Mock()
    comm.receive.return_value = reply
    with pytest.raises(ProtocolResponseError):
        HandExo(comm, send_delay=0).set_control_mode('current_position')
    assert comm.receive.call_args.kwargs['timeout'] == 10


def test_assist_worker_checks_measured_binary_feedback_then_enables_once(connection):
    from PyQt5.QtWidgets import QApplication
    from nml_hand_exo.applications.hand_exo_gui import SerialWorker, SynchronizedHandExo

    app = QApplication.instance() or QApplication([])
    exo, text, binary = connection
    comm = exo.device
    comm.assist_binary_supported = comm.assist_enable_supported = True
    commands = []

    def respond(data):
        request = comm.pb.Request.FromString(read_frame(io.BytesIO(data), .1))
        assert list(request.motor_ids) == [11, 15]
        commands.append(request.command)
        response = comm.pb.Response(sequence=request.sequence, status=comm.pb.OK)
        if request.command == comm.pb.GET_TELEMETRY_FAST:
            stamp = len(commands)*50
            payload = b''.join(struct.pack('<BBhiiiiBQI', mid, 0, 5, 0, 2048,
                                           18000, 0, 0x15, 0, stamp) for mid in (11, 15))
            header = struct.pack('<2sBBBHI', b'NX', 2, 3, 2, len(payload), stamp)
            response.nx_frame = header + struct.pack('<H', (sum(header)+sum(payload)) & 0xffff) + payload
        else:
            assert request.command == comm.pb.ASSIST_ENABLE
            response.assist.state = 1
            response.assist.reason = comm.pb.ASSIST_REASON_CALIBRATING
            response.assist.total_current_budget_mA = 900
        binary.feed(encode_frame(response.SerializeToString()))

    binary.on_write = respond
    text.on_write = lambda data: pytest.fail('Assist startup must use the binary CDC')
    worker = SerialWorker()
    worker.set_exo(SynchronizedHandExo(exo))
    result = []
    worker.assist_completed.connect(lambda *args: result.append(args))
    worker.request_assist('enable', dict(ids=[11, 15], parameters={}))
    worker._handle_assist(*worker._assist_q.get_nowait())
    assert len(result) == 1 and result[0][1]['state'] == 1 and result[0][2] == ''
    assert commands[-1] == comm.pb.ASSIST_ENABLE
    assert len(commands) > 2 and all(command == comm.pb.GET_TELEMETRY_FAST for command in commands[:-1])
    assert app is not None


@pytest.mark.parametrize('motor_ids', [(11, 12), (13, 14, 15), (1, 2), (19,)])
@pytest.mark.parametrize('reply_timing', ['immediate', 'in_flight'])
def test_home_group_preserves_every_ack_on_protobuf_text_port(
    connection, monkeypatch, motor_ids, reply_timing,
):
    from PyQt5.QtWidgets import QApplication
    from nml_hand_exo.applications.hand_exo_gui import SerialWorker, SynchronizedHandExo

    app = QApplication.instance() or QApplication([])
    exo, text, binary = connection
    comm = exo.device
    published = threading.Event()
    publish = comm._publish_reply

    def publish_reply(frame):
        publish(frame)
        published.set()

    monkeypatch.setattr(comm, '_publish_reply', publish_reply)
    writes = []

    def send_home(data):
        writes.append(data)
        mid = int(data.strip().split(b':')[1])
        assert mid in motor_ids
        reply = f'OK: home {mid};\r\n'.encode()
        if reply_timing == 'immediate':
            published.clear()
            text.feed(reply)
            # Make the first ACK available before the next send. The former
            # per-motor send_command() then deterministically discarded it.
            assert published.wait(1), 'Text reader did not publish Home ACK'
        else:
            text.pending.append(reply)

    text.on_write = send_home
    binary.on_write = lambda data: pytest.fail('Home must use the primary text CDC')
    # An older enable ACK may already be queued when the stage begins.
    comm._publish_reply('OK: enable 19')
    flush = Mock(wraps=comm.flush_input)
    monkeypatch.setattr(comm, 'flush_input', flush)
    worker = SerialWorker()
    worker.set_exo(SynchronizedHandExo(exo))
    worker.set_home_generation(1)
    result = []
    worker.home_stage_completed.connect(lambda *args: result.append(args))
    worker._handle_home_stage(1, motor_ids)
    assert result == [(1, '')]
    assert writes == [f'home:{mid}\r\n'.encode() for mid in motor_ids]
    assert comm._replies.empty()
    assert not text.pending
    flush.assert_called_once()
    assert app is not None  # Keep the QApplication alive throughout the stage.
