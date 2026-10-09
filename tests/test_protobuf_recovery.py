"""USB fragmentation, late-reply and corruption recovery without RPC replay."""
import io
from unittest.mock import Mock

import pytest

from nml_hand_exo.interface._protobuf_serial import ProtobufSerialComm, ProtobufFrameReader, encode_frame


def test_rpc_retains_partial_reply_then_discards_its_old_sequence(monkeypatch):
    device = ProtobufSerialComm('COM11', 'COM10', 1000000)
    device._sequence = 0
    clock = iter(index*.0001 for index in range(10000))
    monkeypatch.setattr('nml_hand_exo.interface._protobuf_serial.time.monotonic', lambda: next(clock))
    stream = Mock()
    device._telem = stream
    pending = bytearray()
    tail = b''
    requests = []
    def write(frame):
        nonlocal tail
        request = device.pb.Request.FromString(frame[7:])
        requests.append(request)
        response = device.pb.Response(sequence=request.sequence, status=device.pb.OK,
                                       nx_frame=b'late'*80 if len(requests) == 1 else b'fresh'*60)
        reply = encode_frame(response.SerializeToString())
        if len(requests) == 1:
            pending.extend(reply[:12])
            tail = reply[12:]
        else:
            pending.extend(tail+reply)
        return len(frame)
    def read(size):
        # Emulate USB packet fragmentation, including the old frame's tail.
        size = min(size, 64)
        chunk = bytes(pending[:size])
        del pending[:size]
        return chunk
    stream.write.side_effect = write
    stream.read.side_effect = read
    with pytest.raises(TimeoutError, match=r'GET_TELEMETRY_FAST on COM10.*payload 5/\d+ bytes'):
        device.get_fast_frame([11,16], timeout=.01)
    assert len(device._frame_reader.buffer) == 12
    assert device.get_fast_frame([11,16], timeout=.1) == b'fresh'*60
    assert [request.sequence for request in requests] == [1,2]
    assert all(request.command == device.pb.GET_TELEMETRY_FAST for request in requests)
    assert not device._frame_reader.buffer
    stream.reset_input_buffer.assert_not_called()


@pytest.mark.parametrize('bad', [b'PB\x02\x01\x00\x00\x00x', b'PB\x01\xff\xff\x00\x00'])
def test_reader_resynchronizes_past_bad_envelope(bad):
    reader = ProtobufFrameReader()
    stream = io.BytesIO(b'old text\r\n'+bad+encode_frame(b'valid'))
    assert reader.read(stream, .1) == b'valid'


def test_reader_recovers_after_bad_crc_without_replaying_any_command():
    bad = bytearray(encode_frame(b'bad'))
    bad[-1] ^= 1
    stream = io.BytesIO(bytes(bad)+encode_frame(b'valid'))
    assert ProtobufFrameReader().read(stream, .1) == b'valid'


@pytest.mark.parametrize('data,detail', [(b'', 'no reply bytes received'),
                                       (b'PB\x01', 'header 3/7 bytes'),
                                       (encode_frame(b'abc')[:-1], 'payload 2/3 bytes')])
def test_timeout_reports_which_part_of_reply_is_missing(data, detail):
    reader = ProtobufFrameReader()
    with pytest.raises(TimeoutError, match=detail):
        reader.read(io.BytesIO(data), .002)
    assert bytes(reader.buffer) == data


def test_read_without_reply_never_retries_motion_rpc():
    device = ProtobufSerialComm('COM11', 'COM10', 1000000)
    device._telem = Mock()
    device._telem.read.return_value = b''
    device._telem.write.side_effect = len
    with pytest.raises(TimeoutError, match='SET_CURRENT on COM10.*no reply bytes received'):
        device._rpc(device.pb.SET_CURRENT, [16], 40, timeout=.002)
    device._telem.write.assert_called_once()
