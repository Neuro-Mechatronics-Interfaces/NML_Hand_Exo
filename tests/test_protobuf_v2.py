import io

import pytest

from nml_hand_exo.interface import exo_usb_pb2 as pb
from nml_hand_exo.interface._protobuf_serial import (
    ProtobufFrameReader,
    ProtobufV2SerialComm,
    encode_frame,
    discovery_to_info,
    probe_protobuf_v2_ports,
    read_frame,
)


class Endpoint:
    def __init__(self):
        self.input = bytearray()
        self.writes = []
        self.is_open = True
        self.peer = None

    @property
    def in_waiting(self):
        return len(self.input)

    def reset_input_buffer(self):
        self.input.clear()

    def read(self, size):
        data = bytes(self.input[:size])
        del self.input[:size]
        return data

    def write(self, data):
        self.writes.append(bytes(data))
        request = pb.Request.FromString(read_frame(io.BytesIO(data), .1, version=2))
        response = pb.Response(sequence=request.sequence, status=pb.OK)
        if request.command == pb.TEXT_COMMAND:
            response.text_reply = "INFO: framed"
        elif request.command == pb.SET_CURRENT_LIMITS:
            response.batch.motors = len(request.motor_ids)
        self.peer.input.extend(encode_frame(response.SerializeToString(), version=2))
        return len(data)


def connected_v2():
    comm = ProtobufV2SerialComm("COM-command", "COM-response", 1_000_000)
    comm._cmd = Endpoint()
    comm._telem = Endpoint()
    comm._cmd.peer = comm._telem
    comm._frame_reader = ProtobufFrameReader(2)
    return comm


def test_v2_frame_reader_rejects_v1_and_accepts_v2():
    payload = b"reply"
    reader = ProtobufFrameReader(2)
    assert reader.read(io.BytesIO(encode_frame(payload, version=2)), .1) == payload


def test_v2_maintenance_commands_are_framed_on_command_and_response_ports():
    comm = connected_v2()
    comm.send("info")
    assert comm._cmd.writes
    assert comm._cmd.writes[0].startswith(b"PB\x02")
    assert comm._cmd.writes[0] != b"info\n"
    assert comm.receive(wait_until_return=True, timeout=.1) == "INFO: framed"


def test_v2_current_limit_bulk_is_one_typed_per_motor_rpc():
    comm = connected_v2()
    comm.send_current_limit_batch({11: 300, 12: 300})
    request = pb.Request.FromString(read_frame(io.BytesIO(comm._cmd.writes[0]), .1, version=2))
    assert request.command == pb.SET_CURRENT_LIMITS
    assert list(request.motor_ids) == [11, 12]
    assert list(request.values) == [300, 300]
    assert comm._telem.in_waiting == 0


def test_v2_rpc_lock_timeout_does_not_wait_forever():
    comm = connected_v2()
    assert comm._rpc_lock.acquire()
    try:
        with pytest.raises(TimeoutError, match='transaction lock timed out'):
            comm.get_fast_frame([11], timeout=.001)
    finally:
        comm._rpc_lock.release()


def test_v2_sequence_zero_event_is_delivered_immediately():
    comm = connected_v2()
    events = []
    comm.set_async_event_callback(events.append)
    request = pb.Request(sequence=123, command=pb.HELLO)

    def write(data):
        event = pb.Response(sequence=0, status=pb.OK, text_reply="[assist-fw] phase=read")
        reply = pb.Response(sequence=request.sequence, status=pb.OK)
        comm._telem.input.extend(encode_frame(event.SerializeToString(), version=2))
        comm._telem.input.extend(encode_frame(reply.SerializeToString(), version=2))
        return len(data)

    comm._cmd.write = write
    result = comm._exchange(request, "HELLO", .05)
    assert result.status == pb.OK
    assert events == ["[assist-fw] phase=read"]


@pytest.mark.parametrize('reply_kind', ['typed', 'legacy'])
def test_v2_probe_accepts_typed_hello_and_legacy_text_fallback(monkeypatch, reply_kind):
    class ProbeEndpoint:
        active = []

        def __init__(self, port):
            self.port = port
            self.input = bytearray()
            self.is_open = True
            self.active.append(self)

        @property
        def in_waiting(self):
            return len(self.input)

        def reset_input_buffer(self):
            self.input.clear()

        def read(self, size):
            data = bytes(self.input[:size])
            del self.input[:size]
            return data

        def write(self, data):
            request = pb.Request.FromString(read_frame(io.BytesIO(data), .1, version=2))
            if self.port == 'COM11' and request.command == pb.HELLO:
                peer = next(endpoint for endpoint in self.active if endpoint is not self)
                if reply_kind == 'typed':
                    response = pb.Response(
                        sequence=request.sequence, status=pb.OK,
                        discovery=pb.Discovery(
                            schema_version=1, protocol_version=2,
                            firmware_version='0.9.7', capability_flags=63,
                            cdc_count=2, cdc_role=pb.CDC_ROLE_PRIMARY_TEXT,
                            side='dual', motor_ids=[1, 2, 11, 12],
                        ),
                    )
                else:
                    response = pb.Response(sequence=request.sequence, status=pb.OK,
                                           text_reply='protobuf-v2')
                peer.input.extend(encode_frame(response.SerializeToString(), version=2))
            return len(data)

        def close(self):
            self.is_open = False
            self.active.remove(self)

    monkeypatch.setattr('nml_hand_exo.interface._protobuf_serial.serial.Serial',
                        lambda port, *args, **kwargs: ProbeEndpoint(port))
    result = probe_protobuf_v2_ports('COM10', 'COM11', 1_000_000,
                                     timeout=.01, return_info=True)
    assert result[0] == ('COM11', 'COM10')
    assert result[1]['usb_protocol'] == 'protobuf-v2'
    if reply_kind == 'typed':
        assert result[1]['version'] == '0.9.7'
        assert result[1]['side'] == 'dual'
        assert result[1]['motor_ids'] == [1, 2, 11, 12]
        assert result[1]['motors'][11] == {'id': 11}
    else:
        assert 'version' not in result[1]


def test_typed_discovery_maps_to_backward_compatible_info_shape():
    discovery = pb.Discovery(
        schema_version=1, protocol_version=2, firmware_version='0.9.7',
        capability_flags=pb.DISCOVERY_CAP_BATCH_MOTION,
        cdc_count=2, cdc_role=pb.CDC_ROLE_PRIMARY_TEXT,
        side='right', motor_ids=[11, 19],
    )
    info = discovery_to_info(discovery)
    assert info['version'] == '0.9.7'
    assert info['n_motors'] == 2 and info['motors'][11] == {'id': 11}
    assert info['usb_features'] == ['batch_motion']
