import io

from nml_hand_exo.interface import exo_usb_pb2 as pb
from nml_hand_exo.interface._protobuf_serial import (
    ProtobufFrameReader,
    ProtobufV2SerialComm,
    encode_frame,
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
