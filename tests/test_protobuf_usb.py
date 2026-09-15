import io
import os
from pathlib import Path
import shutil
import subprocess
from unittest.mock import Mock

import pytest

pb = pytest.importorskip('nml_hand_exo.interface.exo_usb_pb2')
from nml_hand_exo.interface import HandExo
from nml_hand_exo.interface._protobuf_serial import ProtobufSerialComm, encode_frame, read_frame

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / 'src/cpp/nml_hand_exo'


@pytest.fixture(scope='module')
def firmware(tmp_path_factory):
    nanopb = Path(os.environ.get('NANOPB_DIR', Path.home() / 'Documents/Arduino/libraries/nanopb'))
    compiler = shutil.which('g++')
    if not compiler or not (nanopb / 'pb_decode.c').is_file():
        pytest.skip('Native g++ and installed nanopb are required')
    folder = tmp_path_factory.mktemp('protobuf-native')
    text = (FIRMWARE / 'protobuf_usb.cpp').read_text()
    text = text[text.index('static exo_pb::Reader'):text.rindex('#endif')]
    header = (FIRMWARE / 'nml_hand_exo.h').read_text(encoding='utf-8')
    records = header[header.index('struct __attribute__((packed)) FastTelemetryRecord'):
                     header.index('/// @brief One absolute motor target')]
    harness = (ROOT / 'tests/cpp/protobuf_usb_harness.cpp').read_text()
    implementation = (FIRMWARE / 'nml_hand_exo.cpp').read_text(encoding='utf-8')
    current_batch = implementation[implementation.index('bool NMLHandExo::setGoalCurrents('):
                                   implementation.index('bool NMLHandExo::setGoalCurrent(')]
    source = folder / 'test.cpp'
    source.write_text(harness.replace('// RECORD_DECLARATIONS', records)
                      .replace('// FIRMWARE_IMPLEMENTATION', text)
                      .replace('// CURRENT_BATCH_IMPLEMENTATION', current_batch))
    executable = folder / 'test.exe'
    result = subprocess.run([compiler, '-std=c++11', '-I', str(FIRMWARE), '-I', str(nanopb),
                             '-include', str(FIRMWARE / 'io_profile.h'), str(source),
                             *[str(nanopb / f'pb_{name}.c') for name in ('common', 'encode', 'decode')],
                             '-o', str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    def run(frame, reject=False, guard=None):
        result = subprocess.run([str(executable), frame.hex(), *([guard] if guard else ['reject'] if reject else [])],
                                capture_output=True, text=True, timeout=5)
        assert result.returncode == 0, result.stderr
        counts, output = result.stdout.splitlines()
        return [float(v) for v in counts.split()], bytes.fromhex(output)
    return run


@pytest.mark.parametrize('command,value,expected', [
    (pb.SET_CURRENT, -80, -80), (pb.SET_VELOCITY, 12, 12),
    (pb.SET_ABSOLUTE_ANGLE, 200, 200), (pb.SET_ANGLE, 20, 80),
])
def test_python_to_real_nanopb_motion_dispatch(firmware, command, value, expected):
    request = pb.Request(sequence=123, command=command, motor_ids=[16], value=value)
    counts, output = firmware(encode_frame(request.SerializeToString()))
    assert counts == [1, 0, 0, expected]
    response = pb.Response.FromString(read_frame(io.BytesIO(output), .1))
    assert response.sequence == 123 and response.status == pb.OK


def test_binary_telemetry_reuses_sdk_nx_decoder(firmware):
    request = pb.Request(sequence=124, command=pb.GET_TELEMETRY_FAST, motor_ids=[11, 16])
    counts, output = firmware(encode_frame(request.SerializeToString()))
    assert counts[:3] == [0, 1, 0]
    response = pb.Response.FromString(read_frame(io.BytesIO(output), .1))
    device = Mock(usb_protocol="protobuf-v1")
    device.get_fast_frame.return_value = response.nx_frame
    result = HandExo(device).get_fast_telemetry(motor_ids=[11, 16])
    assert result[16]['current'] == -80
    assert result[16]['absolute_angle'] == 123.45
    assert result[16]['position_source'] == 'measured'
    device.send.assert_not_called()


@pytest.mark.parametrize('ids,value', [([1], 80), ([16,16], 80), ([],80), ([16], float('nan')),
                                      ([16], float('inf')), ([256], 80), ([16], None)])
def test_invalid_motion_never_reaches_motor_backend(firmware, ids, value):
    request = pb.Request(sequence=125, command=pb.SET_CURRENT, motor_ids=ids)
    if value is not None: request.value = value
    counts, output = firmware(encode_frame(request.SerializeToString()))
    assert counts[:3] == [0, 0, 0]
    assert pb.Response.FromString(read_frame(io.BytesIO(output), .1)).status == pb.INVALID_REQUEST


def test_missing_required_fields_and_bad_crc_cannot_act(firmware):
    for payload in (b'\x08\x01', b'\x08\x01\x10\x7f'):
        counts, _ = firmware(encode_frame(payload))
        assert counts[:3] == [0, 0, 0]
    request = pb.Request(sequence=127, command=pb.SET_CURRENT, motor_ids=[16], value=80)
    frame = bytearray(encode_frame(request.SerializeToString()))
    frame[-1] ^= 1
    counts, output = firmware(frame)
    assert counts[:3] == [0, 0, 0] and not output


def test_rejected_current_is_reported_and_stop_all_has_no_implicit_enable(firmware):
    req = pb.Request(sequence=128, command=pb.SET_CURRENT, motor_ids=[16], value=80)
    _, output = firmware(encode_frame(req.SerializeToString()), reject=True)
    assert pb.Response.FromString(read_frame(io.BytesIO(output), .1)).status == pb.REJECTED
    req = pb.Request(sequence=129, command=pb.STOP)
    counts, _ = firmware(encode_frame(req.SerializeToString()))
    assert counts[:3] == [0, 0, 1]


def test_rpc_timeout_never_retries_a_motion_command():
    device = ProtobufSerialComm('fake', 'fake2', 1000000)
    device._telem = Mock()
    device._telem.write.side_effect = lambda data: len(data)
    device._telem.read.return_value = b''
    with pytest.raises(TimeoutError):
        device._rpc(pb.SET_CURRENT, [16], 80, timeout=.001)
    assert device._telem.write.call_count == 1


def test_typed_sdk_motion_avoids_ascii_formatting():
    device = Mock(usb_protocol="protobuf-v1")
    HandExo(device).set_direct_current(16, 80)
    device.send_motor_command.assert_called_once_with('set_current', 16, 80)
    device.send.assert_not_called()


def test_legacy_streams_disabled_on_binary_backend():
    device = ProtobufSerialComm('fake', 'fake2', 1000000)
    for command in ('get_telemetry_fast:16', 'shadow_start'):
        with pytest.raises(RuntimeError): device.send(command)


MODEL = dict(gain=.2, time_constant=.1, max_velocity=70, stiffness=.001, moment=-.02)


def test_joint_model_sdk_through_real_nanopb_without_ascii_or_motor_io(firmware):
    device = ProtobufSerialComm('fake', 'fake2', 1000000, model_write_supported=True)
    device._cmd = Mock()
    device._telem = Mock()
    captured = []
    def write(data):
        request = pb.Request.FromString(read_frame(io.BytesIO(data), .1))
        captured.append(request)
        counts, output = firmware(data)
        assert counts == [0, 0, 0, .2]
        device._telem.read.side_effect = io.BytesIO(output).read
        return len(data)
    device._telem.write.side_effect = write
    exo = HandExo(device, send_delay=0)
    exo._firmware_version = (0,9,0)
    assert exo.set_joint_model(16, **MODEL, timeout=.1) == 'OK: set_joint_model id=16'
    assert len(captured) == 1 and captured[0].command == pb.SET_JOINT_MODEL
    assert list(captured[0].motor_ids) == [16]
    for name, value in MODEL.items():
        assert getattr(captured[0].joint_model, name) == pytest.approx(value)
    device._cmd.write.assert_not_called()


@pytest.mark.parametrize('field,value', [('gain',float('nan')), ('gain',float('inf')), ('gain',0),
    ('time_constant',0), ('max_velocity',301), ('stiffness',-1), ('moment',2)])
def test_joint_model_invalid_parameters_cannot_mutate_state(firmware, field, value):
    params = dict(MODEL, **{field:value})
    request = pb.Request(sequence=140, command=pb.SET_JOINT_MODEL, motor_ids=[16],
                         joint_model=pb.JointModel(**params))
    counts, output = firmware(encode_frame(request.SerializeToString()))
    assert counts == [0,0,0,0]
    assert pb.Response.FromString(read_frame(io.BytesIO(output), .1)).status == pb.INVALID_REQUEST


def test_model_fields_cannot_be_attached_to_actuator_commands(firmware):
    request = pb.Request(sequence=141, command=pb.SET_CURRENT, motor_ids=[16], value=80,
                         joint_model=pb.JointModel(**MODEL))
    counts, output = firmware(encode_frame(request.SerializeToString()))
    assert counts == [0,0,0,0]
    assert pb.Response.FromString(read_frame(io.BytesIO(output), .1)).status == pb.INVALID_REQUEST


def test_missing_model_fields_and_duplicate_ids_are_rejected(firmware):
    for ids, model in [([16,16],pb.JointModel(**MODEL)), ([16],pb.JointModel(gain=.2))]:
        request = pb.Request(sequence=142,command=pb.SET_JOINT_MODEL,motor_ids=ids,joint_model=model)
        counts, _ = firmware(encode_frame(request.SerializePartialToString()))
        assert counts == [0,0,0,0]


def test_old_protobuf_firmware_requires_reflash_without_ascii_fallback():
    device = ProtobufSerialComm('fake','fake2',1000000)
    device._cmd, device._telem = Mock(), Mock()
    exo = HandExo(device,send_delay=0)
    exo._firmware_version = (0,9,0)
    with pytest.raises(RuntimeError,match='lacks joint_model_write'):
        exo.set_joint_model(16,**MODEL)
    device._cmd.write.assert_not_called()
    device._telem.write.assert_not_called()


def test_legacy_command_string_is_translated_to_binary_model_call():
    device = ProtobufSerialComm('fake','fake2',1000000,model_write_supported=True)
    device.write_joint_model = Mock()
    device.send('set_joint_model:16:0.2:0.1:70:0.001:-0.02\n')
    device.write_joint_model.assert_called_once_with(16,MODEL)
    assert device._replies.get_nowait() == 'OK: set_joint_model id=16'


@pytest.mark.parametrize('method,args,kwargs,opcode,writes,value', [
    ('set_angles', ({16: 20, 17: -10},), {}, pb.SET_ANGLES, 1, 80),
    ('set_angles', ({16: 120, 17: 140},), {'absolute': True}, pb.SET_ABSOLUTE_ANGLES, 1, 120),
    ('set_currents', ({16: 80, 17: -60},), {}, pb.SET_CURRENTS, 2, -60),
    ('set_finger_angles', ({'index': -20.4, 'middle': 0, 'wrist': None},), {}, pb.SET_FINGER_ANGLES, 1, -20),
    ('set_gesture', ('index', 'flex'), {}, pb.SET_GESTURE, 1, 100),
    ('set_gesture_angle', ('index', 30), {}, pb.SET_GESTURE_ANGLE, 1, 30),
])
def test_typed_sdk_batches_through_nanopb_without_ascii(firmware, monkeypatch, method, args, kwargs, opcode, writes, value):
    from nml_hand_exo.interface import _hand_exo as sdk
    # A binary finger call must not format and then reparse the ASCII command.
    monkeypatch.setattr(sdk, 'format_set_finger_angles', Mock(side_effect=AssertionError('ASCII formatter used')))
    device = ProtobufSerialComm('fake', 'fake2', 1000000, batch_motion_supported=True)
    device._cmd, device._telem = Mock(), Mock()
    captured = []
    def write(data):
        request = pb.Request.FromString(read_frame(io.BytesIO(data), .1))
        captured.append(request)
        counts, output = firmware(data)
        assert counts == [writes, 0, 0, value]
        device._telem.read.side_effect = io.BytesIO(output).read
        return len(data)
    device._telem.write.side_effect = write
    exo = HandExo(device, send_delay=0)
    exo._firmware_version = (0, 9, 0)
    getattr(exo, method)(*args, **kwargs)
    assert len(captured) == 1 and captured[0].command == opcode
    if opcode == pb.SET_FINGER_ANGLES:
        assert not captured[0].finger_angles.HasField('thumb')
        assert captured[0].finger_angles.HasField('middle')
        assert captured[0].finger_angles.middle == 0
        ack = device._replies.get_nowait()
        assert 'held=4' in ack and 'motors=2' in ack and 'commanded=2' in ack
    device._cmd.write.assert_not_called()


@pytest.mark.parametrize('command', [pb.SET_ANGLES, pb.SET_ABSOLUTE_ANGLES, pb.SET_CURRENTS])
@pytest.mark.parametrize('ids,values', [([16,17],[20]), ([16,16],[20,30]),
    ([16,1],[20,30]), ([16,17],[20,float('nan')]), ([16,17],[20,float('inf')]), ([],[])])
def test_invalid_batch_cannot_apply_even_its_valid_prefix(firmware, command, ids, values):
    request = pb.Request(sequence=150, command=command, motor_ids=ids, values=values)
    counts, output = firmware(encode_frame(request.SerializeToString()))
    assert counts == [0,0,0,0]
    assert pb.Response.FromString(read_frame(io.BytesIO(output), .1)).status == pb.INVALID_REQUEST


@pytest.mark.parametrize('fields', [dict(index=30,wrist=101), dict(index=-101), {}])
def test_invalid_finger_batch_never_writes(firmware, fields):
    request = pb.Request(sequence=151, command=pb.SET_FINGER_ANGLES, finger_angles=pb.FingerAngles(**fields))
    counts, output = firmware(encode_frame(request.SerializeToString()))
    assert counts == [0,0,0,0]
    assert pb.Response.FromString(read_frame(io.BytesIO(output), .1)).status == pb.INVALID_REQUEST


@pytest.mark.parametrize('name,pose', [('index','typo'), ('typo','flex'), ('index',''), ('','flex')])
def test_unknown_gesture_or_pose_is_not_acknowledged_as_motion(firmware, name, pose):
    request = pb.Request(sequence=152, command=pb.SET_GESTURE, gesture=pb.Gesture(name=name,pose=pose))
    counts, output = firmware(encode_frame(request.SerializeToString()))
    assert counts == [0,0,0,0]
    assert pb.Response.FromString(read_frame(io.BytesIO(output), .1)).status == pb.INVALID_REQUEST


@pytest.mark.parametrize('command,fields', [
    (pb.SET_ANGLES, dict(motor_ids=[16],values=[20],value=20)),
    (pb.SET_CURRENT, dict(motor_ids=[16],value=80,values=[80])),
    (pb.SET_GESTURE, dict(motor_ids=[16],gesture=pb.Gesture(name='index',pose='flex'))),
    (pb.SET_GESTURE_ANGLE, dict(value=30,gesture=pb.Gesture(name='index',pose='flex'))),
    (pb.SET_FINGER_ANGLES, dict(finger_angles=pb.FingerAngles(index=20),joint_model=pb.JointModel(**MODEL))),
])
def test_conflicting_payload_fields_never_act(firmware, command, fields):
    request = pb.Request(sequence=153, command=command, **fields)
    counts, output = firmware(encode_frame(request.SerializeToString()))
    assert counts == [0,0,0,0]
    assert pb.Response.FromString(read_frame(io.BytesIO(output), .1)).status == pb.INVALID_REQUEST


@pytest.mark.parametrize('raw,opcode', [
    ('set_angles:16=20:17=-10',pb.SET_ANGLES),
    ('set_absolute_angles:16=120:17=140',pb.SET_ABSOLUTE_ANGLES),
    ('set_currents:16=80:17=-60',pb.SET_CURRENTS),
    ('set_finger_angles::-20:0',pb.SET_FINGER_ANGLES),
    ('set_gesture:index:flex',pb.SET_GESTURE),
    ('set_gesture_angle:index:30',pb.SET_GESTURE_ANGLE),
    ('set_current:16:80',pb.SET_CURRENT),
])
def test_legacy_gui_strings_dispatch_to_binary(raw, opcode):
    device = ProtobufSerialComm('fake','fake2',1000000,batch_motion_supported=True)
    device._rpc = Mock(return_value=pb.Response(sequence=1,status=pb.OK))
    device.send(raw + '\n')
    assert device._rpc.call_count == 1
    assert device._rpc.call_args.args[0] == opcode
    assert device._replies.get_nowait().startswith('OK:')


@pytest.mark.parametrize('raw', ['set_angles:16=20:16=30', 'set_currents:16=80:17=nan',
    'set_angles:16', 'set_finger_angles:20:bad', 'set_finger_angles:20:1.5',
    'set_finger_angles:20::::::1', 'set_gesture:index:flex:extra', 'set_gesture:index:',
    'set_gesture:index:flex\x00junk'])
def test_invalid_gui_strings_never_reach_rpc(raw):
    device = ProtobufSerialComm('fake','fake2',1000000,batch_motion_supported=True)
    device._rpc = Mock()
    with pytest.raises(ValueError): device.send(raw)
    device._rpc.assert_not_called()


def test_missing_batch_capability_never_falls_back_to_ascii():
    device = ProtobufSerialComm('fake','fake2',1000000)
    device._cmd, device._telem = Mock(), Mock()
    for raw in ('set_angles:16=20', 'set_currents:16=80', 'set_finger_angles::20', 'set_gesture:index:flex'):
        with pytest.raises(RuntimeError, match='lacks batch_motion'):
            device.send(raw)
    device._cmd.write.assert_not_called()
    device._telem.write.assert_not_called()


def test_largest_supported_id_value_batch_fits_existing_request_limit():
    request = pb.Request(sequence=0xffffffff, command=pb.SET_ANGLES,
                         motor_ids=list(range(236,254)),values=[36000.0]*18)
    assert len(request.SerializeToString()) <= 128


@pytest.mark.parametrize('guard', ['hold', 'mode'])
def test_current_batch_prechecks_all_motors_before_first_write(firmware, guard):
    request = pb.Request(sequence=154, command=pb.SET_CURRENTS,motor_ids=[16,17],values=[80,60])
    counts, output = firmware(encode_frame(request.SerializeToString()), guard=guard)
    assert counts == [0,0,0,0]
    assert pb.Response.FromString(read_frame(io.BytesIO(output), .1)).status == pb.REJECTED


def test_current_batch_stops_after_first_bus_failure(firmware):
    request = pb.Request(sequence=155, command=pb.SET_CURRENTS,motor_ids=[16,17],values=[80,60])
    counts, output = firmware(encode_frame(request.SerializeToString()), reject=True)
    assert counts == [1,0,0,80]
    assert pb.Response.FromString(read_frame(io.BytesIO(output), .1)).status == pb.REJECTED
