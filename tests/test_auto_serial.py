from unittest.mock import Mock
import pytest
from nml_hand_exo.interface import _auto_serial as auto
from nml_hand_exo.interface import _hand_exo as sdk
from nml_hand_exo.interface import _protobuf_serial as binary


@pytest.mark.parametrize('features', [[], ['joint_model_write'], ['joint_model_write','batch_motion']])
@pytest.mark.parametrize('protocol,count', [('ascii-nx',1), ('ascii-nx',2), ('protobuf-v1',2)])
def test_info_selects_transport_without_motion_or_user_checkbox(monkeypatch, protocol, count, features):
    info = dict(version='0.9.0', motors={16:{}}, usb_protocol=protocol, usb_cdc_count=count,
                usb_text_port='primary', usb_binary_port='secondary', usb_features=features)
    monkeypatch.setattr(auto, 'find_cdc_sibling', lambda _: ('COM10','COM11') if count==2 else None)
    monkeypatch.setattr(auto.time, 'sleep', lambda _: None)
    probe = Mock()
    probe.info.return_value = info
    monkeypatch.setattr(sdk, 'HandExo', Mock(return_value=probe))
    single, dual, proto = Mock(), Mock(), Mock()
    monkeypatch.setattr(auto, 'SerialComm', single)
    monkeypatch.setattr(auto, 'DualSerialComm', dual)
    monkeypatch.setattr(binary, 'ProtobufSerialComm', proto)
    device = auto.AutoSerialComm('COM10')
    device.connect()
    selected = proto if protocol=='protobuf-v1' else dual if count==2 else single
    assert device.backend is selected.return_value
    if protocol == 'protobuf-v1':
        assert proto.call_args.kwargs['model_write_supported'] == ('joint_model_write' in features)
        assert proto.call_args.kwargs['batch_motion_supported'] == ('batch_motion' in features)
    assert device.is_split == (count==2)
    probe.close.assert_called_once()
    assert [c.args[0] for c in probe.send_command.call_args_list] == ['set_reply_route:both','debug:off']
    device.drain_async_lines()
    selected.return_value.drain_async_lines.assert_called_once()


def test_reversed_binary_port_discovery_closes_both_probes(monkeypatch):
    monkeypatch.setattr(auto, 'find_cdc_sibling', lambda _: ('COM11','COM10'))
    monkeypatch.setattr(auto.time, 'sleep', lambda _: None)
    probes = [Mock(), Mock()]
    probes[0].info.return_value = {}
    probes[1].info.return_value = dict(version='0.9.0', motors={16:{}}, usb_protocol='protobuf-v1',
                                      usb_cdc_count=2, usb_text_port='primary', usb_binary_port='secondary')
    monkeypatch.setattr(sdk, 'HandExo', Mock(side_effect=probes))
    monkeypatch.setattr(auto, 'SerialComm', Mock())
    constructor = Mock()
    monkeypatch.setattr(binary, 'ProtobufSerialComm', constructor)
    device = auto.AutoSerialComm('COM11')
    device.connect()
    assert constructor.call_args.args[:2] == ('COM10','COM11')
    for probe in probes: probe.close.assert_called_once()


def test_auto_serial_consumes_typed_v2_discovery(monkeypatch):
    monkeypatch.setattr(auto, 'find_cdc_sibling', lambda _: ('COM10', 'COM11'))
    monkeypatch.setattr(auto.time, 'sleep', lambda _: None)
    probes = [Mock(), Mock()]
    for probe in probes:
        probe.info.return_value = {}
    monkeypatch.setattr(sdk, 'HandExo', Mock(side_effect=probes))
    monkeypatch.setattr(auto, 'SerialComm', Mock(side_effect=probes))
    typed_info = {
        'name': 'NMLHandExo', 'version': '0.9.7', 'usb_protocol': 'protobuf-v2',
        'usb_cdc_count': 2, 'usb_text_port': 'primary', 'usb_binary_port': 'secondary',
        'usb_features': ['batch_motion'], 'motors': {11: {'id': 11}, 19: {'id': 19}},
    }
    monkeypatch.setattr(binary, 'probe_protobuf_v2_ports',
                        Mock(return_value=(('COM11', 'COM10'), typed_info)))
    constructor = Mock()
    monkeypatch.setattr(binary, 'ProtobufV2SerialComm', constructor)
    device = auto.AutoSerialComm('COM10')
    device.connect()
    assert device.info is typed_info
    assert constructor.call_args.args[:2] == ('COM11', 'COM10')
    assert constructor.call_args.kwargs['motor_ids'] == [11, 19]


def test_unknown_protocol_fails_closed_and_releases_probe(monkeypatch):
    monkeypatch.setattr(auto, 'find_cdc_sibling', lambda _: None)
    monkeypatch.setattr(auto.time, 'sleep', lambda _: None)
    probe = Mock()
    probe.info.return_value = dict(version='0.9.1',motors={16:{}},usb_protocol='unknown')
    monkeypatch.setattr(sdk, 'HandExo', Mock(return_value=probe))
    monkeypatch.setattr(auto, 'SerialComm', Mock())
    with pytest.raises(ConnectionError, match='Unsupported USB'):
        auto.AutoSerialComm('COM10').connect()
    probe.close.assert_called_once()


def test_metadata_parser_exposes_protocol_without_changing_semver():
    comm = Mock()
    comm.receive.return_value = '''Name: NMLHandExo
Version: 0.9.0
USB Protocol: protobuf-v1
USB CDC Count: 2
USB Text Port: primary
USB Binary Port: secondary
USB Features: joint_model_write batch_motion
Model Step Ms: 1
I/O Profile: 1
Number of Motors: 1
Motor 0: {name: index, id: 16, limits: [100, 200]}
'''
    info = sdk.HandExo(comm, send_delay=0).info()
    assert info['usb_protocol'] == 'protobuf-v1' and info['usb_cdc_count'] == 2
    assert info['model_step_ms'] == 1 and info['version'] == '0.9.0'
    assert info['usb_features'] == ['joint_model_write', 'batch_motion']


def test_metadata_parser_exposes_firmware_motor_reachability():
    comm = Mock()
    comm.receive.return_value = '''Name: NMLHandExo
Version: 0.9.5
Number of Motors: 2
Motor 0: {name: wrist, id: 1, reachable: false, limits: [100, 200]}
Motor 1: {name: wrist, id: 11, reachable: true, limits: [100, 200]}
'''
    info = sdk.HandExo(comm, send_delay=0).info()
    assert info['motors'][1]['reachable'] is False
    assert info['motors'][11]['reachable'] is True


def test_ascii_info_and_help_keep_the_legacy_command_surface():
    comm = Mock()
    comm.receive.return_value = (
        'Name: NMLHandExo\nVersion: 0.9.5\nNumber of Motors: 1\n'
        'Motor 0: {name: index, id: 16, limits: [100, 200]}\n'
    )
    exo = sdk.HandExo(comm, send_delay=0)
    info = exo.info()
    exo.send_command('help')
    assert info['name'] == 'NMLHandExo'
    assert info['motors'][16]['name'] == 'index'
    assert [call.args[0] for call in comm.send.call_args_list] == ['info\n', 'help\n']


def test_handexo_info_falls_back_to_typed_v2_discovery_shape():
    comm = Mock(usb_protocol='protobuf-v2')
    comm.discovery_info = {
        'name': 'NMLHandExo', 'version': '0.9.5', 'usb_protocol': 'protobuf-v2',
        'usb_cdc_count': 2, 'n_motors': 1, 'motors': {11: {'id': 11}},
    }
    comm.receive.return_value = ''
    info = sdk.HandExo(comm, send_delay=0).info()
    assert info['version'] == '0.9.5'
    assert info['motors'] == {11: {'id': 11}}
