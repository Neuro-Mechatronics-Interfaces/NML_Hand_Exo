import importlib.util
from pathlib import Path
from unittest.mock import Mock
from types import SimpleNamespace
import pytest
from nml_hand_exo.interface import HandExo


def test_profile_fractions_are_exclusive_and_complete():
    result = HandExo._parse_io_profile('''IO_PROFILE: version=1 window_us=1000
IO_STAGE: name=command calls=5 exclusive_us=100 max_span_us=600
IO_STAGE: name=dxl_read calls=5 exclusive_us=500 max_span_us=500
IO_STAGE: name=other calls=0 exclusive_us=400 max_span_us=0;''')
    assert result['stages']['dxl_read']['fraction'] == .5
    assert sum(s['fraction'] for s in result['stages'].values()) == 1
    with pytest.raises(ValueError):
        HandExo._parse_io_profile('IO_PROFILE: version=1 window_us=1000')


@pytest.mark.parametrize('counter', ['lu', '', '%llu', '123garbage'])
def test_malformed_profile_header_reports_original_value(counter):
    reply = f'IO_PROFILE: version=1 window_us={counter}'
    with pytest.raises(ValueError, match='Malformed I/O profile header') as error:
        HandExo._parse_io_profile(reply)
    assert repr(reply) in str(error.value)


def test_unknown_profile_version_is_distinct_from_broken_formatting():
    with pytest.raises(ValueError, match='Unsupported I/O profile version 2'):
        HandExo._parse_io_profile('IO_PROFILE: version=2 window_us=1000')


def test_profile_header_allows_transport_whitespace():
    result = HandExo._parse_io_profile('IO_PROFILE:  version=1\twindow_us=1000\r\n'
        'IO_STAGE: name=other calls=0 exclusive_us=1000 max_span_us=0;')
    assert result['window_us'] == 1000


def test_benchmark_distinguishes_fresh_measured_samples_and_estimates():
    path = Path(__file__).resolve().parents[1] / 'examples/diagnostics/benchmark_fast_telemetry.py'
    spec = importlib.util.spec_from_file_location('benchmark', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    def record(stamp, measured=True):
        return {16: dict(error=False, position_source='measured' if measured else 'estimated',
                         estimated=not measured, sample_timestamp_ms=stamp)}
    exo = Mock()
    exo.get_fast_telemetry.side_effect = [record(1), record(1), record(11,False), TimeoutError('test'), {}]
    result = module.measure(exo, [16], 5, .01, 0)
    assert result['frames'] == 4 and result['complete_frames'] == 3
    assert result['measured_position_frames'] == 2 and result['frames_with_estimates'] == 1
    assert result['per_id'][16]['fresh_measured'] == 1
    assert result['per_id'][16]['repeated_timestamp'] == 1
    assert sum(result['errors'].values()) == 1


@pytest.mark.parametrize('poll_rate,poll_seconds', [(0, .008), (20, .008), (0, .05)])
def test_mixed_benchmark_schedules_model_writes_without_catchup_bursts(monkeypatch, poll_rate, poll_seconds):
    path = Path(__file__).resolve().parents[1] / 'examples/diagnostics/benchmark_fast_telemetry.py'
    spec = importlib.util.spec_from_file_location('mixed_benchmark', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    clock = [0.0]
    def advance(seconds):
        clock[0] += seconds
    monkeypatch.setattr(module, 'time', SimpleNamespace(perf_counter=lambda: clock[0], sleep=advance))
    writes = []
    parameters = dict(gain=.2, time_constant=.1, max_velocity=70, stiffness=.001, moment=0)
    def write(mid, **kwargs):
        assert mid == 16
        assert kwargs == dict(parameters, timeout=.2)
        writes.append(clock[0])
        advance(.002)
    def read(**kwargs):
        advance(poll_seconds)
        return {16: dict(error=False, position_source='measured', estimated=False,
                         sample_timestamp_ms=int(clock[0] * 1000))}
    exo = SimpleNamespace(device=SimpleNamespace(usb_protocol='protobuf-v1'),
                          set_joint_model=write, get_fast_telemetry=read)
    result = module.measure(exo, [16], 100, .5, poll_rate, command_rate=50,
                            command_id=16, model_parameters=parameters, command_timeout=.2)
    commands = result['commands']
    assert result['complete_frames'] == 100
    assert commands['transport'] == 'protobuf'
    assert commands['successes'] == len(writes)
    assert commands['transaction_ms']['mean'] == pytest.approx(2)
    assert commands['errors'] == {}
    # No bursts of writes to replay missed periods.
    assert min(b-a for a, b in zip(writes, writes[1:])) > .010
    if poll_seconds < .02:
        assert commands['acknowledged_hz'] == pytest.approx(50, abs=2)
        assert commands['skipped_slots'] == 0
        assert commands['start_lateness_ms']['max'] <= 8.001
    else:
        assert commands['skipped_slots'] > 0
        assert commands['attempts'] == 100


def test_model_workload_reports_failed_acknowledgements(monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'examples/diagnostics/benchmark_fast_telemetry.py'
    spec = importlib.util.spec_from_file_location('failed_benchmark', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'time', SimpleNamespace(perf_counter=iter([0, .1]).__next__))
    exo = Mock()
    exo.device.usb_protocol = 'ascii-nx'
    exo.set_joint_model.side_effect = TimeoutError('model ACK')
    workload = module.ModelCommandWorkload(exo, 16, {}, 50, .1, 0)
    workload.run()
    stats = workload.summary(.1)
    assert stats['attempts'] == 1 and stats['successes'] == 0
    assert stats['skipped_slots'] == 5
    assert stats['errors'] == {'TimeoutError: model ACK': 1}
    assert stats['transport'] == 'ascii'


def test_benchmark_reports_old_sdk_before_opening_ports(monkeypatch, capsys):
    path = Path(__file__).resolve().parents[1] / 'examples/diagnostics/benchmark_fast_telemetry.py'
    spec = importlib.util.spec_from_file_location('old_sdk_benchmark', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    class OldHandExo:
        def set_joint_model(self, motor_id, gain=.1):
            pass
    monkeypatch.setattr(module, 'HandExo', OldHandExo)
    ports = Mock()
    monkeypatch.setattr(module, 'AutoSerialComm', ports)
    monkeypatch.setattr('sys.argv', ['benchmark', '--port', 'COM10', '--command-rate', '50', '--command-id', '16'])
    with pytest.raises(SystemExit) as error:
        module.main()
    assert error.value.code == 2
    assert 'python -m pip install -e ".[protobuf]"' in capsys.readouterr().err
    ports.assert_not_called()
