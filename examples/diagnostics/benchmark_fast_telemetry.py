"""Measure SDK telemetry throughput, sample provenance, and firmware I/O time.

Close the GUI/Serial Monitor first. No enable or movement commands are sent.
Optional fixed-rate model writes reapply the selected joint's current parameters.
Test only IDs physically present. A rate of 0 polls telemetry without pacing.
"""
from __future__ import annotations
import argparse
import inspect
import json
import math
from pathlib import Path
import statistics
import time
from collections import Counter
from nml_hand_exo.interface import HandExo, AutoSerialComm


def percentiles(values):
    if not values:
        return {}
    ordered = sorted(values)
    def percentile(p):
        return ordered[min(len(ordered)-1, int((len(ordered)-1)*p + 0.5))]
    return dict(min=min(values), median=statistics.median(values),
                mean=statistics.mean(values), p95=percentile(.95),
                p99=percentile(.99), max=max(values))


class ModelCommandWorkload:
    """Prioritize scheduled model writes between complete telemetry transactions.

    One owner serializes request/reply traffic, like the GUI's serial worker.
    Expired command slots are skipped instead of accumulating a catch-up burst.
    """
    def __init__(self, exo, motor_id, parameters, rate, timeout, start):
        self.exo, self.motor_id, self.parameters = exo, motor_id, dict(parameters)
        self.transport = 'protobuf' if getattr(exo.device, 'usb_protocol', None) == 'protobuf-v1' else 'ascii'
        self.rate, self.timeout, self.start = rate, timeout, start
        self.slot = 0
        self.attempts = self.successes = self.skipped_slots = 0
        self.latencies, self.lateness = [], []
        self.errors = Counter()

    @property
    def deadline(self):
        return self.start + self.slot / self.rate

    def run(self):
        begin = time.perf_counter()
        late = max(0, begin - self.deadline)
        # Coalesce past deadlines into this single write, not multiple writes.
        stale = int(late * self.rate)
        self.slot += stale
        self.skipped_slots += stale
        self.lateness.append(late * 1000)
        self.attempts += 1
        try:
            self.exo.set_joint_model(self.motor_id, **self.parameters, timeout=self.timeout)
            self.successes += 1
        except (RuntimeError, ValueError, ConnectionError, OSError) as exc:
            self.errors[type(exc).__name__ + ': ' + str(exc)] += 1
        end = time.perf_counter()
        self.latencies.append((end-begin) * 1000)
        next_slot = max(self.slot + 1, math.floor((end-self.start) * self.rate) + 1)
        self.skipped_slots += next_slot - self.slot - 1
        self.slot = next_slot

    def summary(self, elapsed):
        return dict(name='set_joint_model', transport=self.transport, motor_id=self.motor_id,
                    parameters=self.parameters, requested_rate_hz=self.rate,
                    attempts=self.attempts, successes=self.successes,
                    acknowledged_hz=self.successes/elapsed, skipped_slots=self.skipped_slots,
                    start_lateness_ms=percentiles(self.lateness),
                    transaction_ms=percentiles(self.latencies), errors=dict(self.errors))


def measure(exo, ids, samples, timeout, rate, *, command_rate=0, command_id=None,
            model_parameters=None, command_timeout=1.0):
    latencies, errors = [], Counter()
    complete = measured = estimated = 0
    per_id = {mid: dict(valid=0, measured=0, estimated=0, missing=0, errors=0, unavailable=0, repeated_timestamp=0,
                       fresh_measured=0, last=None, gaps=[]) for mid in ids}
    start = time.perf_counter()
    workload = (ModelCommandWorkload(exo, command_id, model_parameters, command_rate,
                                    command_timeout, start) if command_rate else None)
    for i in range(samples):
        if workload:
            # Commands have their own schedule, including during paced telemetry
            # sleeps. An in-flight telemetry read remains non-preemptible.
            poll_deadline = start + i / rate if rate else time.perf_counter()
            while True:
                now = time.perf_counter()
                if now >= workload.deadline:
                    workload.run()
                    if time.perf_counter() >= poll_deadline:
                        break
                    continue
                if now >= poll_deadline:
                    break
                time.sleep(max(0, min(poll_deadline, workload.deadline)-now))
        elif rate:
            time.sleep(max(0, start + i / rate - time.perf_counter()))
        begin = time.perf_counter()
        try:
            records = exo.get_fast_telemetry(timeout=timeout, motor_ids=ids)
        except (TimeoutError, ValueError, ConnectionError, OSError) as exc:
            errors[type(exc).__name__ + ': ' + str(exc)] += 1
            continue
        latencies.append((time.perf_counter() - begin) * 1000)
        good = [mid for mid in ids if mid in records and not records[mid]['error']
                and records[mid]['position_source'] != 'unavailable']
        for mid in ids:
            stats = per_id[mid]
            if mid not in records: stats['missing'] += 1
            elif records[mid]['error']: stats['errors'] += 1
            elif records[mid]['position_source'] == 'unavailable': stats['unavailable'] += 1
        complete += len(good) == len(ids)
        measured += len(good) == len(ids) and all(
            records[mid]['position_source'] == 'measured' for mid in ids)
        estimated += any(records[mid]['estimated'] for mid in good)
        for mid in good:
            r, stats = records[mid], per_id[mid]
            stats['valid'] += 1
            is_measured = r['position_source'] == 'measured'
            stats['measured'] += is_measured
            stats['estimated'] += r['position_source'] == 'estimated'
            stamp = r['sample_timestamp_ms']
            if stamp == stats['last']:
                stats['repeated_timestamp'] += 1
            else:
                stats['fresh_measured'] += is_measured
                if stats['last'] is not None:
                    gap = (stamp - stats['last']) & 0xffffffff
                    if gap < 0x80000000: stats['gaps'].append(gap)
            stats['last'] = stamp
    elapsed = time.perf_counter() - start
    for stats in per_id.values():
        stats.pop('last')
        stats['device_sample_gap_ms'] = percentiles(stats.pop('gaps'))
        stats['fresh_measured_hz'] = stats['fresh_measured'] / elapsed
    result = dict(attempts=samples, frames=len(latencies), complete_frames=complete,
                measured_position_frames=measured, frames_with_estimates=estimated,
                elapsed_s=elapsed, frame_hz=len(latencies)/elapsed,
                complete_frame_hz=complete/elapsed, measured_position_frame_hz=measured/elapsed,
                round_trip_ms=percentiles(latencies), errors=dict(errors), per_id=per_id)
    if workload:
        result['commands'] = workload.summary(elapsed)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--telem-port', help='Optional explicit sibling CDC (otherwise discovered automatically)')
    parser.add_argument('--baud', type=int, default=1_000_000)
    parser.add_argument('--ids', nargs='+', type=int, default=list(range(11, 20)))
    parser.add_argument('--samples', type=int, default=500)
    parser.add_argument('--timeout', type=float, default=.5)
    parser.add_argument('--rate', type=float, default=0, help='Target polls/s; 0 = saturation test')
    parser.add_argument('--command-rate', type=float, default=0,
                        help='Independent set_joint_model writes/s; 0 disables mixed load')
    parser.add_argument('--command-id', type=int,
                        help='Explicit DXL ID whose current model parameters are reapplied')
    parser.add_argument('--command-timeout', type=float, default=1.0,
                        help='Maximum model-write acknowledgement wait, seconds')
    parser.add_argument('--json', type=Path, help='Save all statistics for comparison')
    args = parser.parse_args()
    if (args.samples < 1 or not math.isfinite(args.timeout) or args.timeout <= 0
            or not math.isfinite(args.rate) or args.rate < 0):
        parser.error('samples and timeout must be positive; rate must be nonnegative')
    if (not math.isfinite(args.command_rate) or not 0 <= args.command_rate <= 1000
            or not math.isfinite(args.command_timeout) or args.command_timeout <= 0):
        parser.error('command-rate must be 0..1000 Hz and command-timeout must be positive')
    if args.command_rate and args.command_id not in args.ids:
        parser.error('--command-rate requires --command-id from the telemetry --ids list')
    if args.command_id is not None and not args.command_rate:
        parser.error('--command-id requires a positive --command-rate')
    if len(set(args.ids)) != len(args.ids) or not all(1 <= mid <= 253 for mid in args.ids):
        parser.error('Use unique explicit DXL IDs in 1..253')
    if args.command_rate and 'timeout' not in inspect.signature(HandExo.set_joint_model).parameters:
        parser.error('The benchmark imported an older SDK from '
                     f'{inspect.getfile(HandExo)}. In this Python environment run: '
                     'python -m pip install -e ".[protobuf]"')
    comm = AutoSerialComm(args.port, args.baud, sibling=args.telem_port, timeout=.02, progress=print)
    exo = HandExo(comm, send_delay=0)
    try:
        exo.connect()
        exo.set_debug(False)
        info = exo.info()
        if not info.get('version'):
            preview = repr(info.get('_raw', '')[:240])
            raise ConnectionError(f'Expected firmware info on {comm.cmd_port if comm.is_split else comm.port}; '
                                  f'received {preview}')
        # Warm caches and exclude connection/version queries from the run.
        exo.get_fast_telemetry(timeout=args.timeout, motor_ids=args.ids)
        model_parameters = None
        if args.command_rate:
            model = exo.get_joint_model(args.command_id)
            model_parameters = {key: model[key] for key in
                                ('gain', 'time_constant', 'max_velocity', 'stiffness', 'moment')}
            # Validate support and consume the ACK before the timed run.
            exo.set_joint_model(args.command_id, **model_parameters, timeout=args.command_timeout)
            command_transport = 'Protobuf' if comm.usb_protocol == 'protobuf-v1' else 'ASCII'
            print(f'Mixed load: reapply joint model for ID {args.command_id} at '
                  f'{args.command_rate:g} Hz over {command_transport}; parameters={model_parameters}')
        profile_available = True
        try:
            exo.reset_io_profile()
        except (RuntimeError, ValueError, ConnectionError) as exc:
            profile_available = False
            print('Firmware I/O profile unavailable:', exc)
        result = measure(exo, args.ids, args.samples, args.timeout, args.rate,
                         command_rate=args.command_rate, command_id=args.command_id,
                         model_parameters=model_parameters, command_timeout=args.command_timeout)
        result.update(firmware=info['version'], ids=args.ids, requested_rate=args.rate)
        if profile_available:
            try:
                result['io_profile'] = exo.get_io_profile()
            except (RuntimeError, ValueError, ConnectionError) as exc:
                result['io_profile_error'] = str(exc)
        print(json.dumps(result, indent=2))
        if args.json:
            args.json.parent.mkdir(parents=True, exist_ok=True)
            args.json.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    finally:
        exo.close()


if __name__ == '__main__':
    main()
