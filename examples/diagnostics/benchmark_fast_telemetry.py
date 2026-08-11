"""Benchmark the current exo firmware's fast telemetry serial command.

This script intentionally avoids the nml_hand_exo Python API. It talks to the
OpenRB over pyserial only, sends read-only commands, parses the compact ``NX``
telemetry frame, and reports timing.

Close Arduino Serial Monitor, the GUI, and any other serial terminal before
running this. Windows COM ports are exclusive, and two programs cannot safely
own COM3 at the same time.

Example:
    python examples/diagnostics/benchmark_fast_telemetry.py --port COM3 --ids 11 12 13 14 15 16 17 18 19
"""

from __future__ import annotations

import argparse
import statistics
import struct
import time
from dataclasses import dataclass

import serial


HEADER_FMT = "<2sBBBHIH"
RECORD_FMT = "<BBhiiii"
HEADER_LEN = struct.calcsize(HEADER_FMT)
RECORD_LEN = struct.calcsize(RECORD_FMT)


@dataclass
class FastFrame:
    elapsed_ms: float
    flags: int
    count: int
    checksum_ok: bool
    records: list[tuple[int, int, int, int, int, float, float]]


def read_text_response(ser: serial.Serial, command: str, timeout: float) -> str:
    ser.reset_input_buffer()
    ser.write((command + "\r\n").encode("ascii"))
    ser.flush()

    original_timeout = ser.timeout
    ser.timeout = 0.02
    deadline = time.monotonic() + timeout
    data = bytearray()
    try:
        while time.monotonic() < deadline:
            chunk = ser.read(1)
            if chunk:
                data += chunk
                if chunk == b";":
                    break
            else:
                time.sleep(0.001)
    finally:
        ser.timeout = original_timeout
    return data.decode(errors="replace").replace(";", "").strip()


def read_exact_until_deadline(
    ser: serial.Serial, size: int, deadline: float
) -> bytes:
    data = bytearray()
    while len(data) < size and time.monotonic() < deadline:
        chunk = ser.read(size - len(data))
        if chunk:
            data += chunk
        else:
            time.sleep(0.0005)
    return bytes(data)


def read_fast_frame(ser: serial.Serial, ids: list[int], timeout: float) -> FastFrame:
    command = "get_telemetry_fast:" + ":".join(str(mid) for mid in ids)
    ser.reset_input_buffer()
    start = time.perf_counter()
    ser.write((command + "\r\n").encode("ascii"))
    ser.flush()

    original_timeout = ser.timeout
    ser.timeout = 0.005
    deadline = time.monotonic() + timeout
    prefix = bytearray()
    try:
        while time.monotonic() < deadline:
            byte = ser.read(1)
            if byte:
                prefix += byte
                if len(prefix) > 2:
                    prefix = prefix[-2:]
                if bytes(prefix) == b"NX":
                    break
            else:
                time.sleep(0.0005)
        else:
            raise TimeoutError("Timed out waiting for NX frame")

        rest = read_exact_until_deadline(ser, HEADER_LEN - 2, deadline)
        if len(rest) != HEADER_LEN - 2:
            raise TimeoutError(f"Timed out reading header ({len(rest)} bytes)")
        header = b"NX" + rest
        magic, version, flags, count, payload_len, _timestamp_ms, checksum = (
            struct.unpack(HEADER_FMT, header)
        )
        if magic != b"NX" or version != 1:
            raise ValueError(f"Unsupported frame magic/version: {magic!r}/{version}")

        payload = read_exact_until_deadline(ser, payload_len, deadline)
        if len(payload) != payload_len:
            raise TimeoutError(f"Timed out reading payload ({len(payload)} bytes)")
    finally:
        ser.timeout = original_timeout

    elapsed_ms = (time.perf_counter() - start) * 1000.0
    checksum_ok = ((sum(header[: HEADER_LEN - 2]) + sum(payload)) & 0xFFFF) == checksum
    records = []
    for offset in range(0, payload_len, RECORD_LEN):
        if offset + RECORD_LEN > len(payload):
            break
        mid, err, current_ma, velocity_raw, position_ticks, abs_cdeg, rel_cdeg = (
            struct.unpack_from(RECORD_FMT, payload, offset)
        )
        records.append(
            (
                mid,
                err,
                current_ma,
                velocity_raw,
                position_ticks,
                abs_cdeg / 100.0,
                rel_cdeg / 100.0,
            )
        )
    return FastFrame(elapsed_ms, flags, count, checksum_ok, records)


def method_name(flags: int) -> str:
    return {
        0: "failed",
        1: "fallbackRead",
        2: "fastSyncRead",
        3: "syncRead",
    }.get(flags, f"unknown({flags})")


def summarize(values: list[float]) -> str:
    return (
        f"min={min(values):.2f} ms, mean={statistics.mean(values):.2f} ms, "
        f"median={statistics.median(values):.2f} ms, max={max(values):.2f} ms, "
        f"mean_rate={1000.0 / statistics.mean(values):.1f} Hz"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3", help="OpenRB USB serial port")
    parser.add_argument("--baud", type=int, default=2_000_000, help="USB baud rate")
    parser.add_argument(
        "--ids",
        nargs="+",
        type=int,
        default=[11, 12, 13, 14, 15, 16, 17, 18, 19],
        help="Dynamixel IDs to request",
    )
    parser.add_argument("--samples", type=int, default=50)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument(
        "--skip-text-diag",
        action="store_true",
        help="Skip version/telemetry_diag text checks",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print(f"Opening {args.port} at {args.baud} baud")
    with serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=1.0) as ser:
        time.sleep(2.0)
        ser.reset_input_buffer()

        if not args.skip_text_diag:
            for command in (
                "version",
                "telemetry_diag:" + ":".join(str(mid) for mid in args.ids),
            ):
                start = time.perf_counter()
                response = read_text_response(ser, command, args.timeout)
                elapsed_ms = (time.perf_counter() - start) * 1000.0
                print(f"\n{command} ({elapsed_ms:.1f} ms)")
                print(response or "<no response>")

        print("\nFirst fast frame")
        first = read_fast_frame(ser, args.ids, args.timeout)
        print(
            f"elapsed={first.elapsed_ms:.2f} ms, method={method_name(first.flags)}, "
            f"count={first.count}, checksum_ok={first.checksum_ok}"
        )
        for record in first.records:
            mid, err, current_ma, velocity_raw, position_ticks, abs_deg, rel_deg = record
            print(
                f"  id={mid:>2} err={err} current={current_ma:>5} mA "
                f"velocity_raw={velocity_raw:>8} pos_ticks={position_ticks:>8} "
                f"abs={abs_deg:>8.2f} rel={rel_deg:>8.2f}"
            )

        timings = []
        flags = []
        checksum_ok = 0
        for _ in range(args.samples):
            frame = read_fast_frame(ser, args.ids, args.timeout)
            timings.append(frame.elapsed_ms)
            flags.append(frame.flags)
            checksum_ok += int(frame.checksum_ok)

    print("\nBenchmark")
    print(f"samples={len(timings)}, ids={len(args.ids)}, checksum_ok={checksum_ok}")
    print(f"methods={', '.join(method_name(flag) for flag in sorted(set(flags)))}")
    print(summarize(timings))


if __name__ == "__main__":
    main()
