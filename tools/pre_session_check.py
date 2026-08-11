"""Pre-session exoskeleton validation; read-only unless explicitly armed."""

from __future__ import annotations

import argparse
import json
import math
import time

from nml_hand_exo.interface import DualSerialComm, HandExo, SerialComm


MOTION_CONFIRMATION = "I_UNDERSTAND_THIS_MOVES_HARDWARE"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    connection = parser.add_mutually_exclusive_group(required=True)
    connection.add_argument("--port", help="Single-CDC command/reply COM port")
    connection.add_argument("--command-port", help="Dual-CDC command COM port")
    parser.add_argument("--telemetry-port", help="Dual-CDC telemetry COM port")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--sample-interval", type=float, default=0.1)
    parser.add_argument("--exercise-motion", type=int, metavar="DXL_ID")
    parser.add_argument("--exercise-hold", type=int, metavar="DXL_ID")
    parser.add_argument("--motion-rpm", type=float, default=0.5)
    parser.add_argument("--motion-duration", type=float, default=0.2)
    parser.add_argument(
        "--confirm-motion",
        help=f"Required for motion/hold tests: {MOTION_CONFIRMATION}",
    )
    return parser


def validate_args(args: argparse.Namespace):
    if bool(args.command_port) != bool(args.telemetry_port):
        raise ValueError("--command-port and --telemetry-port must be supplied together")
    if args.samples < 2:
        raise ValueError("--samples must be at least 2")
    if args.sample_interval < 0:
        raise ValueError("--sample-interval must be non-negative")
    motion_requested = args.exercise_motion is not None or args.exercise_hold is not None
    if motion_requested and args.confirm_motion != MOTION_CONFIRMATION:
        raise ValueError(
            "Motion-capable checks require --confirm-motion " + MOTION_CONFIRMATION
        )
    if not math.isfinite(args.motion_rpm) or abs(args.motion_rpm) > 1.0:
        raise ValueError("--motion-rpm must be finite and no greater than 1 rpm")
    if not 0.05 <= args.motion_duration <= 0.5:
        raise ValueError("--motion-duration must be between 0.05 and 0.5 seconds")


def make_exo(args: argparse.Namespace) -> HandExo:
    if args.port:
        comm = SerialComm(
            port=args.port,
            baudrate=args.baud,
            response_timeout=0.75,
        )
    else:
        comm = DualSerialComm(
            cmd_port=args.command_port,
            telem_port=args.telemetry_port,
            baudrate=args.baud,
            response_timeout=0.75,
        )
    return HandExo(
        comm,
        auto_connect=True,
        command_delimiter="\r\n",
        send_delay=0.01,
    )


def _assert_all_other_motors_disabled(enabled: dict, target: int):
    active = sorted(
        int(mid) for mid, value in enabled.items()
        if bool(value) and int(mid) != int(target)
    )
    if active:
        raise RuntimeError(
            f"Refusing active test: other enabled motor IDs detected: {active}"
        )


def collect_read_only_report(exo: HandExo, samples: int, interval_s: float) -> dict:
    info = exo.info()
    angle_samples = []
    for index in range(samples):
        angles = exo.get_motor_angle("all")
        if not angles or any(value is None or not math.isfinite(float(value)) for value in angles.values()):
            raise RuntimeError(f"Invalid angle sample {index + 1}: {angles}")
        angle_samples.append({int(mid): float(value) for mid, value in angles.items()})
        if index + 1 < samples:
            time.sleep(interval_s)
    motor_ids = sorted(angle_samples[0])
    if any(sorted(sample) != motor_ids for sample in angle_samples):
        raise RuntimeError("Motor ID set changed between angle samples")
    return {
        "status": "read-only checks passed",
        "info": info,
        "motor_ids": motor_ids,
        "angle_samples_deg": angle_samples,
        "limits_deg": exo.get_motor_limits("all"),
        "enabled": exo.is_enabled("all"),
    }


def exercise_motion(exo: HandExo, motor_id: int, rpm: float, duration_s: float):
    enabled = exo.is_enabled("all")
    _assert_all_other_motors_disabled(enabled, motor_id)
    try:
        exo.set_direct_command_timeout(250)
        exo.set_control_mode("velocity")
        exo.enable_motor(motor_id)
        exo.set_motor_velocity(motor_id, rpm)
        time.sleep(duration_s)
    finally:
        try:
            exo.stop_direct_control(motor_id)
        finally:
            exo.disable_motor(motor_id)
            exo.set_control_mode("position")


def exercise_hold(exo: HandExo, motor_id: int):
    enabled = exo.is_enabled("all")
    _assert_all_other_motors_disabled(enabled, motor_id)
    angle = float(exo.get_motor_angle(motor_id))
    try:
        exo.set_control_mode("velocity")
        exo.hold_motor_position(motor_id, angle)
        time.sleep(0.25)
    finally:
        try:
            exo.release_motor_hold(motor_id)
        finally:
            exo.set_control_mode("position")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    exo = make_exo(args)
    try:
        report = collect_read_only_report(exo, args.samples, args.sample_interval)
        if args.exercise_hold is not None:
            exercise_hold(exo, args.exercise_hold)
            report["hold_test"] = {"motor_id": args.exercise_hold, "status": "passed"}
        if args.exercise_motion is not None:
            exercise_motion(exo, args.exercise_motion, args.motion_rpm, args.motion_duration)
            report["motion_test"] = {
                "motor_id": args.exercise_motion,
                "rpm": args.motion_rpm,
                "duration_s": args.motion_duration,
                "status": "passed",
            }
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    finally:
        exo.close()


if __name__ == "__main__":
    raise SystemExit(main())
