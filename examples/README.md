# Examples

Run examples from the repository root after `python -m pip install -e .`.
Hardware scripts use explicit command-line arguments where practical; start with
`--help`. A successful syntax or help check does not authorize motor motion.

## Directory guide

| Directory | Purpose | Hardware required |
|---|---|---|
| `01_basic` | Serial, Bluetooth-serial, and TCP connections | For connection |
| `02_motor_control` | Motor profiles and batch operations | Yes |
| `03_sensors` | OpenRB IMU reading and feedback | Yes |
| `04_advanced` | Firmware modes and device configuration | Yes |
| `05_applications` | LSL reader and optional joystick UDP control | Depends |
| `06_lsl_streaming` | LSL publishers, subscribers, plots, and gesture bridge | Depends |
| `07_mindrove` | Optional MindRove acquisition integration | MindRove for live use |
| `08_udp` | Maintained UDP gesture receiver and manual sender GUI | Mock mode available |
| `calibration` | Calibration and ROM command-line workflows | Yes |
| `diagnostics` | Fast-telemetry and OpenRB diagnostics | Depends |
| `tests` | Hardware-independent regression tests | No |

## Safe smoke checks

```powershell
python examples/01_basic/example_serial_exo.py --help
python examples/01_basic/example_bluetooth_exo.py --help
python examples/01_basic/example_tcp_exo.py --help
python examples/08_udp/udp_gesture_receiver.py --help
python examples/08_udp/udp_gesture_receiver.py --mock
python examples/calibration/calibrate_exo.py --help
python examples/calibration/rom_assessment.py --help
python examples/diagnostics/benchmark_fast_telemetry.py --help
```

## Common rules

- USB and Dynamixel links use 1,000,000 baud by default. HC-05 Bluetooth uses
  the baud configured for that module, normally 115,200 in current firmware.
- Read motor IDs from `HandExo.info()`. In dual firmware, bare names may select
  the wrong side; calibration and motion code must use integer IDs.
- Do not apply example angles or limits to a worn device. Use a validated,
  side-specific participant calibration profile.
- LSL, LabRecorder, MindRove, pygame, and WebSockets are integrations, not
  dependencies on another local repository. Install optional extras with
  `python -m pip install -e ".[integrations]"`.
- The maintained task-cue application is `nml-task-cue`; older task prototypes
  have been removed rather than presented as supported data-collection tools.

For protocol and dual-CDC details, see `docs/serial_protocol.md` and
`docs/dual_exo_architecture.md`.
