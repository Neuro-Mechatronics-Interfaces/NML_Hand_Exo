# NML Hand Exoskeleton

<p align="center">
  <img src="https://raw.githubusercontent.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/refs/heads/dev/shadow-contact-phase1/docs/assets/nml-hand-exo.png" alt="Rendered NML Hand Exoskeleton and controller" width="900">
</p>

[![Python 3.10–3.12](https://img.shields.io/badge/python-3.10--3.12-blue.svg)](https://www.python.org/)
[![CI](https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/actions/workflows/python-ci.yml/badge.svg)](https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/actions/workflows/python-ci.yml)
[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-b31b34.svg)](https://neuro-mechatronics-interfaces.github.io/NML_Hand_Exo/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

`nml_hand_exo` is the Python SDK, operator software, and OpenRB-150 firmware for
the NML Hand Exoskeleton. It supports explicit Dynamixel control, participant
calibration, Lab Streaming Layer (LSL) integration, event-marked task sessions,
and continuous EMG intent decoding.

This is research software for supervised laboratory use. It is not a medical
device. Verify calibration, current limits, joint limits, and the emergency-stop
workflow before placing the device on a participant.

## Quick links

- [Documentation](https://neuro-mechatronics-interfaces.github.io/NML_Hand_Exo/)
- [Examples](examples/README.md)
- [Serial protocol](docs/serial_protocol.md)
- [Dual-exoskeleton architecture](docs/dual_exo_architecture.md)
- [EMG intent architecture](docs/emg_intent_architecture.md)
- [Issue tracker](https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/issues)

## Features

- USB serial, dual-CDC USB, Bluetooth serial, and TCP communication transports.
- Single-side and dual-side operation with explicit integer Dynamixel IDs.
- PyQt operator GUI for connection, calibration, telemetry, direct control, and
  guarded EMG teleoperation.
- Participant task-cue application with optional LSL event markers.
- Legacy centroid and current LDA-based continuous intent decoders.
- XDF session import, decoder validation, playback utilities, and analysis tools.
- OpenRB-150 firmware with joint-limit, current-limit, and direct-command safety
  handling.
- Hardware-independent fakes and unit tests for host-side development.

## Installation

Python 3.10 through 3.12 is supported. Python 3.11 is used for hardware
development and is the recommended version on Windows.

From PyPI after a release:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install nml-hand-exo
```

From a source checkout:

```powershell
git clone https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo.git
cd NML_Hand_Exo
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e .
```

The Python package does not require a sibling repository, private drive, or
lab-local package.

## Optional dependencies

Extras keep research integrations separate from the supported runtime:

```powershell
# Plotting and tabular analysis tools
python -m pip install -e ".[analysis]"

# MindRove, joystick, and WebSocket integrations
python -m pip install -e ".[integrations]"

# Legacy PyTorch model helpers
python -m pip install -e ".[ml]"

# Tests and packaging tools
python -m pip install -e ".[dev]"
```

LabRecorder is optional external software. It is useful for creating XDF files
that combine EMG and task markers, but it is not needed to import the package,
run the exoskeleton GUI, or use the Python API.

## Getting started

Connect to one OpenRB command port at 1 Mbps and inspect the firmware-reported
motor map:

```python
from nml_hand_exo import HandExo, SerialComm

exo = HandExo(SerialComm(port="COM12", baudrate=1_000_000))
try:
    exo.connect()
    info = exo.info()
    print(info)
    print(exo.get_motor_angle("all"))
finally:
    exo.close()
```

Use integer IDs reported by `info()` for calibration and motion commands. Bare
motor names are ambiguous in dual firmware. Never command motion outside the
active calibration profile's joint limits.

## CLI usage

Installation provides two commands:

```powershell
handexo --help
handexo --version
handexo gui
handexo emg-intent
handexo emg-centroid

nml-task-cue --help
nml-task-cue
```

`handexo gui` is the main operator application. `handexo emg-intent` is the
maintained discovery, validation, visualization, and continuous-decoding
workflow. `handexo emg-centroid` remains available for backward compatibility
and comparison with older recordings.

## Examples

Examples are grouped by purpose under [`examples/`](examples/README.md). Run
them from the repository root so package imports and relative resources resolve
consistently:

```powershell
python examples/01_basic/example_serial_exo.py --help
python examples/01_basic/example_serial_exo.py --port COM12
python examples/calibration/calibrate_exo.py --help
python examples/08_udp/udp_gesture_receiver.py --help
python tools/import_xdf_intent_session.py --help
```

Hardware examples do not move motors during `--help`. Read the safety notes in
their local README before running a motion command.

## Package structure

```text
src/nml_hand_exo/
├── applications/   PyQt operator, task-cue, and decoder applications
├── calibration/    calibration profile and ROM helpers
├── control/        compatibility EMG trigger controllers
├── decoding/       features, sessions, LDA models, and stabilization
├── interface/      serial/TCP/LSL transports and HandExo API
├── ml/             optional legacy PyTorch model helpers
├── plotting/       PyQtGraph visualizers
├── processing/     signal processing and orientation helpers
└── testing/        hardware-independent OpenRB test doubles

src/cpp/nml_hand_exo/   OpenRB-150 firmware
src/pico_server/        optional CircuitPython TCP bridge
examples/               runnable hardware and integration examples
tools/                  offline analysis and session-conversion utilities
docs/                   architecture, protocol, and Sphinx documentation
```

## Documentation

Protocol and architecture documents live directly in [`docs/`](docs/). Build
the Sphinx reference locally with:

```powershell
python -m pip install -e ".[docs]"
python -m sphinx -W --keep-going docs/source docs/build/html
```

## Development and testing

```powershell
python -m pip install -e ".[analysis,dev]"
python -m pytest -q
python -m compileall -q src examples tools tests
python tools/check_protocol_contract.py
git diff --check
```

Hardware is not required for the unit suite. Tests that exercise a live serial
device are documented separately and must be run with the appropriate firmware,
calibration profile, and physical safety setup.

## Release workflow

Releases use the version declared in `pyproject.toml` and
`nml_hand_exo.__version__`. A release commit must be tagged with the matching
`vX.Y.Z` tag before upload.

```powershell
python -m build
python -m twine check dist/*
```

TestPyPI and PyPI uploads are intentionally manual. See
[`docs/releasing.md`](docs/releasing.md) for the complete verification and
upload commands. Never store an API token in this repository.

## Contributing

Open an issue before making a protocol or firmware contract change. Keep host
and firmware command names synchronized, prefer Python-side compatibility fixes,
and include regression tests for parser or safety behavior. Pull requests should
pass the complete test, compile, documentation, and packaging checks above.

## License

NML Hand Exoskeleton is distributed under the [MIT License](LICENSE).
