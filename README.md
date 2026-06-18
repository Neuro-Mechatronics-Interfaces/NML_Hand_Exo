# NML Hand Exoskeleton

<p align="center">
  <img src="docs/source/_static/new exo.png" width="80%"/>
</p>

[![Python](https://img.shields.io/badge/python-3.11-blue)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/actions/workflows/gh-pages.yml/badge.svg)](https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/actions/workflows/gh-pages.yml)
[![Docs](https://img.shields.io/badge/docs-online-blue.svg)](https://neuro-mechatronics-interfaces.github.io/NML_Hand_Exo/)

Firmware and Python tools for the **NML Hand Exoskeleton** — a modular, open-source robotic hand exoskeleton for neuromechanics research. Supports single-hand and dual-hand (bilateral) configurations on a shared Dynamixel bus.

Tested on **Windows 11**, **Python 3.11**.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware / System Architecture](#2-hardware--system-architecture)
3. [Software Architecture](#3-software-architecture)
4. [Connection Modes](#4-connection-modes)
5. [Installation](#5-installation)
6. [Launching the GUI](#6-launching-the-gui)
7. [Calibration Workflow](#7-calibration-workflow)
8. [Serial Command Reference](#8-serial-command-reference)
9. [Serial Command Examples](#9-serial-command-examples)
10. [Important Caveats / Gotchas](#10-important-caveats--gotchas)
11. [Repository Map](#11-repository-map)
12. [How to Cite](#12-how-to-cite)

---

## 1. Project Overview

The NML Hand Exo is a 9-DoF per side robotic hand exoskeleton driven by Dynamixel XL330 motors.
The system has three components:

| Component | Role |
|-----------|------|
| **Firmware** (Arduino C++) | Real-time motor control on OpenRB-150. Parses serial commands, executes gestures, enforces joint limits. |
| **Python backend** (`HandExo`) | High-level host API. Sends commands over serial, parses responses, manages calibration profiles. |
| **PyQt5 GUI** | Desktop application for interactive control, calibration, ROM assessment, and telemetry. |

Both USB and Bluetooth (HC-05) connections are supported. The same command protocol is used over both channels.

---

## 2. Hardware / System Architecture

### One board, one bus, one port

The physical setup is always:

```
PC (Python)  ←→  Serial (USB or BT)  ←→  OpenRB-150  ←→  Dynamixel bus  ←→  Motors
```

There is **one OpenRB-150 microcontroller**, **one Dynamixel bus**, and **one serial port** — even in dual-exo (bilateral) configurations. Both the left and right exo motors share the same bus.

### Motor IDs and names

Each exo has 9 motors. In dual firmware (`BUILD_LEFT_HAND 2`, currently active), all 18 motors live on one bus:

| Side  | DXL Hardware IDs | firmware build flag |
|-------|-----------------|---------------------|
| Left  | 1–9             | `BUILD_LEFT_HAND 1` or `2` |
| Right | 11–19           | `BUILD_LEFT_HAND 0` or `2` |
| Both  | 1–9 + 11–19     | `BUILD_LEFT_HAND 2` (dual, active) |

Motor names (9 per side, in ID order):

```
wrist  wrist2  thumbadd  thumbrot  thumbflex  index  middle  ring  pinky
```

### Duplicate motor names in dual mode

In dual firmware, `MOTOR_NAMES[]` contains both sides' names — so "wrist" appears twice: once for ID 1 (left) and once for ID 11 (right). Firmware name lookup always returns the **first match**, which is always the left motor. See [Section 10](#10-important-caveats--gotchas) for how the software handles this safely.

### Serial connections

| Channel          | Arduino object | Baud  | Physical connection   |
|------------------|----------------|-------|-----------------------|
| USB debug/control | `Serial`      | 57600 | USB port              |
| Dynamixel bus    | `Serial1`      | 57600 | JST DXL connector     |
| HC-05 Bluetooth  | `Serial3`      | 57600 | D13 (TX3), D14 (RX3)  |

HC-05 modules ship at 9600 baud from factory. Use AT command mode to set them to 57600 before use.

---

## 3. Software Architecture

```
src/
├── cpp/nml_hand_exo/
│   ├── nml_hand_exo.ino            Entry point, setup/loop
│   ├── config.h                    Motor IDs, names, limits, baud rates, BUILD_LEFT_HAND flag
│   ├── utils.cpp                   Serial command parser — parseMessage() dispatches all commands
│   ├── nml_hand_exo.cpp/.h         NMLHandExo class — motor control, angle conversion, limits
│   ├── gesture_controller.cpp/.h   Executes named gestures across all firmware motors
│   └── gesture_library.cpp/.h      Gesture definitions (grasp, keygrip, pinch_*, peace)
│
└── nml_hand_exo/
    ├── interface/
    │   ├── _hand_exo.py            HandExo — primary Python API, serial communication, profile apply
    │   ├── _dual_hand_exo.py       DualHandExo — wrapper for two separate HandExo instances
    │   │                           (NOT used by the GUI; provided as an alternative model)
    │   └── _interfaces.py          SerialComm / TCPComm transport layer
    └── applications/
        └── hand_exo_gui.py         PyQt5 GUI — Controls tab, Telemetry tab,
                                    CalibrationDialog, ROMDialog
```

**The GUI uses a single `HandExo` instance** connected to one shared port, regardless of mode (Left Only / Right Only / Dual). The `DualHandExo` class exists as an alternative design for setups with two separate serial ports, but it is not the active GUI architecture.

### Key Python classes

| Class | File | Role |
|-------|------|------|
| `HandExo` | `_hand_exo.py` | Serial command interface, profile application, angle reads |
| `CalibrationDialog` | `hand_exo_gui.py` | Streaming joint-limit calibration — two recording phases |
| `ROMDialog` | `hand_exo_gui.py` | 4-phase range-of-motion assessment protocol |
| `HandExoGUI` | `hand_exo_gui.py` | Main window — connection, motor control, gestures, telemetry |

---

## 4. Connection Modes

The GUI mode combo (selected before connecting, locked while connected) determines which motors are active:

| Mode | Active motor IDs | Widgets shown | Gesture target |
|------|-----------------|---------------|----------------|
| **Right Only** | 11–19 | 9 right motors | Right side only |
| **Left Only** | 1–9 | 9 left motors | Left side only |
| **Dual** | 1–9 + 11–19 | 18 motors (L:/R: prefixed) | Selectable via combo |

### What "active" means

At connect time, `_connect()` builds `_motor_dxl_id` — the list of DXL hardware IDs for the selected mode. Any motor detected on the bus but **not** in that list is immediately disabled (`disable:<id>`). This prevents firmware broadcast gesture commands from moving the inactive side.

In **Dual** mode all detected motors are active, so nothing is pre-disabled. The gesture target combo in the GUI controls which side receives gesture commands.

### Command routing

| Operation | How it targets motors |
|-----------|-----------------------|
| Enable / Disable / Home | Per-ID from `_motor_dxl_id` |
| Set angle (slider) | Per-ID from `_motor_dxl_id` |
| Calibration apply | Per-ID via `name_to_id` mapping |
| `set_gesture` | Firmware broadcast — moves all motors on bus |
| `enable:all` / `disable:all` | Firmware broadcast — affects all motors on bus |

See [Section 10](#10-important-caveats--gotchas) for broadcast implications.

---

## 5. Installation

### Clone

```bash
git clone https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo.git
cd NML_Hand_Exo
```

### Create a virtual environment

**Windows PowerShell:**
```powershell
py -3.11 -m venv .venv
Set-ExecutionPolicy -Scope Process -ExecutionPolicy RemoteSigned
.\.venv\Scripts\Activate.ps1
```

**Windows CMD:**
```cmd
py -3.11 -m venv .venv
.\.venv\Scripts\activate.bat
```

**macOS / Linux:**
```bash
python3.11 -m venv .venv
source .venv/bin/activate
```

### Install dependencies

```bash
python -m pip install --upgrade pip setuptools wheel
python -m pip install -e .
```

### Notes

- **Python 3.11 required.** Package requires `>=3.10`; 3.11 is the tested version.
- Do not commit `.venv/` or similar environment directories.
- If you see stale behavior after pulling changes, recreate the venv.

### Firmware

Open `src/cpp/nml_hand_exo/nml_hand_exo.ino` in the Arduino IDE.

**Board:** OpenRB-150 (install the ROBOTIS board package via Arduino IDE board manager)

**Required Arduino libraries:**
- Dynamixel2Arduino
- Adafruit BNO055
- Adafruit Unified Sensor
- Adafruit SSD1306
- Adafruit GFX Library

Upload → board flashes LED 4× and prints `"Exo device ready to receive commands"`.

Set `BUILD_LEFT_HAND` in `config.h` before flashing:
- `0` = right exo only (IDs 11–19)
- `1` = left exo only (IDs 1–9)
- `2` = dual — both sides on one bus (IDs 1–9 + 11–19)

---

## 6. Launching the GUI

```bash
handexo gui
```

Run this after activating the virtual environment and installing the repository
with `python -m pip install -e .`. The direct Python file command remains available
for development.

The GUI provides:

- **Connection panel** — COM port selector, mode combo (Right Only / Left Only / Dual), Connect/Disconnect
- **Controls tab**
  - Per-motor angle sliders and enable/disable toggles
  - Gesture presets with a lazy-initialize safety gate (profile applied before first gesture)
  - **Calibration** button — opens `CalibrationDialog`
  - **ROM Assessment** button — opens `ROMDialog`
- **Telemetry tab** — live table: position, torque, and current for active motors; manual Refresh
- **Log panel** — timestamped command/response log

The **Settings tab** configures optional telemetry and command integrations:

- LSL streams `NMLHandExoJointAngles` and `NMLHandExoMotorTorque`
- UDP telemetry JSON output, default destination `127.0.0.1:10002`
- UDP command input, default bind `0.0.0.0:10001` and disabled by default
- Continuous telemetry with a configurable target up to 20 Hz; the Telemetry tab
  reports the measured rate because serial round trips may limit the achieved rate

UDP command input accepts `set_gesture:*` commands by default. Advanced mode uses a
restricted allowlist and requires explicit active motor IDs for enable, disable, and
home commands. Angle and calibration commands are not forwarded over UDP. Settings
shows the last received command and whether it was accepted or rejected.

The Teleop tab's WebSocket connection is an outbound telemetry client: it sends
normalized joint-state frames to a WebSocket server. It does not listen for commands.
Use UDP Command Input for inbound commands, even when both endpoints are on localhost.

The **Direct Control tab** supports guarded velocity and current/estimated-torque
control with explicit motor arming. Velocity is limited to ±10 rpm, current to
±910 mA, and the command button must be held down. Releasing it sends zero; the
firmware independently stops stale commands after its watchdog timeout. Direct
control requires firmware version `0.2.14` or newer. Normal multi-query telemetry
polling pauses in direct mode so command refreshes have priority on the serial bus,
then resumes after returning to current-position control.

### Python API (scripting)

```python
from nml_hand_exo.interface._hand_exo import HandExo

exo = HandExo('COM3', baudrate=57600)
exo.enable_motor(1)                  # enable motor ID 1
angle = exo.get_motor_angle(1)       # relative angle
exo.set_motor_angle(1, 45)
exo.disable_motor(1)
```

---

## 7. Calibration Workflow

### Profile storage

Profiles are JSON files in `examples/calibration/profiles/<name>.json`:

```json
{
  "side": "right",
  "motors": {
    "wrist":     { "home": 149.1, "flip": false, "limit_min": -189.0, "limit_max": 2840.0 },
    "index":     { "home": 162.8, "flip": false, "limit_min": 162.8,  "limit_max": 224.93 }
  }
}
```

The `"side"` field determines which side's profile list shows in the GUI. Legacy profiles without `"side"` are treated as right.

`examples/calibration/profiles/config.json` stores defaults per side:

```json
{
  "default_left":  "alice left",
  "default_right": "alice right",
  "default":       "alice right"
}
```

### GUI calibration (streaming, recommended)

1. Select mode and connect
2. Click **Calibration**; select target side in Dual mode
3. Phase 0 (Extension): click Start, move all joints to their open extreme, click Stop
4. Phase 1 (Flexion): move all joints to their closed extreme, click Stop
5. Profile is saved and set as default; GUI offers to apply it immediately

### CLI calibration (updates config.h)

```bash
python examples/calibration/calibrate_exo.py --port COM<N>
```

The CLI also updates `src/cpp/nml_hand_exo/config.h` with the calibrated limits, making them survive firmware reboots. The GUI does not update `config.h`.

### Applying a profile

Profile application pushes three values per motor to the device:

```
set_zero_offset:<id>:<home>
set_motor_limits:<id>:<min>:<max>
set_flip:<id>:<0|1>
```

In dual mode, explicit DXL integer IDs are always used (not bare motor names) to avoid left-motor-always ambiguity. This is handled automatically by `HandExo.apply_calibration(name_to_id=...)`.

**Applied calibration is not persistent across reboots** unless `config.h` is updated and firmware is reflashed. See [docs/apply_and_gesture_state.md](docs/apply_and_gesture_state.md).

### ROM Assessment

```bash
python examples/calibration/rom_assessment.py --port COM<N>
```

Or launch from the GUI via **ROM Assessment**. Runs a 4-phase protocol (unassisted open/close, assisted open/close) and saves results to:

```
output_data/<participant>_rom_<date>_<run>.csv
```

---

## 8. Serial Command Reference

Commands are plain ASCII, colon-delimited, newline-terminated:

```
<command>:<arg1>:<arg2>\n
```

Responses are terminated with `;`. Commands are case-insensitive.

Motor references accept either an **integer DXL ID** (e.g. `11`) or a **bare motor name** (e.g. `wrist`). In dual firmware, always prefer integer IDs to avoid name ambiguity. `all` is accepted where noted.

### Motor enable / disable

| Command | Args | Description |
|---------|------|-------------|
| `enable` | `<motor\|all>` | Enable torque for one motor or all |
| `disable` | `<motor\|all>` | Disable torque for one motor or all |
| `get_enabled` | `<motor\|all>` | Query torque-enable state |

### Angle control

| Command | Args | Description | Notes |
|---------|------|-------------|-------|
| `get_angle` | `<motor\|all>` | Get relative angle (°) from home | Relative to zero offset |
| `set_angle` | `<motor>:<deg>` | Set relative angle | |
| `get_absolute_angle` | `<motor\|all>` | Get multi-turn absolute angle (°) | Key value used by calibration |
| `set_absolute_angle` | `<motor>:<deg>` | Set absolute angle | |
| `home` | `<motor\|all>` | Move to home position | |
| `get_home` | `<motor\|all>` | Query current home value | |
| `set_home` | `<motor>:<deg>` | Set home position | |
| `set_zero_offset` | `<motor\|all>:<deg>` | Set the zero-angle offset (calibration) | Use integer ID in dual mode |
| `set_yaw_angle` | `<motor>:<deg>` | Set angle using yaw/IMU reference | |

### Joint limits and flip

| Command | Args | Description | Notes |
|---------|------|-------------|-------|
| `get_motor_limits` | `<motor\|all>` | Query current joint limits | |
| `set_motor_limits` | `<motor>:<min>:<max>` | Set both limits in one command | Use integer ID in dual mode |
| `set_upper_limit` | `<motor>:<deg>` | Set upper joint limit | |
| `set_lower_limit` | `<motor>:<deg>` | Set lower joint limit | |
| `get_flip` | `<motor\|all>` | Query flip direction | |
| `set_flip` | `<motor>:<0\|1>` | Set flip direction (1 = inverted) | Use integer ID in dual mode |

### Telemetry

| Command | Args | Description | Notes |
|---------|------|-------------|-------|
| `get_current` | `<motor\|all>` | Motor current (mA) | |
| `get_current_lim` | `<motor\|all>` | Current limit (mA) | |
| `set_current_lim` | `<motor>:<mA>` | Set current limit | XC330-T288 build clamps to 910 mA |
| `get_torque` | `<motor\|all>` | Estimated torque (N·m) | Requires reflash of torque fix |
| `get_goal_velocity` | `<motor\|all>` | Velocity limit (rpm) | |
| `set_goal_velocity` | `<motor\|all>:<val>` | Set velocity limit | |
| `get_goal_acceleration` | `<motor\|all>` | Acceleration limit | |
| `set_goal_acceleration` | `<motor\|all>:<val>` | Set acceleration limit | |
| `get_baud` | `<motor\|all>` | Query motor baud rate | |
| `set_baud` | `<motor>:<baud>` | Set motor baud rate | |

### Gesture control

Requires `set_exo_mode:gesture_fixed` first. Gesture commands are **firmware-level broadcasts** — they target all `N_MOTORS` on the bus regardless of mode. Inactive-side motors must be disabled before use.

| Command | Args | Description |
|---------|------|-------------|
| `set_exo_mode` | `<mode>` | Set operating mode (`gesture_fixed`, `gesture_continuous`, `free`) |
| `get_exo_mode` | — | Query current exo mode |
| `set_gesture` | `<name>:<state>` | Execute a gesture at a state (`open` or `close`) |
| `get_gesture` | — | Query current active gesture name |
| `gesture_list` | — | List all available gestures |
| `get_gesture_state` | — | Query current gesture state (`open`/`close`) |
| `set_gesture_state` | `<state>` | Set state of current gesture |
| `cycle_gesture` | — | Advance to next gesture (in gesture mode only) |
| `cycle_gesture_state` | — | Toggle state of current gesture |

Available gestures (6 total): `grasp`, `keygrip`, `pinch_index`, `pinch_middle`, `pinch_ring`, `peace`

### Motor and exo mode

| Command | Args | Description |
|---------|------|-------------|
| `get_motor_mode` | `<motor>` | Query Dynamixel operating mode |
| `set_motor_mode` | `<motor>:<mode>` | Set Dynamixel operating mode |

### System / info

| Command | Args | Description |
|---------|------|-------------|
| `info` | — | Print motor name, ID, and config for all motors |
| `version` | — | Print firmware version string |
| `get_imu` | — | Print IMU orientation data (BNO055) |

---

## 9. Serial Command Examples

### Query motor info

```
info
version
gesture_list
```

### Read angles and telemetry

```
# All motors — returns one block per motor
get_absolute_angle:all
get_angle:all
get_current:all

# Single motor by ID (safe in dual mode)
get_absolute_angle:11
get_current:1
get_enabled:11
```

### Enable / disable by ID (single-exo or dual-aware)

```
# Single motor
enable:1
disable:11

# All motors on bus (broadcast — use with care in dual mode)
enable:all
disable:all
```

### Disable one full side (right exo, IDs 11–19)

Send individually — there is no range syntax:
```
disable:11
disable:12
disable:13
disable:14
disable:15
disable:16
disable:17
disable:18
disable:19
```

### Home motors

```
# Home all (broadcast)
home:all

# Home single motor by ID
home:1
home:11
```

### Gesture commands (dual-aware usage)

Before gestures, disable the inactive side and set gesture mode:

```
# In Right Only use: disable left side first, then gesture
disable:1
disable:2
disable:3
disable:4
disable:5
disable:6
disable:7
disable:8
disable:9
set_exo_mode:gesture_fixed

# Execute a gesture
set_gesture:grasp:open
set_gesture:grasp:close
set_gesture:pinch_index:close

# Cycle through gestures
cycle_gesture
cycle_gesture_state

# Query state
get_gesture
get_gesture_state
gesture_list
```

### Calibration commands (always use IDs in dual mode)

```
# Set zero offset — right wrist (ID 11), safe in dual mode
set_zero_offset:11:149.1

# WRONG in dual mode — name lookup always returns left motor
set_zero_offset:wrist:149.1   # → targets ID 1 (left), not ID 11

# Set limits — right index finger
set_motor_limits:16:162.8:224.93

# Set flip for right middle finger
set_flip:17:1
```

### Velocity and acceleration

```
# Set velocity limit — all motors
set_goal_velocity:all:100

# Set for a specific motor
set_goal_velocity:11:50
get_goal_velocity:all
```

---

## 10. Important Caveats / Gotchas

### Duplicate motor names in dual mode

In dual firmware, "wrist", "index", etc. each appear twice in `MOTOR_NAMES[]` — once for each side. Firmware name lookup returns the **first match**, which is always the left motor. Commands that use bare motor names (e.g. `set_zero_offset:wrist:X`) always target the **left** motor in dual mode, regardless of intent.

**Safe pattern:** always use the integer DXL ID (`set_zero_offset:11:X` for the right wrist).

### Gesture commands are firmware-level broadcasts

`set_gesture` triggers `executeGesture()` in firmware, which iterates **all** `N_MOTORS` on the bus and calls `setAbsoluteAngle()` unconditionally — there is no side filter in the firmware. In Left Only or Right Only GUI mode, the GUI disables inactive-side motors at connect time so they cannot physically move, but the firmware still writes their goal positions.

Do not call `enable:all` in single-side mode — it will re-enable the inactive side.

### Inactive-side safety depends on disable-at-connect

The inactive-side protection is **behavioral, not structural**. Motors are disabled at `_connect()` time. If you reconnect, reboot the device, or call `enable:all`, inactive-side motors become live again. The GUI's protection does not survive a device reboot.

### Calibration is side-safe only when applied by ID

The GUI always calls `apply_calibration(name_to_id={...})` so commands use integer DXL IDs. Direct API usage without `name_to_id` sends bare motor names, which silently target the wrong side in dual firmware.

### Applied calibration does not persist through reboot

`apply_calibration()` updates the device's runtime state over serial. On power cycle, firmware restores its built-in defaults from `config.h` (`HOME_STATES[]`, `jointLimits[][]`, `DEFAULT_FLIPS[]`). To make calibration permanent, run the CLI calibration wizard (which updates `config.h`) and reflash.

### Wrist range is intentionally multi-turn

Wrist (IDs 1 and 11) has a range of approximately −189° to 2840° to support multi-turn motion. Do not clamp it to 360°.

### Torque telemetry requires firmware reflash

The torque telemetry response format has a known firmware bug (wrong variable, missing newline, wrong key capitalization). The fix is committed to `utils.cpp` but the device has not been reflashed. Torque shows `—` in the GUI until the device is reflashed.

---

## 11. Repository Map

| What you need | Where to look |
|---------------|---------------|
| All serial command names and dispatch logic | `src/cpp/nml_hand_exo/utils.cpp` |
| Motor control — angle conversion, limits, flip | `src/cpp/nml_hand_exo/nml_hand_exo.cpp` |
| Gesture execution and library | `src/cpp/nml_hand_exo/gesture_controller.cpp`, `gesture_library.cpp` |
| Motor IDs, names, limits, baud rates, build flag | `src/cpp/nml_hand_exo/config.h` |
| Python API — send commands, parse responses, apply profiles | `src/nml_hand_exo/interface/_hand_exo.py` |
| GUI — connection, controls, calibration, ROM, telemetry | `src/nml_hand_exo/applications/hand_exo_gui.py` |
| Calibration profiles | `examples/calibration/profiles/*.json` |
| Default profile config | `examples/calibration/profiles/config.json` |
| ROM output data | `output_data/<participant>_rom_<date>_<run>.csv` |

### Developer docs

| Doc | When to read |
|-----|-------------|
| [docs/dual_exo_architecture.md](docs/dual_exo_architecture.md) | Full dual-exo model: ID ranges, name resolution, command routing, `_make_name_to_id` |
| [docs/serial_protocol.md](docs/serial_protocol.md) | Command format, baud rates, response parsing, coupling rules |
| [docs/calibration_flow.md](docs/calibration_flow.md) | Calibration streaming phases, profile schema, side-specific apply |
| [docs/apply_and_gesture_state.md](docs/apply_and_gesture_state.md) | Profile apply internals, `_gesture_ready` flag, apply vs config.h |
| [docs/gui_workflow.md](docs/gui_workflow.md) | GUI class map, CalibrationDialog flow, ROMDialog flow |
| [docs/telemetry_architecture.md](docs/telemetry_architecture.md) | Telemetry tab, polling path, `_motor_dxl_id` vs `_motor_idx` |
| [docs/gotchas.md](docs/gotchas.md) | Known bugs, dual-mode traps, firmware quirks |

---

## 12. How to Cite

If you use this project in your research, please cite:

Jonathan Shulgach & Kriti Kacker. (2025). NML Hand Exoskeleton [Computer software]. https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo

```bibtex
@misc{shulgach_kacker_2025_nmlhandexo,
  author       = {Jonathan Shulgach and Kriti Kacker},
  title        = {NML Hand Exoskeleton},
  year         = {2025},
  publisher    = {GitHub},
  journal      = {GitHub repository},
  howpublished = {\url{https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo}}
}
```

## License

MIT License
