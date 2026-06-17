# AGENTS.md — NML Hand Exoskeleton

Dual-stack: Arduino C++ on OpenRB-150 + Python SDK on host PC.
9 Dynamixel XL330 motors per side. USB serial (57600) or HC-05 BT on Serial3 D13/D14 (57600).

## Agent quickstart (do first)

1. Read [docs/gotchas.md](docs/gotchas.md), [docs/serial_protocol.md](docs/serial_protocol.md), and [docs/dual_exo_architecture.md](docs/dual_exo_architecture.md) before changing firmware or parsers.
2. Prefer Python-side fixes first. Touch firmware only when Python-side mitigation is impossible.
3. In dual firmware, always target motors by integer DXL ID for calibration and motor commands.
4. Keep firmware/Python protocol in sync: command names, delimiter, and motor-name mapping are shared contracts.

---

## Canonical commands

```powershell
# Environment + install
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip setuptools wheel
python -m pip install -e .

# Run GUI
handexo gui

# Build wheel
python -m build --wheel

# Common diagnostics
python test_bluetooth_ports.py
python test_hc05_data_mode.py
python test_gui_threading.py
python test_hc05_wiring.py

# Basic connectivity check
python examples/basic/hand_exo_cli.py --list-ports
python examples/basic/hand_exo_cli.py --connect COM5 --info
```

Notes:
- There is no single repo-wide lint/typecheck/test gate configured in the root project; use targeted scripts relevant to your change.
- If you build `src/nml_wtf_exo`, clean `build/`, `dist/`, and `*.egg-info/` first to avoid recursive artifacts.

---

## Key paths

```
src/cpp/nml_hand_exo/
  config.h                 Hardware constants, motor IDs/names, baud rates, BUILD_LEFT_HAND flag
  utils.cpp                Serial command parser — source of truth for all command names
  nml_hand_exo.cpp         NMLHandExo motor-control class
  gesture_controller.cpp   Gesture execution — acts on all firmware-managed motors
  gesture_library.cpp      Gesture definitions (sparse, relative, normalized 0–1)

src/nml_hand_exo/interface/
  _hand_exo.py             HandExo — high-level Python API, response parser
  _dual_hand_exo.py        DualHandExo — wrapper for two separate HandExo instances
                           (NOT used by the GUI; GUI connects a single HandExo to one shared port)
  _interfaces.py           SerialComm / TCPComm

src/nml_hand_exo/applications/
  hand_exo_gui.py          PyQt5 main GUI window. Calibration and ROM are modal QDialogs
                           launched from it, not tabs. The QTabWidget holds Controls and
                           Telemetry only.

examples/calibration/
  calibrate_exo.py         CLI calibration wizard (also updates config.h)
  rom_assessment.py        ROM protocol → output_data/<name>_rom_<date>_<run>.csv
  profiles/<name>.json     Per-user calibration profiles (include "side" metadata field)
```

---

## Dual-exo hardware model

**One OpenRB-150 board, one Dynamixel bus, one serial port** — regardless of GUI mode.

| Side  | Motor IDs | firmware build |
|-------|-----------|----------------|
| Left  | 1–9       | BUILD_LEFT_HAND 1 or 2 |
| Right | 11–19     | BUILD_LEFT_HAND 0 or 2 |
| Both  | 1–9 + 11–19 | BUILD_LEFT_HAND 2 (dual, currently active) |

GUI mode dropdown:
- **Right Only** — widgets and commands target IDs 11–19 only
- **Left Only**  — widgets and commands target IDs 1–9 only
- **Dual**       — widgets show both sides; gesture target combo selects which side receives gestures

At connect time, every motor NOT in the active mode's ID range is explicitly disabled
so firmware broadcast commands (e.g. `set_gesture`) cannot move the inactive side.

**Duplicate motor names in dual firmware.** `MOTOR_NAMES[]` contains "wrist" twice — once
for ID 1 (left) and once for ID 11 (right). Firmware `getMotorIDByName()` returns the
**first** match, always left. Any command using a bare name in dual mode may silently
target the wrong motor. Always use explicit integer IDs for calibration and motor commands.

See [docs/dual_exo_architecture.md](docs/dual_exo_architecture.md) for the full model.

---

## Safety rules (mandatory, every session)

1. Never command a motor outside its `jointLimits`. Physical damage + injury risk.
2. Current limit is 910 mA for XC330-T288 participants with finger spasticity.
   Do not raise `MOTOR_CURRENT_LIMIT` beyond the documented control-table max.
3. Before passive movement: disable only the target-side motors by ID (`disable:<id>`), not
   `disable:all`, to avoid accidentally re-enabling inactive-side motors or confusing GUI state.
4. Do not add motion to `setup()` — `initializeMotors()` holds position.
5. Calibration profiles are safety-critical. Verify plausibility before participant use.
6. Wrist (ID 1) range is intentionally multi-turn (`-189° to 2840°`). Do not clamp to 360°.

---

## Protocol rules (mandatory)

- Do not touch firmware unless Python-only is impossible. Read `utils.cpp` first.
- Command names are a shared contract. Rename in `utils.cpp` → update every Python parser.
- Delimiter is `;` in both `config.h` and `SerialComm.__init__()`. Change one → change both.
- Motor names are the join key: `config.h:MOTOR_NAMES[]` → firmware `info` → Python → `profiles/*.json`.
- In dual firmware, always use integer DXL IDs (not bare names) in calibration commands.
  `HandExo.apply_calibration(name_to_id=...)` enforces this; never call it without `name_to_id`
  when connected to dual firmware.

Additional protocol traps:
- Firmware `get_torque:single` can include a ` N·m` suffix; parse numeric values defensively.
- Keep new variables locally scoped inside `utils.cpp:parseMessage()` branches; avoid shared mutable parser state.

---

## Open tasks

- [ ] Reflash firmware with `utils.cpp` torque fix → verify Torque column in Telemetry tab
- [ ] Mirror streaming calibration into `calibrate_exo.py` CLI (or document divergence)
- [ ] Live device test: apply calibration profile with `name_to_id`, confirm per-motor limits received
- [ ] Decide: should GUI call `update_config_h()` after calibration save? (CLI does; GUI does not)

---

## Docs (load on demand)

| Doc | When to load |
|-----|-------------|
| [docs/dual_exo_architecture.md](docs/dual_exo_architecture.md) | Dual-exo bus model, ID ranges, command routing, name disambiguation |
| [docs/calibration_flow.md](docs/calibration_flow.md) | Calibration, profiles, ROM, side-specific apply |
| [docs/telemetry_architecture.md](docs/telemetry_architecture.md) | Telemetry tab, polling, firmware parsing |
| [docs/apply_and_gesture_state.md](docs/apply_and_gesture_state.md) | Profile apply, default profile, `_gesture_ready`, dual-mode gesture routing |
| [docs/gui_workflow.md](docs/gui_workflow.md) | GUI class map, CalibrationDialog, ROMDialog, mode selection |
| [docs/serial_protocol.md](docs/serial_protocol.md) | Firmware commands, baud rates, response format |
| [docs/gotchas.md](docs/gotchas.md) | Known bugs, traps, firmware quirks, dual-mode traps |
