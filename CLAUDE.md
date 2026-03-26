# CLAUDE.md — NML Hand Exoskeleton

> `[VERIFIED]` = confirmed from source. `[INFERRED]` = reasonable but unverified.

---

## Project overview

Dual-stack project: Arduino C++ firmware on an OpenRB-150 + Python SDK on a host PC.
9 Dynamixel XL330 motors drive a wearable hand exoskeleton (wrist, thumb 3-DOF, 4 fingers).
Communication: USB serial (57600) or HC-05 Bluetooth on Serial2 D13/D14 (9600).

---

## Current development focus

**Per-DoF guided calibration.** Replace the current all-at-once open/close snapshot with
a per-motor loop that prompts the user for each joint individually.

- Current flow: one global OPEN snapshot → one global CLOSED snapshot → derive all limits
- Target flow: iterate over motors, prompt per joint, record open+close per motor
- **No firmware changes needed** `[VERIFIED]` — `get_absolute_angle:<name>`,
  `set_motor_limits:<name>:min:max`, `set_zero_offset:<name>:val`, `set_flip:<name>:0|1`
  all already work per-motor
- All changes are Python/GUI only → see [docs/calibration_flow.md](docs/calibration_flow.md)

---

## Key paths

```
src/cpp/nml_hand_exo/           Arduino firmware (entry: nml_hand_exo.ino)
  config.h                      All hardware constants and baud rates
  utils.cpp                     Serial command parser — source of truth for command names
  nml_hand_exo.cpp              NMLHandExo motor-control class

src/nml_hand_exo/interface/
  _hand_exo.py                  HandExo — high-level Python API
  _interfaces.py                SerialComm / TCPComm

src/nml_hand_exo/applications/
  hand_exo_gui.py               PyQt5 GUI — primary user interface [VERIFIED]

examples/calibration/
  calibrate_exo.py              CLI calibration wizard (also updates config.h)
  rom_assessment.py             ROM protocol → output_data/<name>_rom_<date>_<run>.csv
  profiles/<name>.json          Per-user calibration profiles
```

---

## Safety rules

These apply to every session. Do not skip them.

1. **Never command a motor outside its `jointLimits`.** Exceeding physical bounds
   can damage the mechanism or injure a participant.
2. **Current limit is 200 mA by default.** Do not raise `MOTOR_CURRENT_LIMIT` in
   `config.h` without a specific reason.
3. **`disable:all` before any passive movement.** Torque-enabled motors resist
   movement and can cause injury during calibration or ROM assessment.
4. **No movement in `setup()` without user confirmation.** `initializeMotors()`
   holds current position — do not add `homeAllMotors()` or any motion to startup.
5. **Calibration profiles are safety-critical.** Wrong `home`/limit values cause
   sudden large movements. Verify physiological plausibility before participant use.
6. **Wrist (ID 1) multi-turn range is intentional** (`-189° to 2840°`). Do not clamp to 360°.

---

## Critical protocol rules

Full coupling rules → [docs/serial_protocol.md](docs/serial_protocol.md)

- **Do not touch firmware unless a Python-only solution is impossible.**
  Read `utils.cpp` first. If an existing command covers the need, use it.
- **Command names and response labels are a shared contract.**
  Rename anything in `utils.cpp` and you must update every Python parser that reads it.
- **Delimiter is always `;`** in both `config.h` (`COMMAND_DELIMITER`) and
  `SerialComm.__init__()`. Change one → change both.
- **Motor names are the join key** across firmware, profiles, and Python API calls.
  `config.h:MOTOR_NAMES[]` must stay in sync with `profiles/<name>.json` keys.

---

## Per-DoF calibration checklist

- [x] Audit `CalibrationDialog._record()` — understand what must change for per-motor iteration
- [ ] Decide: extract shared helpers into `calibration_utils.py` first, or add inline?
- [x] Design per-DoF prompt sequence (motor order, anatomical descriptions)
- [x] Implement per-motor loop in `CalibrationDialog` (GUI, Python only)
- [ ] Mirror change in `calibrate_exo.py` CLI or explicitly document divergence
- [ ] Add validation: warn if `limit_min == limit_max` or `home` outside `[min, max]`
- [ ] Decide: should GUI call `update_config_h()` after save? (CLI does; GUI currently does not)
- [ ] Live device test: apply profile and confirm per-motor limits received by firmware

---

## Quick-start commands

```bash
source .venv/Scripts/activate                          # activate venv (Git Bash)
.venv\Scripts\activate                                 # activate venv (cmd/PowerShell)

python src/nml_hand_exo/applications/hand_exo_gui.py  # launch GUI

python examples/calibration/calibrate_exo.py --port COM<N> --name <profile>
python examples/calibration/rom_assessment.py  --port COM<N> --profile <name>
python examples/01_basic/example_serial_exo.py         # USB connectivity check
python examples/01_basic/example_bluetooth_exo.py      # BT connectivity check
```

---

## Detailed docs (load on demand)

| Doc | Load when... |
|-----|-------------|
| [docs/calibration_flow.md](docs/calibration_flow.md) | Working on calibration logic, profiles, or ROM |
| [docs/gui_workflow.md](docs/gui_workflow.md) | Working on `hand_exo_gui.py` or `CalibrationDialog` |
| [docs/serial_protocol.md](docs/serial_protocol.md) | Touching firmware, comms, baud rates, or command parsing |
