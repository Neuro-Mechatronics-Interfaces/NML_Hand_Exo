# Project Memory — March 2026

Snapshot of what was built and discovered in the 2026-03 development sessions.
Use this for orientation at the start of a new session, not as authoritative spec —
read the relevant source files to confirm current state.

---

## What was implemented

### Telemetry tab (`hand_exo_gui.py`)
- Added `QTabWidget` (Controls | Telemetry) to `HandExoGUI`
- Controls tab: existing section builders routed via layout-redirect pattern, untouched
- Telemetry tab: `QTableWidget` (N motors × 4 cols), Refresh button, Auto-refresh checkbox,
  status label showing last poll result with timestamp
- Two independent `QTimer`s: `_angle_timer` (Controls, 500 ms) and `_telem_timer` (Telemetry, 500 ms)
- Precomputed maps on connect: `_motor_idx` (name → serial index), `_motor_row` (name → table row)
- Split try/except per stream: one failure does not block the other two
- `DARK_STYLE` extended with explicit rules for `QTabBar`, `QTableWidget::item`,
  `QHeaderView::section`, `QCheckBox`

### Python parser fix (`_hand_exo.py`)
- `_parse_motor_data_block`: `current` and `current_limit` now strip unit suffixes
  using `re.match(r'[-+]?[\d.]+', val)` before calling `float()`
- Root cause: firmware appends ` mA` / `mA` to these values; `float()` raised `ValueError`

### Firmware fix (`utils.cpp` — source patched, reflash still required)
- `get_torque:all` branch: fixed key case (`Torque` → `torque`), wrong variable
  (`String(val)` → `String(torque, 4)`), missing newline (`}` → `}\n`)
- No Python change needed after reflash
- **The device has not been reflashed. Torque column shows `—` until this is done.**

### Calibration dialog (`hand_exo_gui.py` — earlier sessions)
- Replaced single-snapshot approach with streaming recording (100 ms timer, sample accumulation)
- Two global phases: extension then flexion; operator moves all joints during each window
- Profile derived from median (home), min/max (limits) of sample lists
- `_validate_profile()`: blocks save on `min==max` or `home` outside bounds; warns on range < 2°

---

## Known working

- Position telemetry: populates correctly
- Current telemetry: Python parser fixed, populates correctly
- Calibration dialog: streaming with validation, profile saved and set as default
- ROM dialog: 4-phase streaming, CSV output, optional cal profile derived from assisted phase
- Profile apply: `exo.apply_calibration()` pushes home/limits/flip per motor over serial
- Gesture lazy-init: `_ensure_gesture_ready()` applies default profile + enables motors on
  first gesture click

---

## Known broken / pending

| Item | Status | Fix needed |
|------|--------|------------|
| Torque column shows `—` | Source patched, device not reflashed | Reflash OpenRB-150 |
| Single-motor torque parse | ` N·m` suffix not stripped in Python | Add regex strip to `torque` case in `_parse_motor_data_block` |
| CLI calibration out of sync | `calibrate_exo.py` still uses single-snapshot | Mirror streaming approach or document divergence |
| Calibration does not update `config.h` | GUI intentionally omits this | Design decision pending |

---

## Files touched in 2026-03

| File | Change summary |
|------|---------------|
| `src/nml_hand_exo/applications/hand_exo_gui.py` | Telemetry tab, streaming calibration, CSS rules |
| `src/nml_hand_exo/interface/_hand_exo.py` | Current/current_limit unit suffix parser fix |
| `src/cpp/nml_hand_exo/utils.cpp` | Torque all-mode response format fix (source only, needs reflash) |
