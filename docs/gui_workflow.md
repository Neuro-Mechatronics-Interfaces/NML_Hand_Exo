# GUI Workflow Reference

> `[VERIFIED]` = confirmed from source. `[INFERRED]` = reasonable but unverified.

File: `src/nml_hand_exo/applications/hand_exo_gui.py`

```bash
python src/nml_hand_exo/applications/hand_exo_gui.py
```

---

## Architecture

The GUI is a single-file PyQt5 application. It owns the `HandExo` connection and passes it
into modal `QDialog` subclasses for calibration and ROM assessment. It does **not** call
or subprocess `calibrate_exo.py` — calibration logic is implemented inline. `[VERIFIED]`

Key classes in `hand_exo_gui.py`:

| Class | Purpose |
|-------|---------|
| Main window (unnamed at top level) | Connection panel, motor control, gesture control |
| `CalibrationDialog` | Interactive calibration — open/close recording |
| `ROMDialog` | ROM assessment — 4-phase recording with QTimer polling |

---

## Connection modes `[VERIFIED]`

The mode combo (Right Only / Left Only / Dual) is selected before connecting and locked
while connected. All modes connect through a single `HandExo` to a single serial port.

At connect time, `_connect()` builds:

| Variable | Content |
|---|---|
| `_motor_dxl_id` | DXL hardware IDs for active-side motors, in widget order |
| `_left_motor_names` | bare motor names for left motors detected |
| `_right_motor_names` | bare motor names for right motors detected |
| `motor_names` | display names (bare in single-side, `L:/R:` prefixed in Dual) |
| `motor_widgets` | one dict per active motor for GUI state tracking |

After building `_motor_dxl_id`, any motor detected on the bus but NOT in that list is
immediately disabled (`disable:<id>`) to prevent firmware broadcast commands from
moving the inactive side. In Dual mode, `_motor_dxl_id` includes all detected IDs, so
nothing is disabled.

---

## CalibrationDialog `[VERIFIED]`

### Initialization
- Receives `HandExo`, `motor_names` (target-side only), `profile_name`, `side`, and `dxl_ids`
- Builds `_motor_idx = {name: dxl_id}` from the provided `dxl_ids` list
- Disables only the target-side motors by DXL ID (not `disable:all`)
- Two streaming phases: extension (open), then flexion (close)

### Recording flow (streaming, 100 ms timer)
```
Phase 0 — Extension:
  QTimer → _poll_angles() every 100 ms
    → exo.get_absolute_motor_angle('all')  returns {dxl_id: value}
    → for each motor, angles.get(_motor_idx[name]) → append to _samples_buf[name]
  Stop → commit to _open_samples if ≥ 3 samples per motor

Phase 1 — Flexion:
  same mechanism → commit to _close_samples
  → _save_profile()
```

### `_save_profile()` — what it computes
```python
for name in motor_names:
    o_med = median(_open_samples[name])
    c_med = median(_close_samples[name])
    all_vals = _open_samples[name] + _close_samples[name]
    data["motors"][name] = {
        "home":      round(o_med, 2),
        "flip":      c_med < o_med,
        "limit_min": round(min(all_vals), 2),
        "limit_max": round(max(all_vals), 2),
    }
save_profile(profile_name, data, side=self._side)   # writes profiles/<name>.json
set_default_profile(profile_name, side=self._side)  # always updates default
```

### What `_save_profile()` does NOT do `[VERIFIED]`
It does **not** call `update_config_h()`. After a GUI calibration, `config.h` firmware
defaults (`HOME_STATES[]`, `jointLimits[][]`, `DEFAULT_FLIPS[]`) are **not updated**.
The device receives new values over serial for the current session, but the firmware
defaults persist until the CLI is used or `config.h` is edited manually.

To update `config.h` after GUI calibration: run
```bash
python examples/calibration/calibrate_exo.py --port COM<N> --apply <profile>
```

---

## ROMDialog `[VERIFIED]`

### State machine
Four phases driven by `self._phase` (0–3):
```
0: Phase 1 – Unassisted ROM: OPEN hand
1: Phase 1 – Unassisted ROM: CLOSED hand
2: Phase 2 – Assisted ROM:   OPEN hand
3: Phase 2 – Assisted ROM:   CLOSED hand
```

### Recording mechanism
- Uses a `QTimer` (`self._timer`) polling `_poll_angles()` on each tick
- Accumulates angle samples in `self._phase_data[phase]` as lists per motor
- User clicks "Start Recording" / "Stop Recording" to control the timer
- Motor orientation (home, flip) is auto-detected from the loaded calibration profile

### Output
Saves to `output_data/<participant>_rom_<date>_<run>.csv` with columns:
`participant, date, run, motor, flip, phase,`
`open_abs_max, open_abs_min, closed_abs_max, closed_abs_min,`
`open_norm_max, open_norm_min, closed_norm_max, closed_norm_min, rom_deg`

Normalized angles: `0 = home/open`, positive = closing direction.
`rom_deg = closed_norm_max − open_norm_min`

---

## Duplicated helpers (GUI vs CLI) `[VERIFIED]`

These functions exist in both `hand_exo_gui.py` and the calibration scripts with
identical or near-identical implementations:

```
list_profiles          load_profile           save_profile
get_default_profile_name  set_default_profile
normalize_angle        determine_run_number
```

Before adding new calibration logic, decide whether to extract a shared
`calibration_utils.py` module. Either way, any change must be applied to both.

---

## Profile paths (GUI) `[VERIFIED]`

The GUI resolves paths relative to the repo root via `_repo_root()`:

```python
PROFILES_DIR = <repo_root>/examples/calibration/profiles/
CONFIG_FILE  = <repo_root>/examples/calibration/profiles/config.json
OUTPUT_DIR   = <repo_root>/output_data/
```

These are the same paths used by the CLI scripts — profiles are fully shared.

---

## Adding per-DoF steps to CalibrationDialog

The recording flow to change is `_record()` in `CalibrationDialog`.
Currently: one button click per phase (open / close) reads all motors at once.
Target: iterate over `self.motor_names`, showing per-motor prompts and recording
one motor at a time before advancing.

The `_save_profile()` method does not need to change — it already iterates over
`motor_names` and computes per-motor values from the stored dicts.

See [docs/calibration_flow.md](calibration_flow.md) for the full task checklist.
