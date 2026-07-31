# Project Memory — April 2026

Snapshot of the dual-exo control routing and calibration fixes implemented in April 2026.
Use for orientation at the start of a new session; always verify against source.

---

## What was fixed

### 1. Gesture / command routing (control-routing bug)

**Root cause**: In Left Only / Right Only mode, the GUI selected the correct motors for
Enable/Disable/Home (used `_motor_dxl_id`), but `set_gesture` was a raw `send_command`
broadcast that the firmware executed on ALL `N_MOTORS` regardless of GUI mode. Inactive-side
motors were never explicitly disabled at connect time, so they responded to gesture commands.

**Fix (`_connect()` in `hand_exo_gui.py`)**: After `_motor_dxl_id` is built, compute
`inactive_ids = detected_ids - active_ids` and send `disable:<id>` for each. In Left Only
mode this disables IDs 11–19; in Right Only it disables IDs 1–9; in Dual mode the set is
empty (no-op).

**Why this is the right design**: `setAbsoluteAngle()` in firmware writes the goal position
register unconditionally (no torque check). Disabling motors at connect time ensures they
physically cannot move even if their goal position is written by a firmware broadcast.

---

### 2. Calibration side-mixing bug (apply_calibration)

**Root cause**: `HandExo.apply_calibration()` sent `set_zero_offset:wrist:X` using bare
motor names. In dual firmware, `MOTOR_NAMES[]` has duplicate names ("wrist" at both ID 1
and ID 11). Firmware `getMotorIDByName()` returns the **first match** — always the left
motor. Applying a right-side profile always set left motor offsets. Right motors were
never calibrated.

The epoch-correction block had the same problem: `abs_by_name` was keyed by bare name,
so both "wrist" entries overwrote each other.

**Fix (`_hand_exo.py` + `hand_exo_gui.py`):**

1. Added `name_to_id: dict = None` parameter to `apply_calibration()`.
2. When provided, each motor command uses the explicit integer DXL ID:
   `set_zero_offset:11:X` (not `set_zero_offset:wrist:X`).
3. Epoch correction now uses `abs_by_id = {motor_id: angle ...}` keyed by hardware ID,
   eliminating the name-collision dict overwrite.
4. Added `_make_name_to_id(side)` helper to the GUI. Builds the mapping from
   `_left_motor_names` / `_right_motor_names` zipped with the filtered `_motor_dxl_id`.
5. All 9 `apply_calibration` call sites in the GUI now pass `name_to_id`.

---

### 3. ROM orientation wrong side (minor)

**Root cause**: `ROMDialog._detect_orientation()` called `get_default_profile_name()`
without `side=` parameter, defaulting to "right" in all modes.

**Fix**: Changed to `get_default_profile_name(side=self._side)`.

---

### 4. CalibrationDialog / ROMDialog used `disable_motor('all')`

**Root cause**: Both dialogs sent `disable:all` at init, broadcasting to the whole bus.
In Left Only / Right Only mode, this was harmless (inactive motors were already disabled)
but was inconsistent with the per-ID targeting model.

**Fix**: Replaced with per-ID disable loops using `_motor_idx.values()` (calibration)
and `_motor_dxl_lookup.values()` (ROM).

---

## Architecture truth (post-fix)

- **One board, one port, one HandExo** for all GUI modes.
- `DualHandExo` class exists but is NOT used by the GUI.
- `_motor_dxl_id` is the authoritative "active motor" list. All side-aware commands use it.
- Inactive-side protection is behavioral (motors disabled at connect), not structural.
- Calibration application is now fully ID-based; bare names are never sent in dual mode.
- Profile JSONs carry `"side"` metadata. `config.json` has `"default_left"` / `"default_right"`.

---

## Files changed in April 2026

| File | Change summary |
|------|---------------|
| `src/nml_hand_exo/applications/hand_exo_gui.py` | Inactive-side disable at connect; `_make_name_to_id()`; all `apply_calibration` call sites updated; dialog init per-ID disable; ROM orientation side fix |
| `src/nml_hand_exo/interface/_hand_exo.py` | `apply_calibration(name_to_id=...)`: ID-based commands, `abs_by_id` epoch correction |
| `docs/CLAUDE.md` | Dual-exo model, serial port fix (Serial2→Serial3, 9600→57600), new open tasks |
| `docs/dual_exo_architecture.md` | New — comprehensive dual-exo reference |
| `docs/gotchas.md` | Telemetry index type corrected; dual-mode gotchas added |
| `docs/serial_protocol.md` | HC-05 baud/port fixed; motor name disambiguation section added |
| `docs/calibration_flow.md` | Profile schema updated with `side`; config.json keys updated; dual-mode apply section |
| `docs/apply_and_gesture_state.md` | apply_calibration signature updated; dual-mode gesture paths; default profile side-awareness |
| `docs/gui_workflow.md` | Connection modes section; CalibrationDialog updated to streaming + per-ID |
| `docs/telemetry_architecture.md` | `_motor_idx` vs `_motor_dxl_id` clarified; angle dict key type corrected |
| `docs/project_memory_2026-04.md` | This file |
