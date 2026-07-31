# Dual-Exo Architecture

> `[VERIFIED]` = confirmed from source code.

This document describes how left/right exo control is implemented — hardware layout,
ID assignment, mode selection, command routing, gesture containment, and calibration
application. Read this before touching any code that references left/right side selection.

---

## Hardware model `[VERIFIED]`

**One OpenRB-150 controller. One Dynamixel bus. One physical USB cable.**

Both hands (if present) share the same board and USB connection. There is no separate
left-hand board. The default OpenRB build exposes two logical USB CDC ports on that
single cable so commands and replies/telemetry do not block one another. The GUI still
creates one `HandExo` instance regardless of mode; it uses `DualSerialComm` when the
dual-CDC option is selected and `SerialComm` for the legacy one-port path.

`DualHandExo` (`src/nml_hand_exo/interface/_dual_hand_exo.py`) exists as a class but is
**not used by the GUI**. It was designed for a two-board architecture that was not
adopted. Do not instantiate it for the current single-board setup.

```
PC (Python)
  └── HandExo — one instance
        └── DualSerialComm → command CDC + telemetry CDC → OpenRB-150
            (or SerialComm → one CDC in compatibility mode)
              └── Dynamixel bus (1000000 baud)
                    ├── Left motors:  IDs  1,  2,  3,  4,  5,  6,  7,  8,  9
                    └── Right motors: IDs 11, 12, 13, 14, 15, 16, 17, 18, 19
```

---

## Motor ID assignment `[VERIFIED]`

Defined in `config.h` based on `BUILD_LEFT_HAND` flag:

| ID | Left name   | ID | Right name  |
|----|-------------|----|----|
|  1 | wrist       | 11 | wrist       |
|  2 | wrist2      | 12 | wrist2      |
|  3 | thumbadd    | 13 | thumbadd    |
|  4 | thumbrot    | 14 | thumbrot    |
|  5 | thumbflex   | 15 | thumbflex   |
|  6 | index       | 16 | index       |
|  7 | middle      | 17 | middle      |
|  8 | ring        | 18 | ring        |
|  9 | pinky       | 19 | pinky       |

Current build: `BUILD_LEFT_HAND 2` (dual mode) — all 18 motors active.

In dual firmware, `MOTOR_NAMES[]` contains "wrist" at index 0 (ID 1, left) AND
"wrist" at index 9 (ID 11, right). The first 9 entries are left; the second 9 are right.

---

## Firmware name resolution in dual mode `[VERIFIED]`

`NMLHandExo::getMotorIDByName()` (`nml_hand_exo.cpp:165`) does a **linear scan and
returns the first match**. In dual firmware, bare name commands always resolve to the
left motor:

```
set_zero_offset:wrist:X  →  firmware resolves "wrist" → ID 1 (left)
set_motor_limits:wrist:X:Y  →  ID 1 (left only)
```

**This is the root cause of the calibration side-mixing bug (fixed April 2026).**

### Safe pattern — use integer IDs

Integer IDs bypass `getMotorIDByName()` entirely. `getMotorID()` parses the token as an
integer first (`target.toInt()`); if non-zero, that value is used directly:

```
set_zero_offset:11:X  →  ID 11 (right wrist)  ✓
set_zero_offset:1:X   →  ID 1  (left wrist)   ✓
```

Always use integer IDs for any motor command where the target side matters.

---

## GUI mode selection `[VERIFIED]`

Mode dropdown (in connection panel):

| Mode | Active IDs | `motor_names` prefix | Inactive IDs |
|------|------------|---------------------|--------------|
| Right Only | 11–19 | bare ("wrist", ...) | 1–9 |
| Left Only  | 1–9   | bare ("wrist", ...) | 11–19 |
| Dual       | 1–19  | "L:wrist", "R:wrist" | none |

Mode is locked while connected (`mode_combo.setEnabled(False)` in `_connect()`).

At connect time, `_connect()` builds:

```python
self._motor_dxl_id      # list[int] — DXL IDs for active motors only (widget order)
self._left_motor_names  # list[str] — bare names for left motors detected
self._right_motor_names # list[str] — bare names for right motors detected
self.motor_names        # list[str] — display names (with L:/R: prefix in Dual)
self.motor_widgets      # list[dict] — one entry per active motor
```

**`_motor_dxl_id` is the source of truth for "which motors are active."**
All commands that must respect the mode selection use this list.

---

## Inactive-side containment at connect time `[VERIFIED]`

`set_gesture` and similar firmware broadcasts act on ALL firmware-managed motors
regardless of which side the GUI selected. To prevent the inactive side from moving,
`_connect()` sends `disable:<id>` for every detected motor not in `_motor_dxl_id`:

```python
inactive_ids = sorted(all_detected_ids - set(self._motor_dxl_id))
for _inactive_id in inactive_ids:
    self.exo.disable_motor(_inactive_id)
```

In Left Only mode: IDs 11–19 are disabled at connect.
In Right Only mode: IDs 1–9 are disabled at connect.
In Dual mode: `inactive_ids` is empty; no extra disables.

While inactive motors are disabled, `set_gesture` can still write goal positions to their
Dynamixel registers, but they will not physically move. The GUI never exposes these
motors in `motor_widgets`, so the user cannot re-enable them accidentally.

---

## Command routing — what uses `_motor_dxl_id` `[VERIFIED]`

| Command | Uses `_motor_dxl_id` | Side-safe? |
|---------|---------------------|------------|
| Enable All (`_motor_all`) | Yes — loops over DXL IDs | ✓ |
| Disable All (`_motor_all`) | Yes — loops over DXL IDs | ✓ |
| Home All (`_home_all`) | Yes — loops over DXL IDs | ✓ |
| Per-motor toggle | Yes — widget's `dxl_id` | ✓ |
| Per-side enable/disable (`_motor_side`) | Yes — widget names prefixed L:/R: | ✓ |
| `set_gesture` (open/close buttons) | No — firmware broadcast | protected by inactive-side disable |
| `set_exo_mode:gesture_fixed` | No — firmware broadcast | benign (mode command, no motion) |

---

## Gesture target selection in Dual mode `[VERIFIED]`

In Dual mode, a "Gesture Target" combo box appears with options: Both / Left Only / Right Only.

On every gesture button press, `_apply_gesture_target_motors(target)` runs and:
- Enables all non-`user_disabled` motors on the target side
- Disables all non-`user_disabled` motors on the non-target side

This works because in Dual mode, all motors (both sides) are in `motor_widgets` and
`_motor_dxl_id`, so the GUI can track and control both sides.

In Left Only / Right Only modes, the inactive side is not in `motor_widgets` at all;
containment relies entirely on the connect-time disable described above.

### Signed gesture-angle reference in dual firmware

Firmware 0.6.1 `get_gesture_sang` / `get_gesture_angles` replies follow a
first-motor convention. The percentage is the aggregate of every readable
motor carrying the gesture, including both sides, but the rest-zeroed degree
scale comes from the first matching motor. In the current dual array that is
always the left-side instance because IDs 1-9 precede IDs 11-19.

For example, `wrist` aggregates the left/right `wrist` and `wrist2` motors, then
expresses the result in degrees using left wrist ID 1. The value is not a
right-side-specific physical angle when the two sides have different
calibration spans. This convention is deterministic, but downstream analysis
must account for it or use a side-specific firmware build.

---

## Calibration — side-specific apply `[VERIFIED]`

### Profile schema

```json
{
  "side": "left",
  "motors": {
    "wrist": {"home": 180.0, "flip": false, "limit_min": 160.0, "limit_max": 200.0},
    "index": {...}
  }
}
```

`"side"` is always written by `save_profile()`. Legacy profiles without it default to
`"right"` in the UI filter.

### `config.json` — side-specific defaults

```json
{
  "default_left":  "zach left new",
  "default_right": "zach right",
  "default":       "zach right"
}
```

`"default"` is kept for backward compat with the CLI. `"default_left"` / `"default_right"`
are used by the GUI and `HandExo.apply_calibration(side=...)`.

### `HandExo.apply_calibration(name_to_id=...)` `[VERIFIED]`

All calibration commands use explicit DXL IDs when `name_to_id` is supplied:

```python
self.set_zero_offset(motor_ref, adj_home)        # motor_ref = DXL ID int
self.set_motor_limits(motor_ref, lo, hi)
self.set_flip(motor_ref, flip)
```

Epoch correction (multi-turn alignment) also uses the ID-keyed dict:

```python
abs_by_id = {motor_id: info["absolute_angle"] for motor_id, info in _parsed.items()}
current_abs = abs_by_id.get(dxl_id)             # no name collision possible
```

### `_make_name_to_id(side)` in the GUI `[VERIFIED]`

Every `apply_calibration` call site in the GUI calls this helper first:

```python
# Dual mode, left side:
{"wrist": 1, "index": 6, ...}    # left IDs

# Dual mode, right side:
{"wrist": 11, "index": 16, ...}  # right IDs

# Left Only mode:
{"wrist": 1, "index": 6, ...}    # _motor_dxl_id is already filtered

# Right Only mode:
{"wrist": 11, "index": 16, ...}
```

This is built by zipping `_left_motor_names` / `_right_motor_names` with the
corresponding filtered subset of `_motor_dxl_id`.

---

## Profile visibility filtering `[VERIFIED]`

`_refresh_profiles()` in the GUI filters the profile dropdown by side:

| GUI mode | Shows profiles where |
|----------|---------------------|
| Left Only | `profile["side"] == "left"` |
| Right Only | `profile["side"] == "right"` or no side field |
| Dual | matches `cal_side_combo` selection |

Profiles without a `"side"` field appear in Right Only and Dual/Right views only
(legacy compatibility — treat as right-hand profiles).

---

## Calibration dialog — motor targeting `[VERIFIED]`

`CalibrationDialog` and `ROMDialog` receive already-filtered motor name and ID lists:

- Dual mode: only the `cal_side_combo`-selected side's names and IDs are passed
- Left Only / Right Only: all active motors (already filtered at connect)

Both dialogs:
- Disable only their target motors (per ID) at initialization — not `disable:all`
- Sample angles using `_motor_idx` / `_motor_dxl_lookup` keyed by DXL hardware ID
- Save profiles tagged with the correct `side` field

---

## Things that still require care

1. **Reflash after changing `BUILD_LEFT_HAND`** — the firmware array sizes change.
   Python motor counts come from the `info` response, not from `config.h` directly.

2. **Calibration does not survive device reboot** — `apply_calibration` writes to RAM
   only. `config.h` defaults are restored on power cycle. Run the CLI to update
   `config.h` if persistent calibration is needed.

3. **`set_gesture` target in single-side modes** — the gesture command always goes to
   firmware and always iterates all `N_MOTORS`. Inactive-side protection is
   entirely behavioral (motors disabled), not structural (firmware has no filter).
   If inactive-side motors are somehow re-enabled while in Left Only / Right Only mode,
   they will respond to gesture commands.

4. **`DualHandExo` class is unused** — do not wire it to the GUI without a clear
   requirement. The single-HandExo model is simpler and already handles both sides.
