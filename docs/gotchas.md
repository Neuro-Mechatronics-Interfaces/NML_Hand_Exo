# Gotchas & Known Traps

Hard-won knowledge. Read before touching firmware, the serial protocol, or the GUI.

---

## Firmware response: unit suffixes break float parsing

`_parse_motor_data_block()` in `_hand_exo.py` calls `float(val)` on raw string values.
Several firmware responses include unit suffixes that break this:

| Command | Emitted value | Suffix | Status |
|---------|--------------|--------|--------|
| `get_current:all` | `0.000 mA` | ` mA` | Fixed (2026-03) |
| `get_current_limit:all` | `0.000mA` | `mA` | Fixed (2026-03) |
| `get_torque` single-motor | `0.0000 N·m` | ` N·m` | **Not fixed** — `float()` raises |

Fix pattern applied to `current` and `current_limit`:
```python
_m = re.match(r'[-+]?[\d.]+', val.strip())
motor_info["current"] = float(_m.group()) if _m else float(val)
```

If you see a silent `—` in a telemetry column or a `ValueError` from `_parse_motor_data_block`,
check whether the firmware response includes a unit suffix on that field.

---

## Firmware torque all-mode: three bugs (source fixed, reflash still required)

`get_torque:all` had three simultaneous bugs:
1. Key was `Torque:` (capital T) — Python parser matches only lowercase `torque`
2. Used `String(val)` — `val` is the outer `int val = 0` at `parseMessage()` scope,
   not the local `float torque` computed the line above
3. No `\n` after each motor entry — all motors concatenated on one line

Fix (`utils.cpp:552`):
```cpp
// before
", Torque: " + String(val) + "}"
// after
", torque: " + String(torque, 4) + "}\n"
```

**Source is patched. The device is not yet running the fix. Reflash required.**
Until reflash, the Torque column in the Telemetry tab shows `—`.

---

## `val` variable scope in utils.cpp

`int val = 0;` is declared at `parseMessage()` function scope and reused across all
command branches. Its value at any point depends on which branch ran previously.
This caused the torque bug above. When adding new firmware branches, always use a
locally scoped variable — never rely on the outer `val`.

---

## `_gesture_ready` does not reset on manual profile apply

`_gesture_ready` is the flag that gates lazy motor initialization before the first gesture.
It resets to False on disconnect and after calibration (because CalibrationDialog disables motors).
It does **not** reset when a profile is manually applied via the "Apply" button.

This is usually correct — motors stay enabled and the new profile is live on the device.
But if motors were disabled after the apply (e.g., "Disable All"), gestures will silently
fail to move because `_ensure_gesture_ready()` won't re-run.

See [docs/apply_and_gesture_state.md](apply_and_gesture_state.md) for full behavior.

---

## Qt stylesheet: specificity traps

`QWidget { color: ... }` does NOT cascade into:
- `QTableWidget::item` — requires its own rule
- `QHeaderView::section` — requires its own rule
- `QTabBar::tab` — requires its own rule
- `QCheckBox` — requires its own rule

Missing rules → text appears invisible on dark background. All four now exist in
`DARK_STYLE` in `hand_exo_gui.py`. If a new widget appears unreadable, add an
explicit rule — do not rely on `QWidget` color inheritance.

---

## Controls tab: layout-redirect pattern

`_build_ui()` temporarily redirects `self.main_layout` to the Controls tab container
so all existing `_build_*_section()` methods add to it unchanged:

```python
_saved_layout = self.main_layout
self.main_layout = controls_layout     # all _build_*_section() write here
self._build_motor_section()
# ...
self.main_layout = _saved_layout       # restore
```

This is synchronous, construction-time only, and safe. Do not break this pattern or add
anything between the redirect and restore that depends on `self.main_layout`.

---

## Telemetry index type

`get_absolute_motor_angle('all')` returns `{0: val, 1: val, ...}` keyed by **integer index**
(position in `MOTOR_IDS[]`), not by motor name or hardware ID.
`_motor_idx[name]` gives the correct integer key. `positions.get(name)` always returns `None`.

---

## CalibrationDialog and ROMDialog own their own timers

Both dialogs create a `QTimer(self)` at 100 ms for angle polling.
These are separate from `HandExoGUI._angle_timer` and `HandExoGUI._telem_timer`.
The main window timers keep running while a dialog is open.
Do not share timers across dialog and main window.

---

## `command_delimiter` vs `COMMAND_DELIMITER`

The GUI connects with `command_delimiter='\r\n'` — what the GUI *sends* to terminate commands.
`config.h:COMMAND_DELIMITER = ";"` — what the firmware appends to *terminate responses*.
These are different. `SerialComm.receive()` reads until `;`. The sent delimiter is separate.

---

## Wrist multi-turn range

Wrist (ID 1): `jointLimits = {-189, 2840}` in `config.h`. Multi-turn mechanism — intentional.
Any code that normalizes to 0–360 will produce wrong wrist angles.

---

## Calibration does not update config.h (GUI)

The GUI's `CalibrationDialog` saves a profile JSON and calls `exo.apply_calibration()`,
but does **not** call `update_config_h()`. Firmware defaults in `config.h` persist until
the CLI is used. Applied calibration is lost on device reboot.
