# Apply, Default Profile, and Gesture State

> `[VERIFIED]` = confirmed from source.

File: `src/nml_hand_exo/applications/hand_exo_gui.py`
API: `src/nml_hand_exo/interface/_hand_exo.py`

---

## What "apply" does

`HandExo.apply_calibration(profile_name)` reads a profile JSON and pushes three values
per motor to the device over serial:

```python
for name, vals in cal["motors"].items():
    self.set_zero_offset(name, vals["home"])       # set_zero_offset:<name>:<val>
    self.set_motor_limits(name, vals["limit_min"], vals["limit_max"])  # set_motor_limits:...
    self.set_flip(name, vals["flip"])              # set_flip:<name>:0|1
```

This updates the device's runtime state only. It does **not** write `config.h`.
On device reboot, firmware defaults from `config.h` (`HOME_STATES[]`, `jointLimits[][]`,
`DEFAULT_FLIPS[]`) are restored. Applied calibration is lost on power cycle.

---

## Default profile

`profiles/config.json` stores `{"default": "<name>"}`.

- The GUI's "Apply Profile" dropdown reads the default marker to show `(default)`.
- `CalibrationDialog` always sets the saved profile as default on complete.
- `_ensure_gesture_ready()` applies the default profile automatically before the first gesture.
- `ROMDialog._detect_orientation()` loads the default profile to determine motor flip directions.

If no default is set: gestures will run without calibration (likely wrong angles),
and ROM orientation falls back to `flip=False` for all motors.

---

## Manual apply (GUI)

Two paths:

**1. Calibration section — "Apply" button**
```
profile_combo → _apply_profile()
  → exo.apply_calibration(name)
```
User explicitly picks a profile and clicks Apply. Safe to run at any time.

**2. Post-calibration prompt**
After `CalibrationDialog` completes successfully, the main window asks:
"Apply calibration profile '<name>' to the device now?"
If Yes → `exo.apply_calibration(name)`.

**3. Post-ROM prompt**
After `ROMDialog` completes, if a ROM-derived profile was saved, same question is asked.

---

## `_gesture_ready` flag

```python
self._gesture_ready: bool  # initialized False; set True by _ensure_gesture_ready()
```

`_gesture_ready` gates lazy initialization before the first gesture command.
When False, `_ensure_gesture_ready()` runs once:
1. Loads and applies the default calibration profile (`exo.apply_calibration(default_profile)`)
2. Enables all motors (`exo.enable_motor('all')`)
3. Sets gesture mode (`set_exo_mode:gesture_fixed`)
4. Sets `_gesture_ready = True`

`_gesture_ready` is reset to False on:
- `_disconnect()` — device gone
- `_run_calibration()` — CalibrationDialog disables motors; re-initialization needed

**The flag does not reset when a profile is manually applied via the Apply button.**
If you apply a new profile and then try a gesture, `_ensure_gesture_ready()` will not re-run
because `_gesture_ready` is still True. The previously applied profile is already live on
the device, so this is usually correct. However, if motors were disabled after the profile
was applied (e.g., via "Disable All"), gestures will silently fail to move.

---

## Apply vs config.h

| Method | Updates device runtime | Updates config.h | Survives reboot |
|--------|----------------------|-----------------|-----------------|
| GUI "Apply" button | Yes | **No** | **No** |
| Post-calibration prompt | Yes | **No** | **No** |
| CLI `calibrate_exo.py --apply` | Yes | **No** | **No** |
| CLI `calibrate_exo.py --name` (interactive) | Yes | **Yes** | Yes |

To make calibration survive a reboot, run the CLI interactive calibration or manually
run `calibrate_exo.py --apply <name>` followed by re-flashing (or update `config.h`
manually and reflash).

---

## Traps

- **Gestures after CalibrationDialog**: `_gesture_ready` is reset to False after calibration.
  The next gesture click will call `_ensure_gesture_ready()`, which applies the default profile
  (now updated) and re-enables motors. This is the correct behavior.

- **No default profile set**: If no profile exists, `_ensure_gesture_ready()` logs a warning
  and continues. Motors are enabled but no calibration is applied — gesture angles may be wrong.

- **Motors disabled after apply**: `_gesture_ready` stays True, but motors are off.
  Gestures send commands but nothing moves. Enable motors via "Enable All" or trigger a
  new gesture sequence (which re-runs `_ensure_gesture_ready()` if `_gesture_ready` was reset).

- **ROM orientation without a profile**: `ROMDialog._detect_orientation()` falls back to
  `flip=False` for any motor not in the profile. Normalized ROM angles will be wrong for
  motors where flip should be True (e.g., middle, ring, pinky on typical hardware).
