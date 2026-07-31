# Apply, Default Profile, and Gesture State

> `[VERIFIED]` = confirmed from source.

File: `src/nml_hand_exo/applications/hand_exo_gui.py`
API: `src/nml_hand_exo/interface/_hand_exo.py`

---

## What "apply" does

`HandExo.apply_calibration(profile_name, name_to_id=...)` reads a profile JSON and
pushes three values per motor to the device over serial.

When `name_to_id` is provided (always the case when called from the GUI), the command
uses the explicit DXL integer ID so the correct motor is targeted in dual firmware:

```python
for name, vals in cal["motors"].items():
    dxl_id    = name_to_id.get(name)         # e.g. 11 for right "wrist"
    motor_ref = dxl_id if dxl_id else name   # int preferred over bare string
    self.set_zero_offset(motor_ref, adj_home)    # set_zero_offset:11:<val>
    self.set_motor_limits(motor_ref, lo, hi)     # set_motor_limits:11:<lo>:<hi>
    self.set_flip(motor_ref, flip)               # set_flip:11:0|1
```

Without `name_to_id` (e.g. direct CLI or script use), bare names are sent — safe in
single-exo firmware, but dangerous in dual firmware where names are ambiguous.

An epoch-correction step runs first to align multi-turn profile values to the motor's
current power-on position. It queries `get_absolute_angle:all` and looks up each motor
by DXL ID in the response (keyed by hardware ID, not name) to avoid collisions.

This updates the device's runtime state only. It does **not** write `config.h`.
On device reboot, firmware defaults from `config.h` (`HOME_STATES[]`, `jointLimits[][]`,
`DEFAULT_FLIPS[]`) are restored. Applied calibration is lost on power cycle.

---

## Default profile

`profiles/config.json` stores side-specific default keys:

```json
{
  "default_left":  "alice left",
  "default_right": "alice right",
  "default":       "alice right"
}
```

`"default"` is kept as a right-hand backward-compatibility key. The GUI and CLI prefer `"default_left"` / `"default_right"`; left-hand lookup never falls back to the legacy right-hand key.

- The GUI's "Apply Profile" dropdown reads the default marker to show `(default)`.
- `CalibrationDialog` always sets the saved profile as default (via `set_default_profile(name, side=self._side)`) on complete.
- `_ensure_gesture_ready()` applies the side-correct default profile before the first gesture.
- `ROMDialog._detect_orientation()` loads `get_default_profile_name(side=self._side)` to determine motor flip directions.

If no default is set for a side: gestures run without calibration (likely wrong angles),
and ROM orientation falls back to `flip=False` for motors not found in any profile.

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
When False, `_ensure_gesture_ready(target)` runs once:

**Dual mode:**
1. Applies left default profile if target is "Both" or "Left Only" (with `name_to_id` for left IDs)
2. Applies right default profile if target is "Both" or "Right Only" (with `name_to_id` for right IDs)
3. Enables target-side motors (respects `user_disabled` invariant)
4. Sets gesture mode (`set_exo_mode:gesture_fixed` — broadcast, benign)
5. Sets `_gesture_ready = True`

**Single-side mode (Left Only / Right Only):**
1. Applies the active side's default profile (with `name_to_id` for the active-side IDs)
2. Enables all active motors (respects `user_disabled` invariant)
3. Sets gesture mode
4. Sets `_gesture_ready = True`

In both modes, `_apply_gesture_target_motors(target)` runs on **every** gesture press
in Dual mode (not just the first) so changing the target combo takes immediate effect.
In single-side modes, containment relies on the inactive-side motors being disabled at
connect time.

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
