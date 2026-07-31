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

`get_absolute_motor_angle('all')` and all `_get_motor_attribute('all')` calls return
`{mid: value}` keyed by **DXL hardware ID** (the `id:` field in the firmware response block),
**not** by motor name and **not** by 0-based loop index.

```python
angles = exo.get_motor_angle('all')
# angles == {11: 162.8, 12: 180.0, ...}  — keyed by actual DXL ID
val = angles.get(11)   # ✓  right wrist
val = angles.get("wrist")  # → None  always
val = angles.get(0)        # → None  unless a motor actually has DXL ID 0
```

Use `_motor_dxl_id[i]` (widget index → DXL ID) to look up angle values in the
Controls-tab polling loop.  `_motor_idx` in `HandExoGUI` maps name → enumerate index
and is only used for table row tracking, not for angle dict lookups.

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

## `GESTURE_RESULT` can share the next reply frame

`GESTURE_RESULT` is unsolicited telemetry and does not carry the `;` command
delimiter. `DualSerialComm` retains that line until the next delimited reply,
so a real frame can contain both an older outcome and the current solicited
reply, for example:

```text
GESTURE_RESULT: reached=1 stalled=0 short=0 starved=0
GESTURE_ANGLES: index=42,-3.75 ...
```

Reply consumers must forward or record the outcome **and still account for the
non-outcome line as the current command reply**. Treating the entire frame as
unsolicited stalls the UDP pending-ack queue and prevents later `NGA2` pose
acknowledgements from being sent.

---

## Wrist multi-turn range

Wrist (ID 1): `jointLimits = {-189, 2840}` in `config.h`. Multi-turn mechanism — intentional.
Any code that normalizes to 0–360 will produce wrong wrist angles.

The shortest-path guard in the firmware's goal-setting path is the same trap in
another form: it subtracts a turn from any goal more than 180° from the present
position, which is right for a wrap-around duplicate and wrong for real travel
on a joint whose window is wider than half a turn. Since 0.6.0 the correction is
only applied when the result stays inside the calibrated window
(`NMLHandExo::applyShortestPath`), so a legitimately distant wrist goal survives
it. Do not reinstate an unconditional ±360 snap.

---

## A joint that acks every command and never moves (fixed in 0.6.0)

Before 0.6.0, gesture percentages were a fraction of the **window width**
(`limit_max - limit_min`) added to home, with the direction taken from the flip
flag. That silently assumed home sat on the extension endstop. Where it did not,
the flip side of home was a few degrees wide, every state's target landed past
the boundary, `setAbsoluteAngle()` clamped them all to the same angle, and the
joint held still while every command replied `OK:`.

The stock `config.h` wrist (home 310 in a `[166, 320]` window) and wrist2 (home
180 in `[42, 190]`) are shaped exactly that way: extend, rest and flex all
resolved to the same boundary, 10° from home. The five digits were unaffected
because calibration had put their homes on a window edge.

Travel is now measured on a per-motor axis from home to the flexion endstop
(`NMLHandExo::getGestureSpan`), so a fraction of `1.0` **is** the endstop and
nothing clamps. Three consequences worth knowing:

- `check_limits` prints the resolved `span` per motor and flags `NO_TRAVEL` /
  `SPAN_REVERSED`. That is the first thing to run when a joint will not move.
- A `FLEX_*` constant whose target used to clamp now stops where the fraction
  says, so a joint that used to slam into its endstop may travel visibly less.
  Raise the constant toward `1.0` to restore the old endpoint.
- `set_gesture_angle` percentages now interpolate a gesture's `extend` and
  `flex` postures rather than raw travel, so `0` and `100` ARE those states.
  Since `EXTEND_*` is non-zero, a hand parked at **home** sits below 0% and
  `get_gesture_angle` reports `101` for every joint. That is correct, not a
  fault — home is below the extension posture, not equal to it.

---

## Two motors on one structure: commanding one alone does nothing

`wrist` and `wrist2` are both mounted on the back of the arm and connect to the
dorsal aspect of the wrist, so they act on the same structure. Driving one while
the other holds position means the two fight, and the wrist does not move — with
every command still ACKing. The `wrist` gesture therefore names **both** motors,
and the old `rad` gesture (which drove `wrist2` by itself) was removed in 0.6.0
rather than retuned.

The same coupling exists around the thumb: `thumbadd` sits on the side of the
arm and links to the `thumbflex` motor body, which is itself linked to
`thumbrot`. Treat any of those as single independent axes with care.

---

## Signed gesture degrees use one reference motor (0.6.1)

`get_gesture_sang` and the signed-degree field in `get_gesture_angles` do not
average raw motor degrees. The firmware first computes the same aggregate
gesture position used by `get_gesture_angle`, then maps that position onto the
calibrated span of the **first motor named by the gesture**. Rest is 0 degrees,
toward flex is positive, and toward extend is negative regardless of encoder
direction or `flip`.

This is deliberate for multi-motor gestures, whose motors can have different
calibrated spans:

- `thumb` uses `thumbadd` as its degree reference.
- `wrist` uses `wrist`, not `wrist2`.
- A dual build uses the left-side instance of that motor because IDs 1-9 appear
  before IDs 11-19 in firmware order.

Consequently, a dual-build percentage may reflect both hands while its degree
scale reflects the left reference motor. Do not interpret the value as a
side-specific right-hand angle when the two sides have different calibration
spans. Use a side-specific firmware build or explicitly account for this
reference convention in downstream analysis.

`nan` in a signed reply means the reference degree scale is unavailable. In a
combined reply, the percentage/status field may still be valid if other motors
in the gesture were readable. Code `255,nan` means neither view is available.

---

## Calibration does not update config.h (GUI)

The GUI's `CalibrationDialog` saves a profile JSON and calls `exo.apply_calibration()`,
but does **not** call `update_config_h()`. Firmware defaults in `config.h` persist until
the CLI is used. Applied calibration is lost on device reboot.

---

## Dual-mode: bare motor names always resolve to the left side

In dual firmware (`BUILD_LEFT_HAND 2`), `MOTOR_NAMES[]` has "wrist" at index 0 (ID 1,
left) AND index 9 (ID 11, right). `getMotorIDByName()` returns the **first match** — always
left.

Any firmware command that takes a motor name in dual mode silently targets the left motor:

```
set_zero_offset:wrist:X  →  ID 1 (left wrist) — RIGHT IS NEVER UPDATED
```

**Always use explicit DXL IDs for calibration and limit commands in dual firmware.**
`HandExo.apply_calibration(name_to_id=...)` enforces this. Do not call it without
`name_to_id` when in dual mode. Use `GUI._make_name_to_id(side)` to build the map.

---

## Dual-mode: `set_gesture` is a firmware broadcast

`set_gesture:grasp:close` arrives at `executeGesture()` in firmware, which iterates ALL
`N_MOTORS` and calls `setAbsoluteAngle()` on each one — regardless of which side the GUI
selected, and regardless of whether the motor has torque enabled.

`setAbsoluteAngle()` writes the Dynamixel goal position register unconditionally. If
the motor is torque-off, it won't move yet, but the goal is latched. Re-enabling torque
later will cause an unexpected jump.

Protection strategy (implemented at connect time): inactive-side motors are disabled
immediately when `_connect()` finishes, so they cannot physically move even when their
goal is written. Do not re-enable inactive motors manually while in single-side mode.

---

## Dual-mode: CalibrationDialog `disable_motor` scope

`CalibrationDialog` and `ROMDialog` disable only the motors in their target-side ID list
at initialization (per-ID loop), not `disable:all`. If you add code that uses
`disable_motor('all')` inside these dialogs, it will re-disable inactive-side motors —
harmless in isolation but unnecessary and slightly misleading.
