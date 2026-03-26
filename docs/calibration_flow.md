# Calibration Flow

> `[VERIFIED]` = confirmed from source. `[INFERRED]` = reasonable but unverified.

---

## Current implementation `[VERIFIED]`

Both `CalibrationDialog` (GUI) and `run_interactive_calibration()` (CLI) implement
calibration as a two-snapshot all-at-once pass:

1. Prompt user: "Move ALL fingers to FULLY OPEN"
2. Read `get_absolute_motor_angle('all')` → store as `open_angles`
3. Prompt user: "Move ALL fingers to FULLY CLOSED"
4. Read `get_absolute_motor_angle('all')` → store as `close_angles`
5. For each motor derive:
   - `home = open_angle`
   - `limit_min = min(open, close)`
   - `limit_max = max(open, close)`
   - `flip = (close_angle < open_angle)`
6. Write `profiles/<name>.json`

**Problem:** every motor's limits come from one shared global open and one shared
global closed position. Motors with different natural ranges, or joints that require
independent movement to reach true limits, cannot be accurately calibrated this way.

---

## Target: per-DoF guided calibration

Replace the two-step global pass with a per-motor loop:

```
for each motor in motor_names:
    prompt: "Fully EXTEND your <motor> — e.g. open INDEX finger at MCP joint"
    record open angle for this motor
    prompt: "Fully FLEX your <motor>"
    record close angle for this motor
    derive home, limits, flip for this motor
```

The profile schema, firmware commands, and file format do not change.

### Why no firmware changes are needed `[VERIFIED]`

The firmware already supports per-motor operations:

| Command | Effect |
|---------|--------|
| `get_absolute_angle:<name>` | Read single motor angle |
| `set_zero_offset:<name>:<val>` | Set home for one motor |
| `set_motor_limits:<name>:<min>:<max>` | Set limits for one motor |
| `set_flip:<name>:0\|1` | Set direction for one motor |

`HandExo.get_absolute_motor_angle(name)` also accepts individual motor names.
All per-DoF changes are Python/GUI only.

---

## Profile schema `[VERIFIED]`

File: `examples/calibration/profiles/<name>.json`

```json
{
  "motors": {
    "<motor_name>": {
      "home":      <float>,   // absolute encoder angle at open/rest (degrees)
      "flip":      <bool>,    // true = encoder decreases when closing
      "limit_min": <float>,   // lower absolute encoder bound (degrees)
      "limit_max": <float>    // upper absolute encoder bound (degrees)
    }
  }
}
```

`profiles/config.json` stores `{"default": "<name>"}` — the active profile name.

### Schema evolution rules

1. **Additive only.** Never rename or remove `home`, `flip`, `limit_min`, `limit_max`.
   New fields must have safe defaults so old profiles load without error.
2. **`flip` is always derived from measurement** — `flip = (close < open)`.
   Never pre-populate from `config.h` defaults; they may differ across hardware builds.
3. **Validate after writing.** Warn if `limit_min == limit_max` (failed recording)
   or if `home` is outside `[limit_min, limit_max]` (inconsistent snapshot).

---

## GUI vs CLI surface differences `[VERIFIED]`

| Behaviour | GUI `CalibrationDialog` | CLI `calibrate_exo.py` |
|-----------|------------------------|------------------------|
| Saves profile JSON | Yes | Yes |
| Calls `update_config_h()` | **No** | Yes |
| Applies profile to device over serial | Via `HandExo` API | Via raw `serial.Serial` |
| Sets default profile if first | Yes | Yes |

**The GUI does not update `config.h`** after saving. The CLI does. After a GUI
calibration session, the firmware default arrays (`HOME_STATES[]`, `jointLimits[][]`,
`DEFAULT_FLIPS[]`) are not updated until the CLI is run or `config.h` is edited manually.
Whether to add `update_config_h()` to the GUI is an open design decision. `[INFERRED:
likely intentional for now — auto-writing source files from a GUI is side-effectful]`

---

## Duplicated helpers `[VERIFIED]`

The following functions exist with identical or near-identical implementations in
both `hand_exo_gui.py` and the calibration scripts:

`list_profiles`, `load_profile`, `save_profile`, `get_default_profile_name`,
`set_default_profile`, `normalize_angle`, `determine_run_number`

Before adding new calibration logic, decide whether to extract a shared
`calibration_utils.py` module or keep them duplicated. Either way, any change
must be applied consistently to both surfaces.

---

## Calibration evolution rules

1. **Both surfaces stay in sync.** If calibration logic changes in `CalibrationDialog`,
   mirror it in `calibrate_exo.py` — or explicitly document the intentional divergence here.
2. **Prompts are per-DoF, not per-gesture.** Use anatomical terms
   (e.g. "Fully EXTEND your index finger at the MCP joint"). Gesture names (grasp,
   pinch) involve multiple joints and will produce inaccurate per-motor limits.
3. **Schema changes are additive only.** See schema evolution rules above.
4. **`flip` is always derived from measurement.** Never hard-code it.
5. **Validate after writing.** Log a warning for degenerate profiles (`min == max`
   or `home` outside bounds).

---

## CLI calibration commands

```bash
# Interactive calibration — saves profile and updates config.h
python examples/calibration/calibrate_exo.py --port COM<N> --name <profile>

# Apply a saved profile to the device (no interactive steps)
python examples/calibration/calibrate_exo.py --port COM<N> --apply <profile>

# Apply the default profile
python examples/calibration/calibrate_exo.py --port COM<N> --apply

# Set a profile as the default
python examples/calibration/calibrate_exo.py --set-default <profile>

# List saved profiles
python examples/calibration/calibrate_exo.py --list-profiles

# ROM assessment (two-phase, outputs CSV)
python examples/calibration/rom_assessment.py --port COM<N> --profile <name>
# Output: output_data/<participant>_rom_<date>_<run>.csv
```

---

## Per-DoF calibration task checklist

- [ ] Audit `CalibrationDialog._record()` — understand exactly what must change
      for per-motor iteration vs. the current all-at-once snapshot
- [ ] Decide: extract shared helpers into `calibration_utils.py` first, or add inline?
- [ ] Design per-DoF prompt sequence: motor order, anatomical descriptions,
      whether wrist is included or handled separately (wide multi-turn range)
- [ ] Implement per-motor loop in `CalibrationDialog` (GUI, Python only)
- [ ] Mirror change in `calibrate_exo.py` CLI, or explicitly document divergence
- [ ] Add validation: warn if `limit_min == limit_max` or `home` outside `[min, max]`
- [ ] Decide: should GUI call `update_config_h()` after saving? (CLI does; GUI does not)
- [ ] Live device test: apply profile over serial, confirm per-motor limits received
- [ ] Update this checklist and the root CLAUDE.md checklist when complete
