# Calibration Flow

> `[VERIFIED]` = confirmed from source.

---

## Current implementation — two global streaming phases `[VERIFIED]`

`CalibrationDialog` in `hand_exo_gui.py` is a modal `QDialog` launched from the main window.
It uses a streaming approach: two global recording windows, not per-motor sequential steps.
During each window, the operator manually moves every joint through its range.
The dialog records all motors simultaneously on every 100 ms tick.

**Phase 0 — Extension/Open**
- Operator clicks "Start Recording Extension"
- `QTimer` at 100 ms calls `get_absolute_motor_angle('all')` per tick
- All motor angles accumulate in `_samples_buf[name]` simultaneously
- Operator moves every joint to its open/extended extreme during the window
- "Stop Recording" commits samples to `_open_samples` if ≥ 3 samples per motor

**Phase 1 — Flexion/Close**
- Same mechanism; operator moves every joint to its closed/flexed extreme
- Commits to `_close_samples`; `_save_profile()` called immediately

**Profile derivation from sample lists:**
```python
home      = median(open_samples)
flip      = median(close_samples) < median(open_samples)
limit_min = min(open_samples + close_samples)
limit_max = max(open_samples + close_samples)
```

**Validation (`_validate_profile()`):**
- `limit_min == limit_max` → RuntimeError, profile not saved (motor did not move)
- `home` outside `[limit_min, limit_max]` → RuntimeError, profile not saved
- Observed range < 2° → QMessageBox warning, save continues

---

## Profile schema `[VERIFIED]`

File: `examples/calibration/profiles/<name>.json`

```json
{
  "side": "left",
  "motors": {
    "<name>": {
      "home":      float,
      "flip":      bool,
      "limit_min": float,
      "limit_max": float
    }
  }
}
```

`"side"` is always written by `save_profile()` as of 2026-04. Legacy profiles without it
are treated as `"right"` in profile filtering.

`profiles/config.json` stores side-specific defaults:

```json
{
  "default_left":  "alice left",
  "default_right": "alice right",
  "default":       "alice right"
}
```

`"default"` is kept for backward compat with the CLI. `"default_left"` / `"default_right"`
are used by the GUI and `HandExo.apply_calibration()` when called with `side=...`.

**Schema rules — additive only.** Never remove `home`, `flip`, `limit_min`, `limit_max`,
`side`. New fields need safe defaults so old profiles load without error.
`flip` is always derived from measurement — never hard-coded.

---

## GUI vs CLI `[VERIFIED]`

| Behaviour | GUI `CalibrationDialog` | CLI `calibrate_exo.py` |
|-----------|------------------------|------------------------|
| Saves profile JSON | Yes | Yes |
| Updates `config.h` | **No** | Yes |
| Recording approach | **Streaming (global window)** | Snapshot (single read) |
| Profile validation | **Yes** | No |
| Sets default profile | Yes (always) | Yes (if first) |

After GUI calibration: `config.h` firmware defaults are **not updated** until CLI is run.
The CLI is out of sync with the GUI — it still uses the old single-snapshot approach.

---

## Shared calibration utilities `[VERIFIED]`

Reusable calibration logic is centralized under `src/nml_hand_exo/calibration/`:

- `profiles.py` owns profile paths, listing, loading, saving, and side-specific defaults through `CalibrationProfileStore`.
- `rom.py` owns angle normalization, orientation defaults, and ROM output run numbering.
- Both the GUI and calibration CLI import these helpers; `rom_assessment.py` imports the ROM helpers as well.

The legacy `default` config key is a right-hand compatibility key only. Left-hand lookup requires `default_left`, preventing a right profile from being applied to the left side.

---

## ROM-derived calibration profile

`ROMDialog._finish()` offers to derive a calibration profile from the assisted ROM data
(phases 2 and 3). It uses the same median/min/max arithmetic as `CalibrationDialog`.
Profile is saved via `save_profile()` and optionally applied via `exo.apply_calibration()`.

---

## Side-specific calibration in dual mode `[VERIFIED]`

### Collection — side-correct from the start

`_run_calibration()` in the GUI passes only the target-side motor names and DXL IDs
to `CalibrationDialog`:

- Dual mode: reads `cal_side_combo`, filters `side_motor_names` and `side_dxl_ids`
  to that side only (left: IDs 1–9, right: IDs 11–19).
- Left Only / Right Only: `motor_names` and `_motor_dxl_id` are already filtered at
  connect time; passed directly.

`CalibrationDialog._poll_angles()` uses `self._motor_idx` (name → DXL hardware ID map
built from the `dxl_ids` argument) to look up angle values. Only target-side motors
are sampled.

Profile values (home, flip, limit_min, limit_max) are computed independently per motor
from that motor's own sample lists. There is no cross-motor averaging.

### Application — ID-based, side-safe

**Before April 2026**, `apply_calibration()` sent `set_zero_offset:wrist:X` etc., which
in dual firmware always resolved to the left motor via `getMotorIDByName()`.

**After April 2026**, every `apply_calibration()` call in the GUI passes:

```python
name_to_id = self._make_name_to_id(side)   # {"wrist": 11, "index": 16, ...}
self.exo.apply_calibration(name, name_to_id=name_to_id)
```

Inside `apply_calibration()`, each motor is commanded by integer DXL ID:

```python
motor_ref = name_to_id.get(name)   # e.g. 11 for "wrist" on the right side
self.set_zero_offset(motor_ref, adj_home)   # → set_zero_offset:11:X
```

Epoch correction (multi-turn alignment) also uses `abs_by_id` keyed by DXL ID to
avoid the name-collision problem where two "wrist" entries would overwrite each other
in a name-keyed dict.

---

## Open tasks

- [ ] Mirror streaming calibration into `calibrate_exo.py` CLI (or document divergence explicitly)
- [ ] Live device test: apply profile, confirm per-motor limits received by firmware
- [ ] Decide: GUI call `update_config_h()` after save? (CLI does; GUI does not)
