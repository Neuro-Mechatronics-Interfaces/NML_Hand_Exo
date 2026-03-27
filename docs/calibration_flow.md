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

`profiles/config.json` stores `{"default": "<name>"}`.

**Schema rules — additive only.** Never remove `home`, `flip`, `limit_min`, `limit_max`.
New fields need safe defaults so old profiles load without error.
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

## Duplicated helpers `[VERIFIED]`

Both `hand_exo_gui.py` and the calibration scripts contain:
`list_profiles`, `load_profile`, `save_profile`, `get_default_profile_name`,
`set_default_profile`, `normalize_angle`, `determine_run_number`

Any change must be applied to both, or extracted into a shared `calibration_utils.py`.

---

## ROM-derived calibration profile

`ROMDialog._finish()` offers to derive a calibration profile from the assisted ROM data
(phases 2 and 3). It uses the same median/min/max arithmetic as `CalibrationDialog`.
Profile is saved via `save_profile()` and optionally applied via `exo.apply_calibration()`.

---

## Open tasks

- [ ] Mirror streaming calibration into `calibrate_exo.py` CLI (or document divergence explicitly)
- [ ] Live device test: apply profile, confirm per-motor limits received by firmware
- [ ] Decide: GUI call `update_config_h()` after save? (CLI does; GUI does not)
- [ ] Extract shared helpers into `calibration_utils.py` (reduces duplication risk)
