# 09 — Automatic impedance ROM calibration (by gesture)

Host-side driver for the firmware `calibrate_rom_gesture` endpoint (firmware
**0.8.0+**). Instead of asking the wearer to move to their extremes, the
firmware finds each joint's endstop itself: it ramps current slowly under
current-mode control, watches the joint angle, and marks the angle where the
joint just resists a gentle push. Every sweep returns the joint to where it
started.

`auto_cal_rom.py` calibrates the six functional **gesture axes** —
**wrist, thumb, index, middle, ring, pinky** — the same axes the
`set_finger_angles` batch positioner drives. For each gesture the firmware
decomposes the axis into the motors it actually moves (`thumb` spans
thumbadd/thumbrot/thumbflex, `wrist` spans wrist/wrist2, the fingers are one
motor each) and sweeps every one, in flex and extend. The script prints the
discovered endstops and can optionally write them into the device's joint
limits.

Doing the decomposition in firmware (via `calibrate_rom_gesture`) keeps the
gesture→motor map a single source of truth — the same one `set_finger_angles`
uses — rather than duplicating it in this script.

## Prerequisites

1. **Flash firmware 0.8.0 or newer.** The default build variant is fine — see
   [`src/cpp/nml_hand_exo/README.md`](../../src/cpp/nml_hand_exo/README.md).

   ```
   arduino-cli compile --upload -p %COM% --fqbn OpenRB-150:samd:OpenRB-150 src/cpp/nml_hand_exo
   ```

   **No extra build flags are needed for autocal.** `calibrate_rom` is compiled
   into every variant; nothing gates it at build time. The default firmware is
   **dual** (`BUILD_LEFT_HAND 2`, IDs 1-9 + 11-19), **dual-CDC** (two USB COM
   ports), Axon peripheral **off** — exactly what the command above produces. A
   dual build works even with only one side attached (unreachable IDs are
   skipped). Change `BUILD_LEFT_HAND` in `config.h` only to build for a single
   fixed side, not to enable autocal.

2. **Install the Python package** (from the repo root, in your environment):

   ```
   pip install -e .
   ```

The script refuses to run against firmware older than 0.8.0.

## Run

The default firmware presents **two** USB-CDC COM ports on one cable. Give the
script either one; it finds and pairs the sibling automatically.

```bash
# Report-only, all six gestures, both directions (dual-CDC default firmware)
python auto_cal_rom.py --port COM21

# Name both ports if auto-pairing fails
python auto_cal_rom.py --cmd-port COM21 --telem-port COM22

# Single-CDC firmware (built with -DSINGLE_CDC)
python auto_cal_rom.py --port COM21 --single-cdc

# Only some gestures, one direction
python auto_cal_rom.py --port COM21 --gestures index middle --directions flex

# Also WRITE the discovered endstops into the device joint limits
python auto_cal_rom.py --port COM21 --apply
```

The default gesture set is `thumb index middle ring pinky wrist`. `--apply`
writes limits at runtime only. Persist them through your normal calibration
profile if they should survive a power cycle. Without `--apply` the script only
reports; it never changes stored limits.

## What it does, step by step

1. Connects and checks the firmware version (>= 0.8.0).
2. Puts the device in `current` mode (required by `calibrate_rom_gesture`; this
   turns torque off on all motors, which the firmware re-enables per joint per
   sweep).
3. For each gesture × direction: sends `calibrate_rom_gesture`. The firmware
   resolves the gesture to its motors, sweeps each in turn (ramp current, detect
   onset, confirm endstop, return home), and emits one `ROM_CAL_RESULT` per
   motor; the script collects them all.
4. Prints a per-motor summary table and, with `--apply`, writes `[min, max]`
   limits.
5. On exit (including Ctrl-C) it cancels the campaign and disables torque.

## Output

Each row is one motor; multi-motor gestures (thumb, wrist) contribute several.

```
  gesture    id dir       endstop      home    mA  status
  thumb      13 flex        210.40    220.79   150  ok
  thumb      14 flex        248.10    232.23   165  ok
  thumb      15 flex        140.55    122.76   150  ok
  index      16 flex        231.35    193.34   150  ok
  index      16 extend      162.27    193.34   170  ok
```

`status` is `ok` (endstop found), `ceiling` (reached the firmware current cap
without moving — no endstop), `timeout` (firmware watchdog fired), `aborted`
(cancelled, a precondition changed, or the motor was unreachable and skipped),
or `no_result` (host timed out waiting).

## Safety and tuning

- The endpoint deliberately drives current into a joint on the hand. The
  firmware caps current at `ROM_CAL_MAX_CURRENT_MA` (350 mA default, far below
  the motor limit) and bounds each sweep with a watchdog. Keep an emergency stop
  within reach; `cancel_rom` (also sent automatically on exit) aborts and homes.
- A sweep only finds motion **inside the joint's current stored limits** — the
  firmware's per-command clamp will not drive past them. Widen the stored limits
  first if you want the sweep to discover travel beyond an already-tight limit.
- All sweep tuning (ramp speed, onset sensitivity, nudge, current ceiling,
  timeout) lives in the labeled `ROM_CAL_*` block in
  [`src/cpp/nml_hand_exo/config.h`](../../src/cpp/nml_hand_exo/config.h). Change
  a value there and re-flash to make the sweep faster, gentler, or more/less
  sensitive; that block's header says which knob does what.

See [`docs/serial_protocol.md`](../../docs/serial_protocol.md) for the wire-level
`calibrate_rom_gesture` / `calibrate_rom` / `ROM_CAL_RESULT` protocol.
