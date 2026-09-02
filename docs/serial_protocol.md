# Serial Protocol & Firmware Reference

> `[VERIFIED]` = confirmed from source. `[INFERRED]` = reasonable but unverified.

---

## Communication stack `[VERIFIED]`

```
Python (host PC)
  SerialComm / DualSerialComm / TCPComm
       (src/nml_hand_exo/interface/_interfaces.py)
       |
      | USB serial  → DEBUG_SERIAL  = Serial   (1000000 baud)
       | BT HC-05    → COMMAND_SERIAL = Serial3  (115200 baud, D13=TX D14=RX)
       |
Arduino OpenRB-150
  loop() polls both channels identically
  → parseMessage()  (utils.cpp) — dispatches all command strings
  → NMLHandExo      (nml_hand_exo.cpp) — motor control
   → Serial1 (DXL_SERIAL, 1000000 baud) — Dynamixel bus
```

Both command inputs are active. On OpenRB-150, the default build exposes two
USB CDC interfaces on the same cable: commands use the primary CDC while
replies and telemetry use the second. `set_reply_route:telem` fully decouples
them; `set_reply_route:both` retains single-port compatibility. Bluetooth
commands continue to use the same `parseMessage()` dispatcher.

---

## Command format `[VERIFIED]`

Plain ASCII, colon-delimited, terminated with `\n` (Python) or `;` (BT/COMMAND_SERIAL):

```
<command>:<arg1>:<arg2>...\n
```

Examples:
```
get_absolute_angle:index
get_absolute_angle:all
status:all
set_angle:index:45
set_motor_limits:index:162.8:224.93
set_zero_offset:wrist:149.1
set_flip:middle:1
enable:all
disable:all
enable_ids:11:12:13
disable_ids:11:12:13
set_exo_mode:gesture_fixed
get_telemetry_fast:11:12:13:14:15:16:17:18:19
telemetry_diag:11:12:13:14:15:16:17:18:19
get_gesture_angles:all
info
version
```

The read-only `status:<id|name|all>` command returns a compact motor summary,
including torque state, control modes, absolute and relative angles, stored
home angle, joint limits, present current, goal current, and current limits.
With `all`, it also includes the global current budget, hold current, and
current-governor state.

At firmware startup, the controller probes the configured IDs and caches which
motors respond. In a dual build, gesture commands and current-budget management
skip IDs that did not respond, so operating one connected hand does not wait on
the absent hand.

Text responses are terminated with `;`. `SerialComm.receive()` reads until `;` is seen.

### Fast telemetry binary frame

`get_telemetry_fast:<id>:<id>...` returns one compact binary frame on the serial
stream. `get_telemetry_fast:all` returns all firmware-managed motors. The frame
starts with magic bytes `NX`, version `1`, followed by fixed-size records with
DXL ID, error flag, present current, raw present velocity, position ticks,
absolute angle in centidegrees, and relative angle in centidegrees. Header
`flags` reports the firmware read method: `2` = fastSyncRead, `3` = syncRead,
`1` = short-timeout fallback individual reads, `0` = failed. This command is
intended for GUI polling because it avoids multiple text round trips per motor.

`telemetry_diag:<id>:<id>...` returns a text diagnostic using the same firmware
read path, including method, elapsed microseconds, and per-motor raw values.

### Gesture positioning `[VERIFIED]`

```text
set_gesture_angle:<gesture>:<0-100>
get_gesture_angle:<gesture|all>
get_gesture_sang:<gesture|all>
get_gesture_angles:<gesture|all>
```

The percentage interpolates a gesture between its OWN two end postures:

```text
set_gesture_angle:<g>:0    == set_gesture:<g>:extend
set_gesture_angle:<g>:100  == set_gesture:<g>:flex
set_gesture_angle:<g>:50   == halfway between the two
```

Endpoints come from the `EXTEND_*`/`FLEX_*` constants in `config.h`, so retuning
those moves the axis with them and a host's percentages keep meaning the same
postures. Each named motor travels its own share, so a gesture driving several
motors keeps the ratio between them at every percentage. Each motor's share is
then placed on its calibrated travel — the signed distance from home to its
flexion endstop, resolved from home and `[limit_min, limit_max]` by
`NMLHandExo::getGestureSpan` — so no percentage in range ever clamps, and the
two commands are exact inverses.

Note `0` is the **extend posture, not home**. With a non-zero `EXTEND_*` a hand
parked at home sits below 0% and reads back as `101`.

Addressable gestures are exactly those with a `flex` state: `thumb`,
`thumbadd`, `thumbrot`, `thumbflex`, `index`, `middle`, `ring`, `pinky`, and
`wrist`. The three specific thumb names expose individual research axes while
`thumb` coordinates all three. Multi-joint postures (`grasp`, `keygrip`,
`pinch_*`) are rejected — one percentage cannot describe a posture. `wrist`
drives both dorsal wrist motors (`wrist` and `wrist2`) together; the `rad`
gesture that addressed `wrist2` alone existed only in 0.3.1 – 0.5.x.

`get_gesture_angle` replies with a single line of `name=code` pairs:

```text
GESTURE_ANGLE: thumb=12 index=0 middle=45 ring=101 pinky=100 wrist=255;
```

| Code | Meaning |
|---|---|
| `0`–`100` | Position between the extend and flex postures |
| `101` | Below the extend end |
| `102` | Above the flex end |
| `255` | No position available: no calibrated travel, or the read failed |

A gesture covering several motors (the thumb, the wrist pair, or any joint on a
dual build) reports the mean of the per-motor percentages, skipping joints that
cannot carry one. Positions come from ONE batched Dynamixel read, so polling
this per command costs a single bus transaction.
Emitted through `commandPrint`, so on a dual-CDC build it follows the active
reply route — under `set_reply_route:telem` it lands on the telemetry CDC only.

Needs firmware **>= 0.6.0**; earlier builds ignore the command *silently*, so a
host must gate on the version (or on whether a probe answers) rather than wait
for an error reply. Run `check_limits` when a joint reports `255` or refuses to
move: it prints the resolved `span` per motor and flags `NO_TRAVEL`,
`HOME_OUTSIDE`, `LIMITS_INVERTED` and `SPAN_REVERSED`.

Firmware **0.6.1** adds two rest-zeroed signed-degree views over the same
batched position sample:

```text
get_gesture_sang:<gesture|all>
GESTURE_SANG: index=-12.50 middle=0.00 ring=8.25 wrist=34.75;

get_gesture_angles:<gesture|all>
GESTURE_ANGLES: index=0,-12.50 middle=45,0.00 ring=101,8.25 wrist=255,nan;
```

`get_gesture_sang` (signed angle) returns only the signed physical delta in degrees.
`get_gesture_angles` returns `<percentage-code>,<signed-degrees>` for each
gesture, preserving the exact `get_gesture_angle` code in the first field.
`nan` means the signed angle is unavailable; in the combined reply it can
coexist with a valid percentage, or with code `255` when no position at all is
available.

The degree convention is intentionally independent of encoder direction and
the motor `flip` flag:

- The first motor named by the gesture supplies the calibrated degree scale.
- Its `rest` target is exactly `0 deg`.
- Travel from rest toward `flex` is positive.
- Travel from rest toward `extend` is negative.

If `e`, `r`, and `f` are that reference motor's extend, rest, and flex
fractions, `t` is the underlying gesture position on the extend-to-flex axis,
and `S` is the motor's calibrated `getGestureSpan()` in degrees, the returned
angle is:

```text
reference_fraction = e + t * (f - e)
signed_degrees = (reference_fraction - r) * sign(f - r) * abs(S)
```

The calculation uses the underlying (unclamped) `t`, not status code 101 or
102, so an out-of-range combined result can legitimately be `101,-15.20` or
`102,42.10`. All three query forms still cost one Dynamixel batch read.

For multi-motor gestures, the percentage remains the mean of all usable motor
percentages, while degrees use only the first motor's calibrated span. For
`thumb` that reference is `thumbadd`; for `wrist` it is `wrist`, not `wrist2`.
In a dual build the first matching motor is on the left side because IDs 1-9
precede IDs 11-19. The percentage can therefore aggregate both sides while the
degree scale comes from the left reference motor. Use side-specific firmware
or account for that convention when left and right calibration spans differ.

Firmware older than **0.6.1** ignores both new command names silently. Gate
host calls on the version or probe for a correctly prefixed reply.

### Direct velocity/current control

Direct control is global-mode, per-motor-commanded, and uses explicit Dynamixel IDs:

```text
set_control_mode:all:velocity
set_velocity:16:2.5
set_control_mode:all:current
set_current:16:50
stop:16
stop:all
set_command_timeout:250
hold_position:14:22.5
release_hold:14
```

- `set_velocity` uses signed rpm and is clamped to `DIRECT_VELOCITY_LIMIT_RPM` (50 rpm in the current validated firmware source).
- Before entering Velocity mode, firmware turns torque off, reads each motor's
  hardware `VELOCITY_LIMIT` register, conditionally writes raw `218`
  (approximately 49.9 rpm at 0.229 rpm/unit), and verifies readback. A failed
  verification aborts the mode change with torque remaining off. The EEPROM
  register is not rewritten when it already matches. Dual firmware skips IDs
  that are not physically reachable, but records verification per ID and
  rejects direct velocity commands for any ID that was not verified.
- `set_current` uses signed mA and is clamped to `DIRECT_CURRENT_LIMIT_MA`
  (910 mA for XC330-T288 in this firmware build).
- The GUI permits explicit-ID multi-finger EMG control in Current / Torque mode.
  It first applies each motor row's current ceiling, then proportionally scales
  the complete group to the configured combined-current budget. Any active
  auxiliary position-hold current is reserved before that group budget is
  calculated. This aggregate governor is host-side; firmware independently
  retains its per-ID current clamp, calibrated joint-limit stop, and direct
  command watchdog. Other simultaneous command sources are not included in the
  GUI's aggregate calculation and must not be used during EMG torque control.
- EMG Current / Torque mode is a continuous signed-current controller, not a
  position latch: neutral, low-confidence, or stale intent commands zero current.
  Phase-1 shadow contact recording remains available only in Velocity mode.
- `enable_ids` / `disable_ids` accept colon-separated explicit DXL IDs and
   toggle torque for the provided list in one parser command.
- Mode changes turn torque off. Motors must be explicitly enabled afterward.
- The firmware zeros stale direct commands after the configured watchdog timeout.
- The firmware also zeros a direct command when the motor reaches its calibrated
  joint-limit margin.
- `set_goal_velocity` remains the position-mode profile-velocity setting; it is
  not a direct velocity command.

Firmware **0.6.2** adds an auxiliary mixed-mode position hold for joints that
must keep a fixed posture while other motors receive direct EMG commands:

- `hold_position:<explicit ID>:<relative angle>` atomically stops that motor,
  disables its torque, switches only that ID to current-based position mode,
  writes a goal through the existing calibrated joint-limit clamp, applies the
  configured settled-motor hold current, and re-enables torque.
- `release_hold:<explicit ID>` disables the held motor and restores its
  operating mode to the current global direct mode. Torque remains off.
- Bare motor names are rejected because they are ambiguous in dual firmware.
- The global mode must already be `velocity` or `current` before a hold is
  engaged. Direct velocity/current commands to a held ID are rejected.
- `stop:<id|all>` zeros direct commands but deliberately does not release a
  position hold. This lets a held thumb posture survive neutral or stale intent.
  Call `release_hold` explicitly; the GUI does this for STOP TELEOP,
  STOP ALL MOTION, mode changes, and disconnect.

The current development firmware extends the command with an optional per-hold
current while retaining the 0.6.2 version identifier until release:

- `hold_position:<explicit ID>:<relative angle>:<requested mA>`
- The requested current is clamped to the selected motor's configured
  per-motor current limit, the XC330 firmware maximum, and the configured total
  current budget.
- Omitting the fourth argument preserves the 0.6.2 behavior and uses the global
  settled-motor hold current.
- The acknowledgement reports the current that was actually applied. The GUI
  also reads `get_enabled:<ID>` before declaring the hold verified.

Successful replies are:

```text
OK: hold_position id=14 angle=22.500 current_mA=80
OK: release_hold id=14
```

### Phase-1 shadow contact instrumentation

The development firmware adds an opt-in, read-only sampler for characterizing
contact evidence during direct velocity teleoperation:

```text
shadow_config:2:15:16:17:18:19
shadow_start
shadow_status
shadow_stop
```

- `shadow_config` requires a sample interval followed by unique, explicit DXL
  IDs. Bare names and `all` are intentionally unsupported.
- The sampler is disabled at boot and `shadow_start` succeeds only in global
  `VELOCITY` mode.
- It performs one register read per service pass, alternating
  `PRESENT_CURRENT` and `PRESENT_POSITION`. For five IDs at a 2 ms interval, a
  complete current/position update per ID is nominally 20 ms (50 Hz), subject
  to actual bus latency.
- Velocity is derived from successive relative-position samples; no
  `PRESENT_VELOCITY` read is required.
- `shadow_status` returns buffered values only. It performs no Dynamixel read.
- These commands never enable torque or write goal, mode, limit, current, or
  position registers. Changing out of velocity mode automatically stops the
  sampler.
- Shadow estimates are observation-only and must never be treated as a motor
  command or a validated contact detector.

See [shadow_contact_phase1.md](shadow_contact_phase1.md) for the bench workflow.

---

## Baud rate map `[VERIFIED]`

| Channel          | Arduino object | Baud  | Physical connection  |
|------------------|----------------|-------|----------------------|
| USB debug        | Serial         | 1000000 | USB port             |
| Dynamixel bus    | Serial1        | 1000000 | JST DXL connector    |
| HC-05 Bluetooth  | Serial3        | 115200 | D13 (TX), D14 (RX)   |

All three constants are defined in `src/cpp/nml_hand_exo/config.h`:
`DEBUG_BAUD_RATE`, `DYNAMIXEL_BAUD_RATE`, `COMMAND_BAUD_RATE`.

Live OpenRB/XC330 diagnostics showed that the exo chain was not reliable at a
2 Mbps Dynamixel bus rate: repeated single-motor `PRESENT_POSITION` reads had
timeouts, CRC errors, and buffer overflows even with longer read timeouts and
return delay restored. The same motor was stable at 1 Mbps (`100/100` repeated
position reads, zero timeout/CRC/overflow errors), so 1 Mbps is the recommended
rate for both the USB debug link and the DXL bus.

HC-05 factory default is 9600. The firmware is configured for 115200 (`COMMAND_BAUD_RATE`).
If you swap an HC-05 module, use AT command mode to set it to 115200 before use.

---

## Board serial mapping (OpenRB-150) `[VERIFIED]`

```cpp
#define DEBUG_SERIAL   Serial    // USB CDC
#define DXL_SERIAL     Serial1   // Dynamixel TTL bus
#define COMMAND_SERIAL Serial3   // D13=TX3, D14=RX3  ← HC-05 wired here
```

OpenRB-150 does not use a DIR pin for Dynamixel (`DXL_DIR_PIN = -1`).

---

## Response parsing — load-bearing patterns `[VERIFIED]`

These string patterns are parsed by Python scripts. Changing them in firmware
breaks the corresponding Python code.

| Firmware output pattern | Parsed by | Used for |
|------------------------|-----------|----------|
| `name: <word>` | `calibrate_exo.py`, `rom_assessment.py` | Motor name discovery via `info` command |
| `absolute_angle:<value>` | `calibrate_exo.py`, `rom_assessment.py` | Angle reads |
| `GESTURE_ANGLE: <name>=<code> ...` | `_hand_exo.py:parse_gesture_angles` | Legacy percentage/status read-back |
| `GESTURE_SANG: <name>=<degrees> ...` | `_hand_exo.py:parse_gesture_signed_angles` | Rest-zeroed signed joint angles |
| `GESTURE_ANGLES: <name>=<code>,<degrees> ...` | `_hand_exo.py:parse_gesture_angle_pairs`, `udp_gesture_receiver.py` | Combined joint positions / NGA2 pose acks |
| `GESTURE_RESULT: reached=N ...` | `udp_gesture_receiver.py`, `udp_gesture_gui.py` | Asynchronous move verdicts |
| `SHADOW: {enabled: ...}` plus `Motor N: {...}` | `_hand_exo.py:get_shadow_telemetry` | Buffered read-only current/position/contact evidence |

Regex used: `re.search(r"name:\s*(\w+)", line)` and `line.split("absolute_angle:")`.

---

## Protocol coupling rules `[VERIFIED]`

Violating these causes commands to silently fail or return garbage.

1. **Command names are a shared contract.**
   Any string in `utils.cpp:parseMessage()` must match exactly what `_hand_exo.py` sends.
   Rename a command in C++ → update the Python method, and vice versa.

2. **Delimiter is always `;`.**
   `COMMAND_DELIMITER` in `config.h` and `command_delimiter` in `SerialComm.__init__()`
   must always be the same character.

3. **`info` response format is load-bearing.**
   `calibrate_exo.py` and `rom_assessment.py` parse motor names from the `info` response
   using `re.search(r"name:\s*(\w+)", line)`. Changing the format breaks both scripts.

4. **`get_absolute_angle` response label is load-bearing.**
   Both calibration scripts split on the literal string `"absolute_angle:"`.
   Do not change this label without updating both parsers.

5. **Baud rates are set in `config.h` only.**
   Never hard-code baud rates in `.ino` or Python scripts.

6. **Profile JSON schema is a shared contract.**
   `calibrate_exo.py` and the GUI write it; `rom_assessment.py` and the GUI read it.
   Keys `home`, `flip`, `limit_min`, `limit_max` under `motors.<name>` must stay
   present and consistently named across all readers and writers.

7. **Motor names are the join key.**
   `config.h:MOTOR_NAMES[]` → firmware `info` response → Python motor name list
   → `profiles/<name>.json` keys. All four must stay in sync.

---

## Firmware / protocol change gate

Before touching any C++ firmware or the serial protocol:

1. **Can this be done in Python only?** Read `utils.cpp` for the full command list first.
   Per-motor calibration, guided prompts, profile logic — all achievable without
   firmware changes using existing commands.

2. **Does it need new device-side state or a query no existing command covers?**
   If yes: document the proposed command name, argument format, and response format
   before writing any code. Only then proceed to firmware.

3. **If a protocol change is truly needed: update both sides atomically.**
   Add command to `utils.cpp` → add Python method to `_hand_exo.py` → update
   the coupling rules above → re-flash → test.

**Default: do not touch firmware.** Protocol is stable. Python changes don't require re-flashing.

---

## Dual-mode motor name disambiguation `[VERIFIED]`

In dual firmware (`BUILD_LEFT_HAND 2`), `MOTOR_NAMES[]` contains duplicate bare names:
"wrist" exists at index 0 (ID 1, left) and index 9 (ID 11, right).

The command resolver now detects which side responds on the bus the first time a
bare motor name is used, then caches that active side. With one hand connected,
bare names target that hand:

```
set_zero_offset:wrist:X    →  connected hand's wrist
set_motor_limits:index:X:Y →  connected hand's index
```

Side-qualified names remain available using `/` (the protocol already uses `:`
as the argument delimiter):

```
set_angle:R/index:45      →  ID 16 (right index)
get_current:L/pinky      →  ID 9 (left pinky)
```

Numeric DXL IDs are still accepted for unambiguous low-level control:

```
set_zero_offset:11:X   →  ID 11 (right wrist)  ✓
set_motor_limits:11:X:Y →  ID 11 (right wrist) ✓
```

If both sides respond, bare names are rejected as ambiguous rather than silently
choosing the left side.

`HandExo.apply_calibration(name_to_id={...})` enforces ID-based commands automatically
when the GUI passes a `name_to_id` mapping. See [docs/dual_exo_architecture.md](dual_exo_architecture.md).

---

## Key constants (config.h)

| Constant                | Value    | Meaning                              |
|-------------------------|----------|--------------------------------------|
| `DEBUG_BAUD_RATE`       | 1000000  | USB serial baud                      |
| `COMMAND_BAUD_RATE`     | 115200   | HC-05 Bluetooth baud (firmware side) |
| `DYNAMIXEL_BAUD_RATE`   | 1000000  | Dynamixel bus baud                   |
| `MOTOR_CURRENT_LIMIT`   | 910      | XC330-T288 current cap per motor (mA) |
| `DXL_PROTOCOL_VERSION`  | 2.0      | Dynamixel protocol version           |
| `PULSE_RESOLUTION`      | 4096     | Encoder ticks per revolution         |
| `XC330_T288_TORQUE_CONSTANT` | 0.00115  | Estimated N*m per mA at 11.1 V       |
| `N_GESTURES`            | 15       | Gestures in the library              |
| `GESTURE_MIN_TRAVEL_DEG` | 2.0     | Least travel a joint needs to be positionable |
| `GESTURE_FRACTION_TOLERANCE` | 0.02 | Slack before a joint reads as out of range |
| `GESTURE_AXIS_MIN_SEPARATION` | 0.02 | Least EXTEND_* to FLEX_* gap that carries a position |
| `STATUS_LED_PIN`        | 0        | Onboard LED pin                      |
| `COMMAND_DELIMITER`     | `;`      | End-of-message character             |

---

## Firmware build & flash

Open `src/cpp/nml_hand_exo/nml_hand_exo.ino` in the Arduino IDE.

**Board:** OpenRB-150 (install ROBOTIS board package in Arduino IDE board manager)

**Required libraries** (Arduino Library Manager):
- Dynamixel2Arduino
- Adafruit BNO055
- Adafruit Unified Sensor
- Adafruit SSD1306
- Adafruit GFX Library

Upload → board flashes LED 4× and prints `"Exo device ready to receive commands"`.
