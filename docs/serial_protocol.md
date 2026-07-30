# Serial Protocol & Firmware Reference

> `[VERIFIED]` = confirmed from source. `[INFERRED]` = reasonable but unverified.

---

## Communication stack `[VERIFIED]`

```
Python (host PC)
  SerialComm / TCPComm  (src/nml_hand_exo/interface/_interfaces.py)
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

Both channels are always active. Any command sent to either port is processed
by the same `parseMessage()` dispatcher. Responses go to both channels.

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
set_angle:index:45
set_motor_limits:index:162.8:224.93
set_zero_offset:wrist:149.1
set_flip:middle:1
enable:all
disable:all
enable_ids:11:12:13
disable_ids:11:12:13
set_exo_mode:gesture_fixed
get_telemetry_fast:11:12:13
info
version
```

Responses are terminated with `;`. `SerialComm.receive()` reads until `;` is seen.

### Compact telemetry frame

`get_telemetry_fast:<id>:<id>...` returns one binary frame, intended only for
the GUI's single serial worker. The version-1 frame has `NX` magic bytes, a
13-byte little-endian header, and 20-byte records keyed by DXL ID. Its checksum
is the low 16 bits of the sum of the header bytes before the checksum plus the
payload bytes.

The current firmware uses the conservative `fallbackRead` method (`flags = 1`):
it reports relative and absolute position only. The record's zero current and
velocity fields are placeholders in this mode and **must be shown as unavailable,
not as measured zero**. Other flag values are reserved for future validated
multi-register reads. Text polling remains the compatibility fallback for older
firmware or malformed frames.

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
```

- `set_velocity` uses signed rpm and is clamped to `DIRECT_VELOCITY_LIMIT_RPM`.
- `set_current` uses signed mA and is clamped to `DIRECT_CURRENT_LIMIT_MA`
  (910 mA for XC330-T288 in this firmware build).
- `enable_ids` / `disable_ids` accept colon-separated explicit DXL IDs and
   toggle torque for the provided list in one parser command.
- Mode changes turn torque off. Motors must be explicitly enabled afterward.
- The firmware zeros stale direct commands after the configured watchdog timeout.
- The firmware also zeros a direct command when the motor reaches its calibrated
  joint-limit margin.
- `set_goal_velocity` remains the position-mode profile-velocity setting; it is
  not a direct velocity command.

---

## Baud rate map `[VERIFIED]`

| Channel          | Arduino object | Baud  | Physical connection  |
|------------------|----------------|-------|----------------------|
| USB debug        | Serial         | 1000000 | USB port             |
| Dynamixel bus    | Serial1        | 1000000 | JST DXL connector    |
| HC-05 Bluetooth  | Serial3        | 115200 | D13 (TX), D14 (RX)   |

All three constants are defined in `src/cpp/nml_hand_exo/config.h`:
`DEBUG_BAUD_RATE`, `DYNAMIXEL_BAUD_RATE`, `COMMAND_BAUD_RATE`.

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

Firmware `getMotorIDByName()` performs a linear scan and **returns the first match**.
In dual mode this is always the left motor. Any command using a bare name in dual mode
silently targets the wrong side:

```
set_zero_offset:wrist:X   →  ID 1 (left), regardless of intent
set_motor_limits:wrist:X:Y →  ID 1 (left)
```

**Safe pattern**: use the integer DXL ID. `getMotorID()` parses the token as an integer
first; if non-zero, the integer is used directly without name lookup:

```
set_zero_offset:11:X   →  ID 11 (right wrist)  ✓
set_motor_limits:11:X:Y →  ID 11 (right wrist) ✓
```

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
| `N_GESTURES`            | 6        | Gestures in the library              |
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
