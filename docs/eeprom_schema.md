# Gesture EEPROM Schema

Firmware file: `src/cpp/nml_hand_exo/gesture_eeprom.h` / `gesture_eeprom.cpp`  
Python API: `src/nml_hand_exo/interface/_hand_exo.py` — `flash_calibration_to_firmware()`

---

## Purpose

The EEPROM stores two things that would otherwise reset to compile-time defaults on every power cycle:

1. **Gesture fraction values** — the `[0.0, 1.0]` normalized open/close targets for every joint, gesture, state, and **side**. `0.0` = home/open end of the calibrated range; `1.0` = fully closed/flexed end.
2. **Calibration profile name** — a short string (≤ 15 chars) identifying which participant profile was last loaded via `flash_calibration_to_firmware()`.

Motor home positions, joint limits, and flip flags are **not** stored here — those are re-pushed from Python on every session via `apply_calibration()`.

---

## Physical layout

Hardware: SAMD21G18A (OpenRB-150).  
Library: `FlashStorage_SAMD` (Arduino Library Manager), which emulates EEPROM on Flash pages.

```
Offset  Size    Field
──────  ──────  ──────────────────────────────────────────────────────
0       4 B     magic       uint32_t — schema version tag (see below)
4       16 B    calName     char[16] — null-terminated, 15 usable chars
20      864 B   values      float[6][2][2][9] — gesture × state × side × joint
                            (single-exo builds: float[6][2][1][9] = 432 B)
──────  ──────
Total (dual):    884 B
Total (single):  452 B
```

The `values` array is the serialised form of
`gestureLibrary[side][g].states[s].namedPairs[j].value`
for all valid indices.  Indices correspond to the **canonical orders** defined below.

---

## Canonical index orders

These orders are fixed by the initialiser list in `gesture_library.cpp` and the
`config.h` constants.  
**Do not reorder any of these without also bumping the magic constant.**

### Gesture index (`g`, 0–5)

| Index | Name          |
|-------|---------------|
| 0     | `grasp`       |
| 1     | `keygrip`     |
| 2     | `pinch_index` |
| 3     | `pinch_middle`|
| 4     | `pinch_ring`  |
| 5     | `peace`       |

### State index (`s`, 0–1)

| Index | Name    |
|-------|---------|
| 0     | `open`  |
| 1     | `close` |

### Side index (`si`, 0–`EEPROM_N_SIDES`−1)

| Index | Name    | Motor IDs | Build condition       |
|-------|---------|-----------|-----------------------|
| 0     | `left`  | 1–9       | always present        |
| 1     | `right` | 11–19     | dual only (`N_HAND_SIDES == 2`) |

`EEPROM_N_SIDES = N_HAND_SIDES`.  Single-exo firmware stores only side 0 and the
struct is 432 B.  **EEPROM written by single-exo firmware cannot be read by dual
firmware** — the magic mismatch on schema version handles this safely.

### Joint index (`j`, 0–8)

This order matches the `namedPairs` array in every gesture state in `gesture_library.cpp`.  
All nine joints must appear in every state in this exact order.

| Index | Joint name   |
|-------|--------------|
| 0     | `wrist`      |
| 1     | `wrist2`     |
| 2     | `thumbadd`   |
| 3     | `thumbflex`  |
| 4     | `thumbrot`   |
| 5     | `index`      |
| 6     | `middle`     |
| 7     | `ring`       |
| 8     | `pinky`      |

---

## Magic constant and schema versioning

```cpp
constexpr uint32_t GESTURE_EEPROM_MAGIC = 0xCA130003u;
//                                          ^^^^  ^^^^
//                                          |     schema version (lower 16 bits)
//                                          namespace marker (upper 16 bits, fixed)
```

On boot, `gestureCalLoadFromEEPROM()` reads the stored magic and compares it to
`GESTURE_EEPROM_MAGIC`. A mismatch means the stored data is either uninitialised
(first flash, `0xFFFFFFFF` from erased flash) or from a different schema — in either
case the firmware falls back to the compile-time defaults in `gesture_library.cpp`
and returns `false`.

### When to bump the magic

Increment the lower 16 bits (`...0003` → `...0004`, etc.) and add a history entry
in `gesture_eeprom.h` **any time**:

- The number of gestures (`N_GESTURES`) changes, **or**
- The order of entries in `gestureLibrary[0][]` changes, **or**
- The number of states (`EEPROM_N_STATES`) changes, **or**
- The number of joints per state (`EEPROM_N_JOINTS`) changes, **or**
- The order of `namedPairs` within any gesture state changes, **or**
- `N_HAND_SIDES` changes (i.e. `EEPROM_N_SIDES` changes).

### Schema history

| Magic        | Description |
|--------------|-------------|
| `0xCA130001` | Original, pre-versioning. No explicit order guarantee. |
| `0xCA130002` | 6 gestures × 2 states × 9 joints (single side). |
| `0xCA130003` | **Current.** Adds side dimension: `values[gesture][state][side][joint]`. `EEPROM_N_SIDES = N_HAND_SIDES` (1 for single-exo, 2 for dual). |

---

## Per-side gesture fractions

In **dual mode** (`BUILD_LEFT_HAND == 2`) the firmware maintains two independent
gesture libraries — `gestureLibrary[0]` (left) and `gestureLibrary[1]` (right).
Each has its own set of `[0, 1]` fractions for every gesture × state × joint.

This means left and right can have **different ROM profiles** (e.g. the left index
closes to 0.8 of its calibrated range while the right closes to 0.6), even when
both hands execute the same named gesture.

The firmware initialises `gestureLibrary[1]` as a copy of `gestureLibrary[0]` at
boot (via `gestureLibraryInit()`), so both sides start with identical defaults.
`gestureCalLoadFromEEPROM()` then overwrites both with per-side EEPROM data if
valid data is present.

### Flashing per-side fractions from Python

```python
# Flash left-side profile
exo.flash_calibration_to_firmware("alice_left",  name_to_id=left_name_to_id,  side="left")

# Flash right-side profile
exo.flash_calibration_to_firmware("alice_right", name_to_id=right_name_to_id, side="right")

# Flash the same profile to both sides (default behaviour)
exo.flash_calibration_to_firmware("alice", name_to_id=name_to_id)
```

### Serial protocol

```
set_gesture_cal:<gesture>:<state>:<joint>:<value>          ← both sides
set_gesture_cal:<gesture>:<state>:<joint>:<value>:left     ← left only
set_gesture_cal:<gesture>:<state>:<joint>:<value>:right    ← right only

set_gesture:<gesture>:<state>                              ← both sides
set_gesture:<gesture>:<state>:left                         ← left only
set_gesture:<gesture>:<state>:right                        ← right only
```

---

## Default values (compile-time fallback)

When no valid EEPROM data exists, `gestureLibrary[][]` retains its compile-time
values.  Both sides start with these same defaults (copied by `gestureLibraryInit()`).

| Gesture       | Joint        | open | close |
|---------------|--------------|------|-------|
| grasp         | wrist        | 0.0  | 0.0   |
|               | wrist2       | 0.0  | 0.0   |
|               | thumbadd     | 0.0  | 1.0   |
|               | thumbflex    | 0.0  | 1.0   |
|               | thumbrot     | 0.0  | 1.0   |
|               | index        | 0.0  | 1.0   |
|               | middle       | 0.0  | 1.0   |
|               | ring         | 0.0  | 1.0   |
|               | pinky        | 0.0  | 1.0   |
| keygrip       | wrist        | 0.0  | 0.0   |
|               | wrist2       | 0.0  | 0.0   |
|               | thumbadd     | 0.0  | 1.0   |
|               | thumbflex    | 0.0  | 1.0   |
|               | thumbrot     | 0.0  | 0.0   |
|               | index        | 1.0  | 1.0   |
|               | middle       | 1.0  | 1.0   |
|               | ring         | 1.0  | 1.0   |
|               | pinky        | 1.0  | 1.0   |
| pinch_index   | wrist        | 0.0  | 0.0   |
|               | wrist2       | 0.0  | 0.0   |
|               | thumbadd     | 0.0  | 1.0   |
|               | thumbflex    | 0.0  | 1.0   |
|               | thumbrot     | 0.0  | 1.0   |
|               | index        | 0.0  | 1.0   |
|               | middle       | 0.0  | 0.0   |
|               | ring         | 0.0  | 0.0   |
|               | pinky        | 0.0  | 0.0   |
| pinch_middle  | wrist        | 0.0  | 0.0   |
|               | wrist2       | 0.0  | 0.0   |
|               | thumbadd     | 0.0  | 1.0   |
|               | thumbflex    | 0.0  | 1.0   |
|               | thumbrot     | 0.0  | 1.0   |
|               | index        | 0.0  | 0.0   |
|               | middle       | 0.0  | 1.0   |
|               | ring         | 0.0  | 0.0   |
|               | pinky        | 0.0  | 0.0   |
| pinch_ring    | wrist        | 0.0  | 0.0   |
|               | wrist2       | 0.0  | 0.0   |
|               | thumbadd     | 0.0  | 1.0   |
|               | thumbflex    | 0.0  | 1.0   |
|               | thumbrot     | 0.0  | 1.0   |
|               | index        | 0.0  | 0.0   |
|               | middle       | 0.0  | 0.0   |
|               | ring         | 0.0  | 1.0   |
|               | pinky        | 0.0  | 0.0   |
| peace         | wrist        | 0.0  | 0.0   |
|               | wrist2       | 0.0  | 0.0   |
|               | thumbadd     | 0.0  | 1.0   |
|               | thumbflex    | 0.0  | 1.0   |
|               | thumbrot     | 0.0  | 1.0   |
|               | index        | 0.0  | 0.0   |
|               | middle       | 0.0  | 0.0   |
|               | ring         | 0.0  | 1.0   |
|               | pinky        | 0.0  | 1.0   |

---

## How to add a new gesture

1. Append the new `GestureMap` entry to the **end** of the `[0]` initialiser in `gesture_library.cpp`.
2. Increment `N_GESTURES` in `config.h`.
3. Bump `GESTURE_EEPROM_MAGIC` and add a history entry in `gesture_eeprom.h`.
4. Update the gesture index table above.
5. Add the new gesture to `_DEFAULT_GESTURE_FRACTIONS` in `_hand_exo.py`.

**Never insert a gesture in the middle of the array** — append only, or bump the magic.

## How to add a new joint

1. Append the new `{joint_name, value}` pair to the **end** of every gesture state's `namedPairs` array in `gesture_library.cpp`.
2. Increment `EEPROM_N_JOINTS` in `gesture_eeprom.h`.
3. Bump `GESTURE_EEPROM_MAGIC` and add a history entry.
4. Update the joint index table above.

**Never insert a joint in the middle of `namedPairs`** — append only, or bump the magic.
