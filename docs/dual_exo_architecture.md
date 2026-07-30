# Dual-Exo Architecture

## Hardware model

One OpenRB-150 controls one Dynamixel bus and one host serial connection. The
GUI uses one `HandExo` instance even when both hands are present. `DualHandExo`
exists for a two-board design and is not used by the GUI.

| Side | Dynamixel IDs | Firmware build |
|---|---:|---|
| Left | 1-9 | `BUILD_LEFT_HAND 1` or `2` |
| Right | 11-19 | `BUILD_LEFT_HAND 0` or `2` |
| Both | 1-9 and 11-19 | `BUILD_LEFT_HAND 2` |

The per-side order is wrist, wrist2, thumbadd, thumbrot, thumbflex, index,
middle, ring, pinky.

## Name ambiguity

Dual firmware has duplicate bare motor names. `getMotorIDByName()` returns the
first match, so a bare name selects the left motor. The safe pattern is explicit
integer ID targeting:

```text
set_zero_offset:11:<angle>  # right wrist
set_motor_limits:16:<min>:<max>  # right index
```

This is mandatory for calibration and any side-specific operation.

## GUI modes and containment

The GUI exposes Right Only, Left Only, and Dual modes. At connection time it
builds an active DXL-ID list and explicitly disables every detected motor not in
that list. This contains firmware gesture broadcasts in single-side modes.

In Dual mode, the Gesture Target selector chooses Both, Left Only, or Right
Only. It enables the requested side's eligible motors and disables the other
side before a gesture. Changing the GUI mode is locked while connected.

## Calibration

Profiles include a `side` value and per-motor `home`, `flip`, `limit_min`, and
`limit_max` data. The GUI supplies its selected-side `name_to_id` mapping to
`HandExo.apply_calibration`, ensuring calibration commands use IDs rather than
ambiguous names. Calibration and ROM dialogs disable only their target-side IDs,
never `disable:all`.

## Gesture routing

Firmware gesture execution resolves matching motor names across its active
motor set. In a dual build that can mean both left and right copies. Inactive
side containment therefore depends on torque being disabled, and gesture goals
may still be latched for disabled motors. Re-enabling an inactive side later can
move it to a previously written goal.

The same rule applies to normalized `set_gesture_angle` control. To research a
single physical motor or side, use an explicit-ID direct-control workflow only
after confirming mode, limits, current limit, and participant safety.
