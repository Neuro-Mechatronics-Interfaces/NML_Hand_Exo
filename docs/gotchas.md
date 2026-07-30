# Gotchas & Known Traps

Read this before changing firmware, the serial protocol, calibration, or the
GUI. These are safety and integration constraints, not optional conventions.

## Dual firmware: bare motor names select the left side

In a dual build (`BUILD_LEFT_HAND 2`), each motor name appears twice. Firmware
name lookup returns the first match, which is the left-side motor. Use explicit
integer Dynamixel IDs for calibration, limits, enable/disable, and all
side-specific motor commands. In particular, call
`HandExo.apply_calibration(..., name_to_id=...)` when connected to dual
firmware.

## Gesture commands are firmware broadcasts

`set_gesture` and `set_gesture_angle` resolve every matching firmware motor.
They do not inherently know which GUI side is selected. On connection, the GUI
disables motors outside the active mode; do not re-enable those inactive motors
while operating in a single-side mode. In dual mode, use the Gesture Target
selector before issuing a gesture.

## Never bypass calibrated joint limits

All movement must stay inside `jointLimits`. The wrist is intentionally
multi-turn; do not normalize it to 0-360 degrees or reintroduce an
unconditional shortest-path turn correction. Direct current is capped at
910 mA for XC330-T288 participants with finger spasticity.

## Calibration is applied at runtime

The GUI saves calibration profiles and applies them to the connected device,
but does not rewrite `config.h`. Device reboot returns to compiled defaults.
The calibration CLI can update the source configuration when that is explicitly
desired. Profiles need a `side` field; legacy profiles without it are treated as
right-side profiles by the GUI.

## Protocol delimiters have different roles

The host normally sends commands terminated by newline/CRLF. Firmware responses
are terminated by `;`, which is the delimiter consumed by `SerialComm.receive`.
Do not change either side independently.

## Firmware replies can contain units

When parsing device responses, do not assume every value is directly acceptable
to `float()`: firmware output may include units such as `mA` or `N·m`. Parse the
leading numeric portion defensively and retain unavailable telemetry as missing,
not zero.

## Normalized gesture-angle control requires matching firmware

`set_gesture_angle:<gesture>:<percent>` requires firmware 0.2.16 or newer. The
supported gestures are `thumb`, `thumbadd`, `thumbrot`, `thumbflex`, `index`,
`middle`, `ring`, `pinky`, and `wrist`. The three thumb-axis commands are
research-facing controls; use them cautiously because the thumb mechanisms are
mechanically coupled.

## UDP must have one serial owner

The HandExo GUI's UDP Command Input routes accepted packets through its serial
worker. Do not run another UDP-to-serial bridge against the same board while the
GUI is connected. UDP normalized-angle input is intentionally constrained to
finite 0-100 values and is coalesced before it reaches the serial link.
