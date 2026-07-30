# Scripts #
This folder contains one-off scripts implementing ad hoc functionality without the full Qt GUI.

## UDP Forwarder ##
Usage (from `.handexo` virtual environment):
```bash
python examples/scripts/udp_gesture_receiver.py --port 10003
python examples/scripts/udp_gesture_receiver.py --mock       # no exo attached
```
Binds a UDP socket and forwards each received integer to the exo as one or more
serial commands. Per-joint moves use `set_gesture_angle:<joint>:<percent>`, so
the target position lives in this script's map rather than being fixed by the
firmware constants.

**Requires firmware >= 0.6.0.** From that version `set_gesture_angle`
percentages interpolate each gesture's own extend and flex postures, and
`wrist` drives both dorsal wrist motors together (the separate `rad` joint that
addressed `wrist2` alone was removed). Pose acks also need 0.6.0; the receiver
probes for them at startup and turns them off if the device does not answer, so
older firmware still runs.

### Value scheme ###

| Value | Meaning |
|---|---|
| `0` | Whole-hand release (`set_gesture:grasp:open`) |
| `+1 .. +6` | Flex joint N |
| `-1 .. -6` | Extend joint N (0% — the extend posture) |
| `+11 .. +16` | Rest joint N (intermediate posture) |

Joint order is `1 thumb, 2 index, 3 middle, 4 ring, 5 pinky, 6 wrist`. `wrist`
drives the `wrist` **and** `wrist2` motors together: both are mounted on the
back of the arm and pull on the dorsal aspect of the wrist, so commanding one
alone leaves the other holding position against it.

Percentages interpolate each joint's two end postures — `0` is exactly
`set_gesture:<joint>:extend` and `100` is exactly `set_gesture:<joint>:flex`,
as set by `EXTEND_*`/`FLEX_*` in `src/cpp/nml_hand_exo/config.h`. Retuning those
constants moves both ends, so these percentages keep meaning the same postures.
Edit `FLEX_PERCENT` / `REST_PERCENT` in the script to retune the UDP path alone.

Note `0` is the extend posture, **not** home: with a non-zero `EXTEND_*` a hand
parked at home sits below 0% and reads back as out of range (code `101`).

Integers above `64` are read as a return-port announcement rather than a
command; the receiver echoes each handled value back once the **device** has
replied, so an ack means the exo executed it, not just that the datagram landed.

### Pose acks ###

Each ack is followed by a second datagram carrying where **all six joints**
now sit, so a consumer learns the resulting pose without polling for it. The
receiver gets it by appending `get_gesture_angle:all` to every command batch —
after the move command, never before it — and the device answers from one
batched position read.

The frame is packed binary, little-endian:

| Field | Type | Meaning |
|---|---|---|
| magic | `4s` | `NGA1` |
| value | `h` | The same integer the ASCII ack carried |
| count | `B` | Number of joint codes that follow (6) |
| codes | `B` × count | One per joint, in the value-scheme order above |

Codes are `0`–`100` for a position between the extend and flex postures,
`101`/`102` for a joint sitting below or above them, and `255` when no position
is available — the joint has no calibrated travel, or the read failed. Use
`unpack_pose_ack(datagram)` from the receiver module to decode; `255` on a joint
you just commanded means the exo accepted the command and physically cannot
move that joint, which no ack alone can tell you.

The ASCII integer ack itself is **unchanged** and still goes out first, so
consumers that only parse integers are unaffected. `--no-pose-ack` skips the
query entirely and saves one device round trip per command.

### Current limits ###

Two different knobs, and the distinction matters:

| Flag | Firmware command | Bounds |
|---|---|---|
| `--current-ma` | `set_current_lim` | What **one** motor may push with |
| `--total-current-ma` | `set_total_current_lim` | What **all** motors may draw together |

Per-motor limits do not protect the supply: 18 motors each honouring a 200 mA
limit still draw up to 3.6 A together, which is what browns out the board when
`0` (whole-hand release) drives every joint onto its endstop at once. Size
`--total-current-ma` for the **supply**, and `--current-ma` for what a single
joint needs to move — before firmware 0.4.0 the per-motor value had to absorb
both jobs, which is why it kept getting detuned downward.

Both are sent before torque is enabled, budget first, so the motors never
energize at a wider allocation than intended. Watch `current_status` on the
device for the measured aggregate draw and the per-motor allocation; `applied`
below `nominal` means the budget is actively clamping.

## UDP Gesture GUI ##
```bash
python examples/scripts/udp_gesture_gui.py
python examples/scripts/udp_gesture_gui.py --host 192.168.1.50 --port 10003
```
A small tkinter panel that speaks the receiver's protocol so gestures can be
triggered by hand — handy for checking wiring, calibration and travel without a
decoder or EMG rig in the loop. Defaults target a receiver on localhost at its
own default port, so with a default receiver running it needs no arguments;
destination host/port and the local ack port are editable in the window.

Buttons cover extend/rest/flex per joint plus the whole-hand release, and a
manual field sends any integer. The value table comes from the receiver module
itself, so the two cannot drift apart. Sent commands and acks are logged, and
each ack is followed by the decoded pose line — a joint whose percentage never
changes, or that shows `--`, is one the exo accepted a command for and could
not actually move.

Each joint row also has a **Manual %** field for positioning it anywhere
between its two end postures (`0` = extend, `100` = flex), prefilled with that
joint's flex percent. Enter or **Send** transmits it.

Because no mapped integer can carry an arbitrary percentage, these travel as the
command form `set_gesture_angle:<joint>:<percent>` rather than as an integer.
The receiver accepts that **one** command shape directly, validating the joint
name and the 0–100 range before forwarding — it is deliberately not a general
command passthrough, since the receiver binds `0.0.0.0` by default and anything
it forwards is reachable from the network. Passthroughs are acked with the
sentinel `1000` instead of an echoed value.
