# Scripts #
This folder contains one-off scripts implementing ad hoc functionality without the full Qt GUI.

## UDP Forwarder ##
Usage (from `.handexo` virtual environment):
```bash
python scripts/udp_gesture_receiver.py --port 10003
python scripts/udp_gesture_receiver.py --mock       # no exo attached
```
Binds a UDP socket and forwards each received integer to the exo as one or more
serial commands. Per-joint moves use `set_gesture_angle:<joint>:<percent>`, so
the target position lives in this script's map rather than being fixed by the
firmware constants.

**Requires firmware >= 0.3.0** for `set_gesture_angle` and the `wrist` gesture,
and **>= 0.3.1** for `rad`.

### Value scheme ###

| Value | Meaning |
|---|---|
| `0` | Whole-hand release (`set_gesture:grasp:open`) |
| `+1 .. +7` | Flex joint N |
| `-1 .. -7` | Extend joint N (0% — home / the extension endstop) |
| `+11 .. +17` | Rest joint N (intermediate posture) |

Joint order is `1 thumb, 2 index, 3 middle, 4 ring, 5 pinky, 6 wrist, 7 rad`.
`rad` is the second wrist axis, driven by the `wrist2` motor — its anatomy is
unverified, see the `EXTEND_RAD` note in `src/cpp/nml_hand_exo/config.h`.

Percentages are a fraction of each joint's calibrated range measured from home
in the flexion direction, and the mapping is flip-aware — higher always means
more flexion. Edit `FLEX_PERCENT` / `REST_PERCENT` in the script to retune.

Integers above `64` are read as a return-port announcement rather than a
command; the receiver echoes each handled value back once the **device** has
replied, so an ack means the exo executed it, not just that the datagram landed.

## UDP Gesture GUI ##
```bash
python scripts/udp_gesture_gui.py
python scripts/udp_gesture_gui.py --host 192.168.1.50 --port 10003
```
A small tkinter panel that speaks the receiver's protocol so gestures can be
triggered by hand — handy for checking wiring, calibration and travel without a
decoder or EMG rig in the loop. Defaults target a receiver on localhost at its
own default port, so with a default receiver running it needs no arguments;
destination host/port and the local ack port are editable in the window.

Buttons cover extend/rest/flex per joint plus the whole-hand release, and a
manual field sends any integer. The value table comes from the receiver module
itself, so the two cannot drift apart. Sent commands and acks are logged.
