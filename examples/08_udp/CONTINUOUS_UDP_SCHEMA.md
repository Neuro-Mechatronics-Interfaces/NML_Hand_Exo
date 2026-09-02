# NML continuous UDP schema

`continuous_to_udp_bridge.py` sends one UTF-8 JSON object per UDP datagram. It does not append a newline or NUL byte. The datagram schema identifier is `nml.continuous.v1`.

The input datagram contract is `v1` and unchanged. `v2` adds ONE thing on top of it: an optional binary upstream **ack** returned by the receiver for every accepted datagram (see "Upstream ack" below). A `v1` sender that ignores its source socket is fully `v2`-compatible; nothing about the datagram it sends changes.

```json
{
  "schema": "nml.continuous.v1",
  "sequence": 17,
  "source_time_s": 1788364800.12,
  "channel_names": ["Thumb", "Index", "Pinky"],
  "values": [0.25, -0.5, 0.0]
}
```

## Field contract

| Field | Type | Contract |
|---|---|---|
| `schema` | string | Exactly `nml.continuous.v1`. |
| `sequence` | integer | Unsigned 32-bit packet sequence; wraps from `4294967295` to `0`. |
| `source_time_s` | number | Finite source-sample timestamp in seconds from the CTRL-R stream clock. It is not the UDP receive time. |
| `channel_names` | array of strings | Ordered, non-empty, case-insensitively unique names. |
| `values` | array of numbers | Same length/order as `channel_names`; every value is finite and within `[-1,1]`. Positive is Flexion, negative is Extension, and zero is Rest for an N2 paired model. |

No additional fields are present in v1. The bridge affinely maps its configured `input_min..input_max` into `-1..1`, then clamps roundoff/outliers. With the N2 decoder defaults (`input_min=-1`, `input_max=1`) this mapping is the identity.

The sender consumes the 20 ms decoder stream and transmits samples 3, 6, 9, and so on: one datagram every 60 ms. It processes all queued samples, so batching inside CTRL-R does not alter the decimation phase. UDP remains best-effort; a missing sequence means at least one datagram was lost or reordered.

## Receiver dispatch

This schema can share the legacy Exo UDP port safely:

1. `NGA2` identifies the existing binary pose acknowledgement.
2. A datagram beginning with `{` is decoded as JSON and accepted only when its
   `schema` is exactly `nml.continuous.v1`.
3. Otherwise, retain the existing ASCII-integer gesture-command handling.

Reference Python validation is available as `decode_continuous_packet()` in `continuous_to_udp_bridge.py`. Equivalent Exo handler logic should reject malformed UTF-8/JSON, unknown schema versions, unknown or reordered channels, duplicate names, non-finite values, values outside `[-1,1]`, or mismatched name/value lengths. Establish the expected channel order from the first accepted packet or an explicit Exo configuration, then require it to remain unchanged.

The receiver should also apply its own safety policy: ignore stale/duplicate sequence numbers, clamp once more before converting values to actuator targets, and return all channels to a safe neutral state if valid continuous packets stop arriving for the Exo-configured watchdog interval.

The input `v1` datagram is one-way: continuous datagrams carry no echo request, and this bridge does not use the legacy integer connection or heartbeat messages. In `v1` liveness is therefore determined solely by the receiver watchdog and the advancing `sequence` field. `v2` adds an optional upstream ack (below) that a monitoring consumer may read, but the datagram itself is still one-way and a sender is never required to read the ack.

## Upstream ack (schema v2)

Under `v2` the receiver returns one compact binary ack for every **accepted** datagram, sent to that datagram's UDP source address. Rejected, stale, and duplicate datagrams are not acked. The frame is little-endian:

| Bytes | Field | Type | Meaning |
|---|---|---|---|
| 0-3 | magic | 4 bytes | ASCII `NGA3`. Distinct from the `NGA2` gesture pose ack so both can share a port. |
| 4-7 | sequence | uint32 | The `sequence` of the datagram being acked. |
| 8 | count | uint8 | Number of joint records that follow. |
| 9.. | values | int8 × count | Signed value dispatched to each joint, in the fixed joint order `thumb, index, middle, ring, pinky, wrist`. |

Each per-joint `int8` is in `[-100, 100]`: the value actually sent to the actuator that frame, where `-100` is that joint's extend posture, `0` its rest posture, and `+100` its flex posture. A held joint is reported as `0` (rest) by convention. Reference pack/unpack: `pack_continuous_ack()` / `unpack_continuous_ack()` in the SDK (`nml_hand_exo.interface._gesture_protocol`).

### Ack timing

The ack for a frame is emitted when the **device's serial reply for that frame arrives**, not when the receiver first writes the actuator command. It therefore attests that the exo *answered* the frame, not merely that the host sent it. The reference receiver dispatches exactly one `set_finger_angles` command per accepted datagram and, because the dual USB-CDC link drains replies on a background thread, the reply for one frame never delays the command write for the next. Each accepted frame is queued and the oldest queued frame is retired (and acked) as each solicited device reply is drained. The command reply (`OK: finger_angles ...`) is solicited and retires a frame; the asynchronous `GESTURE_RESULT:` move-outcome line is not, and does not.

If the host briefly outruns the device's reply rate, the receiver bounds the queue and drops the oldest un-acked frame rather than growing without limit; a consumer sees this as a gap in the acked `sequence` values.

## Reference receiver

`continuous_udp_receiver.py` beside this module is the host-side reference implementation. It decodes each datagram with the same strict validation, resolves `channel_names` (case-insensitively, with a small alias table) to the exo's joints, drives the hand from one batch command per frame, and returns the `NGA3` ack above.

Each `[-1, 1]` value is scaled to a signed integer in `[-100, 100]` by `round(value * 100)` and clamped. The firmware anchors that signed value at each joint's calibrated `rest` posture: `-100` is the joint's `extend` posture, `0` its `rest` posture, `+100` its `flex` posture, interpolating rest<->flex above 0 and rest<->extend below 0, per motor. So the three anchors reproduce those postures exactly even on a multi-motor gesture, and `0` lands on the calibrated rest even when rest is not the travel midpoint. Fingers no channel drives are held at `0` (rest) every frame, so a decoder sending fewer channels than the hand has joints still leaves the rest in a known neutral pose.

`set_finger_angles:<thumb>:<index>:<middle>:<ring>:<pinky>[:<wrist>]` is a firmware batch form of `set_gesture_angle`, taking signed `[-100, 100]` fields as above (firmware >= 0.6.4), so the whole vector travels in one serial write and one reply instead of one per joint. An empty field holds that joint unchanged. This command is **required**: the reference receiver has no per-joint fallback, so startup aborts if the device reports a firmware version older than 0.6.4 (or no version). One command per frame is what makes the one-reply-per-frame ack correlation above exact.

To keep the console readable at 60 ms per frame, the reference receiver logs accepted datagrams in batches: one averaged line per `--print-every` (default 10) accepted frames. Rejects, stale drops, and watchdog trips still print immediately.

## Suggested sender change: ack-gated flow control

The `v1` bridge sends on a fixed stride (every Nth decoder sample, one datagram per 60 ms) regardless of whether the receiver and exo can keep up. Because `v2` gives the sender an ack per accepted frame — emitted only once the exo has actually answered that frame — the bridge can instead pace itself to the exo's real throughput.

Suggested scheme (credit-based, one outstanding frame):

1. The bridge starts with one send **credit**.
2. It may send a datagram only while it holds a credit; sending consumes it.
3. A credit is restored by either an incoming `NGA3` ack (the exo answered the last frame) **or** an explicit manual/parameter trigger, whichever the operator selects.
4. With no credit, the bridge holds the newest decoder sample and sends it as soon as a credit returns, rather than sending on the fixed stride.

This makes the send rate self-tuning: when the exo answers a frame in well under 60 ms, the next frame goes out immediately and throughput rises above the fixed `1000/60 ≈ 16.7` Hz ceiling; when the exo is slow or a datagram is lost, the bridge naturally backs off to the exo's real rate instead of piling up frames the receiver would only drop. The manual-trigger option keeps a single-step debugging mode. The datagram format is unchanged — this is purely a change to *when* the existing bridge is permitted to send, and it requires the bridge to read the `NGA3` acks on its source socket (which the current send-only bridge does not yet do). A watchdog on the ack side (fall back to timed sending, or to a safe hold, if no ack returns within a bound) keeps a lost final ack from stalling the stream permanently.
