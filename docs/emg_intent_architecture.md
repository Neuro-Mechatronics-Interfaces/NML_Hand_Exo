# EMG intent discovery architecture

The participant-facing decoder is launched with:

```powershell
handexo emg-intent
```

`handexo emg-intent-decoder` is an equivalent long-form alias. The existing
`handexo emg-centroid` application remains available for the original quick
two-state workflow.

## Module boundaries

The decoder is intentionally separate from exoskeleton control. Modules under
`src/nml_hand_exo/decoding` accept arrays and return data contracts; none send
serial commands or enable motor torque.

| Module | Responsibility |
| --- | --- |
| `layout.py` | Device channel mapping for 8-channel and 128-channel inputs |
| `preprocessing.py` | Sample-rate-aware EMG band-pass and line-noise filtering |
| `features.py` | Common-mode removal, RMS features, and signal-quality checks |
| `orientation.py` | IMU angles and continuous orientation-conditioned rest subtraction |
| `models.py` | Shrinkage-LDA classification behind a small model interface |
| `selection.py` | Group-held-out candidate-pair ranking and false-activation metrics |
| `session.py` | Capture records and atomic, reloadable session files |
| `pipeline.py` | Confidence rejection and continuous rest-to-active-reference mapping |
| `contracts.py` | Stable decisions, orientation samples, and evaluation records |

The GUI orchestrates those functions, receives EMG and optional IMU through
separate LSL streams, and publishes `NMLIntentV1` only when the operator starts
publishing. The hand-exo GUI remains responsible for explicit motor selection,
arming, current/velocity limits, telemetry, watchdog behavior, and stopping.
The new and legacy decoders use the same source ID expected by the hand-exo GUI;
run only one decoder publisher at a time.

### MindRove channel contracts

The live combined `MindRoveStream` begins with a package/status channel. Its
EMG channels are `1-8`, accelerometer channels are `9-11`, and gyroscope
channels are `12-14`. The XDF playback application publishes already-split
`MindRove_EMG` and `MindRove_IMU` streams, so those mappings start at zero
(`0-7`, `0-2`, and `3-5`). Training and live decoding must use the same eight
EMG channels in the same order.

## Participant workflow

1. Connect EMG and verify channel quality.
2. Choose the orientation mode before ranking and fitting. The default global
   EMG baseline needs no IMU. Orientation compensation is opt-in; when selected,
   fitting uses recorded orientation and runtime requires fresh live IMU.
3. Record repeated rest segments and comfortable voluntary attempts with the
   Task GUI markers and LabRecorder, then build or load the decoder NPZ on the
   **Session Data** tab.
4. Rank candidate pairs with complete-recording-held-out cross-validation.
5. Choose which candidate physically maps to open and close, confirm that
   semantic mapping explicitly, then fit the final decoder. Pair ranking does
   not assign actuator meaning by itself.
6. Monitor predictions before explicitly starting `NMLIntentV1` publishing.

## Continuous rest-to-active-reference output

The selected open and close recordings are treated as participant-specific,
comfortable-effort control references. They are not assumed to be maximum
voluntary contractions. The shrinkage-LDA model still chooses direction, while two
one-vs-rest Fisher/LDA projections estimate normalized magnitude. For each
direction, the 95th percentile of resting projection noise maps to 0 and the
90th percentile of the recorded active projection maps to 1. Values are shaped
by the operator-selected gain and response exponent, then clamped before publishing:

```text
open effort reference = -1     rest = 0     close effort reference = +1
```

Direction comes from the relative open/close LDA probabilities; magnitude comes
from the matching rest-to-reference projection. Activations are not subtracted because
related forearm gestures can cross-activate both one-vs-rest axes. Confidence is
the probability support for rest plus the stronger direction, so legitimate
partial contractions can pass through the rest-to-reference transition while reject
or opposing-direction ambiguity resolves to zero.

`NMLIntentV1` remains four channels: continuous `signed_intent`, its absolute
`effort`, confidence, and active state. This is a normalized control signal, not
a force estimate or percentage MVC. The present recordings validate rest and
comfortable-effort endpoints; a later
graded-effort protocol is required to test physiological linearity at 25%, 50%,
and 75% voluntary effort.

The **Monitor and Run** tab reuses the older decoder's PyQtGraph number-line
visualization. Fitted rest/open/close windows appear as jittered class rows on
the normalized -1 to +1 axis, diamond markers show their medians, and a yellow
line/marker shows the live value. The computation readout reports LDA open/close
probabilities, both normalized rest-to-reference activations, the selected direction,
and the exact signed value published through LSL.

Live output passes through a causal stabilizer with exponential smoothing, a
per-update slew limit, separate enter/release thresholds, and a three-sample
direction-switch confirmation. This suppresses isolated opposite-direction
windows without turning the continuous command into a discrete classifier.

Validation holds out complete source recordings rather than individual trials
or overlapping windows. Adjacent windows and rest segments from one XDF file
therefore cannot appear in both training and validation folds.

### Synthetic sine test

After fitting a model, **Synthetic intent test** becomes available and can
temporarily replace live decoding with a bounded sine wave. Starting the test
creates the `NMLIntentV1` outlet automatically if it is not already publishing. The
conservative defaults are amplitude 0.25 and a 10-second open/close cycle;
amplitude is limited to 1.0 and the period cannot be shorter than 4 seconds.
The waveform starts at zero, uses the normal four-channel LSL contract, and
publishes zero when stopped. It does not bypass the hand-exo GUI's explicit
target selection, arming, per-motor current/velocity limits, watchdog, joint
limits, or STOP controls. Stop the sine test before assessing live EMG intent.

### Exoskeleton power-grasp workflow

For a right-hand power grasp with the thumb rotated into opposition:

1. In **Advanced**, apply Velocity mode. All motors begin torque-off.
2. In **Setup > Position and Hold**, select `thumbrot` (right ID 14; left ID 4).
   Either place the torque-off joint at the desired participant-specific pose
   and click **Hold Current Position**, or enter a calibrated target and click
   **Move & Hold**. Basic hold requires firmware 0.6.2 or newer; adjustable
   per-hold current and verified applied-current feedback require the current
   development firmware build.
3. Confirm that the motor row reads `HOLD <angle>`. The same state appears as a
   compact `AUX HOLD` indicator in Operate; clicking it returns to Setup.
4. In **Advanced**, choose the Power Grasp preset and apply IDs 15-19. The held
   thumb-rotation ID is unchecked, disabled in the DIRECT arming list, and
   cannot also be included in the EMG target.
5. Connect the `NMLIntentV1` LSL stream and satisfy the calibration/firmware
   safety-envelope check.
6. Start EMG teleop. An already-active hold remains active; a configured hold
   that was released by the previous STOP is re-engaged before finger commands.

Neutral or stale intent zeros the finger commands but keeps the auxiliary hold.
STOP TELEOP, STOP ALL MOTION, a control-mode change, or disconnect releases the
hold and disables the held motor. STOP TELEOP retains the configured angle for
the next start; **Release Hold** in Setup also removes that reservation.

Session files retain the raw EMG windows, derived features, class labels,
repetition groups, continuous roll/pitch values, participant/device metadata,
and can be reloaded for later analysis or alternative feature extraction.
Formal participant validation reports and continuous source-stream archival
remain follow-on work; the session file does not replace the source recording
required by a research protocol.

## Runtime fail-safe behavior

- Low-confidence or directionally ambiguous predictions resolve to rest and zero signed intent.
- Global-baseline models decode without IMU. Orientation-compensated models
  require fresh IMU and publish zero if it becomes unavailable.
- Stale EMG forces a zero-intent output. Missing or stale IMU matters only when
  the operator explicitly selected orientation compensation.
- Stopping or disconnecting destroys the LSL outlet.
- The decoder does not bypass exo-side arming or command watchdogs.
