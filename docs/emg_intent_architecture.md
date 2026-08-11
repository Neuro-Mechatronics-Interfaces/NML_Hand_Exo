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
| `pipeline.py` | Confidence rejection and continuous rest-to-MVC open/close mapping |
| `contracts.py` | Stable decisions, orientation samples, and evaluation records |

The GUI orchestrates those functions, receives EMG and optional IMU through
separate LSL streams, and publishes `NMLIntentV1` only when the operator starts
publishing. The hand-exo GUI remains responsible for explicit motor selection,
arming, current/velocity limits, telemetry, watchdog behavior, and stopping.
The new and legacy decoders use the same source ID expected by the hand-exo GUI;
run only one decoder publisher at a time.

## Participant workflow

1. Connect EMG and verify channel quality.
2. Optionally connect the forearm IMU. Fresh IMU samples select the learned
   orientation-conditioned rest baseline. With no IMU, or when IMU samples are
   stale, decoding continues with the fitted global EMG rest baseline.
3. Record repeated rest segments and comfortable voluntary attempts with the
   Task GUI markers and LabRecorder, then build or load the decoder NPZ on the
   **Session Data** tab.
4. Rank candidate pairs with repetition-held-out cross-validation.
5. Choose which candidate maps to open and close, then fit the final decoder.
6. Monitor predictions before explicitly starting `NMLIntentV1` publishing.

## Continuous rest-to-MVC output

The selected open and close recordings are treated as participant-specific MVC
references. The shrinkage-LDA model still chooses direction, while two
one-vs-rest Fisher/LDA projections estimate normalized magnitude. For each
direction, the 95th percentile of resting projection noise maps to 0 and the
median recorded MVC projection maps to 1. Values are clamped before publishing:

```text
maximum open MVC = -1     rest = 0     maximum close MVC = +1
```

Direction comes from the relative open/close LDA probabilities; magnitude comes
from the matching rest-to-MVC projection. Activations are not subtracted because
related forearm gestures can cross-activate both one-vs-rest axes. Confidence is
the probability support for rest plus the stronger direction, so legitimate
partial contractions can pass through the rest-to-MVC transition while reject
or opposing-direction ambiguity resolves to zero.

`NMLIntentV1` remains four channels: continuous `signed_intent`, its absolute
`effort`, confidence, and active state. This is normalized EMG activation, not a
force estimate. The present recordings validate rest and MVC endpoints; a later
graded-effort protocol is required to test physiological linearity at 25%, 50%,
and 75% voluntary effort.

The **Monitor and Run** tab reuses the older decoder's PyQtGraph number-line
visualization. Fitted rest/open/close windows appear as jittered class rows on
the normalized -1 to +1 axis, diamond markers show their medians, and a yellow
line/marker shows the live value. The computation readout reports LDA open/close
probabilities, both normalized rest-to-MVC activations, the selected direction,
and the exact signed value published through LSL.

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
- A model trained with orientation uses compensation while fresh orientation is
  available and otherwise uses its fitted global rest baseline.
- Stale EMG forces a zero-intent output. Missing or stale IMU falls back to the
  global EMG baseline and does not stop decoding.
- Stopping or disconnecting destroys the LSL outlet.
- The decoder does not bypass exo-side arming or command watchdogs.
