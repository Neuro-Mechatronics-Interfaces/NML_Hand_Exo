# EMG latent-intent decoder: incremental architecture

## Decision

Add a **prototype-mixture latent-intent mode** to `handexo emg-centroid`. It keeps rest as the origin and estimates (1) effort magnitude, (2) a continuous activation for every calibrated gesture prototype, and (3) a confidence/unknown state. It is the recommended first increment because it reuses the current windowing, features, rest capture, orientation tags, and centroid-calibration workflow. It also leaves the existing signed scalar available unchanged for comparison and backward compatibility.

Do not initially train a separate finger/DOF regressor. That is the useful second model once calibration includes reliable continuous finger labels.

## Current state (verified in source)

`handexo emg-centroid` starts `EmgCentroidDecoderGUI` (`src/handexo_cli.py:23-31`). Its live pipeline is:

```
LSL EMG -> latest window -> preprocessing -> RMS + common-mode removal
                                   |-> covariance -> optional Riemannian tangent feature
                         -> optional nearest forearm-roll-bin decoder
                         -> rest-relative signed close/open scalar in [-1, 1]
                         -> one-channel LSL UserIntent sample
```

- `_tick()` in `emg_centroid_decoder_gui.py:2015-2133` computes the RMS feature, always retains a covariance matrix, and uses the fitted Riemannian extractor when available.
- `CentroidDirectionDecoder.fit()` (`:204-259`) records rest/flex/extend means, uses `normalize(c_flex - c_extend)` as one direction, and derives a 95th-percentile rest-residual gate and scalar scale. `project_signed()` (`:261-280`) is necessarily one-dimensional.
- `OrientationGatedDecoder` (`:296-372`) fits one such decoder per populated 5-degree roll bin and otherwise uses the global decoder; it selects the nearest populated bin rather than interpolating models.
- `_push_intent_sample()` (`:1959-1997`) currently publishes **one float** at 20 Hz through LSL (`UserIntent`).

The HandExo GUI is already the correct actuator boundary. `UDPCommandWorker` (`hand_exo_gui.py:1227-1291`) receives UDP, and `_on_udp_command()` (`:2234-2406`) validates a restricted command set. Direct `set_velocity` and `set_current` commands require an active explicit ID, a matching configured mode, and that exact ID to be armed; it coalesces streams to newest-per-motor. The direct-control UI configures a watchdog and arms motors by integer ID (`:4122-4355`).

**Implementation status (2026-07-30).** The centroid GUI now retains the legacy scalar LSL outlet and publishes a parallel `NMLIntentV1` outlet. The HandExo GUI now has an opt-in LSL intent receiver and guarded single-ID velocity mapper; it is the in-process bridge, not a third GUI and not a UDP client. No firmware change is necessary.

**Continuous two-direction status (2026-08-10).** The separate `handexo
emg-intent` workflow now publishes a continuous signed open/close value using
rest-to-active-reference shrinkage-LDA projections. Rest noise is anchored at its
directional 95th percentile and the 90th percentile of recorded comfortable
gesture effort anchors the control reference. This implements the
bounded two-direction subset of the architecture below; multi-prototype mixture
output and graded-effort validation remain future work.

## Why the signed axis is insufficient

It remains a good low-burden binary open/close baseline: it is interpretable, requires only three captures, and has a useful rest gate. It cannot represent orthogonal intent, however. A partial index activation, pinch, or equal ambiguity between nearby gestures is projected onto the same line and becomes an apparently decisive open/close value. The scalar also conflates *which* intent was seen with *how strongly* it was expressed.

## Candidate designs

| Design | Strength | Limitation | Recommendation |
|---|---|---|---|
| Gesture-simplex/prototypes | Reuses centroid captures; emits mixtures and unknown; modest calibration burden | Outputs gesture components, not anatomically independent fingers | **Implement first** |
| Finger/DOF latent regression | Directly maps to controllable groups and supports genuinely mixed fingers | Needs synchronized continuous labels or carefully scripted graded trials; greater overfit risk | Collect data now; implement after validation |
| Hierarchical hybrid | Separates rest/quality/broad mode from fine continuous intent; safest eventual control design | Adds policy and calibration complexity | Use its safety/confidence layer around the prototype first implementation |

## Recommended model

### Feature and geometry

Keep the selectable, already implemented feature representation: default common-mode-removed per-channel RMS; optional covariance projected to a tangent space at the pooled Riemannian mean. Fit per orientation model only when each class has enough coverage; otherwise use global model and lower confidence.

For rest centroid `c0` and each non-rest prototype `ci`, define

```
di = (ci - c0) / ||ci - c0||
z  = x - c0
r  = ||z||
qi = z^T Wi di / sqrt(di^T Wi di)       # signed directional evidence
ai = max(0, qi / si)                    # prototype activation
```

`Wi` starts as identity, so this is the current feature-space geometry. Where sample counts support it, use a regularized pooled inverse covariance `W = (Sigma + lambda I)^-1`; this is a Mahalanobis whitening step, not a new firmware/device dependency. `si` is a robust training scale (e.g., 95th-percentile positive projection for prototype `i`). Preserve effort separately:

```
effort = clip((r - r_rest95) / (r_active95 - r_rest95), 0, 1)
p_i = softmax((a_i + log prior_i) / T)       over calibrated non-rest prototypes
```

Fit temperature `T` by held-out negative log likelihood where calibration data are sufficient; otherwise expose a conservative fixed default and label the values *normalized activations*, not calibrated probabilities. Do not apply a softmax to low-effort/rest windows: set the intent state to `rest` and retain `p_i` only for visualization.

Confidence should combine effort, prototype agreement, and distance-to-manifold:

```
margin = p_top1 - p_top2
recon  = ||z - sum_i clip(qi, 0, inf) di||
confidence = effort * clamp(margin / margin_ref, 0, 1)
             * exp(-recon^2 / (2 sigma_recon^2)) * orientation_coverage
```

The output activation vector is `u_i = effort * p_i` only when confidence exceeds its configured floor; otherwise it is all zero with state `rest` or `unknown`. This explicitly makes ambiguous mixtures visible without converting weak/noisy EMG into confident motion.

### Mixed gestures and future DOFs

For a pinch-like sample, multiple `u_i` values can be non-zero. The first implementation maps only a **pre-approved, non-conflicting** prototype mixture to an actuator synergy. It must not sum opposite actions on the same joint. The mapping is a bounded matrix `M` from prototype activations to named joint groups, followed by per-group clipping and calibration-limit enforcement:

```
v_group = clip(M u, -1, 1)
```

For the later finger/DOF decoder, replace `u` with a regularized multi-output regression `v = B phi(x)` trained from continuous labels (tracked finger angle, scripted target level, or a validated glove). Retain the same rest, confidence, temporal, and actuator boundary layers.

## Temporal policy

Run at the current 20 Hz initially. Apply an EMA to `u` (100-200 ms time constant) only after the confidence gate. Enter `active` after confidence is above threshold for 150-250 ms; return to rest after a lower threshold for 200-300 ms. Use separate enter/exit thresholds and hold the last *zero-safe* output during `unknown`; do not hold a nonzero motor command. A gesture label can use dwell (e.g. 300 ms) for UI feedback, while continuous activation should not wait for label dwell.

Orientation gating remains optional. Prefer blending the two nearest populated orientation models by circular roll distance rather than abruptly switching bins. If IMU is stale, unavailable, or outside recorded coverage, use the global model with an explicit coverage penalty; never silently claim a high-confidence orientation-specific decode.

## Output and control boundary

Publish a versioned, continuous intent record beside the legacy one-float LSL stream. An LSL outlet can carry a fixed numerical vector; the explicit schema belongs in StreamInfo metadata. For a bridge/UDP message, use JSON:

```json
{
  "schema":"nml.intent.v1",
  "timestamp_monotonic":1234.56,
  "sequence":812,
  "state":"active",
  "effort":0.64,
  "confidence":0.78,
  "orientation":{"roll_deg":-12.4,"coverage":0.91},
  "activations":{"close":0.42,"open":0.00,"pinch_index":0.22},
  "joint_groups":{"index_flex":0.51,"thumb_flex":0.22},
  "legacy_signed":0.37
}
```

This is an **outbound intent command**. Exoskeleton angle/current/torque, participant report, and sensor streams are separate feedback/adaptation inputs; they are not the other direction of an EMG control command.

The HandExo GUI's EMG Teleop component is the only component that converts intent to actuator commands. Its implemented baseline maps to one selected, explicit active Dynamixel ID; future reviewed synergies may map `joint_groups` to several IDs. Never use a bare motor name: dual mode has duplicate names and firmware resolves a bare name to the left-side motor. Position-mode mapping must clamp every requested target to the active, calibration-derived joint limits before transmission. Direct velocity/current mapping must obey the GUI/firmware limits (10 rpm and 910 mA respectively) and the configured command watchdog.

Live control is opt-in and must require all of: connected actuator GUI, matching direct Velocity mode, an explicitly armed allowlisted ID, active calibration for that ID, a physical/software deadman held, fresh decoder samples, and `state == active` with confidence above threshold. On deadman release, decoder stale timeout, low confidence, rest, unknown, disconnect, or mapping error, transmit `stop:<id>` for each previously commanded ID and cease nonzero commands. Never use `disable:all` as a normal teleop action. Do not raise the documented 910 mA limit.

## Calibration and validation

Retain rest, close, and open recordings, but capture multiple comfortable effort levels and repetitions. Add optional prototype recordings only for actions that have a reviewed actuator synergy: e.g. index-dominant close, thumb/index pinch, and whole-hand close. Record rest before/after every block; record the same posture/orientation coverage for every class when orientation gating is enabled. For a later DOF decoder, add synchronized target level or reference kinematics for each controllable group—class names alone are not continuous labels.

Split repetitions, not adjacent windows, into train/test sets. Report per-class confusion, probability calibration, rest false-activation rate, time to active, mixture plausibility, orientation-bin coverage, and every safety-gate rejection. Do not enable a prototype for live use unless its held-out confidence and its safe joint mapping are reviewed.

## Staged plan

1. **Visualization/offline validation.** Implement no actuator behavior. Display prototype rays, effort, activations, confidence, rest/unknown state, orientation coverage, and replay metrics next to the legacy scalar.
2. **Parallel intent stream.** Completed for the baseline contract: `NMLIntentV1` is published alongside the legacy scalar stream. Preserve legacy behavior by default.
3. **Safe direct-control mapping.** The first opt-in implementation maps to one explicit armed ID with direct Velocity mode, calibrated profile, deadman, freshness, confidence, and stop-on-fault gates. Extend only after reviewed multi-motor synergy maps and log/replay validation.
4. **Personalization/adaptation.** Add conservative rest drift correction only during verified rest; later evaluate recalibration prompts, per-user whitening/temperature, and supervised adaptation from explicit user labels.

## Open questions and risks

- Which gestures have clinically meaningful, non-conflicting joint synergies? The mapping must be reviewed before live control.
- Multi-motor synergy maps remain unimplemented by design. Their joint directions, limit margins, and participant-facing behavior require review before live use.
- Riemannian features may improve robustness but need more data and can make geometry less transparent; RMS+CMR should remain the reference baseline.
- Adaptive rest can absorb weak intended contractions. Restrict it to verified resting/deadman-released periods and record every update.
- Calibration distributions drift with electrode shift, fatigue, posture, and spasticity. A low-confidence/unknown decision is a required safe outcome, not a failure to be forced into a gesture.

