# Modeling workflow

## Modeling ladder

1. EMG-only grouped LDA baseline.
2. EMG plus measured exoskeleton state.
3. Regularized linear state-space identification.
4. Reduced second-order physics model with explicit inertia, passive stiffness,
   damping, EMG-to-torque map, and exoskeleton torque.
5. Physics prediction plus a learned residual, after the physics-only model has
   been evaluated.

## Coordinate policy

No anatomical coordinate is hard-coded. A `ReducedGeometry` must specify:

- coordinate names and units;
- valid ranges;
- explicit motor IDs;
- motor-to-coordinate signs and transmission ratios;
- confidence/provenance for each mapping.

Start with identifiable reduced coordinates, such as one functional flexion
coordinate per digit or a whole-hand synergy. Eight forearm EMG channels do not
justify claiming independent identification of every anatomical muscle force.

## Identification conditions

- Empty exoskeleton movement identifies actuator/transmission effects.
- Relaxed worn movement identifies combined passive hand/exoskeleton effects.
- Isometric attempts estimate posture-conditioned EMG-to-balancing torque.
- Transparent dynamic trials identify motion dynamics.
- Assisted trials test transfer under human–robot coupling.

## Validation

All overlapping windows from one trial stay in the same split. Report separate
leave-trial, leave-recording, leave-posture, and leave-assistance results.
The decisive comparison is whether state/physics conditioning improves an
unseen posture or assistance level over the EMG-only baseline.

Requested command channels are zero-order-held exogenous inputs only when the
relevant `requested_*` value is finite. Do not replace semantic gesture events
or unknown per-motor requests with guessed firmware goals. Before claiming a
physics result, report whether inputs came from host requests, measured current,
or future authoritative firmware goal-register telemetry.
