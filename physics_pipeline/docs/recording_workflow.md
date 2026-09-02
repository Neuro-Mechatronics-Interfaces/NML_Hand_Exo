# Recording workflow

This workflow is an engineering acquisition checklist. Participant effort,
assistance, stopping criteria, and safe transparent/isometric modes still
require study-specific approval.

## Before LabRecorder

1. Start the MindRove LSL source and verify the eight EMG channels.
2. Start `handexo gui`, connect the correct side, and apply the correct
   participant calibration.
3. In **Settings → Telemetry Sampling and LSL**, enable LSL and leave
   **Publish complete state (v1)**, **Publish requested commands (v1)**, and
   **Publish command/events (v1)** checked.
4. Select a telemetry rate supported by the connected hardware. The complete
   state stream publishes on every acquired fast-telemetry frame.
5. Start `nml-task-cue`, load a reviewed physics prompt plan, and verify that
   hardware-dependent fields no longer contain `TO_BE_SPECIFIED`.
6. Start any independent anatomical-angle LSL producer, if available.

## LabRecorder stream checklist

Required:

- MindRove EMG source
- `NML_TaskMarkers`

Expected for exoskeleton-on modeling:

- `NMLHandExoStateV1`
- `NMLHandExoCommandV1`
- `NMLHandExoEventsV1`

Optional:

- `NMLHandKinematicsV1`
- `NMLIntentV1`
- legacy `NMLHandExoJointAngles` and `NMLHandExoMotorTorque`

Confirm that each source appears exactly once. Do not record a prefixed replay
stream in place of live data unless the session is explicitly a bench replay.

## State-stream interpretation

The complete state stream records actual encoder/current feedback. Torque is
computed as present motor current multiplied by the configured XC330-T288 motor
torque constant. It does not isolate linkage friction, passive hand resistance,
interaction force, or participant-generated torque.

The command stream records host-transmitted requests. Confirm that command
channels are named `requested_*`; they are not firmware goal-register feedback.
Unresolved gesture goals should remain NaN. The event stream should contain a
`connection` event and any commands exercised during the pre-recording check.

## Immediately after recording

1. Stop the prompt task and LabRecorder.
2. Preserve the XDF without editing it.
3. Create the sidecar manifest with firmware, calibration, geometry, electrode,
   and condition information.
4. Run `python -m physics_pipeline.xdf_inspect recording.xdf`.
5. Check stream presence, sample rates, timestamp reversals, gaps, and labels.
6. Import a derivative with `python -m physics_pipeline.xdf_import`.
7. Review the valid fractions before fitting any model.
8. Confirm that `command_valid_fraction` is nonzero for exoskeleton-on trials
   and inspect failed/rejected events before model fitting.

## Bench replay

Use the supplied reference recording without hardware commands:

```powershell
python -m physics_pipeline.xdf_replay `
  C:\Users\jonat\Documents\temp\jonathan_mindrove\exp001\jonathan-handclose_1.xdf `
  --prefix REPLAY_ --mindrove-split
```

The replay utility only publishes LSL samples. It does not connect to or command
the exoskeleton.
