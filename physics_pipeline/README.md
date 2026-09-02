# NML EMG–Exoskeleton Physics Pipeline

This isolated package adds recording contracts, XDF tools, synchronization,
baseline models, and reduced physics/state-space scaffolding around the existing
NML Hand Exoskeleton control software. It does not command hardware.

## Current commands

Inspect an XDF:

```powershell
python -m physics_pipeline.xdf_inspect path\recording.xdf
```

Replay an XDF through isolated LSL stream names:

```powershell
python -m physics_pipeline.xdf_replay path\recording.xdf --prefix REPLAY_ --mindrove-split
```

`--mindrove-split` additionally publishes the known MindRove channel views used
by the decoder: eight EMG channels (`1:9` of the combined recording) and six
accelerometer/gyroscope channels (`9:15`). The original combined stream is also
replayed. The default `REPLAY_` prefix prevents collision with live hardware;
select the prefixed stream explicitly in the decoder.

Build a synchronized processed session from the current combined MindRove
stream and task markers:

```powershell
python -m physics_pipeline.xdf_import path\recording.xdf data\processed_session.npz
```

Compare EMG-only and state-conditioned intent baselines using whole-trial
grouped validation:

```powershell
python -m physics_pipeline.evaluate_session data\processed_session.npz
```

Recordings containing `NMLHandExoStateV1`, `NMLHandExoCommandV1`, and
`NMLHandKinematicsV1` are aligned automatically. State and kinematics are
interpolated with freshness checks; requested commands use zero-order hold.
`NMLHandExoEventsV1` JSON and timestamps are preserved without resampling.
Missing optional streams remain zero-column arrays with validity set false.

## Safety and interpretation

- This package does not enable motors or issue exoskeleton commands.
- The existing GUI remains the actuator and safety boundary.
- Command channels describe host-transmitted requests, not authoritative
  firmware goal registers.
- Current-derived motor torque is labeled as an estimate and must not be called
  participant or anatomical joint torque.
- Participant-specific geometry and sign conventions remain configuration.
- Use replay and synthetic tests before any participant-facing experiment.

See [docs/data_contract.md](docs/data_contract.md) for the stream contract and
`protocols/physics_calibration_template.json` for an intentionally incomplete
protocol template whose hardware-dependent fields must be supplied explicitly.
The operator sequence is in [docs/recording_workflow.md](docs/recording_workflow.md),
and the model ladder is in [docs/modeling_workflow.md](docs/modeling_workflow.md).
