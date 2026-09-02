# Physics-pipeline data contract

## Scope

This contract supports synchronized EMG, exoskeleton state, commands, task
markers, and optional anatomical hand kinematics. It does not define a clinical
protocol or claim that motor current is human joint torque.

## Source recording

LabRecorder XDF is the immutable source recording. A JSON manifest stores static
session, participant, geometry, calibration, and provenance fields. Processed
NPZ files are reproducible derivatives and never replace the XDF.

## Required streams

- MindRove EMG: raw forearm EMG with explicit channel ordering.
- `NML_TaskMarkers`: irregular string markers.

## Optional streams

- `NMLHandExoStateV1`, schema `nml.hand_exo.state.v1`.
- `NMLHandExoCommandV1`, schema `nml.hand_exo.command.v1`.
- `NMLHandExoEventsV1`, schema `nml.hand_exo.events.v1`.
- `NMLHandKinematicsV1`, schema `nml.hand_kinematics.v1`.
- `NMLIntentV1`, the existing decoder baseline.

## Complete exoskeleton state

The v1 state stream begins with frame sequence, firmware timestamp, and fast-read
flags. Each motor then contributes relative angle, absolute angle, encoder ticks,
velocity, present current, current-derived motor torque, and a validity flag.
Every motor channel contains side, bare motor name, and integer DXL ID.

`estimated_motor_torque_from_current_Nm` is a motor-side estimate using the
configured torque constant. It is not interaction torque, anatomical joint
torque, or participant-generated torque.

## Host-request command stream

`NMLHandExoCommandV1` is a fixed-width numeric snapshot repeated at the
measured-state cadence and immediately after observed command changes. Fields
are named `requested_*`: they are commands transmitted by the host, not
authoritative Dynamixel goal-register readback. Firmware gesture expansion,
clamping, watchdogs, current budgeting, and communication failures can make the
applied result different. Unknown or inapplicable values remain NaN.

The stream includes requested control mode, command source, watchdog and current
budget settings, plus per-motor requested angle, velocity, current, limits,
enable state, and direct-command activity where those quantities are explicit
on the wire. A semantic `set_gesture` event does not invent per-motor goals.

`NMLHandExoEventsV1` is an irregular JSON string stream. It preserves command
send/acknowledgement/failure observations, connection lifecycle, and explicit
safety-stop events. Event observation is non-invasive and cannot block control.

## Synchronization

Processed EMG windows use their center LSL timestamp. Numeric state is linearly
interpolated only when a measured sample lies within `max_state_age_s`; otherwise
the row is marked invalid and its state values are NaN. Validation splits use
whole trials or recordings, never neighboring overlapping windows.

Command snapshots use zero-order hold from the latest preceding sample, never
linear interpolation. Their freshness is bounded independently by
`max_command_age_s`. Event JSON and event timestamps are preserved without
resampling.

## Unknown values

Unknown geometry, sign conventions, assistance levels, or sensor semantics must
remain explicit `unknown`/`TO_BE_SPECIFIED` fields. They must not be inferred from
filenames or silently assigned by import code.
