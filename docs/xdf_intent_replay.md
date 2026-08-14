# XDF intent-session import and replay

The repository can import event-marked MindRove XDF files into the decoder's
reloadable session format. It does not bundle LabRecorder or a MindRove-to-LSL
playback application; those are optional external acquisition tools.

## Import a session

```powershell
python tools/import_xdf_intent_session.py --help
python tools/import_xdf_intent_session.py `
  path\to\participant\exp001 `
  data\intent_sessions\participant_exp001.npz `
  --participant participant
```

For file-level holdout validation, exclude one or more recordings from the
training session and import the holdout separately:

```powershell
python tools/import_xdf_intent_session.py `
  path\to\participant\exp001 `
  data\intent_sessions\training.npz `
  --exclude held_out_recording.xdf

python tools/import_xdf_intent_session.py `
  path\to\participant\exp001\held_out_recording.xdf `
  data\intent_sessions\holdout.npz

python tools/evaluate_intent_session.py `
  data\intent_sessions\training.npz `
  data\intent_sessions\holdout.npz
```

A same-session file holdout tests recording-level robustness, not independent
day or participant generalization.

## Load and monitor

Start `handexo emg-intent`, open **Session Data**, load the training NPZ, rank
candidate pairs, select the intended open/close mapping, fit, and monitor before
publishing. The decoder can fit EMG-only data; orientation compensation requires
a matching live IMU stream.

## Optional LSL replay

Any compatible external XDF player may be used if it publishes the expected
split streams:

- `MindRove_EMG`, type `EMG`, eight channels, approximately 500 Hz.
- `MindRove_IMU`, type `IMU`, nine channels, when orientation is available.

Select the **MindRove XDF playback** preset for split outlets. Its EMG indices
are `0-7`. The live combined `MindRoveStream` has a leading status channel and
uses EMG indices `1-8`; do not interchange those mappings.

`tools/build_alternating_xdf_playback.py` can concatenate marked open/close XDF
segments into a generic recording NPZ for a compatible player. That recording
NPZ is not an intent-session NPZ and cannot be loaded directly in Session Data.

## Guarded exoskeleton check

First test playback on a bench with no participant wearing the device:

1. Confirm the connected side and every expected integer Dynamixel ID.
2. Apply and inspect the correct participant calibration.
3. Connect the decoder's `NMLIntentV1` outlet with teleoperation stopped.
4. Select and arm only the intended IDs.
5. Begin at 1–2 rpm with the firmware watchdog enabled.
6. Verify sign, confidence, stale-input behavior, and motion away from limits.
7. Confirm both the host stop control and firmware watchdog command zero.

Never use a bare motor name, disable all motors indiscriminately, or increase
speed/current to force motion at a joint limit.
