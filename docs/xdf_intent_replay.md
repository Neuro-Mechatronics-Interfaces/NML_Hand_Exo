# XDF intent replay and guarded exoskeleton test

This workflow replays an event-marked MindRove XDF file into LSL, runs the
participant-specific intent decoder, and optionally sends its versioned intent
stream to the guarded explicit-ID motor or finger-group adapter in the HandExo GUI.

## Prepared holdout

- Replay: `jonathan-handclose_2.xdf`
- Training session: `data/intent_sessions/jonathan_exp001_without_handclose_2.npz`
- Final all-data session: `data/intent_sessions/jonathan_exp001_all_files.npz`
- Reference-only holdout import: `data/intent_sessions/jonathan_handclose_2_reference.npz`

The replay file was excluded from the training session. This is a same-day
file-level holdout, not an independent-session validation.

Use the excluded-file session when measuring replay performance. After that
check is complete, use the all-data session for the strongest final fitted
decoder; replaying a file included in that session is a functional test, not
an unbiased accuracy estimate.

## 1. Start XDF playback

From `C:\Users\jonat\Documents\GitHub\python-mindrove-emg`:

```powershell
.\.venv\Scripts\python.exe .\scripts\mindrove_lsl_streamer.py `
  --source playback `
  --playback-file "C:\Users\jonat\Documents\temp\jonathan_mindrove\exp001\jonathan-handclose_2.xdf"
```

Start playback with looping enabled. It publishes:

- `MindRove_EMG`, type `EMG`, 8 channels, approximately 500 Hz
- `MindRove_IMU`, type `IMU`, 9 channels

## 2. Run and fit the intent decoder

From this repository:

```powershell
$env:PYTHONPATH = (Resolve-Path .\src).Path
.\.venv\Scripts\python.exe -m nml_hand_exo.applications.emg_intent_decoder_gui
```

In the decoder GUI:

1. Select **MindRove XDF playback**.
2. Connect EMG and IMU.
3. Open **Session Data** and load
   `data\intent_sessions\jonathan_exp001_without_handclose_2.npz`.
4. Open **Select and Validate**, rank the pairs, and select
   `attempt_hand_open / attempt_hand_close`.
5. Map open to `attempt_hand_open`, close to `attempt_hand_close`, and fit.
6. Monitor the complete replay before enabling output.
7. Start publishing `NMLIntentV1` only after the monitor behavior is correct.

The decoder publishes zero intent for rest, low confidence, stale input, or a
missing required IMU stream.

## 3. Connect the HandExo GUI in monitor-only mode

```powershell
$env:PYTHONPATH = (Resolve-Path .\src).Path
.\.venv\Scripts\python.exe -m nml_hand_exo.applications.hand_exo_gui
```

The OpenRB currently enumerates as USB serial ports COM12 and COM15. Confirm
their command/telemetry roles in the connection panel rather than assuming the
ordering is stable.

1. Select the correct right/left/dual mode and connect.
2. Confirm firmware `0.6.1`, the expected build side, and plausible limits for
   every active integer DXL ID.
3. Load the participant calibration. The validated firmware-ROM fallback may
   be used only when the GUI explicitly marks it available.
4. In EMG Teleop, connect to source ID `nml-emg-centroid-intent-v1` and observe
   samples with teleop stopped.

## 4. Supervised low-speed motion and joint-limit test

Perform the first test on the bench with no participant wearing the device.

1. Choose **Velocity** direct mode and a 250 ms watchdog.
2. Select one explicit active integer ID; for a right index test use ID 16.
3. Set maximum EMG velocity to 1-2 rpm and arm only that ID.
4. Verify the intent direction before moving toward a limit.
5. Start teleop. The holdout alternates recorded rest and hand-close periods;
   rest must produce a stop and hand-close should produce the configured signed
   command only when confidence exceeds the threshold.
6. Approach the calibrated limit slowly. Within the firmware's 2-degree margin,
   a command farther into the limit must be written as zero and the background
   safety service must keep it stopped.
7. Use a low-speed manual command in the opposite direction to confirm motion
   away from the limit is still allowed.
8. Stop the XDF player or decoder and confirm the GUI stale-input gate and the
   firmware watchdog both return the command to zero.

Do not use `disable:all`, bare motor names, or a speed/current increase to force
motion at a limit. Stop immediately if the reported position or limits are not
plausible.

## Rebuild the imported sessions

```powershell
.\.venv\Scripts\python.exe .\tools\import_xdf_intent_session.py `
  "C:\Users\jonat\Documents\temp\jonathan_mindrove\exp001" `
  ".\data\intent_sessions\jonathan_exp001_without_handclose_2.npz" `
  --exclude jonathan-handclose_2.xdf

.\.venv\Scripts\python.exe .\tools\import_xdf_intent_session.py `
  "C:\Users\jonat\Documents\temp\jonathan_mindrove\exp001\jonathan-handclose_2.xdf" `
  ".\data\intent_sessions\jonathan_handclose_2_reference.npz"
```

Score the holdout offline:

```powershell
.\.venv\Scripts\python.exe .\tools\evaluate_intent_session.py `
  ".\data\intent_sessions\jonathan_exp001_without_handclose_2.npz" `
  ".\data\intent_sessions\jonathan_handclose_2_reference.npz"
```
