# Phase-1 Shadow Contact Characterization

This phase measures whether motor current, motion, intent, and joint-limit
distance can distinguish free motion from object contact. It does **not** alter
exo motion or switch control modes in response to its estimates.

## Safety boundary

- The firmware sampler is disabled at boot.
- It accepts explicit Dynamixel IDs only.
- It runs only in global velocity mode and stops automatically after a mode
  change.
- It performs only `PRESENT_CURRENT` and `PRESENT_POSITION` reads.
- The GUI's `free`, `candidate`, `contact`, `limit`, and `stale` labels are
  unvalidated observations. They never change a motor command.
- Use the existing current limits, velocity limits, joint limits, watchdog,
  arming, and stop controls exactly as before.
- Complete bench testing before asking a participant to wear the device.

## Before the first run

1. Flash the normal exo firmware containing the Phase-1 source. Codex does not
   flash the board.
2. Start the GUI and connect normally.
3. Verify the expected firmware version/build, motor IDs, joint limits, current
   limits, velocity ceilings, and emergency stop behavior.
4. Begin with a low GUI maximum velocity and conservative per-motor current
   limits.
5. Keep the hand off the participant for the initial characterization.

## Record a session

1. In **Operate > Intent Input**, connect the LSL decoder.
2. Select and arm the explicit finger IDs used for the power grasp.
3. Expand **Show advanced intent settings**.
4. Check **Record read-only shadow contact evidence (Phase 1)**.
5. Give the block a short **Shadow session label**, such as `free_close`,
   `foam_block`, or `rigid_block`.
6. Start EMG teleoperation normally.
7. Run short, labeled blocks of:
   - rest with no motion;
   - free opening and closing with no object;
   - gentle contact with a compliant block;
   - sustained contact at several voluntary intent levels;
   - intentional release/opening;
   - approach near a configured joint limit without pushing the mechanism into
     the limit.
8. Press **STOP TELEOP** after each block or session.

The GUI writes a timestamped CSV to:

```text
logs/shadow_contact/shadow_contact_YYYYMMDD_HHMMSS_LABEL.csv
```

The file contains the session label, firmware and host timestamps, selected ID,
raw current, relative angle, derived velocity, sample age, decoder
intent/confidence, commanded velocity, joint limits, estimator state, filtered
evidence, dwell, and cumulative firmware read errors.

## What to verify first

Before interpreting contact labels, verify instrumentation health:

- `firmware_sequence` advances continuously.
- `firmware_read_errors` stays constant or increases only rarely.
- `sample_age_ms` remains well below the 150 ms stale threshold.
- EMG commands remain responsive and the 250 ms direct-command watchdog does
  not fire.
- Enabling shadow recording does not introduce visible oscillation, command
  jitter, or serial disconnects.
- Velocity polarity matches the relative-angle trajectory for every selected
  digit.

For five selected IDs and the default 2 ms register-read interval, the nominal
complete update period is 20 ms per ID. This is a design target, not an assumed
measurement; use the CSV timestamps and error count to determine the actual
rate on the physical bus.

## Evidence needed before Phase 2

Collect enough trials to answer these questions per digit:

1. How much current is required for free motion at different speeds and joint
   angles?
2. Does object contact create a repeatable current increase while derived
   velocity falls?
3. Can joint-limit approaches be separated from object contact using limit
   distance?
4. How variable are the signatures across objects, repetitions, and hand
   postures?
5. What dwell and hysteresis values reject brief current spikes without making
   contact detection feel late?

Do not promote the estimator into the motor-control path until these traces
show a stable operating region and the firmware sampler has demonstrated that
it does not degrade direct-control timing.
