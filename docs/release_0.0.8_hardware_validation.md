# 0.0.8 Hardware Validation Checklist

Run this checklist with no participant attached until every motion and stop path
has been observed. Keep an accessible power disconnect throughout.

## Firmware and connection

- [ ] Flash firmware version 0.2.15 (release 0.0.8) to the OpenRB-150.
- [ ] Confirm the USB connection and Dynamixel bus are both 1 Mbps.
- [ ] Confirm the HC-05 link, if used, is 115200 baud.
- [ ] Run `handexo gui`, select the correct side/mode, and confirm detected DXL IDs.
- [ ] In a single-side mode, confirm the other side is disabled by explicit ID.

## Telemetry

- [ ] Confirm the Telemetry tab updates relative and absolute position via the compact frame.
- [ ] Confirm fallback compact telemetry shows current and torque as unavailable (`—`), not zero.
- [ ] Disconnect or use an older firmware and confirm automatic text-poll fallback works.

## Direct and EMG teleop

- [ ] Apply an appropriate calibration profile using explicit DXL IDs.
- [ ] In Direct Control, select Velocity mode, arm one motor, and verify velocity limits.
- [ ] Start `handexo emg-centroid`, publish `NMLIntentV1`, and connect it in EMG Teleop.
- [ ] Verify live mode is blocked without calibration, direct velocity mode, an armed ID, or a held deadman.
- [ ] With the deadman held, verify a fresh active intent commands only the selected ID.
- [ ] Verify rest, stale input, low confidence, invalid samples, deadman release, disconnect, and application close each send a stop command.

## Release decision

- [ ] Record firmware version, board/port, DXL IDs, and observed behavior.
- [ ] Review any unexpected movement or serial errors before participant use.
