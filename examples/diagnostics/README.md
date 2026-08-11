# Diagnostics

These tools are for isolating serial and Dynamixel bus speed issues. Use only
one program at a time on the OpenRB USB port. Windows COM ports are exclusive:
if Arduino Serial Monitor is open, Python cannot safely open the same COM port.

## Safe Motor Console

Flash this first when the bus is acting flaky:

```text
examples/diagnostics/openrb_motor_console/openrb_motor_console.ino
```

Use Arduino Serial Monitor at `1000000` baud with line ending `Newline`.

Supported commands:

```text
help
protocol 2
baud 1000000
timeout 10
ids 11 12 13 14 15 16 17 18 19
ping
cache
fw 11
rd 11
srl 11
setrd 11 50
setmotorbaud 11 1000000
recover
pos 11
posbench 11 20 5
cur 11
vel 11
syncpos
fastsyncpos
scan
```

Retired / do-not-run commands:

```text
readct
read
repeat 20
once
bench 200
readone
```

Why: these commands either no longer exist in the current motor-console sketch
or were too aggressive while the bus was unstable. `readct` in particular ran a
multi-register sweep across all selected motors and caused Arduino Serial
Monitor to freeze during live testing. Prefer one-register commands (`pos`,
`cur`, `vel`) with explicit single IDs.

If a command aborts after repeated read errors, send `recover` before trying a
smaller ID subset. If Arduino Serial Monitor still does not respond, close it
and unplug/replug the OpenRB.

Recommended sequence:

```text
ids 11 12 13 14 15 16 17 18 19
cache
setrd 11 100
timeout 10
rd 11
pos 11
posbench 11 100 20
timeout 20
posbench 11 100 20
```

If a motor is still configured at 2 Mbps, switch the diagnostic DXL port to 2
Mbps before changing that motor, then return to 1 Mbps:

```text
cache
baud 2000000
setmotorbaud 12 1000000
baud 1000000
cache
setrd 12 100
timeout 20
rd 12
pos 12
posbench 12 100 20
```

To restore a motor to 2 Mbps for troubleshooting:

```text
setmotorbaud 11 2000000
baud 2000000
```

`setmotorbaud` changes DYNAMIXEL EEPROM and turns torque off for that one motor
before writing. Use one explicit ID at a time.

### Current baud conclusion

The full exo chain was unreliable at 2 Mbps: repeated single-motor position
reads produced timeout, CRC, and buffer-overflow errors. Motor 11 was stable at
1 Mbps with `100/100` successful position reads and zero errors. The normal exo
firmware now defaults both `DEBUG_BAUD_RATE` and `DYNAMIXEL_BAUD_RATE` to
`1000000`.

Before flashing/running normal firmware, make sure every motor on the active
chain has been moved to 1 Mbps. Use the diagnostic sketch and change one motor
at a time:

```text
baud 2000000
cache
setmotorbaud 12 1000000
baud 1000000
cache
pos 12
posbench 12 100 20
```

For the next motor, switch the OpenRB DXL port back to the baud that motor is
currently using before writing it. If only motor 11 has been changed so far,
IDs 12-19 are still at 2 Mbps until you change them.

## Retired Raw Dynamixel Bus Benchmark

Flash:

```text
examples/diagnostics/openrb_fast_sync_read_benchmark/openrb_fast_sync_read_benchmark.ino
```

This sketch is kept for reference but is not the recommended live test path
right now. It can produce misleading timeout timings and too much Serial
Monitor output while the bus is unstable.

Previously used commands:

```text
ids 11 12 13 14 15 16 17 18 19
ping
baudscan
fullscan
once
bench 200
```

After testing any diagnostic sketch, flash the normal exo firmware back.

If `ping` reports no responsive motors, do not trust `once` or `bench` timing yet:
they are measuring timeouts, not successful motor data. Use `baudscan` to check
whether the selected motor IDs respond at common protocol-2 baud rates. If that
finds nothing, use `fullscan` to scan all IDs across protocol 1 and 2 at common
baud rates. Then set the matching bus baud with `baud <value>` before
benchmarking.

## Exo Firmware Fast-Telemetry Benchmark

Use this only after flashing the normal exo firmware, and only when Arduino
Serial Monitor is closed:

```powershell
python examples\diagnostics\benchmark_fast_telemetry.py --port COM3 --ids 11 12 13 14 15 16 17 18 19
```

This script opens COM3 through pyserial, so it cannot run at the same time as
Arduino Serial Monitor, the GUI, or any other serial terminal.

## Phase-1 Shadow Contact Recorder

The normal GUI now contains the supported live recorder. Expand advanced intent
settings and enable **Record read-only shadow contact evidence (Phase 1)**
before starting EMG teleoperation. The recorder uses the GUI's existing serial
worker, so it does not contend with the GUI for a Windows COM port.

See `docs/shadow_contact_phase1.md` for the full bench procedure and CSV field
definitions.
