# OpenRB Motor Console

Minimal read-only Dynamixel console for OpenRB-150.

Use this instead of the full exo firmware when debugging basic motor
communication. It follows the ROBOTIS example pattern and does not initialize
OLED, IMU, GUI protocol, sync-read benchmarks, torque, operating modes, or
motion commands.

Serial Monitor:

```text
Baud: 1000000
Line ending: Newline
```

Supported commands:

```text
help
scan
ids 11 12 13 14 16 17 18 19
timeout 10
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

`readct` caused Arduino Serial Monitor to freeze during live testing because it
ran current, velocity, and position reads across every selected motor. Use
single-register commands with explicit IDs instead.

The console uses bounded DXL RX draining and aborts register sweeps after two
consecutive read errors. Send `recover` before trying a smaller ID subset. If
Serial Monitor still does not respond, close it and unplug/replug the OpenRB.

`setrd ID VALUE` writes `RETURN_DELAY_TIME` to one explicit motor ID. It does
not command motion, but it does change a DYNAMIXEL EEPROM setting. Use
intermediate values like `50` or `100` while testing; `0` was fast for one-byte
reads but made 4-byte present-position reads unreliable on the current setup.

`setmotorbaud ID BAUD` changes one motor's DYNAMIXEL EEPROM baud setting and
turns torque off before writing, following the ROBOTIS baudrate example pattern.
After a successful write, the console switches its DXL port to the new baud.
Use one explicit ID at a time. For example: `setmotorbaud 11 1000000`, then
test with `baud 1000000`; restore with `setmotorbaud 11 2000000`.

Live testing found the chain unreliable at 2 Mbps and stable for motor 11 at
1 Mbps (`100/100` position reads, zero errors). The normal exo firmware now
defaults the USB debug link and DXL bus to 1 Mbps. Before running that firmware,
convert every active motor to 1 Mbps one at a time. If the bus is mixed-baud,
switch the console DXL port to the baud of the motor you are about to change.

`timeout MS` changes the Dynamixel read timeout used by `ping`, register reads,
and grouped reads. Use `timeout 10` or `timeout 20` when checking reliability;
the default `3` fails fast for recovery but may be too short for benchmarks.

`posbench ID SAMPLES GAP_MS` runs repeated position reads for one motor with a
small delay between samples and reports error counts (`timeout`, `crc`,
`overflow`, `other`). Increase `GAP_MS` or `timeout` if reads are intermittent.

`fw`, `rd`, and `srl` are one-byte registers. The console prints both the signed
library value and the unsigned byte value; `RETURN_DELAY_TIME` may appear as
`value=-6 unsigned=250` before `setrd0`, which means the default 250 units
(500 us), not a negative setting.
