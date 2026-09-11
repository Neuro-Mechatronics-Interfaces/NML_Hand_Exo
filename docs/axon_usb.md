# Optional SciFi Axon composite USB build

Compile all sketch C++ translation units with `-DEXO_AXON_USB=1` for the
OpenRB-150 SciFi integration. Without this flag, the existing dual CDC build
is unchanged. Use a clean Arduino build when changing modes.

The flag reserves vendor interface 0 for Axon discovery/angle polling and retains one CDC
at control interface 1/data interface 2 for the existing ASCII command/reply
protocol. A second CDC cannot fit alongside Axon. Do not define CDC_DISABLED.
`AxonUsbPeripheral` must construct with `init_priority(101)` before SerialUSB.

The firmware answers zero-payload ENUMERATE type 3 with an outer-framed type 4
response advertising hardware type `0xF002`. This is not the host-assigned
Synapse peripheral ID. Other Axon traffic cannot reach the motor command
parser. Motor logic, limits, startup behavior and Bluetooth commands retain
their existing behavior. Discovery starts when the main loop starts.

Driver 0.2.0 polls measured absolute encoder angles with type 0xF210, payload
uint32 little-endian `[version=1, request_token, count, motor_ids...]` (1..18
distinct integer Dynamixel IDs). Type 0xF211 replies echo the token and contain
`[version, token, snapshot_ms_low, snapshot_ms_high, count]` followed by four
int16 fields per motor: angle in 0.1 degree, age-ms low word, age-ms high word,
status (0 measured, 1 unavailable). Age words are unsigned bit patterns.
Subtract age from snapshot uptime to recover each read's source start time.
Unavailable/out-of-range positions use angle -32768 plus status 1, never zero.
Absolute encoder angles retain multiple turns; these are not anatomical angles
or the six gesture percentages. No homing, torque, limits or mode changes occur.

Sampling runs only on request, one position transaction per loop pass, bounded
to 2ms by the Dynamixel read timeout. The loop remains cooperative, not strictly
wait-free; other existing motor-control code can add latency. USB TX advances
one <=64-byte packet only when the IN bank is ready, avoiding the core's 70ms
wait. Commands still run between polling passes. Requests are not replayed as
motion, and the raw serial parser never sees Axon payloads. Timestamp alignment
to SciFi is unknown; host receipt and source timestamps remain distinct.

The host requires the updated server VID allowlist, and a matching `axon_exo`
driver registered for `0xF002`. In the ScienceXYZ parent repository, see
`firmware/axon-exo/README.md` for build, package and operator acceptance steps.
Hub-manager must select CDC interface 1; legacy interface 0/2 fallback is for
the non-Axon firmware. Local compile and parser tests do not establish bench
enumeration, control, synchronization or motion acceptance.
