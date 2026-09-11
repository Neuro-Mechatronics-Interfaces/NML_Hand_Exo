# Optional SciFi Axon composite USB build

Compile all sketch C++ translation units with `-DEXO_AXON_USB=1` for the
OpenRB-150 SciFi integration. Without this flag, the existing dual CDC build
is unchanged. Use a clean Arduino build when changing modes.

The flag reserves vendor interface 0 for Axon discovery and retains one CDC
at control interface 1/data interface 2 for the existing ASCII command/reply
protocol. A second CDC cannot fit alongside Axon. Do not define CDC_DISABLED.
`AxonUsbPeripheral` must construct with `init_priority(101)` before SerialUSB.

The firmware answers zero-payload ENUMERATE type 3 with an outer-framed type 4
response advertising hardware type `0xF002`. This is not the host-assigned
Synapse peripheral ID. Other Axon traffic cannot reach the motor command
parser. Motor logic, limits, startup behavior and Bluetooth commands retain
their existing behavior. Discovery starts when the main loop starts.

The host requires the updated server VID allowlist, and a matching `axon_exo`
driver registered for `0xF002`. In the ScienceXYZ parent repository, see
`firmware/axon-exo/README.md` for build, package and operator acceptance steps.
Hub-manager must select CDC interface 1; legacy interface 0/2 fallback is for
the non-Axon firmware. Local compile and parser tests do not establish bench
enumeration, control, synchronization or motion acceptance.
