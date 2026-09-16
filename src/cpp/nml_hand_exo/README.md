# NML Hand Exo firmware

Firmware for the NML Hand Exoskeleton, targeting the **ROBOTIS OpenRB-150**
(SAMD21) controller driving Dynamixel XC330 servos. The sketch entry point is
[`nml_hand_exo.ino`](nml_hand_exo.ino); the device is controlled over a serial
command interface (USB, and optionally an HC-05 Bluetooth UART).

**v0.9.0:** motion-time telemetry comes from a bounded per-joint model, with field-source flags and UTC/sample-uptime timestamps. The GUI synchronizes UTC on connection. See the [implementation and protocol contract](../../../docs/v0.9.0_implementation.md) for tuning, compatibility, safety-feedback exceptions, and bench validation.

This [document](../../../docs/arduino_cli_setup.md) covers how to install `arduino-cli`. Below are steps to use the `arduino-cli` tool to compile, and flash the board, plus a short tour of the serial command interface. For the full protocol reference see [`docs/serial_protocol.md`](../../../docs/serial_protocol.md).

## Choose the build variant

Two build-time selectors in [`config.h`](config.h) change what the firmware is built for. Set them before compiling.

**Hand side** — `BUILD_LEFT_HAND` (near the top of `config.h`):

| Value     | Build            | Motor IDs      |
|-----------|------------------|----------------|
| `2` (default) | dual (both)  | 1-9 + 11-19    |
| `0`       | right hand only  | 11-19          |
| `1`       | left hand only   | 1-9            |

The default is **dual**: it enumerates every ID, and a hand with only one side physically attached still works — the unreachable IDs are reported as skipped rather than blocking the attached side. Build for a single fixed side (`0`/`1`) only when you want exactly those motors. In single-hand builds the `ENABLE_*` flags below the selector can exclude a motor that is not physically connected.

**USB layout** — by default an OpenRB build exposes **two USB CDC serial ports** on one cable (`DUAL_CDC`): the first carries host commands, the second carries replies and telemetry. This decouples command writes from telemetry reads. A legacy single-port host still works because either port accepts commands and replies mirror to both by default. Define `SINGLE_CDC` at build time to force one port. The opt-in `EXO_AXON_USB` composite build (for the SciFi/Axon integration) is mutually exclusive with dual CDC.

---

## Compile

From the repository root:

```bash
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 src/cpp/nml_hand_exo
```

A clean build ends with a flash/RAM usage summary, e.g.

```
Sketch uses 149852 bytes (57%) of program storage space. Maximum is 262144 bytes.
Global variables use 15132 bytes (46%) of dynamic memory ...
```

You may see `LITTLE_ENDIAN redefined` warnings from the SAMD core headers; those come from the platform, not this firmware, and are harmless.

To add a build flag without editing `config.h` (e.g. force single CDC), use `compiler.cpp.extra_flags`. Overriding `build.extra_flags` removes OpenRB MCU/USB
definitions such as `__SAMD21G18A__` and causes missing `Sercom`/`TCC_INST_NUM` errors:

```bash
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-property "compiler.cpp.extra_flags=-DSINGLE_CDC" src/cpp/nml_hand_exo
```

For example to compile and upload the optional `Protobuf` build (requires nanopb):

Current firmware reports **0.9.1**, with measured 30-ms shaped pulse-response calibration.
See [the v0.9.1 defaults and diagnostics](../../../docs/v0.9.1_pulse_calibration.md).

```bash
arduino-cli compile --upload -p %COM% --fqbn OpenRB-150:samd:OpenRB-150 --build-property "compiler.cpp.extra_flags=-DEXO_USB_PROTOBUF=1" src/cpp/nml_hand_exo
```

The primary CDC retains `info`/`help`; the second carries typed Protobuf telemetry
and direct motion/model-update requests. `info` advertises `USB Protocol: protobuf-v1` so the
SDK/GUI discovers the layout automatically. See [USB diagnostics and protocol](../../../docs/usb_protocol_and_io_diagnostics.md)
for compile-only commands, the Python extra, schema regeneration, and telemetry
plus fixed-rate model-write benchmarks. `USB Features: joint_model_write`
identifies builds supporting typed `set_joint_model` requests. The additional
`batch_motion` feature enables ID-keyed angle/current batches, finger angles,
and named gestures through Protobuf. Update both
firmware and Python for this feature. The default ASCII/NX build is unchanged unless the flag
is enabled.

---

## Flash

Find the board's serial port:

```bash
arduino-cli board list
```

On Windows `cmd` terminal, to set `COM` environment variable (with OpenRB board plugged in):  
```cmd
set "COM=" && for /f "tokens=1" %i in ('arduino-cli board list ^| findstr "OpenRB-150:samd:OpenRB-150"') do @if not defined COM set "COM=%i"
```
or on Mac/Linux:
```bash
export COM=$(arduino-cli board list | grep "OpenRB-150:samd:OpenRB-150" | head -n1 | awk '{print $1}')
echo "$COM"
```

Look for the row identified as OpenRB-150 (or the port that appears when you
plug the board in) — for example `%COM%` on Windows, `/dev/ttyACM0` on Linux, or
`/dev/cu.usbmodemXXXX` on macOS.

Compile and upload in one step (`--upload` compiles first):

```cmd
arduino-cli compile --upload -p %COM% --fqbn OpenRB-150:samd:OpenRB-150 src/cpp/nml_hand_exo
```

On success the board flashes its status LED **4 times** and prints
```cmd
Exo device ready to receive commands
```

**If the upload cannot open the port:** close any serial monitor or GUI holding it. If the board does not enter the bootloader on its own, double-press the board's reset button to force the SAMD bootloader (a new port usually appears
for a few seconds); pass that port to `-p`.

---

## Serial interface at a glance

The controller listens on:

| Channel        | Arduino object | Baud      | Physical                     |
|----------------|----------------|-----------|------------------------------|
| USB (commands) | `Serial`       | 1000000   | USB cable                    |
| Dynamixel bus  | `Serial1`      | 1000000   | JST DXL connector            |
| HC-05 (BT)     | `Serial3`      | 115200    | D13 (TX3) / D14 (RX3)        |

Commands are plain ASCII, colon-delimited, one per line:

```
<command>:<arg1>:<arg2>...
```

Terminate a command with `\n` over USB, or `;` over the Bluetooth/`COMMAND_SERIAL` path. Text responses are terminated with `;`.

Examples:

```text
version                              # firmware version string
info                                 # device + motor summary
get_absolute_angle:all               # read every motor's absolute angle
set_angle:index:45                   # move a joint to a relative angle
enable:all                           # torque on
disable:all                          # torque off
help                                 # list every command
```

Use an **explicit integer Dynamixel ID** rather than a bare name when it
matters: in dual-hand builds names like `wrist` are ambiguous (they exist on
both sides) and resolve to the left motor. IDs are unambiguous.

### Direct current / velocity control

The global control mode must be set before issuing direct commands; a mode
change turns torque off, so re-enable afterward:

```text
set_control_mode:all:current         # or velocity, position, current_position
enable:16
set_current:16:50                    # signed mA (current mode)
stop:16                              # zero the direct goal
```

### Impedance range-of-motion (ROM) calibration (firmware >= 0.8.0)

Discovers a joint's endstop in one direction by pushing on it gently under
current control and watching for motion, rather than asking the wearer to move
to their extremes. One joint, one direction per command; the host orchestrates
a full sweep.

```text
set_control_mode:all:current         # required first
enable:11
calibrate_rom:11:flex                # arm the sweep; endstop arrives async
```

The endstop is reported (not applied) on a `ROM_CAL_RESULT:` line, e.g.

```text
OK: calibrate_rom id=11 dir=flex
ROM_CAL_RESULT: id=11 dir=flex home=228.45 endstop=262.10 current_mA=145 status=ok
```

`cancel_rom` aborts a running sweep and returns the joint home. Every sweep
tuning value lives in the labeled `ROM_CAL_*` block in [`config.h`](config.h);
see [`docs/serial_protocol.md`](../../../docs/serial_protocol.md) for the full
behavior and safety notes.

> Safety: this endpoint deliberately drives current into a joint on a
> participant's hand. Keep the current ceiling conservative, keep `cancel_rom`
> within reach, and confirm the reported `home` matches where the joint actually
> started before adopting any endstop.

---

## File map

| File | Purpose |
|------|---------|
| `nml_hand_exo.ino` | Sketch entry point: `setup()` / `loop()`, USB/serial wiring |
| `config.h` | Build selectors, motor IDs/names, limits, baud rates, all tuning constants |
| `nml_hand_exo.{h,cpp}` | `NMLHandExo` class: motor control, telemetry, calibration |
| `utils.{h,cpp}` | Serial command parser (`parseMessage`) and helpers |
| `gesture_controller.{h,cpp}`, `gesture_library.{h,cpp}` | Named gestures and button handling |
| `oled.{h,cpp}` | OLED status display |
| `axon_config.h`, `AxonUsbPeripheral.{h,cpp}`, `AxonEnumProtocol.h` | Opt-in Axon/SciFi composite-USB peripheral |

---

## Host-side control

The Python package under [`src/nml_hand_exo`](../../nml_hand_exo) drives this
firmware over serial (`HandExo`), including a `calibrate_rom(...)` helper for the ROM sweep above. Python changes do not require re-flashing; only firmware or
protocol changes do.
