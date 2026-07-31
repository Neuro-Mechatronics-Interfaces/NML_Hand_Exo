# NML Hand Exo Examples

This directory contains example scripts demonstrating how to use the NML Hand Exoskeleton Python API.

##  Directory Structure

Examples are organized by complexity and topic:

```
examples/
|- README.md                           # This file
|- 01_basic/                           # Getting started
|  |- example_serial_exo.py            # Basic serial connection
|  |- example_tcp_exo.py               # TCP/IP connection (WiFi)
|  `- hand_exo_cli.py                  # Command-line interface tool
|- 02_motor_control/                   # Motor control basics
|  |- motor_test.py                    # Simple motor movement test
|  |- joint_range_test.py              # Sweep motors through full range
|  |- example_motor_config.py          # Velocity/acceleration/limits
|  `- example_batch_operations.py      # Batch operations with 'all'
|- 03_sensors/                         # Sensor integration
|  |- example_imu_control.py           # IMU-based wrist control
|  `- imu/
|     `- imu_serial.py                 # Read IMU data (roll, pitch, yaw)
|- 04_advanced/                        # Advanced features
|  |- example_advanced_config.py       # Debug, OLED, modes, baudrate
|  `- UART_uno_pico/                   # UART communication examples
|- 05_applications/                    # Real-world applications
|  |- example_pylsl_read.py            # LSL reading example
|  |- live_decoder_from_lsl_stream.py  # Real-time decoder
|  |- pca_viewer.py                    # PCA visualization
|  `- task/                            # Task-specific applications
|     |- training_task.py              # Training task with GUI
|     |- task_gui_minimal.py           # Minimal task GUI
|     `- task_config.json              # Task configuration
|- 06_lsl_streaming/                   # Lab Streaming Layer
|  `- LSL/
|     |- lsl_classifier_trigger.py     # EMG classifier control
|     |- lsl_gesture_controller.py     # LSL marker control
|     |- lsl_state_trigger.py          # State-based EMG control
|     |- lsl_broadcast_test.py         # LSL broadcasting
|     |- lsl_subscribe_test.py         # LSL subscription
|     |- lsl_stacked_plot.py           # Stacked EMG plotting
|     |- lsl_grid_plot.py              # Grid EMG visualization
|     `- lsl_rms_barplot.py            # RMS bar plot
|- 07_mindrove/                        # MindRove control panel
|- 08_udp/                             # UDP receiver and manual gesture GUI
|- calibration/                        # Calibration & range-of-motion
|  |- calibrate_exo.py                 # CLI calibration wizard (updates config.h)
|  |- rom_assessment.py                # ROM protocol -> output_data/*.csv
|  `- profiles/                        # Per-user calibration profiles (JSON)
|- udp_bindings/                       # Integer UDP binding profiles
|- scripts/                            # Standalone tools (no Qt)
|  |- README.md                        # UDP forwarder + panel docs
|  |- udp_gesture_receiver.py          # UDP integer -> serial gesture forwarder
|  |- udp_gesture_gui.py               # Tkinter panel that drives the receiver
|  `- diagnostics/                     # Port scans, CDC latency, pose capture
`- tests/                              # Unit tests (python -m pytest examples/tests -q)
```

`tests/` imports the UDP receiver from `../scripts`, so the two must stay
siblings under `examples/`.

---

##  Getting Started

### Basic Examples - Connection & Setup

#### Serial Connection (`01_basic/example_serial_exo.py`)
Basic example showing how to connect via serial and query device information.

```bash
python examples/01_basic/example_serial_exo.py
```

**Key Features:**
- Serial communication setup
- Device info and version queries
- Motor angle/velocity/torque readings
- Gesture state queries

#### TCP/IP Connection (`01_basic/example_tcp_exo.py`)
Example for connecting over TCP/IP (e.g., using Pico W with WiFi).

```bash
python examples/01_basic/example_tcp_exo.py
```

#### Command-Line Interface (`01_basic/hand_exo_cli.py`)
Full-featured CLI tool for sending commands and monitoring serial output.

```bash
# List available serial ports
python examples/01_basic/hand_exo_cli.py --list-ports

# Connect and print device info
python examples/01_basic/hand_exo_cli.py --connect COM5 --baud 57600 --info

# Home all motors
python examples/01_basic/hand_exo_cli.py --connect COM5 --home

# Send custom command
python examples/01_basic/hand_exo_cli.py --connect COM5 --send "led:1:on"

# Monitor serial output
python examples/01_basic/hand_exo_cli.py --connect COM5 --monitor
```

---

### Motor Control Examples

#### Motor Configuration (`02_motor_control/example_motor_config.py`)
Demonstrates setting velocity, acceleration, and motor limits.

```bash
python examples/02_motor_control/example_motor_config.py
```

**Features:**
- Query current velocity/acceleration/limits
- Set custom motion profiles
- Test movement with new settings
- Restore original configuration

#### Batch Operations (`02_motor_control/example_batch_operations.py`)
Using the 'all' keyword for simultaneous control of all motors.

```bash
python examples/02_motor_control/example_batch_operations.py
```

**Features:**
- Read all motor states at once
- Enable/disable all motors
- Synchronized position control
- LED control for all motors

#### Motor Test (`02_motor_control/motor_test.py`)
Simple motor movement test - moves motor 0 to different angles.

```bash
python examples/02_motor_control/motor_test.py
```

#### Joint Range Test (`02_motor_control/joint_range_test.py`)
Systematically sweeps each motor through its full range of motion.

```bash
python examples/02_motor_control/joint_range_test.py
```

**Features:**
- Queries home positions and joint limits
- Sweeps to upper limit, returns home
- Sweeps to lower limit, returns home
- Processes all motors sequentially

---

### Sensor Examples

#### IMU-Based Control (`03_sensors/example_imu_control.py`)
Control motor angles based on IMU wrist orientation.

```bash
python examples/03_sensors/example_imu_control.py
```

**Features:**
- Read IMU orientation (heading/pitch/roll)
- Set wrist motor to track target yaw angles
- Flex/extend direction control
- Closed-loop IMU feedback

#### Reading IMU Data (`03_sensors/imu/imu_serial.py`)
Continuously read and display roll, pitch, and yaw angles from the IMU.

```bash
python examples/03_sensors/imu/imu_serial.py
```

---

### Advanced Examples

#### Advanced Configuration (`04_advanced/example_advanced_config.py`)
Demonstrates debug mode, OLED control, motor modes, and baudrate settings.

```bash
python examples/04_advanced/example_advanced_config.py
```

**Features:**
- Enable/disable Arduino debug output
- OLED display control
- Switch motor control modes (position/velocity/current_position)
- Switch exo operating modes (FREE/GESTURE_FIXED/GESTURE_CONTINUOUS)
- Query motor baudrate settings
- Complete device information summary

---

### Application Examples

These examples demonstrate complete applications and task implementations.

#### LSL Reading (`05_applications/example_pylsl_read.py`)
Example of reading EMG data from LSL streams.

```bash
python examples/05_applications/example_pylsl_read.py
```

#### Live Decoder (`05_applications/live_decoder_from_lsl_stream.py`)
Real-time gesture decoding from EMG streams.

```bash
python examples/05_applications/live_decoder_from_lsl_stream.py
```

#### Training Task (`05_applications/task/training_task.py`)
Complete training task with GUI for data collection.

```bash
python examples/05_applications/task/training_task.py
```

---

### LSL (Lab Streaming Layer) Examples

These examples demonstrate real-time EMG streaming and gesture control using LSL.

#### Prerequisites
Install LSL support:
```bash
pip install pylsl
```

#### EMG Classifier Trigger (`06_lsl_streaming/LSL/lsl_classifier_trigger.py`)
Uses an EMG classifier to trigger gesture changes.

```bash
python examples/06_lsl_streaming/LSL/lsl_classifier_trigger.py --port COM4 --baudrate 115200
```

#### LSL Gesture Controller (`06_lsl_streaming/LSL/lsl_gesture_controller.py`)
Listens to LSL marker streams and executes corresponding gestures.

```bash
python examples/06_lsl_streaming/LSL/lsl_gesture_controller.py --port COM4 --type Markers --name EMGGesture
```

**Arguments:**
- `--type`: LSL stream type (default: "Markers")
- `--name`: LSL stream name (default: "EMGGesture")
- `--port`: Serial port (default: "COM4")
- `--baudrate`: Baud rate (default: 115200)
- `--verbose`: Enable verbose output

#### State Trigger (`06_lsl_streaming/LSL/lsl_state_trigger.py`)
Triggers gestures based on EMG RMS thresholds.

```bash
python examples/06_lsl_streaming/LSL/lsl_state_trigger.py --port COM4
```

#### Visualization Tools

**Stacked Plot:**
```bash
python examples/06_lsl_streaming/LSL/lsl_stacked_plot.py --type EMG --name OpenEphysEMG
```

**Grid Plot:**
```bash
python examples/06_lsl_streaming/LSL/lsl_grid_plot.py --type EMG --name OpenEphysEMG
```

**RMS Bar Plot:**
```bash
python examples/06_lsl_streaming/LSL/lsl_rms_barplot.py --type EMG --name OpenEphysEMG
```

---

### UDP Gesture Control (`08_udp`)

The two scripts in `08_udp` are designed to run together. `udp_gesture_receiver.py` owns the dual-CDC connection to the exo, translates UDP integers into gesture commands, and returns acknowledgements. `udp_gesture_gui.py` is the manual UDP control panel used to register the acknowledgement port and send those integers.

> **Safety:** The receiver arms and homes the exo by default. Keep hands and obstructions clear before starting it.

1. Start the receiver in the first terminal. Its defaults use `COM10` for commands and `COM11` for telemetry; pass the appropriate ports if your system enumerated them differently.

   ```bash
   python examples/08_udp/udp_gesture_receiver.py --cmd-port COM10 --telem-port COM11
   ```

2. Wait until the receiver finishes connecting, arming, and binding its UDP socket. It must print the following line before you proceed:

   > Send an integer > 64 first to register the port acks should return on.

Do not click **Connect** in the GUI before this line appears. It means the receiver is ready for the GUI's initial return-port announcement.

3. Start the GUI in a second terminal:

   ```bash
   python examples/08_udp/udp_gesture_gui.py
   ```

4. Click **Connect**. The GUI sends its listening port as an integer greater than 64, and the receiver registers that port for acknowledgements.

5. Test every joint in this order, clicking each numbered button once:

   - **Rest:** `+11`, `+12`, `+13`, `+14`, `+15`, `+16`
   - **Flex:** `+1`, `+2`, `+3`, `+4`, `+5`, `+6`
   - **Extend:** `-1`, `-2`, `-3`, `-4`, `-5`, `-6`

The values address the thumb, index, middle, ring, pinky, and wrist in that order. You should hear the corresponding motors move after every click, with movement beginning nearly instantly.

If every movement produces the expected immediate motor response, the exo and its acknowledgement path are configured correctly and it is ready for UDP-driven control.

---

##  Common Usage Patterns

### Basic Connection Setup

```python
from nml_hand_exo.interface import HandExo, SerialComm

# Create communication interface
comm = SerialComm(port="COM6", baudrate=57600)

# Create HandExo instance
exo = HandExo(comm, verbose=False)

# Connect to device
exo.connect()

# Use the device
print(exo.version())
print(exo.info())

# Clean up
exo.close()
```

### Motor Control

```python
# Enable motor
exo.enable_motor(motor_id=0)

# Set motor angle (relative to home position)
exo.set_motor_angle(motor_id=0, angle=45)

# Get current angle
angle = exo.get_motor_angle(motor_id=0)

# Set motor to home position
exo.home(motor_id=0)

# Disable motor
exo.disable_motor(motor_id=0)
```

### Reading Sensor Data

```python
# Get IMU data
imu_data = exo.get_imu_data()
print(f"Heading: {imu_data['heading']}, Pitch: {imu_data['pitch']}, Roll: {imu_data['roll']}")

# Get specific IMU angles
heading = exo.get_imu_heading()
roll = exo.get_imu_roll()
pitch = exo.get_imu_pitch()

# Get motor torque
torque = exo.get_motor_torque(motor_id=0)

# Get motor current
current = exo.get_motor_current(motor_id=0)
```

### Gesture Control

```python
# Set exo to gesture mode
exo.set_exo_mode("GESTURE_FIXED")

# Set a specific gesture
exo.set_gesture("grasp", "open")

# Get current gesture
current_gesture = exo.get_gesture()

# Cycle through gestures
exo.cycle_gesture()

# Cycle through gesture states
exo.cycle_gesture_state()
```

---

##  Known Issues & Improvements Needed

### Import Inconsistencies
 **FIXED** - All examples now use consistent import patterns:
-  Correct: `from nml_hand_exo.interface import HandExo, SerialComm`
-  All examples include proper error handling with try/finally
-  All examples call `exo.close()` to clean up connections

### Method Call Issues
 **FIXED** - Removed deprecated parameters from all examples

### Documentation
 **COMPLETE** - Comprehensive README with organized structure
-  Examples organized by complexity (basic to lsl_streaming)
-  Clear usage instructions for all examples
-  Common patterns and best practices documented

---

##  Best Practices

1. **Always use explicit imports**: `from nml_hand_exo.interface import HandExo, SerialComm`
2. **Use context managers or try/finally**: Ensure `exo.close()` is called
3. **Enable verbose mode during debugging**: `HandExo(comm, verbose=True)`
4. **Check device info on connect**: Verify correct device and version
5. **Home motors before absolute positioning**: Ensures accurate reference point
6. **Respect motor limits**: Query limits with `get_motor_limits()` before commanding angles

---

##  Example Coverage

Complete coverage of all API features:
-  Basic connection (serial, TCP)
-  Motor control (position, velocity, acceleration, limits)
-  Batch operations ('all' keyword)
-  IMU sensor reading
-  IMU-based control (`set_yaw_angle`)
-  Debug mode control
-  OLED display control
-  Motor mode switching
-  Exo mode switching
-  Gesture control
-  LSL streaming integration
-  Real-time visualization
-  Task applications

All newly fixed API methods are demonstrated in the examples!

---

##  Reporting Issues

If you find bugs or inconsistencies in the examples, please report them at:
https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/issues

---

##  Additional Resources

- **API Documentation**: See `docs/` folder
- **Firmware Source**: `src/cpp/nml_hand_exo/`
- **Python Package**: `src/nml_hand_exo/`
- **Command Reference**: `COMMAND_INCONSISTENCIES.md` (now resolved)
