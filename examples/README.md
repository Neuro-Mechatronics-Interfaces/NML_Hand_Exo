# NML Hand Exo Examples

This directory contains example scripts demonstrating how to use the NML Hand Exoskeleton Python API.

## 📁 Directory Structure

Examples are organized by complexity and topic:

```
examples/
├── README.md                           # This file
├── 01_basic/                          # Getting started
│   ├── example_serial_exo.py          # Basic serial connection
│   ├── example_tcp_exo.py             # TCP/IP connection (WiFi)
├── 02_motor_control/                  # Motor control basics
│   ├── motor_test.py                  # Simple motor movement test
│   ├── example_motor_config.py        # Velocity/acceleration/limits
│   └── example_batch_operations.py    # Batch operations with 'all'
├── 03_sensors/                        # Sensor integration
│   ├── example_imu_control.py         # IMU-based wrist control
│   └── imu/
│       └── imu_serial.py              # Read IMU data (roll, pitch, yaw)
├── 04_advanced/                       # Advanced features
│   ├── example_advanced_config.py     # Debug, OLED, modes, baudrate
│   └── UART_uno_pico/                 # UART communication examples
├── 05_applications/                   # Real-world applications
│   ├── example_pylsl_read.py          # LSL reading example
│   ├── pca_viewer.py                  # PCA visualization
│   └── task/                          # Task-specific applications
│       ├── training_task.py           # Training task with GUI
│       ├── task_gui_minimal.py        # Minimal task GUI
│       └── task_config.json           # Task configuration
└── 06_lsl_streaming/                  # Lab Streaming Layer
    └── LSL/
        ├── lsl_classifier_trigger.py  # EMG classifier control
        ├── lsl_gesture_controller.py  # LSL marker control
        ├── lsl_state_trigger.py       # State-based EMG control
        ├── lsl_broadcast_test.py      # LSL broadcasting
        ├── lsl_subscribe_test.py      # LSL subscription
        ├── lsl_stacked_plot.py        # Stacked EMG plotting
        ├── lsl_grid_plot.py           # Grid EMG visualization
        └── lsl_rms_barplot.py         # RMS bar plot
```

---

## 🚀 Getting Started

### 01. Basic Examples - Connection & Setup

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

### 02. Motor Control Examples

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

### 03. Sensor Examples

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

### 04. Advanced Examples

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

### 05. Application Examples

These examples demonstrate complete applications and task implementations.

#### LSL Reading (`05_applications/example_pylsl_read.py`)
Example of reading EMG data from LSL streams.

```bash
python examples/05_applications/example_pylsl_read.py
```

#### Training Task (`05_applications/task/training_task.py`)
Complete training task with GUI for data collection.

```bash
python examples/05_applications/task/training_task.py
```

---

### 06. LSL (Lab Streaming Layer) Examples

These examples demonstrate real-time EMG streaming and gesture control using LSL.

#### Prerequisites
Install LSL support:
```bash
pip install pylsl
```

#### EMG Classifier Trigger (`06_lsl_streaming/LSL/lsl_classifier_trigger.py`)
Uses an EMG classifier to trigger gesture changes.

```bash
python examples/06_lsl_streaming/LSL/lsl_classifier_trigger.py --port COM4 --baudrate 1000000
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
- `--baudrate`: Baud rate (default: 1000000 for USB serial)
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

## 🔧 Common Usage Patterns

### Basic Connection Setup

```python
from nml_hand_exo.interface import HandExo, SerialComm

# Create communication interface
comm = SerialComm(port="COM6", baudrate=1000000)

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
motor_id = sorted(exo.info()["motors"])[0]
exo.enable_motor(motor_id=motor_id)

# Set motor angle (relative to home position)
exo.set_motor_angle(motor_id=motor_id, angle=45)

# Get current angle
angle = exo.get_motor_angle(motor_id=motor_id)

# Set motor to home position
exo.home(motor_id=motor_id)

# Disable motor
exo.disable_motor(motor_id=motor_id)
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
torque = exo.get_motor_torque(motor_id=motor_id)

# Get motor current
current = exo.get_motor_current(motor_id=motor_id)
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

## ⚠️ Known Issues & Improvements Needed

### Import Inconsistencies
✅ **FIXED** - All examples now use consistent import patterns:
- ✅ Correct: `from nml_hand_exo.interface import HandExo, SerialComm`
- ✅ All examples include proper error handling with try/finally
- ✅ All examples call `exo.close()` to clean up connections

### Method Call Issues
✅ **FIXED** - Removed deprecated parameters from all examples

### Documentation
✅ **COMPLETE** - Comprehensive README with organized structure
- ✅ Examples organized by complexity (01-06)
- ✅ Clear usage instructions for all examples
- ✅ Common patterns and best practices documented

---

## 📝 Best Practices

1. **Always use explicit imports**: `from nml_hand_exo.interface import HandExo, SerialComm`
2. **Use context managers or try/finally**: Ensure `exo.close()` is called
3. **Enable verbose mode during debugging**: `HandExo(comm, verbose=True)`
4. **Check device info on connect**: Verify correct device and version
5. **Home motors before absolute positioning**: Ensures accurate reference point
6. **Respect motor limits**: Query limits with `get_motor_limits()` before commanding angles

---

## 🆕 Example Coverage

Complete coverage of all API features:
- ✅ Basic connection (serial, TCP)
- ✅ Motor control (position, velocity, acceleration, limits)
- ✅ Batch operations ('all' keyword)
- ✅ IMU sensor reading
- ✅ IMU-based control (`set_yaw_angle`)
- ✅ Debug mode control
- ✅ OLED display control
- ✅ Motor mode switching
- ✅ Exo mode switching
- ✅ Gesture control
- ✅ LSL streaming integration
- ✅ Real-time visualization
- ✅ Task applications

All newly fixed API methods are demonstrated in the examples!

---

## 🐛 Reporting Issues

If you find bugs or inconsistencies in the examples, please report them at:
https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/issues

---

## 📚 Additional Resources

- **API Documentation**: See `docs/` folder
- **Firmware Source**: `src/cpp/nml_hand_exo/`
- **Python Package**: `src/nml_hand_exo/`
- **Command Reference**: `COMMAND_INCONSISTENCIES.md` (now resolved)

