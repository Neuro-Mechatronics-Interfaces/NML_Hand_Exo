---
layout: single
title: C++ Firmware API
permalink: /cpp-api/
toc: true
toc_sticky: true
description: "C++ Arduino firmware API for the NML Hand Exoskeleton microcontroller."
header:
  overlay_color: "#1e1e1e"
  overlay_filter: "0.5"
---

# Overview

**Note:** This page provides an overview and quickstart guide. For the complete auto-generated C++ API reference with detailed class documentation, see the [**Doxygen Reference**](/NML_Hand_Exo/cpp_api_doxygen/index.html){: .btn .btn--info}.
{: .notice--info}

The **C++ firmware** runs on the OpenRB-150 microcontroller and provides real-time motor control, serial communication, and gesture management for the NML Hand Exoskeleton.

**Key Features:**
- **Dynamixel motor control** - Position, velocity, current control
- **Serial command protocol** - Text-based commands over USB/UART
- **Gesture library** - Pre-defined hand poses and movements
- **IMU integration** - Orientation tracking (optional)
- **OLED display** - Status information (optional)
- **Safety limits** - Current limits, joint angle limits

**Source Code:** [`src/cpp/nml_hand_exo/`](https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/tree/main/src/cpp/nml_hand_exo)

---

# Architecture

## Files

- **`nml_hand_exo.h/cpp`** - Main device class
- **`gesture_controller.h/cpp`** - Gesture management and execution
- **`gesture_library.h/cpp`** - Pre-defined gesture database
- **`config.h`** - Pin definitions and configuration constants
- **`utils.h/cpp`** - Helper functions
- **`oled.h/cpp`** - OLED display driver (optional)

## Dependencies

```cpp
#include <Dynamixel2Arduino.h>  // Dynamixel motor control
#include <Wire.h>                // I2C for IMU/OLED
#include <Adafruit_Sensor.h>     // Sensor abstraction (optional)
#include <Adafruit_BNO055.h>     // IMU driver (optional)
```

---

# Setup & Flashing

## Prerequisites

1. **Arduino IDE** (1.8.19 or 2.x)
2. **OpenRB-150 Board Support** - Install via Board Manager
3. **Dynamixel2Arduino Library** - Install via Library Manager

## Board Configuration

```
Board: OpenRB-150
Upload Speed: 115200
Port: (Select your COM port)
```

## Upload Firmware

1. Clone the repository:
```bash
git clone https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo.git
cd NML_Hand_Exo/src/cpp/nml_hand_exo
```

2. Open `nml_hand_exo.ino` in Arduino IDE

3. Configure `config.h` for your hardware:
```cpp
// Motor IDs (must match Dynamixel IDs programmed via Wizard)
const uint8_t MOTOR_IDS[] = {1, 2, 3, 4, 5};
const uint8_t NUM_MOTORS = 5;

// Joint limits [min, max] in degrees
const float JOINT_LIMITS[][2] = {
  {-10.0, 90.0},   // Thumb
  {-5.0, 85.0},    // Index
  {-5.0, 85.0},    // Middle
  {-5.0, 85.0},    // Ring
  {-5.0, 85.0}     // Pinky
};

// Serial communication
#define COMMAND_SERIAL Serial  // USB serial
#define DXL_SERIAL Serial1     // Dynamixel serial
#define BAUD_RATE 1000000
```

4. Click **Upload** (Ctrl+U)

---

# NMLHandExo Class

The main class for controlling the exoskeleton.

## Constructor

```cpp
NMLHandExo(const uint8_t* ids, uint8_t numMotors, 
           const float jointLimits[][2], 
           const float* homeState = nullptr);
```

**Parameters:**
- `ids` - Array of motor IDs
- `numMotors` - Number of motors
- `jointLimits` - 2D array of [min, max] angles in degrees
- `homeState` - Optional home positions (defaults to zero)

**Example:**
```cpp
const uint8_t MOTOR_IDS[] = {1, 2, 3, 4, 5};
const float JOINT_LIMITS[][2] = {
  {-10.0, 90.0},
  {-5.0, 85.0},
  {-5.0, 85.0},
  {-5.0, 85.0},
  {-5.0, 85.0}
};

NMLHandExo exo(MOTOR_IDS, 5, JOINT_LIMITS);
```

---

## Initialization

### `initializeSerial(int baud)`
Initialize the Dynamixel serial port.

```cpp
exo.initializeSerial(1000000);
```

### `initializeMotors()`
Configure all motors: disable torque, set position mode, re-enable torque.

```cpp
exo.initializeMotors();
```

---

## Motor Control

### Position Control

```cpp
// Get relative angle (degrees) - relative to home position
float angle = exo.getRelativeAngle(motorID);

// Set relative angle (degrees)
exo.setRelativeAngle(motorID, 45.0);

// Get absolute angle (degrees) - raw encoder value
float absAngle = exo.getAbsoluteAngle(motorID);

// Set absolute angle (degrees)
exo.setAbsoluteAngle(motorID, 2048.0);
```

### Velocity Control

```cpp
// Get current velocity (units depend on mode)
float vel = exo.getVelocity(motorID);

// Set velocity
exo.setVelocity(motorID, 100.0);
```

### Torque/Current

```cpp
// Enable/disable motor torque
exo.enableTorque(motorID);
exo.disableTorque(motorID);

// Check torque state
bool enabled = exo.isTorqueEnabled(motorID);

// Get current draw (mA)
float current = exo.getCurrent(motorID);

// Set current limit (mA)
exo.setCurrentLimit(motorID, 1000.0);
```

---

## Homing & Calibration

```cpp
// Set current position as zero offset for a motor
exo.setZeroOffset(motorID);

// Get stored zero offset
float offset = exo.getZeroOffset(motorID);

// Reset all motors to current positions
exo.resetAllZeros();

// Start calibration mode
exo.beginCalibration(enableTimed=true, duration=5);

// Check calibration status
if (exo.isExoCalibrating()) {
  exo.updateCalibration();
}
```

---

## Operating Modes

```cpp
// Set exo operating mode
exo.setExoOperatingMode("GESTURE_FIXED");
// Options: "FREE", "GESTURE_FIXED", "GESTURE_CONTINUOUS", "GESTURE_CALIBRATION"

// Get current mode
String mode = exo.getExoOperatingMode();

// Cycle through modes
exo.cycleExoOperatingMode();

// Set motor control mode
exo.setMotorControlMode("position");
// Options: "position", "velocity", "current_position", "extended_position"

// Get motor control mode
String motorMode = exo.getMotorControlMode();
```

---

## Gestures

### GestureController Class

```cpp
#include "gesture_controller.h"

// Create gesture controller
GestureController gestureCtrl(&exo);

// Set a gesture by name
gestureCtrl.setGesture("IndexPinch");

// Get current gesture
String currentGesture = gestureCtrl.getCurrentGesture();

// Set gesture state
gestureCtrl.setGestureState("open");   // or "close"

// Get gesture state
String state = gestureCtrl.getGestureState();

// Cycle through gestures
gestureCtrl.cycleGesture();

// Cycle gesture state
gestureCtrl.cycleGestureState();

// Get list of available gestures
String gestures = gestureCtrl.getGestureList();
```

### Built-in Gestures

The gesture library includes:
- **Fist** - All fingers closed
- **Open** - All fingers extended
- **IndexPinch** - Thumb-index pinch
- **MiddlePinch** - Thumb-middle pinch
- **RingPinch** - Thumb-ring pinch
- **PinkyPinch** - Thumb-pinky pinch
- **ThreeFingerPinch** - Thumb-index-middle
- **Point** - Index extended, others closed
- **Peace** - Index and middle extended

---

## Utility Functions

```cpp
// Get motor ID by name
int id = exo.getMotorIDByName("THUMB");

// Get motor ID by index
uint8_t id = exo.getMotorIDByIndex(0);

// Get motor name by ID
String name = exo.getMotorNameByID(1);

// Get device info string
String info = exo.getDeviceInfo();

// Get motor count
int count = exo.getMotorCount();

// Update exo state (call in loop)
exo.update();
```

---

# Serial Command Protocol

The firmware listens for semicolon-delimited commands on the serial port.

## Command Format

```
command:arg1:arg2;
```

**Rules:**
- Commands are case-insensitive
- Arguments separated by `:`
- Commands end with `;` (configurable)
- Motor ID can be `all` or a specific number

## Common Commands

### Motor Control
```
enable:all;                    # Enable all motors
disable:2;                     # Disable motor 2
set_angle:1:45.0;             # Set motor 1 to 45 degrees
get_angle:1;                   # Get motor 1 angle
set_velocity:all:100;          # Set all motors to velocity 100
```

### Homing
```
home:all;                      # Home all motors
set_home:1;                    # Set motor 1 current position as home
get_home:3;                    # Get motor 3 home position
```

### Limits
```
set_current_limit:1:1000;      # Set motor 1 current limit to 1000mA
set_limits:2:-5.0:85.0;        # Set motor 2 angle limits
get_limits:1;                  # Get motor 1 limits
```

### Gestures
```
set_gesture:IndexPinch;        # Set gesture to IndexPinch
get_gesture;                   # Get current gesture
set_gesture_state:open;        # Set state to open
cycle_gesture;                 # Cycle to next gesture
get_gesture_list;              # List all gestures
```

### System
```
help;                          # List available commands
version;                       # Get firmware version
info;                          # Get device info
reboot;                        # Reboot controller
```

### LEDs
```
led:1:on;                      # Turn on motor 1 LED
led:all:off;                   # Turn off all LEDs
```

---

# Example Sketch

```cpp
#include "nml_hand_exo.h"
#include "gesture_controller.h"
#include "config.h"

// Motor configuration
const uint8_t MOTOR_IDS[] = {1, 2, 3, 4, 5};
const uint8_t NUM_MOTORS = 5;
const float JOINT_LIMITS[][2] = {
  {-10.0, 90.0},  // Thumb
  {-5.0, 85.0},   // Index
  {-5.0, 85.0},   // Middle
  {-5.0, 85.0},   // Ring
  {-5.0, 85.0}    // Pinky
};

// Create exo instance
NMLHandExo exo(MOTOR_IDS, NUM_MOTORS, JOINT_LIMITS);
GestureController gestureCtrl(&exo);

void setup() {
  Serial.begin(1000000);
  exo.initializeSerial(1000000);
  exo.initializeMotors();
  
  Serial.println("NML Hand Exo Ready!");
  Serial.println(exo.getDeviceInfo());
}

void loop() {
  // Process serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil(';');
    processCommand(cmd);
  }
  
  // Update exo state
  exo.update();
}

void processCommand(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("enable:")) {
    String arg = cmd.substring(7);
    if (arg == "all") {
      for (int i = 0; i < NUM_MOTORS; i++) {
        exo.enableTorque(MOTOR_IDS[i]);
      }
    }
  }
  else if (cmd.startsWith("set_angle:")) {
    // Parse: set_angle:1:45.0
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    int motorID = cmd.substring(firstColon + 1, secondColon).toInt();
    float angle = cmd.substring(secondColon + 1).toFloat();
    exo.setRelativeAngle(motorID, angle);
  }
  else if (cmd == "help") {
    Serial.println("Available commands: enable, disable, set_angle, get_angle, home, info");
  }
}
```

---

# Advanced Topics

## Custom Gestures

Define custom gestures in `gesture_library.cpp`:

```cpp
void initializeGestures(GestureLibrary& library) {
  // Add custom gesture
  Gesture customGesture;
  customGesture.name = "MyCustomGesture";
  customGesture.numStates = 2;
  
  // State 0: open
  customGesture.states[0].angles[0] = 0.0;   // Thumb
  customGesture.states[0].angles[1] = 0.0;   // Index
  customGesture.states[0].angles[2] = 0.0;   // Middle
  // ... etc
  
  // State 1: closed
  customGesture.states[1].angles[0] = 45.0;
  customGesture.states[1].angles[1] = 80.0;
  // ... etc
  
  library.addGesture(customGesture);
}
```

## IMU Integration

```cpp
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  if (!bno.begin()) {
    Serial.println("IMU initialization failed!");
  }
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  
  float roll = event.orientation.x;
  float pitch = event.orientation.y;
  float yaw = event.orientation.z;
  
  // Use IMU data for control...
}
```

---

# Troubleshooting

**Motor not responding:**
- Check Dynamixel ID matches firmware configuration
- Verify baud rate (1000000 default for USB/Dynamixel; 115200 for HC-05)
- Check power supply (12V, adequate current)
- Test motor individually with Dynamixel Wizard

**Serial communication issues:**
- Ensure correct COM port selected
- Verify 1000000 for USB/Dynamixel or 115200 for HC-05
- Check USB cable quality
- Use `help;` command to verify connection

**Gestures not executing:**
- Verify motors are enabled: `enable:all;`
- Check joint limits in `config.h`
- Ensure gesture exists: `get_gesture_list;`

**Compilation errors:**
- Install Dynamixel2Arduino library via Library Manager
- Update Arduino IDE to latest version
- Check board support package is installed

---

# Next Steps

<div class="feature__wrapper">
  <div class="feature__item">
    <div class="archive__item">
      <div class="archive__item-body">
        <h2 class="archive__item-title">🐍 Python API</h2>
        <div class="archive__item-excerpt">
          <p>Control the exoskeleton from your PC using the Python package.</p>
        </div>
        <p><a href="/NML_Hand_Exo/python-api/" class="btn btn--primary">View Python API</a></p>
      </div>
    </div>
  </div>

  <div class="feature__item">
    <div class="archive__item">
      <div class="archive__item-body">
        <h2 class="archive__item-title">🛠️ Assembly</h2>
        <div class="archive__item-excerpt">
          <p>Hardware assembly and firmware flashing instructions.</p>
        </div>
        <p><a href="/NML_Hand_Exo/assembly/" class="btn btn--primary">View Assembly</a></p>
      </div>
    </div>
  </div>

  <div class="feature__item">
    <div class="archive__item">
      <div class="archive__item-body">
        <h2 class="archive__item-title">🚀 Examples</h2>
        <div class="archive__item-excerpt">
          <p>Example projects and tutorials.</p>
        </div>
        <p><a href="/NML_Hand_Exo/examples/" class="btn btn--primary">View Examples</a></p>
      </div>
    </div>
  </div>
</div>
