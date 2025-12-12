# Basic Examples - Getting Started

This folder contains introductory examples for connecting to and communicating with the NML Hand Exoskeleton.

## Examples

### 1. `example_serial_exo.py`
**Basic serial connection and device queries**

Connect via USB serial port and query device information.

```bash
python example_serial_exo.py
```

**What it demonstrates:**
- Creating a SerialComm interface
- Connecting to the exoskeleton
- Querying version, mode, angles, velocities
- Reading gesture states
- Proper connection cleanup

---

### 2. `example_tcp_exo.py`
**TCP/IP connection over WiFi**

Connect to a Pico W or similar device over network.

```bash
python example_tcp_exo.py
```

**What it demonstrates:**
- Creating a TCPComm interface
- Network-based communication
- Same API works for both serial and TCP

---

### 3. `hand_exo_cli.py`
**Command-line interface tool**

Full-featured CLI for device control and monitoring.

```bash
# List available ports
python hand_exo_cli.py --list-ports

# Connect and get info
python hand_exo_cli.py --connect COM5 --info

# Home all motors
python hand_exo_cli.py --connect COM5 --home

# Send commands
python hand_exo_cli.py --connect COM5 --send "led:1:on"

# Monitor output
python hand_exo_cli.py --connect COM5 --monitor
```

**What it demonstrates:**
- Port detection
- Command sending
- Serial monitoring
- Error handling

---

## Next Steps

After mastering these basics, move on to:
- `02_motor_control/` - Control motors precisely
- `03_sensors/` - Work with IMU and sensors
- `04_advanced/` - Advanced configuration
