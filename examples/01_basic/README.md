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

## Next Steps

After mastering these basics, move on to:
- `02_motor_control/` - Control motors precisely
- `03_sensors/` - Work with IMU and sensors
- `04_advanced/` - Advanced configuration
