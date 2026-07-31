# Advanced Configuration Examples

Examples demonstrating advanced device configuration and debugging features.

## Examples

### 1. `example_advanced_config.py`
**Debug, OLED, motor modes, and baudrate**

Comprehensive example of advanced device settings.

```bash
python example_advanced_config.py
```

**What it demonstrates:**

#### Debug Mode Control
- Enable/disable verbose Arduino output
- Useful for firmware debugging
- `set_debug(True)` / `set_debug(False)`

#### OLED Display Control
- Query OLED status
- Enable/disable display
- `enable_oled()` / `disable_oled()` / `get_oled_status()`

#### Motor Control Modes
Switch between control modes:
- **position**: Standard position control (default)
- **velocity**: Velocity-based control
- **current_position**: Current-based position control

#### Exo Operating Modes
Switch between operating modes:
- **FREE**: Manual control via API
- **GESTURE_FIXED**: Fixed gesture execution
- **GESTURE_CONTINUOUS**: Continuous gesture tracking

#### Baudrate Settings
- Query motor communication baudrate
- Change baudrate (requires reconnection)
- `get_baudrate()` / `set_baudrate()`

#### Device Information
- Complete device metadata
- Motor configuration summary
- Version and status info

---

## UART Communication

The `UART_uno_pico/` folder contains examples for:
- Arduino Uno ↔ Raspberry Pi Pico communication
- Custom UART protocols
- Bridging different microcontrollers

---

## When to Use These Features

**Debug Mode:**
- Firmware development
- Troubleshooting communication issues
- Understanding command processing

**OLED Control:**
- Battery saving (disable when not needed)
- Reducing visual distractions
- Custom display management

**Motor Modes:**
- Position mode: Standard point-to-point movement
- Velocity mode: Speed control applications
- Current mode: Force/torque control

**Exo Modes:**
- FREE: API-driven control (development)
- GESTURE_FIXED: Pre-programmed gesture library
- GESTURE_CONTINUOUS: Real-time gesture streaming

---

## Caution

⚠️ **Baudrate Changes:**
Changing motor baudrate requires:
1. Device reconnection
2. Matching host baudrate
3. Potential re-flashing if settings are lost

Only change baudrate if you know what you're doing!
