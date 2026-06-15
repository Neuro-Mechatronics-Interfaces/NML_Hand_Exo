# Sensor Examples - IMU Integration

Examples demonstrating IMU sensor reading and control.

## Examples

### 1. `imu/imu_serial.py`
**Continuous IMU data reading**

Read and display roll, pitch, and yaw angles in real-time.

```bash
python imu/imu_serial.py
```

**What it demonstrates:**
- Reading IMU orientation data
- Continuous polling loop
- Proper cleanup with Ctrl+C

---

### 2. `example_imu_control.py`
**IMU-based motor control**

Control wrist motor angle based on IMU orientation feedback.

```bash
python example_imu_control.py
```

**What it demonstrates:**
- Reading current IMU heading (yaw)
- Setting target wrist angles via IMU
- Flex vs extend direction control
- Closed-loop IMU feedback
- Short-form direction codes ('f', 'e')

---

## IMU Axes

The IMU provides 3-axis orientation:
- **Heading (Yaw)**: Rotation around vertical axis (0-360°)
- **Pitch**: Forward/backward tilt (-90 to 90°)
- **Roll**: Left/right tilt (-180 to 180°)

## IMU-Based Control

`set_yaw_angle(motor_id, target_angle, direction)` uses closed-loop feedback:

1. Reads current IMU yaw
2. Moves motor incrementally
3. Checks IMU yaw after each step
4. Stops when target reached (within 0.5°) or timeout (150 steps)

**Direction options:**
- `"flex"` or `"f"` - Flexion movement
- `"extend"` or `"e"` - Extension movement

---

## Applications

IMU-based control is useful for:
- Wrist angle stabilization
- Orientation-dependent assistance
- Tremor compensation
- Gravity compensation
