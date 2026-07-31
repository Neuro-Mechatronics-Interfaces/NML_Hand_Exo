# Motor Control Examples

Examples demonstrating motor control, configuration, and batch operations.

## Examples

### 1. `motor_test.py`
**Simple motor movement test**

Quick test moving a single motor to different positions.

```bash
python motor_test.py
```

---

### 2. `example_motor_config.py`
**Velocity, acceleration, and limit configuration**

Configure motor motion profiles and safety limits.

```bash
python example_motor_config.py
```

**What it demonstrates:**
- Query current velocity/acceleration
- Set custom motion profiles (slower/faster)
- Configure joint angle limits
- Test movement with new settings
- Restore original configuration

---

### 3. `example_batch_operations.py`
**Batch operations using 'all' keyword**

Control all motors simultaneously.

```bash
python example_batch_operations.py
```

**What it demonstrates:**
- Reading all motor states at once
- Enabling/disabling all motors
- Synchronized position commands
- LED control for all motors
- Current monitoring across all motors

---

## Key Concepts

**Motor IDs vs Names:**
- Query `exo.info()["motors"]` and use the reported numeric Dynamixel IDs.
- In dual firmware, always use those integer IDs rather than a bare motor name.
- Use 'all' to command all motors simultaneously

**Motion Profiles:**
- Velocity: Maximum speed (deg/s)
- Acceleration: Rate of speed change (deg/s²)
- Limits: Safe angle bounds (lower, upper)

**Relative vs Absolute Angles:**
- Relative: Position relative to home/zero (most common)
- Absolute: Raw encoder position
