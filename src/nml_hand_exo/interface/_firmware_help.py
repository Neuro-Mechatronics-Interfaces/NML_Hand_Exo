"""The firmware's human-readable ``help`` response.

The firmware currently emits this response with one ``commandPrint`` call per
line.  That is useful on an interactive serial monitor, but it is not a
single request/reply frame on dual-CDC transports.  Keep the supported
Python-facing presentation in one host-side value so GUI and SDK clients can
provide the same help command without consuming unrelated replies.  The
firmware-only ``info_verbose`` diagnostic is intentionally omitted because it
does not provide a bounded response on the Python/Protobuf path.

Update this text when the corresponding supported portion of the help block in
``utils.cpp`` changes.
"""

FIRMWARE_HELP_TEXT = """ ================================== List of commands ======================================
 led                   |  ID/NAME/ALL:ON/OFF  | // Turn motor or system LED on/off
 help                  |                      | // Display available commands
 home                  |  ID/NAME/ALL         | // Set specific motor (or all) to home position
 info                  |                      | // Gets information about exo device. Returns string of metadata with comma delimiters
 debug                 |  ON/OFF              | // Set verbose output on/off
 set_reply_route       |  BOTH/TELEM/CMD      | // Route replies: both(legacy)/telem(decoupled)/cmd (dual-CDC)
 get_reply_route       |                      | // Get current reply/telemetry CDC route
 check_limits          |                      | // Per-motor gesture span; flags NO_TRAVEL / HOME_OUTSIDE joints
 io_profile            |                      | // Exclusive stage times and call maxima
 reset_io_profile      |                      | // Reset I/O and loop statistics
 loop_stats            |                      | // Loop period: n / mean / max microseconds
 reset_loop_stats      |                      | // Clear loop statistics
 reboot                |  ID/NAME/ALL         | // Reboot motor
 version               |                      | // Get current software version
 enable                |  ID/NAME             | // Enable torque for motor
 disable               |  ID/NAME             | // Disable torque for motor
 enable_ids            |  ID:ID:ID...         | // Enable torque for a list of DXL IDs
 disable_ids           |  ID:ID:ID...         | // Disable torque for a list of DXL IDs
 get_enable            |  ID/NAME             | // Get the torque enable status of the motor
 get_baud              |  ID/NAME             | // Get baud rate for motor
 set_baud              |  ID/NAME:VALUE       | // Set baud rate for motor
 get_goal_velocity     |  ID/NAME             | // Get current velocity profile for motor
 set_goal_velocity     |  ID/NAME/ALL:VALUE   | // Set velocity profile for motor
 get_velocity          |  ID/NAME/ALL         | // Get present velocity in rpm
 set_velocity          |  ID:SIGNED_RPM       | // Direct velocity command (velocity mode)
 get_goal_acceleration |  ID/NAME             | // Get current acceleration profile for motor
 set_goal_acceleration |  ID/NAME/ALL:VALUE   | // Set acceleration limit for motor
 get_home              |  ID/NAME             | // Get stored zero position
 set_home              |  ID/NAME:VALUE       | // Set current position as new zero angle
 get_angle             |  ID/NAME             | // Get relative motor angle
 set_angle             |  ID/NAME:ANGLE       | // Set motor angle
 get_absolute_angle    |  ID/NAME/ALL         | // Get absolute motor angle
 set_absolute_angle    |  ID/NAME:ANGLE       | // Set absolute motor angle
 get_torque            |  ID/NAME             | // Get torque output reading from motor
 get_current           |  ID/NAME             | // Get current draw from motor
 set_current_lim       |  ID/NAME:VAL         | // Set per-motor current draw limit
 set_total_current_lim |  MA                  | // Set COMBINED current budget across all motors
 get_total_current_lim |                      | // Get combined current budget
 set_hold_current      |  MA                  | // Current allowed to a settled/shed motor
 get_hold_current      |                      | // Get hold current
 set_current_governor  |  ON/OFF              | // Closed-loop budget enforcement (off = static clamp)
 current_status        |                      | // Budget, measured aggregate draw, per-motor allocation
 get_goal_current      |  ID/NAME/ALL         | // Get direct current goal in mA
 set_current           |  ID:SIGNED_MA        | // Direct current command (current mode)
 stop                  |  ID/ALL              | // Zero direct velocity/current goals
 hold_position         |  ID:ANGLE[:MA]       | // Hold one explicit ID with optional per-hold current
 release_hold          |  ID                  | // Disable held ID and restore global mode
 set_command_timeout   |  MILLISECONDS        | // Set direct-control watchdog (50-5000 ms)
 get_telemetry_fast    |  ID:ID:ID.../ALL     | // Binary current/velocity/position telemetry frame
 set_utc_time          | UNIX_MS | // Synchronize UTC; no effect on control timing
 get_utc_time          |         | // UTC ms, synchronization flag and uptime
 assist_enable         | THRESHOLD_MA:GAIN:CAP_MA:INERTIA:VISCOUS:COULOMB:ID[:ID...] | // Grouped startup, automatic bias and activation
 assist_config         | ID:THRESHOLD_MA:VELOCITY_GAIN_DEG_S_PER_MA:CAP_MA | // Configure secondary-goal assist while off
 assist_gains          | ID:POSITION_P:POSITION_D | // Configure Assist position gains while off
 assist_settle_config  | VELOCITY_LIMIT_DEG_S:POSE_DRIFT_DEG:INTENT_DELAY_S | // Configure settling and Assist intent dwell while off
 assist_observer       | ID:INERTIA:VISCOUS:COULOMB | // Nm-based observer model terms; all zero = current residual
 assist_calibrate      | ID[:ID...]           | // Relaxed bias capture; selected motors enabled in CURRENT_POSITION
 assist_start / assist_stop / assist_heartbeat / assist_status | // Explicit assist, 1-second lease, measured status
 reset_motors          | ID[:ID...]           | // Safe torque-off, re-probe, and telemetry recovery
 set_joint_model       | ID:GAIN:TAU:VMAX:STIFFNESS:MOMENT | // Tune telemetry model only
 get_joint_model       | ID | // Query model and trigger threshold
 set_estimate_holdoff  | MS | // Telemetry quiet period (50..5000 ms)
 set_joint_trigger     | ID:NM | // Store future assistance threshold; no motion
 clear_joint_trigger   | ID | // Clear threshold
 telemetry_diag        |  ID:ID:ID.../ALL     | // Text diagnostics for fast telemetry reads
 shadow_config         |  INTERVAL:ID:ID...   | // Configure read-only contact evidence sampling
 shadow_start          |  None                 | // Start sampling (VELOCITY mode only)
 shadow_stop           |  None                 | // Stop sampling; no motor state changes
 shadow_status         |  None                 | // Buffered current/position/derived-velocity snapshot
 get_motor_limits      |  ID/NAME             | // Get motor limits (upper and lower bounds)
 set_motor_limits      |  ID/NAME:VAL:VAL     | // Set motor limits (upper and lower bounds)
 set_upper_limit       |  ID/NAME:ANGLE       | // Set the absolute upper bound position limit for the motor
 set_lower_limit       |  ID/NAME:ANGLE       | // Set the absolute lower bound position limit for the motor
 get_motor_mode        |                      | // Get motor control mode
 set_motor_mode        |  VALUE               | // Set POSITION, CURRENT_POSITION, VELOCITY, or CURRENT
 set_control_mode      |  ALL:VALUE           | // Safe direct-mode alias; torque remains off
 get_exo_mode          |                      | // Get exo device operation mode
 set_exo_mode          |  VALUE               | // Set exo device operation mode (FREE', 'GESTURE_FIXED', 'GESTURE_CONTINUOUS')
 gesture_list          |                      | // Get gestures in library
 set_gesture           |  NAME:VALUE          | // Set exo gesture
 get_gesture           |                      | // Get exo gesture
 set_gesture_state     |  NAME:VALUE          | // Set exo gesture state
 set_gesture_angle     |  NAME:0-100          | // Interpolate a gesture: 0=its extend state, 100=its flex state
 set_finger_angles     |  T:I:M:R:P[:W]       | // Batch signed -100..100 (thumb:index:middle:ring:pinky[:wrist]): -100 extend, 0 rest, +100 flex; empty holds
 get_gesture_angle     |  NAME/ALL            | // Read positions as 0-100 (101/102 out of range, 255 no travel)
 get_gesture_sang      |  NAME/ALL            | // Read signed degrees from rest: flex positive, extend negative
 get_gesture_angles    |  NAME/ALL            | // Read <percent-code>,<signed-degrees> pairs
 get_gesture_state     |                      | // Get exo gesture state
 cycle_gesture         |                      | // Executes the next gesture in the library
 cycle_gesture_state   |                      | // Cycles the next gesture state
 set_zero_offset       |  ID/NAME/ALL:VALUE   | // Set the zero offset for a motor to an arbitrary angle
 set_flip              |  ID/NAME:0/1         | // Set motor direction flip (1=inverted, 0=normal)
 get_flip              |  ID/NAME/ALL         | // Get motor direction flip status
 calibrate_exo         |  VALUE:VALUE         | // start the calibration routine for the exo
 calibrate_rom         |  ID:FLEX/EXTEND      | // Impedance ROM sweep of one joint (needs CURRENT mode); reports ROM_CAL_RESULT
 calibrate_rom_gesture |  NAME:FLEX/EXTEND    | // Impedance ROM sweep of every motor a gesture drives (thumb/wrist span several)
 cancel_rom            |                      | // Abort a running impedance ROM sweep and return the joint home
 get_imu               |                      | // Returns list of accel & gyro values
 set_yaw_angle         |  ID/NAME:ANGLE       | // Set motor angle via IMU wrist angle
 oled                  |  VALUE               | // Turn OLED on/off, get status
 ==========================================================================================
"""

# These parser branches intentionally do not call ``commandPrint`` on the
# legacy ASCII/dual-CDC transport.  Protobuf motion handlers may synthesize an
# acknowledgement for some of the same command names, so this set is used
# only after a transaction has actually returned an empty reply.
FIRMWARE_SILENT_COMMANDS = frozenset({
    "enable",
    "disable",
    "set_goal_velocity",
    "set_goal_acceleration",
    "set_angle",
    "set_absolute_angle",
    "set_home",
    "set_upper_limit",
    "set_lower_limit",
    "set_motor_limits",
    "led",
    "set_exo_mode",
    "set_zero_offset",
    "set_flip",
    "reboot",
    "calibrate_exo",
    "cycle_gesture",
    "cycle_gesture_state",
})


def firmware_help_text() -> str:
    """Return a fresh copy of the firmware-compatible help response."""
    return FIRMWARE_HELP_TEXT


def firmware_command_may_be_silent(command: str) -> bool:
    """Return whether a known firmware command may validly emit no reply."""
    head = str(command).strip().split(":", 1)[0].lower()
    return head in FIRMWARE_SILENT_COMMANDS
