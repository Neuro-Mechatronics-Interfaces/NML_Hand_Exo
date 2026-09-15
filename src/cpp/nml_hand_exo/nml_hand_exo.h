/**
 * @file nml_hand_exo.h
 * @brief API header for the NML Hand Exoskeleton device.
 *
 * This file declares the NMLHandExo class and helper utilities for managing
 * and controlling the exoskeleton using Dynamixel servos.
 */
#ifndef NML_HAND_EXO_H
#define NML_HAND_EXO_H

#include "config.h"
#include "joint_state_model.h"
#include "rom_pulse_fit.h"
#include "utc_clock.h"
#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

/// @brief Verbose output toggle for debugging.
extern bool VERBOSE;

/// @brief Where command replies / telemetry are emitted (dual-CDC).
/// One of REPLY_ROUTE_BOTH / REPLY_ROUTE_TELEM / REPLY_ROUTE_CMD (see config.h).
/// Default REPLY_ROUTE_BOTH keeps legacy single-port hosts working.
extern uint8_t gReplyRoute;

/// @brief Bluetooth serial stream used for commands.
//extern Stream& COMMAND_SERIAL;
//extern Stream* debugStream;

/// @brief Debugging print helper function.
/// @param msg The message to print.
void debugPrint(const String& msg);

/// @brief Emits a line to the reply/telemetry CDC(s) per gReplyRoute.
/// @param msg The message to print.
void telemetryPrintln(const String& msg);

/// @brief Mode press function
void onModeButtonPress();

/// @brief Enum to manage the operating mode of the exo
enum ExoOperatingMode {
  FREE = 0,
  GESTURE_FIXED,
  GESTURE_CONTINUOUS,
  GESTURE_CALIBRATION
};

struct __attribute__((packed)) FastTelemetryRecord {
  uint8_t id;
  uint8_t error;
  int16_t current_mA;
  int32_t velocity_raw;
  int32_t position_ticks;
  int32_t absolute_cdeg;
  int32_t relative_cdeg;
  uint8_t sources; // two bits each: position, current, velocity; 0/1/2 = unavailable/measured/estimated
  uint64_t utc_ms; // Unix UTC milliseconds; zero until synchronized
  uint32_t sample_ms; // sample uptime (millis), independent of UTC adjustments
};

static constexpr uint8_t TELEM_SOURCE_ESTIMATED = 0x80;
static_assert(sizeof(FastTelemetryRecord) == 33, "NX v2 wire record size");

/// @brief One absolute motor target for a multi-motor Sync Write.
struct MotorAngleTarget {
  uint8_t id;
  float angleDeg;
};

/// @brief Buffered, read-only evidence used by the Phase-1 shadow estimator.
struct ShadowTelemetryRecord {
  uint8_t id = 0;
  uint8_t error = 0;
  int16_t current_mA = 0;
  int32_t position_ticks = 0;
  int32_t absolute_cdeg = 0;
  int32_t relative_cdeg = 0;
  int32_t velocity_cdeg_s = 0;
  uint32_t current_sample_ms = 0;
  uint32_t position_sample_ms = 0;
};

/// @brief Outcome of one commanded move, reported in GESTURE_RESULT lines.
///
/// The command ack only says the firmware ACCEPTED a goal; these say what the
/// motor then did. "Accepted but never moved" is otherwise invisible to a host.
enum MoveVerdict : uint8_t {
  MOVE_VERDICT_NONE = 0,
  MOVE_VERDICT_REACHED,   ///< Stopped within tolerance of its goal.
  MOVE_VERDICT_STALLED,   ///< Drew hard without arriving, or was load-shed.
  MOVE_VERDICT_SHORT,     ///< Stopped pulling, but not at its goal.
  MOVE_VERDICT_STARVED    ///< Never funded: the budget had no slot for it.
};

enum FastTelemetryMethod : uint8_t {
  FAST_TELEM_METHOD_FAILED = 0,
  FAST_TELEM_METHOD_FALLBACK_READ = 1,
  FAST_TELEM_METHOD_FAST_SYNC_READ = 2,
  FAST_TELEM_METHOD_SYNC_READ = 3,
  FAST_TELEM_METHOD_MODEL = 4
};

/// @brief Direction of an impedance ROM calibration sweep.
///
/// Expressed on the same home->flexion axis the gesture system uses
/// (getGestureSpan): FLEX drives toward the flexion endstop, EXTEND toward the
/// extension endstop. The routine resolves the physical current sign for each
/// joint at runtime, so this stays flip-flag agnostic.
enum RomCalDirection : uint8_t {
  ROM_CAL_DIR_FLEX = 0,
  ROM_CAL_DIR_EXTEND
};

/// @brief State of the non-blocking impedance ROM calibration state machine.
enum RomCalPhase : uint8_t {
  ROM_CAL_IDLE = 0,   ///< No sweep running.
  ROM_CAL_RAMP,       ///< Powered portion of one bounded current pulse.
  ROM_CAL_REST,       ///< Zero current; wait for feedback before another pulse.
  ROM_CAL_RETURN,     ///< Endstop found (or sweep aborted); returning to home.
  ROM_CAL_DONE        ///< Terminal; result already reported. Cleared to IDLE.
};

/// @brief Why an impedance ROM calibration sweep ended, reported to the host.
enum RomCalStatus : uint8_t {
  ROM_CAL_STATUS_NONE = 0,
  ROM_CAL_STATUS_OK,        ///< Endstop found by motion onset + resisted nudge.
  ROM_CAL_STATUS_CEILING,   ///< Reached the current ceiling without moving.
  ROM_CAL_STATUS_TIMEOUT,   ///< Overall watchdog fired before concluding.
  ROM_CAL_STATUS_ABORTED,   ///< Cancelled, or a precondition failed mid-sweep.
  ROM_CAL_STATUS_LIMIT      ///< Stored joint boundary reached; not a physical endstop.
};

/// @brief Class to manage the NML Hand Exoskeleton, providing initialization, motor control, and telemetry.
class NMLHandExo {
  public:
    bool setUtcTime(uint64_t epochMs) { return utcClock_.set(epochMs, millis()); }
    uint64_t utcTime() { return utcClock_.now(millis()); }
    bool telemetryEstimated();
    void serviceJointModels();
    bool setJointModel(uint8_t id, const JointModelParams& params);
    String getJointModel(uint8_t id);
    bool setEstimateHoldoff(uint32_t ms);
    bool setJointTrigger(uint8_t id, float torque);
    bool clearJointTrigger(uint8_t id);
    /// @brief Constructor.
    /// @param ids Pointer to array of motor IDs.
    /// @param numMotors Number of motors in the device.
    /// @param jointLimits Pointer to 2D array defining [min, max] joint limits in degrees.
    /// @param homeState Optional array of home positions (in degrees). Defaults to zero offsets.
    NMLHandExo(const uint8_t* ids, uint8_t numMotors, const float jointLimits[][2], const float* homeState = nullptr);

    /// @brief Destructor.
    ~NMLHandExo() {
      delete[] jointLimits_;
      delete[] zeroOffsets_;
      delete[] currentLimits_;
      delete[] flipMotor_;
      delete[] lastDirectCommandMs_;
      delete[] directCommandActive_;
      delete[] directCommandDirection_;
      delete[] directVelocityLimitBlock_;
      delete[] directVelocityLimitVerified_;
      delete[] positionHoldActive_;
      delete[] appliedCurrents_;
      delete[] motorMoving_;
      delete[] motorAdmitted_;
      delete[] admissionMs_;
      delete[] goalAngle_;
      delete[] goalAngleValid_;
      delete[] motorReachable_;
      delete[] verdictPending_;
      delete[] lastVerdict_;
      delete[] goalIssuedMs_;
      delete[] stallSinceMs_;
    }

    // -----------------------------------------------------------
    // Utility functions
    // -----------------------------------------------------------

    /// @brief Initialize the serial port for Dynamixel communication.
    /// @param baud Baud rate to initialize.
    void initializeSerial(int baud);

    /// @brief Initialize all motors: disables torque, sets position mode, then re-enables torque.
    void initializeMotors();

    /// @brief Gets the motor ID according to the passed index
    /// @param index Index of array
    uint8_t getMotorIDByIndex(const int index);

    /// @brief Get the current operating mode of the motors
    /// @return The motor control mode.
    String getMotorControlMode();

    /// @brief Set the operating mode of the motors
    /// @param name Name of the mode (e.g. "position", "current_position", "velocity").
    bool setMotorControlMode(const String& name);

    /// @brief Get the motor ID from a user-supplied token (either name or ID as a string).
    /// @param token The token string (e.g. "WRIST" or "1").
    /// @return The motor ID or -1 if not found.
    int getMotorID(const String& token);

    /// @brief Get the index of a motor in the internal arrays from its ID.
    /// @param id The motor ID.
    /// @return The index in the motor array or -1 if not found.
    int getIndexById(uint8_t id);

    /// @brief Get the motor ID by name.
    /// @param name Name of the motor (e.g. "WRIST").
    /// @return The motor ID or -1 if not found.
    int getMotorIDByName(const String& name);

    // @brief Get the name of the motor by index
    // @param index Index of array
    //const char* getMotorName(int index);

    /// @brief Set the unique names for motors (Must match the number of IDs)
    /// @param names A list of "names" separated by comma.
    void setMotorNames(const char* const* names);

    /// @brief Get the motor name from its ID.
    /// @param id The motor ID.
    /// @return The name of the motor.
    String getMotorNameByID(uint8_t id);

    /// @brief Convert a relative angle (degrees) to Dynamixel tick counts.
    /// @param angle_deg Angle in degrees.
    /// @param index Index of the motor.
    /// @return Ticks equivalent to the angle.
    int angleToTicks(float angle_deg, int index);

    /// @brief Calibrate the zero offset for a motor by reading its current position.
    /// @param id Motor ID.
    void setZeroOffset(uint8_t id);

    /// @brief Get the stored zero offset for a motor.
    /// @param id Motor ID.
    /// @return Zero offset in degrees.
    float getZeroOffset(uint8_t id);

    /// @brief Fill one compact telemetry record for host streaming.
    /// @param id Motor ID.
    /// @param record Destination record.
    /// @return True if the record has a usable position.
    bool getFastTelemetryRecord(uint8_t id, FastTelemetryRecord& record);
    // Read-only Axon sampler: one position transaction, bounded to 2 ms.
    // Returns raw encoder angle in tenths of a degree; no wrap/clamp or motion.
    bool readAxonAngle(uint8_t id, int16_t& angle);

    /// @brief Read compact telemetry records for multiple motor IDs.
    /// @param ids Array of requested Dynamixel IDs.
    /// @param count Number of requested IDs.
    /// @param records Destination record array with at least count entries.
    /// @param methodOut Transport method in low bits, TELEM_SOURCE_ESTIMATED in bit 7.
    /// @param timeoutMs Bounded timeout for the idle Sync Read; no fallback reads.
    /// @return Number of records filled.
    uint8_t getFastTelemetryRecords(
      const uint8_t* ids,
      uint8_t count,
      FastTelemetryRecord* records,
      uint8_t& methodOut,
      uint32_t timeoutMs = 10
    );

    /// @brief Configure explicit IDs for read-only shadow instrumentation.
    bool configureShadowTelemetry(
      const uint8_t* ids, uint8_t count, unsigned long intervalMs
    );
    /// @brief Start shadow sampling. Requires VELOCITY mode and a configuration.
    bool startShadowTelemetry();
    /// @brief Stop sampling without changing any motor state.
    void stopShadowTelemetry();
    bool isShadowTelemetryEnabled() const;
    uint8_t getShadowTelemetryCount() const;
    unsigned long getShadowTelemetryIntervalMs() const;
    uint32_t getShadowTelemetrySequence() const;
    uint32_t getShadowTelemetryReadErrors() const;
    uint8_t copyShadowTelemetryRecords(
      ShadowTelemetryRecord* records, uint8_t capacity
    ) const;

    /// @brief Reset zero offsets for all motors using their current positions.
    void resetAllZeros();

    /// @brief Get a string summarizing the device information.
    // @return Information string.
    String getDeviceInfo(bool includeLiveTelemetry = false);

    /// @brief Get the hand side this board is compiled for ("right" or "left").
    /// @return HAND_SIDE constant as a C-string.
    const char* getSide() const;

    /// @brief Get the number of motors
    /// @return integer, number of motors.
    int getMotorCount();

    bool isMotorFlipped(uint8_t id);

    // -----------------------------------------------------------
    // Calibration functions
    // -----------------------------------------------------------

    /// @brief Start the calibration process for the exoskeleton.
    /// @param enableTimedCalibration If true, enables timed calibration mode.
    /// @param duration Duration in seconds for timed calibration.
    void beginCalibration(bool enableTimedCalibration, int duration);

    /// @brief Update the calibration state, checking if the calibration is complete.
    void updateCalibration();

    /// @brief Check if the exoskeleton is currently calibrating.
    /// @return True if in calibration mode, false otherwise.
    bool isExoCalibrating();

    // -----------------------------------------------------------
    // Impedance range-of-motion (ROM) calibration
    // -----------------------------------------------------------
    //
    // Drives ONE joint in ONE direction with bounded current pulses, zero-current
    // rests, and measured feedback (per ROM_CAL_* in config.h). Repeated powered
    // pulses without further progress after prior travel identify the endstop.
    // Valid responses refine the directional pulse model and telemetry gain.
    // Automatic return is disabled by default. This runs as a non-blocking state
    // machine serviced from update(): begin*() only arms it and returns.
    //
    // The host triggers this per joint/direction (calibrate_rom:<id>:<dir>) and
    // orchestrates a full six-joint, two-direction sweep itself. The firmware
    // reports the discovered endstop but does NOT overwrite jointLimits_; the
    // host decides whether to apply it via set_motor_limits.

    /// @brief Arm an impedance ROM sweep for one joint and direction.
    ///
    /// Requires the global motor control mode to already be CURRENT and the ID
    /// to be reachable; fails (returns false, arms nothing) otherwise. On
    /// success it enables torque on the target only and records the present
    /// angle as the return-home reference.
    /// @param id Target Dynamixel ID.
    /// @param direction FLEX or EXTEND on the gesture axis.
    /// @return True if the sweep was armed.
    bool beginRomCalibration(uint8_t id, RomCalDirection direction);

    /// @brief Arm a sequence of ROM sweeps over several joints, one at a time.
    ///
    /// Used by the gesture-level calibration (calibrate_rom_gesture): a gesture
    /// like "thumb" or "wrist" drives several motors, and this sweeps each of
    /// them in the requested direction back to back. Each motor still emits its
    /// own ROM_CAL_RESULT line as it concludes; the state machine advances to
    /// the next queued motor automatically. The first motor is armed exactly as
    /// beginRomCalibration() would, so the same CURRENT-mode / reachability
    /// preconditions apply to it.
    /// @param ids Motor IDs to sweep, in order.
    /// @param count Number of IDs (bounded by N_MOTORS).
    /// @param direction FLEX or EXTEND, applied to every motor in the batch.
    /// @return True if at least the first reachable motor was armed.
    bool beginRomCalibrationBatch(const uint8_t* ids, uint8_t count,
                                  RomCalDirection direction);

    /// @brief Service the ROM calibration state machine. Called from update().
    /// Non-blocking: does at most a bounded amount of Dynamixel I/O per call.
    void serviceRomCalibration();

    /// @brief Whether an impedance ROM sweep is currently running.
    bool isRomCalibrating() const;

    /// @brief Side-effect-free precheck for whether a ROM sweep may start now
    /// (global mode is CURRENT and no sweep is already running). Lets a caller
    /// emit its acknowledgement before any ROM_CAL_RESULT line is produced.
    bool canStartRomCalibration() const;

    /// @brief Abort any running sweep immediately: zero current on the target,
    /// return it home, and report status=aborted. Safe to call when idle.
    void cancelRomCalibration();

    // -----------------------------------------------------------
    // Mode functions
    // -----------------------------------------------------------

    /// @brief Assign pin for mode switch interrupt.
    /// @param pin Interrupt pin.
    void setModeSwitchButton(int pin);

    /// @brief Define the operating mode for exo.
    /// @param name Operating mode ("GESTURE_FIXED", "GESTURE_CONTINUOUS", "FREE")
    void setExoOperatingMode(const String& name);

    /// @brief Get the current operating mode for exo.
    /// @return The current operating mode as a string.
    String getExoOperatingMode();

    /// @brief Get the current operating mode as an enum.
    /// @return The current operating mode as an ExoOperatingMode enum.
    ExoOperatingMode getExoOperatingModeEnum();

    /// @brief Check if the mode switch button was pressed.
    bool checkModeSwitchButtonPressed();

    /// @brief Update the exo state, including checking for button pressed, mode switching, and internal routines
    void update();

    /// @brief Cycle through the exo operating modes.
    void cycleExoOperatingMode();


    // -----------------------------------------------------------
    // Position commands
    // -----------------------------------------------------------

    /// @brief Get the relative angle (degrees) of a motor.
    /// @param id Motor ID.
    /// @return Relative angle in degrees.
    float getRelativeAngle(uint8_t id);

    /// @brief Command the motor to a relative angle.
    /// @param id Motor ID.
    /// @param angleDeg Relative angle in degrees.
    void setRelativeAngle(uint8_t id, float angleDeg);

    /// @brief Get the absolute angle (degrees) of a motor.
    /// @param id Motor ID.
    /// @return Absolute angle in degrees.
    float getAbsoluteAngle(uint8_t id);

    /// @brief Command the motor to an absolute angle.
    /// @param id Motor ID.
    /// @param angleDeg Absolute angle in degrees.
    void setAbsoluteAngle(uint8_t id, float angleDeg);

    /// @brief Atomically command several absolute angles with one DXL packet.
    ///
    /// Every target is validated, clamped, and shortest-path-resolved before
    /// anything is transmitted. Duplicate/unknown IDs or an unreadable initial
    /// position is omitted from the packet so one offline ID cannot block the
    /// rest of a dual-hand frame.
    /// @param targets Array of motor IDs and absolute angles.
    /// @param count Number of targets in the array.
    /// @param writtenOut Optional out: number of targets packed and written.
    /// @param skippedOut Optional out: number omitted because they are offline.
    /// @param libErrorOut Optional out: Dynamixel library error on failure.
    /// @return True when at least one target was transmitted successfully.
    bool setAbsoluteAnglesSync(const MotorAngleTarget* targets, uint8_t count,
                               uint8_t* writtenOut = nullptr,
                               uint8_t* skippedOut = nullptr,
                               int16_t* libErrorOut = nullptr);

    /// @brief Get the stored zero angle of a motor.
    /// @param id Motor ID.
    /// @return Zero angle in degrees.
    float getZeroAngle(uint8_t id);

    /// @brief Home the motor to its stored zero position.
    /// @param id Motor ID.
    void setHome(uint8_t id);

    /// @brief Home all motors to their stored zero positions.
    void homeAllMotors();

    /// @brief Command the motor to an angle by ID.
    /// @param id Motor ID.
    /// @param angleDeg Angle in degrees.
    void setAngleById(uint8_t id, float angleDeg);

    /// @brief Command a motor using an alias (name) and angle.
    /// @param alias Motor name (e.g. "WRIST").
    /// @param angleDeg Angle in degrees.
    void setAngleByAlias(const String& alias, float angleDeg);

    /// @brief Set the lower joint limit (in degrees) for a motor.
    /// @param id Motor ID.
    /// @param lowerBound New lower bound in degrees.
    void setMotorLowerBound(uint8_t id, float lowerBound);

    /// @brief Set the upper joint limit (in degrees) for a motor.
    /// @param id Motor ID.
    /// @param upperBound New upper bound in degrees.
    void setMotorUpperBound(uint8_t id, float upperBound);

    /// @brief Get the joint angle limits for a motor.
    /// @param id Motor ID.
    /// @return A string in the format "[min, max]" or error message.
    String getMotorLimits(uint8_t id);

    /// @brief Get the lower joint limit for a motor.
    /// @param id Motor ID.
    /// @return Lower limit in degrees, or -1 if invalid ID.
    float getMotorLimitMin(uint8_t id);

    /// @brief Get the upper joint limit for a motor.
    /// @param id Motor ID.
    /// @return Upper limit in degrees, or -1 if invalid ID.
    float getMotorLimitMax(uint8_t id);

    // -----------------------------------------------------------
    // Gesture fractional axis
    // -----------------------------------------------------------
    //
    // Every gesture percentage -- the EXTEND_*/REST_*/FLEX_* constants,
    // `set_gesture_angle`, `get_gesture_angle` -- rides on ONE per-motor axis:
    // 0 at home, 1 at the flexion endstop. Defining it in one place is what
    // makes those three agree, and what makes the mapping invertible so a
    // measured angle can be reported back as the percentage that produced it.
    //
    // The old form scaled by the whole window width (limit_max - limit_min) and
    // took its direction solely from the flip flag, which assumed home sat on
    // the extension endstop. Where it did not -- the wrist and wrist2 axes --
    // every state landed past the boundary, setAbsoluteAngle() clamped them all
    // to the same angle, and the joint stopped moving while still acking OK.

    /// @brief The 0% anchor for a motor: home, clamped into its limit window.
    ///
    /// A home outside the window is unreachable (setAbsoluteAngle clamps), so
    /// anchoring on the clamped value keeps set/get exact inverses instead of
    /// reporting a percentage the joint can never occupy. `check_limits` still
    /// flags the underlying HOME_OUTSIDE condition.
    /// @param id Motor ID.
    /// @return Absolute angle of the 0% end, in degrees.
    float getGestureOrigin(uint8_t id);

    /// @brief Signed travel from home to the flexion endstop, in degrees.
    ///
    /// Sign carries direction, so callers never need the flip flag themselves.
    /// Magnitude below GESTURE_MIN_TRAVEL_DEG means the joint cannot be
    /// meaningfully positioned by any gesture.
    /// @param id Motor ID.
    /// @return Signed span in degrees; 0 for an unknown ID.
    float getGestureSpan(uint8_t id);

    /// @brief Absolute angle for a point on this motor's gesture axis.
    /// @param id Motor ID.
    /// @param fraction 0 = home, 1 = flexion endstop.
    /// @return Absolute target in degrees, inside the limit window by
    ///         construction for fraction in [0, 1].
    float gestureFractionToAngle(uint8_t id, float fraction);

    /// @brief Where an absolute angle sits on this motor's gesture axis.
    ///
    /// Exact inverse of gestureFractionToAngle(). Values outside [0, 1] mean
    /// the joint is beyond an end of its calibrated travel, which is a real
    /// state a hand can be in (moved by hand, or a stale multi-turn epoch).
    /// @param id Motor ID.
    /// @param angleDeg Absolute angle in degrees.
    /// @return Fraction, or NAN if the motor has no usable travel.
    float gestureAngleToFraction(uint8_t id, float angleDeg);

    /// @brief Set the joint angle limits for a motor.
    /// @param id Motor ID.
    /// @param lowerLimit New lower limit in degrees.
    /// @param upperLimit New upper limit in degrees.
    void setMotorLimits(uint8_t id, float lowerLimit, float upperLimit);



    // -----------------------------------------------------------
    // Torque commands
    // -----------------------------------------------------------

    /// @brief Check if torque is enabled for a motor.
    /// @param id Motor ID.
    bool getTorqueEnabledStatus(uint8_t id);

    /// @brief Enable or disable torque for a motor.
    /// @param id Motor ID.
    /// @param enable True to enable torque, false to disable.
    void enableTorque(uint8_t id, bool enable);

    /// @brief Get the current draw from a motor.
    /// @param id Motor ID.
    /// @return Raw current value.
    int16_t getCurrent(uint8_t id);

    /// @brief Get the current limit in mA for a motor.
    /// @param id Motor ID.
    /// @return Current limit in milliamps.
    int16_t getCurrentLimit(uint8_t id);

    /// @brief Set the current limit for a motor.
    /// @param id Motor ID.
    /// @param current_mA Current limit in milliamps.
    void setCurrentLimit(uint8_t id, uint16_t current_mA);

    /// @brief Command signed motor current in mA while in CURRENT mode.
    bool setGoalCurrent(uint8_t id, float current_mA);
    /// Validate the whole request, then apply guarded per-motor current writes.
    /// Bus failures may leave an applied prefix; this is not a DXL Sync Write.
    bool setGoalCurrents(const uint8_t* ids, const float* currents, uint8_t count);

    /// @brief Read the currently configured goal current in mA.
    float getGoalCurrent(uint8_t id);

    // -----------------------------------------------------------
    // Combined-motor current budget
    // -----------------------------------------------------------

    /// @brief Set the combined current budget across all motors, in mA.
    /// @param budget_mA New fleet budget; clamped to a sane range.
    void setTotalCurrentBudget(uint16_t budget_mA);

    /// @brief Get the combined current budget across all motors, in mA.
    uint16_t getTotalCurrentBudget() const;

    /// @brief Set the current allowed to a settled/stalled motor, in mA.
    void setHoldCurrent(uint16_t hold_mA);

    /// @brief Get the current allowed to a settled/stalled motor, in mA.
    uint16_t getHoldCurrent() const;

    /// @brief Enable or disable the closed-loop half of the budget enforcement.
    ///
    /// Disabling stops all current sampling and leaves the conservative
    /// feed-forward clamp in charge, which is strictly safer for the supply but
    /// gives up per-motor effort. It does NOT remove the budget.
    void setCurrentGovernorEnabled(bool enabled);

    /// @brief Whether the closed-loop half of the budget enforcement is active.
    bool getCurrentGovernorEnabled() const;

    /// @brief Aggregate current measured on the most recent sample, in mA.
    /// @return Measured total, or -1 if nothing has been sampled yet.
    int32_t getMeasuredTotalCurrent() const;

    /// @brief Fraction of nominal effort currently allowed, in [0, 1].
    float getCurrentBudgetScale() const;

    /// @brief Human-readable budget state for the `current_status` command.
    String getCurrentBudgetStatus();

    /// @brief Set the zero offset for a motor to an arbitrary value.
    /// @param id Motor ID.
    /// @param offset_deg Zero offset in degrees.
    void setZeroOffsetValue(uint8_t id, float offset_deg);

    /// @brief Set the flip direction flag for a motor.
    /// @param id Motor ID.
    /// @param flip True to invert the motor direction, false for normal.
    void setFlipMotor(uint8_t id, bool flip);

    /// @brief Get the flip direction flag for a motor.
    /// @param id Motor ID.
    /// @return True if the motor direction is inverted.
    bool getFlipMotor(uint8_t id);

    /// @brief Set the calculated torque in N·m for a motor.
    /// @param id Motor ID.
    /// @return Torque in Newton-meters.
    void setTorque(uint8_t id, float torque_Nm);

    /// @brief Get the calculated torque in N·m for a motor.
    /// @param id Motor ID.
    /// @return Torque in Newton-meters.
    float getTorque(uint8_t id);

    // -----------------------------------------------------------
    // Velocity commands
    // -----------------------------------------------------------

    /// @brief Set the velocity limit of a motor.
    /// @param id Motor ID.
    /// @param vel Velocity limit.
    void setVelocityLimit(uint8_t id, uint32_t vel);

    /// @brief Get the velocity limit of a motor.
    /// @param id Motor ID.
    /// @return Velocity limit.
    uint32_t getVelocityLimit(uint8_t id);

    /// @brief Command signed motor velocity in rpm while in VELOCITY mode.
    bool setGoalVelocity(uint8_t id, float velocity_rpm);

    /// @brief Read present motor velocity in rpm, using calibrated direction.
    float getPresentVelocity(uint8_t id);

    /// @brief Immediately zero direct velocity/current goals for one motor.
    void stopDirectControl(uint8_t id);

    /// @brief Immediately zero direct goals for every firmware-managed motor.
    void stopAllDirectControl();

    /// @brief Configure the direct-command watchdog timeout.
    void setDirectCommandTimeout(unsigned long timeout_ms);

    /// @brief Return the direct-command watchdog timeout.
    unsigned long getDirectCommandTimeout() const;

    /// @brief Hold one explicit motor at a relative angle while the remaining
    /// motors stay in the global velocity/current mode.
    bool holdRelativePosition(
      uint8_t id, float relativeAngle, uint16_t requestedCurrentMa = 0);

    /// @brief Return the current applied to an active auxiliary hold, or zero.
    uint16_t getPositionHoldCurrent(uint8_t id) const;

    /// @brief Disable a held motor and restore its operating mode to the
    /// current global motor-control mode. Torque remains off.
    bool releasePositionHold(uint8_t id);

    /// @brief Return whether a motor is currently in auxiliary position hold.
    bool isPositionHoldActive(uint8_t id) const;

    // -----------------------------------------------------------
    // Acceleration commands
    // -----------------------------------------------------------

    /// @brief Set the acceleration limit of a motor.
    /// @param id Motor ID.
    /// @param acc Acceleration limit.
    void setAccelerationLimit(uint8_t id, uint32_t acc);

    /// @brief Get the acceleration limit of a motor.
    /// @param id Motor ID.
    /// @return Acceleration limit.
    uint32_t getAccelerationLimit(uint8_t id);

    // -----------------------------------------------------------
    // Motor-specific commands
    // -----------------------------------------------------------

    /// @brief Reboot a motor.
    /// @param id Motor ID.
    void rebootMotor(uint8_t id);

    /// @brief Ping a motor to verify communication.
    /// @param id Motor ID.
    void getMotorInfo(uint8_t id);

    /// @brief Set the baud rate of a motor.
    /// @param id Motor ID.
    /// @param baudrate New baud rate.
    bool setBaudRate(uint8_t id, uint32_t baudrate);

    /// @brief Get the baud rate of a motor.
    /// @param id Motor ID.
    /// @return Baud rate.
    uint32_t getBaudRate(uint8_t id);

    /// @brief Set the LED state of a motor.
    /// @param id Motor ID.
    /// @param state True for on, false for off.
    void setMotorLED(uint8_t id, bool state);

    /// @brief Set the LED state of all motors.
    /// @param state True for on, false for off.
    void setAllMotorLED(bool state);

    /// @brief Set the current control mode of a motor.
    /// @param id Motor ID.
    /// @param mode Control mode as a string (POSITION, CURRENT_POSITION, VELOCITY, CURRENT).
    void setMotorControlMode(uint8_t id, const String& mode);

    /// @brief Get the current control mode of a motor.
    /// @param id Motor ID.
    /// @return Control mode as a string.
    String getMotorControlMode(uint8_t id);

    /// @brief Set the control mode for all motors.
    /// @param mode Control mode as a string (e.g. "POSITION", "CURRENT_POSITION", "VELOCITY").
    void setMotorMode(const String& mode);

    /// @brief Get the current control mode of all motors.
    /// @return Control mode as a string.
    String getMotorMode();

    /// @brief Whether absolute-position telemetry is a controlled/meaningful
    /// quantity in the active control mode.
    ///
    /// PRESENT_POSITION is always readable on the XC330 regardless of operating
    /// mode, but it is only a commanded/held quantity when the motor is in a
    /// position-controlling mode (POSITION or CURRENT_POSITION). In VELOCITY or
    /// pure CURRENT the shaft position is free, so the reported angle is a
    /// bystander reading, not a controlled measurement. The Axon telemetry marks
    /// the angle field unavailable when this is false so a consumer never treats
    /// an uncontrolled position as commanded state. Keyed off the firmware's
    /// single tracked control mode (motorControlMode_), which every motor shares.
    bool modeControlsPosition() const;

    /// @brief Whether present-current (and derived torque) telemetry is a
    /// controlled/meaningful quantity in the active control mode.
    ///
    /// PRESENT_CURRENT is always readable, but only reflects a commanded effort
    /// when the motor is in a current-controlling mode (CURRENT or
    /// CURRENT_POSITION). In POSITION or VELOCITY the current is whatever the
    /// position/velocity loop happens to draw, so it is marked unavailable in
    /// telemetry (and torque, derived from it, follows). Keyed off
    /// motorControlMode_.
    bool modeControlsCurrent() const;

    /// @brief Current software version.
    ///
    /// 0.3.0 -- per-joint gestures gained a third "rest" state, "extend" is now
    /// anchored at home (EXTEND_* = 0.0), a "wrist" gesture was added, and
    /// set_gesture_angle:<gesture>:<0-100> was introduced. Hosts that need to
    /// know whether those exist should gate on >= 0.3.0.
    ///
    /// 0.3.1 -- added the "rad" gesture on the wrist2 motor. Gate on >= 0.3.1.
    ///
    /// 0.4.0 -- combined-motor current budget. GOAL_CURRENT is now owned by the
    /// budget allocator rather than written directly by set_current_lim, which
    /// sets the per-motor NOMINAL effort instead. Adds set_total_current_lim,
    /// get_total_current_lim, set_hold_current, get_hold_current,
    /// set_current_governor and current_status. Gate on >= 0.4.0.
    ///
    /// 0.5.0 -- asynchronous GESTURE_RESULT move verdicts, so a host can tell
    /// "the firmware accepted the goal" from "the joint got there".
    ///
    /// 0.6.0 -- three related changes to gesture positioning:
    ///   * Travel is now measured on a home -> flexion-endstop axis resolved
    ///     per motor (getGestureSpan), fixing joints whose home sits mid-window
    ///     -- the wrist axes -- where every state used to clamp onto the same
    ///     boundary and the joint never moved while still acking OK.
    ///   * set_gesture_angle:<g>:<0-100> now interpolates the gesture's own
    ///     extend -> flex postures, so 0 and 100 ARE those states. It used to
    ///     drive every named motor to the same fraction of its own travel,
    ///     which did not reproduce either state on a multi-motor gesture.
    ///   * The `rad` gesture is GONE and `wrist` drives both dorsal motors
    ///     together: wrist and wrist2 act on the same structure, so commanding
    ///     one alone left the other holding position against it.
    /// Adds get_gesture_angle:<gesture|all>. Gate on >= 0.6.0.
    ///
    /// 0.6.1 -- adds rest-zeroed, OpenSim-style signed gesture angles. The
    /// first motor named by a gesture supplies its calibrated degree scale;
    /// motion from rest toward flex is positive and toward extend is negative.
    /// Adds get_gesture_sang:<gesture|all> (signed degrees only) and
    /// get_gesture_angles:<gesture|all> (percentage code plus signed degrees).
    /// Gate on >= 0.6.1.
    ///
    /// 0.6.2 -- adds per-ID auxiliary position hold during global direct
    /// velocity/current control: hold_position:<id>:<relative-angle> and
    /// release_hold:<id>. Gate on >= 0.6.2.
    /// Development extension -- hold_position accepts optional per-hold
    /// current in mA and reports the applied, safety-clamped value.
    ///
    /// 0.6.3 -- adds set_finger_angles:<thumb>:<index>:<middle>:<ring>:<pinky>
    /// [:<wrist>], a positional batch form of set_gesture_angle that positions
    /// every named joint from ONE command instead of one per joint. An EMPTY
    /// field holds that joint unchanged, so a host that only drives some
    /// fingers leaves the rest where they are with a single write. Cuts the
    /// per-frame command and reply count on the continuous UDP path from
    /// one-per-joint to one. Gate on >= 0.6.3.
    ///
    /// 0.6.4 -- set_finger_angles fields are now SIGNED INTEGERS in [-100, 100]
    /// anchored at each gesture's calibrated REST posture (-100 extend, 0 rest,
    /// +100 flex), replacing the 0.6.3 unsigned 0..100 extend->flex percentage.
    /// This matches the continuous decoder convention (positive flex, negative
    /// extend, zero rest) so the host sends round(value*100) directly, and it is
    /// rest-anchored per motor so signed 0 lands on rest even on multi-motor
    /// gestures. Adds GestureController::setGestureSignedAngle. The 0.6.3 wire
    /// form never shipped, so this replaces rather than extends it. Gate on
    /// >= 0.6.4.
    ///
    /// 0.7.0 -- set_finger_angles now resolves and validates the complete pose
    /// before moving, then sends every motor goal in one Protocol 2.0 Sync
    /// Write. Steady-state shortest-path resolution uses the last accepted
    /// goal, eliminating one present-position round trip per motor per frame.
    /// Also fixes set_baud to use Dynamixel2Arduino's baud-value mapping.
    /// Gate on >= 0.7.0 for atomic single-packet finger frames.
    ///
    /// 0.7.1 -- dual firmware omits IDs that were unreachable during startup
    /// from set_finger_angles Sync Write packets. A single attached nine-motor
    /// hand therefore moves normally while the other nine IDs are reported as
    /// skipped_offline instead of rejecting the whole frame.
    ///
    /// 0.8.0 -- adds impedance-based range-of-motion calibration:
    /// calibrate_rom:<id>:<flex|extend> drives one joint in one direction under
    /// current-mode control, ramping current slowly until the joint just starts
    /// to move and then resists a small extra nudge, marks that angle as the
    /// endstop, and returns the joint to where it started. It runs as a
    /// non-blocking on-device state machine (serviced from update()) with a hard
    /// current ceiling well below MOTOR_CURRENT_LIMIT and an overall watchdog;
    /// all sweep tuning lives in the ROM_CAL_* block of config.h. The endstop is
    /// REPORTED (async ROM_CAL_RESULT line), not auto-applied -- the host writes
    /// it back with set_motor_limits if it wants to. Requires the global mode to
    /// be CURRENT first. Adds calibrate_rom and cancel_rom. Gate on >= 0.8.0.
    ///
    /// axon-0.3.0 -- the read-only Axon telemetry block grows from 4 to 8
    /// channels per motor, adding current_mA (+status) and derived torque (N*m)
    /// alongside the existing angle/age/status. The reply schema id becomes
    /// 0xF212 so a driver expecting the 4-field 0xF211 layout rejects rather
    /// than misparses. No motion or actuation is added; readback only.
#if EXO_AXON_USB
    static constexpr const char* VERSION = "0.9.1-axon-0.3.0";
#else
    static constexpr const char* VERSION = "0.9.1";
#endif

  private:
    ExoUtcClock utcClock_;
    JointStateModel jointModels_[N_MOTORS];
    JointTelemetryGate telemetryGate_;
    bool torqueEnabled_[N_MOTORS] = {};
    uint32_t lastPositionMonitorMs_ = 0;
    bool motionActive() const;
    void servicePositionMonitor();
    void fillModelRecord(int index, FastTelemetryRecord& record);
    /// @brief Dynamixel2Arduino object for motor communication.
    Dynamixel2Arduino dxl_;              // Handle to Dynamixel object

    /// @brief Pointer to array of motor IDs.
    const uint8_t* motorIds_;                 // List of motor IDs passed by the user

    /// @brief Pointer to array of motor names.
    const char* const* motorNames_;

    /// @brief Number of motors configured.
    uint8_t numMotors_ = 0;                  // Number of motors being used, should be detected by the length of motor ids passed

    /// @brief Pointer to 2D array of joint limits [min, max] for each motor.
    float (*jointLimits_)[2];      // Pointer to 2D array of joint limits

    /// @brief Array of zero offsets for each motor.
    float* zeroOffsets_;                 // Track absolute zero positions

    /// @brief Array of current limits for each motor
    uint16_t* currentLimits_;

    /// @brief Mode switch pin
    int modeSwitchPin = -1;

    /// @brief Mode switch flag for interrupt callback
    static volatile bool modeSwitchFlag;

    /// @brief Last interrupt time for mode switch button
    bool lastButtonState = false; // Last state of the mode switch button

    /// @brief Current state of the mode switch button
    int buttonState = HIGH;

    /// @brief Last debounce time for mode switch button
    unsigned long lastDebounceTime = 0;

    /// @brief Debounce delay for mode switch button
    const unsigned long debounceDelay = 50;  // 50 ms debounce

    /// @brief Operating mode of motors.
    String motorControlMode_;            

    /// @brief Per-motor direct-command watchdog state.
    unsigned long* lastDirectCommandMs_;
    bool* directCommandActive_;
    float* directCommandDirection_;
    int8_t* directVelocityLimitBlock_;
    bool* directVelocityLimitVerified_;
    bool* positionHoldActive_;
    unsigned long directCommandTimeoutMs_ = DIRECT_COMMAND_TIMEOUT_MS;

    /// @brief Taper an outward velocity near a limit and latch at the hard margin.
    float limitDirectVelocity(int index, float velocity_rpm, float position);

    /// @brief Ensure the motor's EEPROM VELOCITY_LIMIT register matches the
    /// firmware direct-command ceiling. Torque must already be off.
    bool ensureDirectVelocityLimit(uint8_t id);

    /// @brief Enforce direct-control watchdogs and calibrated position limits.
    void serviceDirectControlSafety();

    /// @brief Snap a goal to within +/-180 deg of present, without leaving the
    /// calibrated window.
    ///
    /// In extended/current-based position mode the motor travels the linear
    /// tick line from present to goal, so a goal 360 deg away is the same
    /// physical angle but costs a full revolution. Subtracting the turn avoids
    /// that -- but on a joint whose window is WIDER than 180 deg (the wrist is
    /// deliberately multi-turn) the correction can push a legitimately distant
    /// goal straight outside the limits it was just clamped to. Applying it
    /// only when the result stays in the window keeps the short-route guard for
    /// the wrap case it exists for and drops it for genuine long travel.
    /// @param index Motor index.
    /// @param goal Absolute goal in degrees, already clamped to the limits.
    /// @return Goal to actually command.
    float applyShortestPath(int index, float goal);

    /// @brief Pure shortest-path calculation using a supplied reference.
    float applyShortestPathFromReference(int index, float goal, float reference) const;

    // -- Combined-motor current budget state --------------------------
    // currentLimits_[i] above is the NOMINAL per-motor effort (what
    // set_current_lim asked for). appliedCurrents_[i] is what is actually in
    // the motor's GOAL_CURRENT register after the budget has had its say, so
    // the two differ whenever the fleet is being clamped.

    /// @brief GOAL_CURRENT actually written per motor, in mA.
    uint16_t* appliedCurrents_;

    /// @brief True while a motor has an outstanding goal it has not concluded.
    bool* motorMoving_;

    /// @brief True while a motor is FUNDED to move, not merely wanting to.
    ///
    /// The budget admits movers in waves: a motor can want to move (moving_)
    /// yet be unfunded (not admitted_) because giving it a share now would drop
    /// every mover below the current it takes to actually travel.
    bool* motorAdmitted_;

    /// @brief millis() when each motor was admitted, for the move timeout.
    unsigned long* admissionMs_;

    /// @brief Absolute goal each motor was last sent, for judging arrival.
    float* goalAngle_;

    /// @brief Whether goalAngle_ is safe to use as a shortest-path reference.
    bool* goalAngleValid_;

    /// @brief Whether startup obtained a trustworthy position from this ID.
    bool* motorReachable_;

    /// @brief True between a goal being issued and its verdict being recorded.
    bool* verdictPending_;

    /// @brief Per-motor MoveVerdict for the batch being summarised.
    uint8_t* lastVerdict_;

    uint8_t verdictsCollected_ = 0;
    uint8_t verdictFailures_ = 0;

    /// @brief millis() when each motor's current goal was issued.
    unsigned long* goalIssuedMs_;

    /// @brief millis() when each motor first drew stall-level current, or 0.
    unsigned long* stallSinceMs_;

    /// @brief Combined budget across all motors, in mA.
    uint16_t totalCurrentBudgetMa_ = TOTAL_CURRENT_BUDGET_MA;

    /// @brief Current allowed to a settled or stalled motor, in mA.
    uint16_t holdCurrentMa_ = HOLD_CURRENT_MA;

    /// @brief Fraction of nominal effort the budget currently allows, in [0, 1].
    float currentBudgetScale_ = 1.0f;

    /// @brief Closed-loop half of the enforcement; feed-forward runs regardless.
    bool currentGovernorEnabled_ = true;

    /// @brief Aggregate measured on the last sample, or -1 if never sampled.
    int32_t measuredTotalMa_ = -1;

    /// @brief Scale the motors' GOAL_CURRENT registers actually reflect.
    ///
    /// Distinct from currentBudgetScale_, which is what the control law has
    /// decided. The gap between them is the write deadband: sub-threshold
    /// corrections accumulate here rather than being discarded one at a time.
    float appliedScale_ = 1.0f;

    /// @brief Set when the allocation needs pushing to the motors.
    ///
    /// Commands that touch every motor -- a whole-hand gesture, or
    /// `set_current_lim:all` -- would otherwise rewrite the whole fleet once per
    /// motor. The flag collapses that into one pass, flushed from update().
    bool allocationDirty_ = false;

    /// @brief Which motor the incremental current sweep reads next.
    int governorCursor_ = 0;

    /// @brief Aggregate accumulated so far by the in-progress sweep, in mA.
    uint32_t sweepAccumMa_ = 0;

    unsigned long lastGovernorSampleMs_ = 0;
    unsigned long lastGoalIssuedMs_ = 0;
    uint8_t governorReadFails_ = 0;
    bool governorMeasurementTrusted_ = true;

    /// @brief Record that a goal was issued for one motor, and pre-clamp the
    /// fleet to the static worst case so the first transient is bounded.
    /// @param goalAngle Absolute target, kept so arrival can be judged later.
    void noteGoalCommanded(int index, float goalAngle);

    /// @brief How many motors may travel at once inside the budget.
    uint8_t maxConcurrentMovers() const;

    /// @brief End a move, record why it ended, and free its admission slot.
    void concludeMove(int index, uint8_t verdict);

    /// @brief Emit one GESTURE_RESULT summary once a whole batch has concluded.
    void reportMoveVerdicts();

    /// @brief Worst-case scale that fits every moving motor inside the budget.
    float staticBudgetScale() const;

    /// @brief Push the budget-derived GOAL_CURRENT to every motor that needs it.
    void refreshCurrentAllocation();

    /// @brief Write one motor's GOAL_CURRENT, skipping no-op writes.
    void applyGoalCurrent(int index, uint16_t current_mA);

    /// @brief Read PRESENT_CURRENT defensively; clears ok on a bus error.
    int16_t readPresentCurrentMa(uint8_t id, bool& ok);

    /// @brief Sample the aggregate draw and adjust the clamp. Called from update().
    void serviceCurrentGovernor();

    /// @brief Operating mode of exo
    ExoOperatingMode exoMode_ = FREE; // Default mode is free

    /// brief Flag to indicate if the exoskeleton is currently calibrating.
    bool isCalibrating = false;

    // @brief Flag for calibration timed mode
    bool calibrationTimedMode = false;

    /// @brief Start time for calibration.
    unsigned long calibrationStartTime;

    /// @brief Duration for calibration in milliseconds.
    unsigned long calibrationDuration;

    // -- Impedance ROM calibration state -------------------------------
    // Single-joint, single-direction at a time, so scalar state is enough.
    // The state machine reads only the ROM_CAL_* constants from config.h for
    // its tuning; see serviceRomCalibration() in nml_hand_exo.cpp.
    RomCalPhase romCalPhase_ = ROM_CAL_IDLE;
    RomCalDirection romCalDir_ = ROM_CAL_DIR_FLEX;
    RomCalStatus romCalStatus_ = ROM_CAL_STATUS_NONE;
    uint8_t romCalId_ = 0;               ///< Target Dynamixel ID for the sweep.
    int romCalIndex_ = -1;               ///< Cached motor index of romCalId_.
    float romCalSign_ = 0.0f;            ///< Commanded-current sign (+/-1), locked at onset.
    float romCalCurrentMa_ = 0.0f;       ///< Current magnitude the ramp has reached, mA.
    bool romCalDirLocked_ = false;       ///< Whether romCalSign_ has been resolved.
    float romCalHomeAngle_ = 0.0f;       ///< Absolute angle to return to, deg.
    float romCalLastAngle_ = 0.0f;       ///< Absolute angle at the previous step, deg.
    float romCalEndstopAngle_ = 0.0f;    ///< Furthest advanced position, i.e. the endstop, deg.
    unsigned long romCalStartMs_ = 0;
    unsigned long romCalLastStepMs_ = 0;
    unsigned long romCalPulseStartMs_ = 0;
    unsigned long romCalRestStartMs_ = 0;
    unsigned long romCalPulseOnMs_ = 0;
    unsigned long romCalReturnStartMs_ = 0;
    float romCalPulseStartAngle_ = 0;
    float romCalPulseStartVelocity_ = NAN;
    float romCalPulsePeakAngle_ = 0;
    uint16_t romCalPulseCount_ = 0;
    uint16_t romCalFitSamples_ = 0;
    uint8_t romCalStallPulses_ = 0;
    bool romCalSignReversed_ = false;
    bool romCalStopRequested_ = false;
    const char* romCalReason_ = "none";
    unsigned long romCalMaxPulseOnMs_ = 0;
    float romCalFeedbackAngle_ = NAN;
    uint16_t romCalRecoveries_ = 0;
    uint8_t romCalFeedbackFailures_ = 0;
    bool romCalPulseValid_ = true;
    bool romCalPulseLimited_ = false;
    float romCalMaxExcursionDeg_ = 0;
    const char* romCalPulseStop_ = "none";
    const char* romCalFitReason_ = "no_completed_pulse";
    float romCalPredictedDeg_ = NAN;
    float romCalObservedDeg_ = NAN;
    const char* romCalResponseReason_ = "no_completed_pulse";
    void romCalReportPulse();
    bool romCalCheckTravel(float angle);
    void romCalEndPulse(const char* reason);
    bool romCalReadPosition(float& angle);
    void romCalRecoverFeedback();
    bool romCalReadFeedback(float& angle, float& velocity);
    bool romCalStartPulse(float angle, float velocity);
    void romCalCompletePulse(float angle, float velocity);

    // Batch/gesture sweeps: a queue of motor IDs swept one at a time. Empty for
    // a single-joint calibrate_rom; populated by beginRomCalibrationBatch so a
    // multi-motor gesture (thumb, wrist) sweeps each of its joints in turn.
    uint8_t romCalQueue_[N_MOTORS];
    uint8_t romCalQueueCount_ = 0;   ///< IDs remaining in the queue.
    uint8_t romCalQueueCursor_ = 0;  ///< Next queue slot to arm.
    RomCalDirection romCalQueueDir_ = ROM_CAL_DIR_FLEX;

    /// @brief Arm one motor's sweep. The shared core of beginRomCalibration and
    /// each step of a batch; assumes the queue has already been set as intended.
    bool romCalArmMotor(uint8_t id, RomCalDirection direction);

    /// @brief Apply the ROM-specific current cap. Pulse service enforces the
    /// stored joint window with measured position and aborts on failed feedback.
    bool writeRomSweepCurrent(float current_mA);

    /// @brief Arm the next queued motor, or clear the queue when exhausted.
    /// Called after each sweep concludes. No-op if the queue is empty.
    void romCalAdvanceQueue();

    /// @brief Finish without motion by default; optionally return after success.
    /// Faults and cancellation never initiate a return move.
    void romCalBeginReturnHome();

    /// @brief Release the current target's hold, report it, and advance the
    /// queue (arm the next motor, or idle when the campaign is done).
    void romCalFinish();

    /// @brief Emit one ROM_CAL_RESULT line for the current target. Pure
    /// reporting -- no motor I/O, no queue advance -- so romCalFinish() and the
    /// skip path in romCalAdvanceQueue() can both use it without recursing.
    void romCalReport();


    bool* flipMotor_;

    // Phase-1 shadow telemetry. Fixed storage keeps RAM use deterministic and
    // makes it impossible for instrumentation to fragment the MCU heap.
    ShadowTelemetryRecord shadowRecords_[SHADOW_TELEMETRY_MAX_MOTORS];
    uint8_t shadowTelemetryCount_ = 0;
    uint8_t shadowTelemetryCursor_ = 0;
    bool shadowTelemetryReadCurrent_ = true;
    bool shadowTelemetryEnabled_ = false;
    unsigned long shadowTelemetryIntervalMs_ =
      SHADOW_TELEMETRY_DEFAULT_INTERVAL_MS;
    unsigned long shadowTelemetryLastReadMs_ = 0;
    uint32_t shadowTelemetrySequence_ = 0;
    uint32_t shadowTelemetryReadErrors_ = 0;

    bool readPresentPositionTicks(uint8_t id, int32_t& ticks);
    void serviceShadowTelemetry();

};

#endif // NML_HAND_EXO_H
