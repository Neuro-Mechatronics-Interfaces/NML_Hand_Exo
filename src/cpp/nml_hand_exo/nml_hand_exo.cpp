/**
 * @file nml_hand_exo.cpp
 * @brief Implementation file for the NML Hand Exoskeleton API.
 *
 * This file contains the implementation of the NMLHandExo class, which handles
 * initialization, motor control, angle management, and device telemetry for the
 * exoskeleton.
 */
#include "config.h"
//#include "utils.h"
#include "nml_hand_exo.h"
#include <Dynamixel2Arduino.h>


/// @brief Verbose output toggle for debugging.
bool VERBOSE = DEFAULT_VERBOSE; // default to true

/// @brief Reply/telemetry routing for dual-CDC. Legacy-safe default (BOTH).
uint8_t gReplyRoute = REPLY_ROUTE_BOTH;

/// @brief Flag for mode switching
static volatile bool modeSwitchFlag = false;

/// @brief Checking last interrupt time to check for unwanted presses, control debounce
static unsigned long lastInterruptTime = 0;

Stream* debugStream = &TELEM_SERIAL;

/// @brief Mode press function
void onModeButtonPress() {
  debugPrint(F("Button pressed"));
  unsigned long now = millis();
  if (now - lastInterruptTime > BUTTON_DEBOUNCE_DURATION) {  // debounce
    digitalWrite(LED_BUILTIN, HIGH);
    modeSwitchFlag = true;
    lastInterruptTime = now;
  }
}

NMLHandExo::NMLHandExo(const uint8_t* ids, uint8_t numMotors, const float jointLimits[][2], const float* homeState)
  : dxl_(DXL_SERIAL, DXL_DIR_PIN), motorIds_(ids), numMotors_(numMotors) //jointLimits_(jointLimits),
{

  // Allocate and copy joint limits
  jointLimits_ = new float[numMotors_][2];
  for (int i = 0; i < numMotors_; ++i) {
    jointLimits_[i][0] = jointLimits[i][0]; // min
    jointLimits_[i][1] = jointLimits[i][1]; // max
  }

  zeroOffsets_ = new float[numMotors_];

  // Create offsets if values passed for homeState
  // NOTE: These will be OVERRIDDEN in initializeMotors() by reading actual motor positions
  // This prevents assumptions about where motors are physically located
  if (homeState != nullptr) {
    for (int i = 0; i < numMotors_; ++i) {
      zeroOffsets_[i] = homeState[i];
    }
  } else {
    // Default all to 0.0
    for (int i = 0; i < numMotors_; ++i) {
      zeroOffsets_[i] = 0.0f;
    }
  }

  // Allocate and initialize current limits
  currentLimits_ = new uint16_t[numMotors_];
  for (int i = 0; i < numMotors_; ++i) {
      currentLimits_[i] = MOTOR_CURRENT_LIMIT;
  }

  // Allocate flip flags and initialize from config defaults (overwritten by calibration)
  flipMotor_ = new bool[numMotors_];
  lastDirectCommandMs_ = new unsigned long[numMotors_];
  directCommandActive_ = new bool[numMotors_];
  directCommandDirection_ = new float[numMotors_];
  directVelocityLimitBlock_ = new int8_t[numMotors_];
  directVelocityLimitVerified_ = new bool[numMotors_];
  positionHoldActive_ = new bool[numMotors_];
  appliedCurrents_ = new uint16_t[numMotors_];
  motorMoving_ = new bool[numMotors_];
  motorAdmitted_ = new bool[numMotors_];
  admissionMs_ = new unsigned long[numMotors_];
  goalAngle_ = new float[numMotors_];
  verdictPending_ = new bool[numMotors_];
  lastVerdict_ = new uint8_t[numMotors_];
  goalIssuedMs_ = new unsigned long[numMotors_];
  stallSinceMs_ = new unsigned long[numMotors_];
  for (int i = 0; i < numMotors_; ++i) {
    flipMotor_[i] = DEFAULT_FLIPS[i];
    lastDirectCommandMs_[i] = 0;
    directCommandActive_[i] = false;
    directCommandDirection_[i] = 0;
    directVelocityLimitBlock_[i] = 0;
    directVelocityLimitVerified_[i] = false;
    positionHoldActive_[i] = false;
    appliedCurrents_[i] = 0;
    motorMoving_[i] = false;
    motorAdmitted_[i] = false;
    admissionMs_[i] = 0;
    goalAngle_[i] = 0.0f;
    verdictPending_[i] = false;
    lastVerdict_[i] = MOVE_VERDICT_NONE;
    goalIssuedMs_[i] = 0;
    stallSinceMs_[i] = 0;
  }
  motorControlMode_ = "CURRENT_POSITION";


  // If jointLimits_, zeroOffsets_, currentLimits_ were dynamically allocated, make sure to add a destructor.
}
// ====================================================================================
// ================================ Utility functions =================================
// ====================================================================================
void NMLHandExo::initializeSerial(int baud) {
  // Match ROBOTIS examples: set protocol before opening the DXL port.
  dxl_.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Initialize serial communication with DYNAMIXEL hardware using the specified baudrate. Has to match hardware
  dxl_.begin(baud);
}
void NMLHandExo::initializeMotors() {
  // Configure motor operating modes and torque, but DON'T move them yet
  for (int i = 0; i < numMotors_; i++) {
    uint8_t id = motorIds_[i];
    dxl_.torqueOff(id);
    dxl_.setOperatingMode(id, OP_CURRENT_BASED_POSITION);
    dxl_.torqueOn(id);
    // Ceiling stays at the part maximum so set_current_lim can raise effort at
    // runtime, but the COMMANDED effort starts low. Writing GOAL_CURRENT at
    // the ceiling makes every motor push at max whenever it is resisted, and
    // several digits held flexed at once will brown out the supply.
    //
    // The register write goes direct rather than through setCurrentLimit(),
    // which now also owns the NOMINAL effort: the two are different numbers
    // here (part ceiling vs. working effort) and must not be conflated.
    dxl_.writeControlTableItem(CURRENT_LIMIT, id, MOTOR_CURRENT_LIMIT);
    currentLimits_[i] = DEFAULT_GOAL_CURRENT_MA;   // nominal, not the ceiling
    motorMoving_[i] = false;                       // boots holding, not moving
  }
  // One allocation pass gives every motor its GOAL_CURRENT through the budget,
  // so appliedCurrents_ starts consistent with what is really in the registers.
  currentBudgetScale_ = staticBudgetScale();
  refreshCurrentAllocation();
  delay(500);

  // --- Multi-turn offset correction ---
  // In OP_CURRENT_BASED_POSITION mode the motor tracks multi-turn position.
  // After power-on, the reported position may be HOME + N*360 for some integer N.
  // If we keep zeroOffsets_ = HOME_STATES (single-turn), the first "home" command
  // targets HOME but the motor is at HOME + N*360, causing N full rotations.
  //
  // Fix: read actual position and snap zeroOffsets_ (and jointLimits_) to the
  // nearest equivalent of HOME_STATES modulo 360 degrees.
  for (int i = 0; i < numMotors_; i++) {
    uint8_t id = motorIds_[i];
    float currentPos = dxl_.getPresentPosition(id, UNIT_DEGREE);

    // Sanity check: if read returns exactly 0.0 but home is far away,
    // assume the read failed and keep the original HOME_STATES offset.
    if (currentPos == 0.0f && zeroOffsets_[i] > 10.0f) {
      debugPrint("[WARN] Motor " + String(id) + " position read returned 0.0, "
                 "keeping HOME_STATES offset " + String(zeroOffsets_[i], 2));
      continue;
    }
    // Reject wildly out-of-range reads (>100 turns)
    if (currentPos < -36000.0f || currentPos > 36000.0f) {
      debugPrint("[WARN] Motor " + String(id) + " position read out of range: "
                 + String(currentPos, 2) + ", keeping HOME_STATES offset");
      continue;
    }

    float home = zeroOffsets_[i];  // original HOME_STATES value
    float diff = currentPos - home;
    float turns = round(diff / 360.0f);
    float offset = turns * 360.0f;

    zeroOffsets_[i] = home + offset;
    jointLimits_[i][0] += offset;
    jointLimits_[i][1] += offset;

    if (turns != 0.0f) {
      debugPrint("[INIT] Motor " + String(id) + " multi-turn correction: "
                 + String(turns, 0) + " turn(s), new offset "
                 + String(zeroOffsets_[i], 2));
    }
  }

  debugPrint("========== MOTOR INITIALIZATION COMPLETE ==========");
  debugPrint("Motors configured. Multi-turn offsets corrected.");
}
int NMLHandExo::getMotorID(const String& token) {
  String target = token;
  target.trim();
  target.toUpperCase();
  int id = target.toInt(); // Try converting to integer

  // Check if it was a valid number (e.g., not "WRIST")
  if (id != 0 || target == "0") {
    return id;
  }
  return getMotorIDByName(target);  // Otherwise, try name lookup
}
uint8_t NMLHandExo::getMotorIDByIndex(const int index) {
  return motorIds_[index];
}
int NMLHandExo::getIndexById(uint8_t id) {
  for (int i = 0; i < numMotors_; i++) {
    if (motorIds_[i] == id) return i;
  }
  return -1;
}
int NMLHandExo::getMotorIDByName(const String& name) {
  String target = name; target.toLowerCase();
  for (int i = 0; i < numMotors_; ++i) {
    if (motorNames_ && target.equalsIgnoreCase(String(motorNames_[i]))) {
      return motorIds_[i];
    }
  }
  return -1;
}
void NMLHandExo::setMotorNames(const char* const* names) {
  motorNames_ = names;
}
String NMLHandExo::getMotorNameByID(uint8_t id) {
  for (int i = 0; i < numMotors_; ++i) {
    if (motorIds_[i] == id) {
      return motorNames_ ? String(motorNames_[i]) : "unnamed";
    }
  }
  return "unknown";
}
int NMLHandExo::angleToTicks(float angle_deg, int index) {
  // Map degrees to ticks: assume full range = 4096 ticks = 360 deg
  float deg_per_tick = 360.0 / PULSE_RESOLUTION;
  int ticks = static_cast<int>(angle_deg / deg_per_tick);
  return ticks;
}
void NMLHandExo::setZeroOffset(uint8_t id) {
  int index = getIndexById(id);
  if (index != -1) {
    float current_angle = dxl_.getPresentPosition(id, UNIT_DEGREE);
    zeroOffsets_[index] = current_angle;
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Calibrated zero for motor %d: %f deg", id, current_angle);
    debugPrint(buffer);
    //debugPrint("Calibrated zero for motor " + String(id) + ": " + String(current_angle, 2) + " deg");
  } else {
    debugPrint("[ERROR] Invalid motor ID for zero calibration: " + String(id));
  }
}
float NMLHandExo::getZeroOffset(uint8_t id) {
  int index = getIndexById(id);
  return (index != -1) ? zeroOffsets_[index] : 0.0f;
}
bool NMLHandExo::getFastTelemetryRecord(uint8_t id, FastTelemetryRecord& record) {
  int index = getIndexById(id);
  record.id = id;
  record.error = 0;
  record.current_mA = 0;
  record.velocity_raw = 0;
  record.position_ticks = 0;
  record.absolute_cdeg = 0;
  record.relative_cdeg = 0;
  if (index == -1) {
    record.error = 1;
    return false;
  }

  int16_t current_mA = (int16_t)dxl_.readControlTableItem(PRESENT_CURRENT, id);
  int32_t velocity_raw = (int32_t)dxl_.readControlTableItem(PRESENT_VELOCITY, id);
  float absolute_deg = dxl_.getPresentPosition(id, UNIT_DEGREE);
  float relative_deg = absolute_deg - zeroOffsets_[index];
  if (flipMotor_[index]) {
    relative_deg *= -1.0f;
  }

  record.current_mA = current_mA;
  record.velocity_raw = velocity_raw;
  record.position_ticks = (int32_t)round(absolute_deg * (float)PULSE_RESOLUTION / 360.0f);
  record.absolute_cdeg = (int32_t)round(absolute_deg * 100.0f);
  record.relative_cdeg = (int32_t)round(relative_deg * 100.0f);
  return true;
}
uint8_t NMLHandExo::getFastTelemetryRecords(
  const uint8_t* ids,
  uint8_t count,
  FastTelemetryRecord* records,
  uint8_t& methodOut,
  uint32_t timeoutMs
) {
  static constexpr uint8_t MAX_FAST_TELEM_IDS = 32;

  if (count > MAX_FAST_TELEM_IDS) {
    count = MAX_FAST_TELEM_IDS;
  }

  auto clearRecords = [&]() {
    for (uint8_t i = 0; i < count; ++i) {
      records[i].id = ids[i];
      records[i].error = 0;
      records[i].current_mA = 0;
      records[i].velocity_raw = 0;
      records[i].position_ticks = 0;
      records[i].absolute_cdeg = 0;
      records[i].relative_cdeg = 0;
    }
  };

  auto clearDxlRx = [&]() {
    while (DXL_SERIAL.available() > 0) {
      DXL_SERIAL.read();
    }
  };

  auto readItem = [&](uint8_t item, uint8_t id, bool& ok) -> int32_t {
    clearDxlRx();
    int32_t value = dxl_.readControlTableItem(item, id, timeoutMs);
    if (dxl_.getLastLibErrCode() != DXL_LIB_OK) {
      ok = false;
    }
    return value;
  };

  clearRecords();
  methodOut = FAST_TELEM_METHOD_FAILED;
  if (count == 0) {
    return 0;
  }

  // Keep this path conservative: diagnostics showed multi-register and repeated
  // per-motor telemetry reads can corrupt or wedge the current bus. For the GUI
  // fast path, read position only and leave current/velocity at zero until the
  // bus is stable enough for those additional registers.
  methodOut = FAST_TELEM_METHOD_FALLBACK_READ;
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t id = ids[i];
    records[i].id = id;
    records[i].error = 0;
    int index = getIndexById(id);
    if (index == -1) {
      records[i].error = 1;
      continue;
    }

    bool ok = true;
    int32_t position_ticks = readItem(PRESENT_POSITION, id, ok);
    if (!ok) {
      records[i].error = 1;
    }
    records[i].current_mA = 0;
    records[i].velocity_raw = 0;
    records[i].position_ticks = position_ticks;
    float absolute_deg = records[i].position_ticks * 360.0f / (float)PULSE_RESOLUTION;
    float relative_deg = absolute_deg - zeroOffsets_[index];
    if (flipMotor_[index]) {
      relative_deg *= -1.0f;
    }
    records[i].absolute_cdeg = (int32_t)round(absolute_deg * 100.0f);
    records[i].relative_cdeg = (int32_t)round(relative_deg * 100.0f);
  }
  return count;
}
void NMLHandExo::resetAllZeros() {
  for (int i = 0; i < numMotors_; ++i) {
    uint8_t id = motorIds_[i];
    float current_angle = dxl_.getPresentPosition(id, UNIT_DEGREE);
    zeroOffsets_[i] = current_angle;
    debugPrint("[DEBUG] Zero offset set for motor " + String(id) + ": " + String(current_angle, 2) + " deg");
  }
}
const char* NMLHandExo::getSide() const {
  return HAND_SIDE;
}

String NMLHandExo::getDeviceInfo(bool includeLiveTelemetry) {

    // Need to return a single string with all the information
    String info = "Name: NMLHandExo\n";
    info += "Version: " + String(VERSION) + "\n";
    info += "Side: " + String(HAND_SIDE) + "\n";
    info += "Number of Motors: " + String(numMotors_) + "\n";
    for (int i = 0; i < numMotors_; ++i) {

      uint8_t id = getMotorIDByIndex(i);
      String name = getMotorNameByID(id);
      float minLimit = jointLimits_[i][0];
      float maxLimit = jointLimits_[i][1];

      info += "Motor " + String(i) + ": {name: " + String(name) +
            ", id: " + String(id) +
            ", limits: [" + String(minLimit, 2) + ", " + String(maxLimit, 2) + "]";
      if (includeLiveTelemetry) {
        float angle = getRelativeAngle(id);
        float abs = getAbsoluteAngle(id);
        float torque = getTorque(id);
        bool isEnabled = getTorqueEnabledStatus(id);
        info += ", angle: " + String(angle, 2) +
              ", absolute_angle: " + String(abs, 2) +
              ", torque: " + String(torque, 2) +
              ", enabled: " + (isEnabled ? "true" : "false");
      }
      info += "}\n";
      }
    info += ";";
    return info;
}
int NMLHandExo::getMotorCount() {
  return numMotors_;
}
bool NMLHandExo::isMotorFlipped(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return false;
  return flipMotor_[index];
}
// ====================================================================================
// ============================ Calibration commands ==================================
// ====================================================================================
void NMLHandExo::beginCalibration(bool enableTimedCalibration=false, int duration=10) {
  isCalibrating = true;
  calibrationTimedMode = enableTimedCalibration;
  calibrationStartTime = millis();
  calibrationDuration = duration * 1000;

  // Initialize calibration state and ask user to move their fingers to extremes. The calibration process will last as long as the calibration duration in seconds
  // This is done only if timedCalibration is true. If off, step through the calibration process starting with asking the user to close their fingers, followed by opening their fingers
  if (calibrationTimedMode) {
      telemetryPrintln("[Gesture] Timed calibration mode. You have " + String(duration) + " seconds to complete.");
  } else {
      telemetryPrintln(F("[Gesture] Step-through calibration mode. Follow prompts to set limits."));
  }
  for (uint8_t i = 0; i < numMotors_; ++i) {
    jointLimits_[i][0] = 1e6; // Initialize to a very large value
    jointLimits_[i][1] = -1e6; // Initialize to a very small value
  }
  debugPrint(F("[Exo Calibration] Started. Move all motors to full range."));
}
void NMLHandExo::updateCalibration() {
  if (!isCalibrating) return;

  unsigned long currentTime = millis();
  unsigned long elapsedMs = (currentTime - calibrationStartTime);

  // Update joint limits
  for (uint8_t i = 0; i < numMotors_; ++i) {
    uint8_t id = motorIds_[i];
    float angle = getAbsoluteAngle(id);
    if (angle < jointLimits_[i][0]) jointLimits_[i][0] = angle;
    if (angle > jointLimits_[i][1]) jointLimits_[i][1] = angle;
  }

  // Stop condition (timed or button)
  if ((calibrationTimedMode && elapsedMs >= calibrationDuration) ||
      (!calibrationTimedMode && this->checkModeSwitchButtonPressed())) {
    isCalibrating = false;
    telemetryPrintln("[Gesture] Calibration complete.");
    for (uint8_t i = 0; i < this->getMotorCount(); ++i) {
      telemetryPrintln("Motor " + String(i) + ": Min = " + String(jointLimits_[i][0]) +
                       ", Max = " + String(jointLimits_[i][1]));
    }
  }
}
bool NMLHandExo::isExoCalibrating() {
  // Check if the exoskeleton is currently in calibration mode
  return isCalibrating;
}

// ====================================================================================
// ================================= Mode commands ====================================
// ====================================================================================
void NMLHandExo::update() {

    serviceDirectControlSafety();

    // Flush any allocation batched up by this pass's commands before the
    // governor gets a say, so the feed-forward clamp reaches the motors within
    // the same loop iteration that issued their goals.
    if (allocationDirty_) refreshCurrentAllocation();
    serviceCurrentGovernor();
    reportMoveVerdicts();

    // First check if we are calibrating
    if (isExoCalibrating()) {
        updateCalibration();
        return; // Skip other updates while calibrating
    }

    // Check for button pushes
    if (checkModeSwitchButtonPressed()) {
        debugPrint(F("Mode switch button pressed"));
        String exo_mode = getExoOperatingMode();
        cycleExoOperatingMode();
        // if (exo_mode == "GESTURE_FIXED" || exo_mode == "GESTURE_CONTINUOUS") {
        //     // === Button was pressed ===
        //     debugPrint(F("[HandExo] Button press detected, cycling exo mode."));
        //     cycleExoOperatingMode();
        // } else {
        //     debugPrint(F("[HandExo] Button press detected, but exo is in FREE mode. No action taken."));
        // }
    }
}
void NMLHandExo::setModeSwitchButton(int pin) {
  modeSwitchPin = pin;
  pinMode(modeSwitchPin, INPUT_PULLUP);

  lastButtonState = HIGH;
  buttonState = HIGH;
  lastDebounceTime = 0;

  char buffer[64];
  debugPrint("Mode switch button set on pin " + String(modeSwitchPin));
}
void NMLHandExo::setExoOperatingMode(const String& modeStr) {
  String m = modeStr;
  m.toUpperCase();

  if (m == "FREE") {
    exoMode_ = FREE;
  } else if (m == "GESTURE_FIXED") {
    exoMode_ = GESTURE_FIXED;
  } else if (m == "GESTURE_CONTINUOUS") {
    exoMode_ = GESTURE_CONTINUOUS;
  } else {
    debugPrint(F("[ERROR] Invalid EXO mode passed"));
  }
  debugPrint("Exo mode set to: " + m);
}
ExoOperatingMode NMLHandExo::getExoOperatingModeEnum() {
  // Return the current operating mode of the exoskeleton as an enum
  return exoMode_;
}
String NMLHandExo::getExoOperatingMode() {
  // Return the current operating mode of the exoskeleton
  ExoOperatingMode mode = getExoOperatingModeEnum();
  switch (mode) {
      case FREE:
      return "FREE";
      case GESTURE_FIXED:
      return "GESTURE_FIXED";
      case GESTURE_CONTINUOUS:
      return "GESTURE_CONTINUOUS";
      default:
      return "UNKNOWN";
  }
}
bool NMLHandExo::checkModeSwitchButtonPressed() {
  if (modeSwitchPin == -1) return false;
  int reading = digitalRead(modeSwitchPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        // === Button was pressed ===
        return true;
      }
    }
  }
  lastButtonState = reading;
  return false;
}
void NMLHandExo::cycleExoOperatingMode() {
  exoMode_ = static_cast<ExoOperatingMode>((exoMode_ + 1) % 3);  // cycles 0–2
  switch (exoMode_) {
    case FREE:
      debugPrint(F("Mode changed to: FREE"));
      break;
    case GESTURE_FIXED:
      debugPrint(F("Mode changed to: GESTURE_FIXED"));
      break;
    case GESTURE_CONTINUOUS:
      debugPrint(F("Mode changed to: GESTURE_CONTINUOUS"));
      break;
  }
}



// ====================================================================================
// ============================== Position commands ====================================
// ====================================================================================
float NMLHandExo::getRelativeAngle(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return -1;

  float abs_angle = dxl_.getPresentPosition(id, UNIT_DEGREE);
  float rel_angle = abs_angle - zeroOffsets_[index];

  // Flip if necessary
  if (flipMotor_[index]) {
    rel_angle *= -1;
  }

  return rel_angle;
}
void NMLHandExo::setRelativeAngle(uint8_t id, float relativeAngle) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID: " + String(id));
    return;
  }

  // Flip the relative angle if necessary
  if (flipMotor_[index]) {
    relativeAngle *= -1;
  }

  // Compute the absolute angle by adding the stored offset
  float abs_goal = zeroOffsets_[index] + relativeAngle;

  // Clamp the absolute goal to the joint limits (if necessary)
  abs_goal = constrain(abs_goal, jointLimits_[index][0], jointLimits_[index][1]);

  // Shortest-path guard: avoid 360° rotations in extended position mode
  abs_goal = applyShortestPath(index, abs_goal);

  // Register the move first: the budget pre-clamps effort before the motor
  // starts drawing, which is the whole point of the feed-forward half.
  noteGoalCommanded(index, abs_goal);
  dxl_.setGoalPosition(id, abs_goal, UNIT_DEGREE);

  char buffer[128];
  snprintf(buffer, sizeof(buffer), "Motor %d set to relative angle %.2f deg (absolute: %.2f deg)", id, relativeAngle, abs_goal);
  debugPrint(buffer);
}
float NMLHandExo::getAbsoluteAngle(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID: " + String(id));
    return -1;
  }
  return dxl_.getPresentPosition(id, UNIT_DEGREE);
}
void NMLHandExo::setAbsoluteAngle(uint8_t id, float absoluteAngle) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID: " + String(id));
    return;
  }
  float clamped = constrain(absoluteAngle, jointLimits_[index][0], jointLimits_[index][1]);

  // In extended/current-based position mode the motor travels directly from
  // present to goal on a linear tick number line (no shortest-path wrapping).
  // If present and goal differ by ~360° they represent the same physical angle
  // but the motor will spin a full revolution.  Snap goal to within ±180° of
  // the current position to always take the short route.
  clamped = applyShortestPath(index, clamped);

  noteGoalCommanded(index, clamped);
  dxl_.setGoalPosition(id, clamped, UNIT_DEGREE);
  debugPrint("[NMLHandExo] Setting motor " + String(id) + " to absolute angle " + String(clamped, 2));
}
float NMLHandExo::getZeroAngle(uint8_t id){
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint(F("Invalid motor ID"));
    return -1;
  }

  return zeroOffsets_[index];
}
void NMLHandExo::setHome(uint8_t id){
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint(F("Invalid motor ID"));
    return;
  }

  // Command the motor to move to the stored zero offset position
  float homeAngle = zeroOffsets_[index];

  // Shortest-path guard: avoid 360° rotations in extended position mode
  homeAngle = applyShortestPath(index, homeAngle);

  noteGoalCommanded(index, homeAngle);
  dxl_.setGoalPosition(id, homeAngle, UNIT_DEGREE);
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Motor %d homing to %.2f deg", id, homeAngle);
  debugPrint(buffer);
}
void NMLHandExo::homeAllMotors() {
  for (int i = 0; i < numMotors_; ++i) {
    uint8_t id = motorIds_[i];
    setHome(id);
  }
}
void NMLHandExo::setAngleById(uint8_t id, float angle_deg) {
  int index = getIndexById(id);
  if (index == -1) return;

  // Apply offset to relative angle position
  float abs_goal = zeroOffsets_[index] + angle_deg;

  // Clamp angle to joint limits (in degrees)
  abs_goal = constrain(abs_goal, jointLimits_[index][0], jointLimits_[index][1]);

  // Shortest-path guard: avoid 360° rotations in extended position mode
  abs_goal = applyShortestPath(index, abs_goal);

  // Set new goal tick position
  noteGoalCommanded(index, abs_goal);
  dxl_.setGoalPosition(id, abs_goal, UNIT_DEGREE);
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "[NMLHandExo] Setting motor %d to abs angle %.2f deg", id, abs_goal);
  debugPrint(buffer);
}
void NMLHandExo::setAngleByAlias(const String& alias, float angleDeg) {
  int id = getMotorIDByName(alias);
  if (id != -1) setAngleById((uint8_t)id, angleDeg);
}
void NMLHandExo::setMotorLowerBound(uint8_t id, float lowerBound) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID for lower bound update: " + String(id));
    return;
  }

  if (lowerBound > jointLimits_[index][1]) {
    debugPrint("Lower bound exceeds current upper bound for motor " + String(id));
    return;
  }

  jointLimits_[index][0] = lowerBound;
  directVelocityLimitBlock_[index] = 0;
  debugPrint("Set lower bound for motor " + String(id) + " to " + String(lowerBound) + " deg");
}
void NMLHandExo::setMotorUpperBound(uint8_t id, float upperBound) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID for upper bound update: " + String(id));
    return;
  }

  if (upperBound < jointLimits_[index][0]) {
    debugPrint("Upper bound below current lower bound for motor " + String(id));
    return;
  }

  jointLimits_[index][1] = upperBound;
  directVelocityLimitBlock_[index] = 0;
  debugPrint("Set upper bound for motor " + String(id) + " to " + String(upperBound) + " deg");
}
String NMLHandExo::getMotorLimits(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) {
    return "[ERROR] Invalid motor ID: " + String(id);
  }

  float min = jointLimits_[index][0];
  float max = jointLimits_[index][1];
  return "[" + String(min, 2) + ", " + String(max, 2) + "]";
}
void NMLHandExo::setMotorLimits(uint8_t id, float lowerLimit, float upperLimit) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID for setting limits: " + String(id));
    return;
  }

  // Check if limits are valid
  if (lowerLimit > upperLimit) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Invalid limits for motor %d: [%.2f, %.2f]", id, lowerLimit, upperLimit);
    debugPrint(buffer);
    return;
  }

  jointLimits_[index][0] = lowerLimit;
  jointLimits_[index][1] = upperLimit;
  directVelocityLimitBlock_[index] = 0;
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Set limits for motor %d: [%.2f, %.2f]", id, lowerLimit, upperLimit);
  debugPrint(buffer);
}
float NMLHandExo::getMotorLimitMin(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return -1;
  return jointLimits_[index][0];
}
float NMLHandExo::getMotorLimitMax(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return -1;
  return jointLimits_[index][1];
}

// ====================================================================================
// ============================ Gesture fractional axis ===============================
// ====================================================================================
float NMLHandExo::getGestureOrigin(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return 0.0f;
  const float lo = min(jointLimits_[index][0], jointLimits_[index][1]);
  const float hi = max(jointLimits_[index][0], jointLimits_[index][1]);
  return constrain(zeroOffsets_[index], lo, hi);
}
float NMLHandExo::getGestureSpan(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return 0.0f;

  const float lo = min(jointLimits_[index][0], jointLimits_[index][1]);
  const float hi = max(jointLimits_[index][0], jointLimits_[index][1]);
  const float origin = constrain(zeroOffsets_[index], lo, hi);

  // Travel actually available on each side of home.
  const float up = hi - origin;
  const float down = origin - lo;

  // The flip flag names the flexion direction; calibration normally leaves all
  // the travel on that side. Trust it unless the window says otherwise.
  const bool flipped = flipMotor_[index];
  const float preferred = flipped ? -down : up;
  const float opposite  = flipped ?  up   : -down;

  // Home mid-window with the flip side a stub: the flag cannot be describing
  // this joint's travel, so take the long side as flexion. Without this the
  // stub is the entire range, every state rounds onto the same boundary, and
  // the joint holds still while every command still acks.
  if (fabsf(opposite) > fabsf(preferred) * GESTURE_SPAN_OVERRIDE_RATIO &&
      fabsf(opposite) - fabsf(preferred) >= GESTURE_MIN_TRAVEL_DEG) {
    return opposite;
  }
  return preferred;
}
float NMLHandExo::gestureFractionToAngle(uint8_t id, float fraction) {
  return getGestureOrigin(id) + fraction * getGestureSpan(id);
}
float NMLHandExo::gestureAngleToFraction(uint8_t id, float angleDeg) {
  const float span = getGestureSpan(id);
  if (fabsf(span) < GESTURE_MIN_TRAVEL_DEG) return NAN;
  return (angleDeg - getGestureOrigin(id)) / span;
}
float NMLHandExo::applyShortestPath(int index, float goal) {
  if (index < 0 || index >= (int)numMotors_) return goal;
  const float present = dxl_.getPresentPosition(motorIds_[index], UNIT_DEGREE);
  const float diff = goal - present;
  if (fabsf(diff) <= 180.0f) return goal;

  const float lo = min(jointLimits_[index][0], jointLimits_[index][1]);
  const float hi = max(jointLimits_[index][0], jointLimits_[index][1]);
  const float wrapped = goal + (diff > 0.0f ? -360.0f : 360.0f);
  // Only a wrap-around duplicate of the same physical angle is worth taking;
  // if the shorter route leaves the calibrated window it was not a duplicate,
  // it was real travel on a joint whose window exceeds half a turn.
  if (wrapped < lo || wrapped > hi) return goal;
  return wrapped;
}


// ====================================================================================
// ============================ Torque commands =======================================
// ====================================================================================

bool NMLHandExo::getTorqueEnabledStatus(uint8_t id) {
  // Check if torque is enabled for the specified motor ID
  return dxl_.getTorqueEnableStat(id);
}

void NMLHandExo::enableTorque(uint8_t id, bool enable) {
  if (enable) {
    dxl_.torqueOn(id);
    debugPrint("Motor " + String(id) + " enabled");
  } else {
    dxl_.torqueOff(id);
    debugPrint("Motor " + String(id) + " disabled");
  }
}

int16_t NMLHandExo::getCurrentLimit(uint8_t id) {
  // Reads the current limit in mA from the motor's control table.
  int index = getIndexById(id);
  if (index == -1) {
      debugPrint(F("Invalid motor ID"));
      return -1;
  }
  return currentLimits_[index];
}

void NMLHandExo::setCurrentLimit(uint8_t id, uint16_t current_mA) {
  int index = getIndexById(id);
  if (index == -1) {
      debugPrint(F("Invalid motor ID"));
      return;
  }
  current_mA = min(current_mA, (uint16_t)MOTOR_CURRENT_LIMIT);
  currentLimits_[index] = current_mA;
  bool wasEnabled = getTorqueEnabledStatus(id);
  if (wasEnabled) dxl_.torqueOff(id);
  dxl_.writeControlTableItem(CURRENT_LIMIT, id, current_mA);
  if (wasEnabled) dxl_.torqueOn(id);
  if (motorControlMode_ == "CURRENT_POSITION") {
    // This sets the NOMINAL effort only. GOAL_CURRENT is owned by the combined
    // budget now, so writing it here directly would be overwritten on the next
    // allocation pass -- and would briefly bypass the fleet cap in the meantime.
    // Deferred for the same reason as noteGoalCommanded: `set_current_lim:all`
    // calls this once per motor.
    currentBudgetScale_ = staticBudgetScale();
    allocationDirty_ = true;
  }
  char buffer[64];
  // %u, not %.2f: current_mA is an integer, and passing it to a float
  // conversion is undefined behaviour that printed garbage for this line.
  snprintf(buffer, sizeof(buffer), "Set current limit for motor %u: %u mA",
           (unsigned)id, (unsigned)current_mA);
  debugPrint(buffer);
}
int16_t NMLHandExo::getCurrent(uint8_t id) {
  // XC330 PRESENT_CURRENT uses about 1 mA per raw unit.
  return dxl_.readControlTableItem(PRESENT_CURRENT, id);
}
float NMLHandExo::getTorque(uint8_t id) {
  float current_mA = NMLHandExo::getCurrent(id);
  float torque_Nm = current_mA * XC330_T288_TORQUE_CONSTANT;
  return torque_Nm;  // in N·m
}
bool NMLHandExo::setGoalCurrent(uint8_t id, float current_mA) {
  int index = getIndexById(id);
  if (index == -1 || motorControlMode_ != "CURRENT" || positionHoldActive_[index]) return false;

  current_mA = constrain(
      current_mA,
      -(float)DIRECT_CURRENT_LIMIT_MA,
      (float)DIRECT_CURRENT_LIMIT_MA);
  if (flipMotor_[index]) current_mA *= -1.0f;

  float position = getAbsoluteAngle(id);
  if ((current_mA > 0 && position >= jointLimits_[index][1] - DIRECT_LIMIT_MARGIN_DEG) ||
      (current_mA < 0 && position <= jointLimits_[index][0] + DIRECT_LIMIT_MARGIN_DEG)) {
    current_mA = 0;
  }

  dxl_.writeControlTableItem(GOAL_CURRENT, id, (int16_t)round(current_mA));
  lastDirectCommandMs_[index] = millis();
  directCommandActive_[index] = (current_mA != 0);
  directCommandDirection_[index] = current_mA;
  return true;
}
float NMLHandExo::getGoalCurrent(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return 0;
  float current_mA = (int16_t)dxl_.readControlTableItem(GOAL_CURRENT, id);
  return flipMotor_[index] ? -current_mA : current_mA;
}
// ====================================================================================
// ====================== Combined-motor current budget ===============================
// ====================================================================================
//
// Per-motor limits alone cannot protect the supply: N motors each honouring a
// 200 mA limit still draw up to N * 200 mA together. Enforcement here is two
// sided, and the two halves solve different problems.
//
//   Feed-forward (noteGoalCommanded -> staticBudgetScale) runs the instant a
//   goal is issued, with no measurement, so a posture that commands every joint
//   at once is already clamped before the first amp flows. On its own this is
//   the "divide the budget" scheme, which is safe but weak.
//
//   Feedback (serviceCurrentGovernor) then measures what the fleet actually
//   draws and relaxes that clamp whenever there is real headroom. Motors that
//   reached their target draw near zero, so in the common case of one or two
//   joints working the clamp lifts back to full per-motor effort within a few
//   samples. This is what stops the budget from making the exo feel dead.
//
// A motor drawing stall-level current for STALL_HOLD_MS is pushing against
// something rather than travelling, and is demoted to HOLD_CURRENT_MA. That
// costs no extra bus traffic -- it reuses the governor's own samples -- and is
// what keeps a hand parked on its endstops from holding at full effort forever.

void NMLHandExo::setTotalCurrentBudget(uint16_t budget_mA) {
  // The floor keeps the budget above what every motor needs just to hold, or
  // the allocator could not satisfy its own worst case.
  uint16_t floor_mA = (uint16_t)(numMotors_ * holdCurrentMa_);
  totalCurrentBudgetMa_ = constrain(budget_mA, floor_mA,
                                    (uint16_t)(numMotors_ * MOTOR_CURRENT_LIMIT));
  if (budget_mA < floor_mA) {
    debugPrint("[Budget] Requested " + String(budget_mA) + " mA is below the "
               + String(floor_mA) + " mA needed to hold " + String(numMotors_)
               + " motors; clamped. Lower hold current to go lower.");
  }
  currentBudgetScale_ = staticBudgetScale();
  refreshCurrentAllocation();
  debugPrint("[Budget] Total current budget: " + String(totalCurrentBudgetMa_) + " mA");
}
uint16_t NMLHandExo::getTotalCurrentBudget() const {
  return totalCurrentBudgetMa_;
}
void NMLHandExo::setHoldCurrent(uint16_t hold_mA) {
  // Every motor may sit at hold current simultaneously, so the whole fleet's
  // worth of it has to fit inside the budget.
  uint16_t ceiling = (numMotors_ > 0) ? (uint16_t)(totalCurrentBudgetMa_ / numMotors_)
                                      : holdCurrentMa_;
  holdCurrentMa_ = min(hold_mA, ceiling);
  if (hold_mA > ceiling) {
    debugPrint("[Budget] Hold current capped at " + String(ceiling) +
               " mA so all " + String(numMotors_) + " motors fit the budget");
  }
  refreshCurrentAllocation();
}
uint16_t NMLHandExo::getHoldCurrent() const {
  return holdCurrentMa_;
}
void NMLHandExo::setCurrentGovernorEnabled(bool enabled) {
  currentGovernorEnabled_ = enabled;
  if (!enabled) {
    // Without measurement the only defensible allocation is the static worst
    // case, so fall back to it rather than leaving a relaxed clamp in place.
    measuredTotalMa_ = -1;
    currentBudgetScale_ = staticBudgetScale();
    refreshCurrentAllocation();
  }
  debugPrint(String("[Budget] Governor ") + (enabled ? "enabled" : "disabled"));
}
bool NMLHandExo::getCurrentGovernorEnabled() const {
  return currentGovernorEnabled_;
}
int32_t NMLHandExo::getMeasuredTotalCurrent() const {
  return measuredTotalMa_;
}
float NMLHandExo::getCurrentBudgetScale() const {
  return currentBudgetScale_;
}
float NMLHandExo::staticBudgetScale() const {
  // Worst case if every currently-moving motor stalled at once. Motors that are
  // not moving are charged at hold current, since that is all they can draw.
  uint32_t movingNominal = 0;
  uint32_t heldTotal = 0;
  for (int i = 0; i < numMotors_; ++i) {
    if (motorMoving_[i]) {
      movingNominal += currentLimits_[i];
    } else {
      heldTotal += holdCurrentMa_;
    }
  }
  if (movingNominal == 0) return 1.0f;
  // Held motors are charged against the budget first; whatever is left is what
  // the moving ones may share.
  uint32_t available = (heldTotal >= totalCurrentBudgetMa_)
                         ? 0
                         : (totalCurrentBudgetMa_ - heldTotal);
  if (movingNominal <= available) return 1.0f;
  return (float)available / (float)movingNominal;
}
void NMLHandExo::applyGoalCurrent(int index, uint16_t current_mA) {
  // Never exceed what the user asked for, nor the part maximum.
  current_mA = min(current_mA, currentLimits_[index]);
  current_mA = min(current_mA, (uint16_t)MOTOR_CURRENT_LIMIT);
  if (appliedCurrents_[index] == current_mA) return;   // skip no-op bus writes
  appliedCurrents_[index] = current_mA;
  dxl_.writeControlTableItem(GOAL_CURRENT, motorIds_[index], current_mA);
}
uint8_t NMLHandExo::maxConcurrentMovers() const {
  // Largest k for which k motors can each hold MIN_MOVE_CURRENT_MA while the
  // remaining (N - k) sit at hold current, all inside the budget. Admitting
  // more than this is what starves the fleet into moving nothing at all.
  uint8_t best = 0;
  for (uint8_t k = 1; k <= numMotors_; ++k) {
    uint32_t heldTotal = (uint32_t)(numMotors_ - k) * holdCurrentMa_;
    if (heldTotal >= totalCurrentBudgetMa_) break;
    uint32_t available = totalCurrentBudgetMa_ - heldTotal;
    if (available < (uint32_t)k * MIN_MOVE_CURRENT_MA) break;
    best = k;
  }
  // Always let one motor through. If even a single mover does not fit, the
  // budget is smaller than one joint needs and nothing could ever move; one
  // motor at a time is the least-bad reading of that configuration.
  return best > 0 ? best : 1;
}
void NMLHandExo::refreshCurrentAllocation() {
  allocationDirty_ = false;
  if (motorControlMode_ != "CURRENT_POSITION") return;
  appliedScale_ = currentBudgetScale_;   // what the motors now actually reflect

  // --- Admission ---------------------------------------------------
  // Motors wanting to move are funded in waves. Already-admitted ones keep
  // their slot so a wave is not reshuffled mid-travel; freed slots go to
  // whoever is still waiting, in index order.
  const uint8_t limit = maxConcurrentMovers();
  uint8_t admitted = 0;
  for (int i = 0; i < numMotors_; ++i) {
    if (motorAdmitted_[i] && motorMoving_[i] && admitted < limit) {
      admitted++;
    } else {
      motorAdmitted_[i] = false;
    }
  }
  for (int i = 0; i < numMotors_ && admitted < limit; ++i) {
    if (motorMoving_[i] && !motorAdmitted_[i]) {
      motorAdmitted_[i] = true;
      admissionMs_[i] = millis();
      admitted++;
    }
  }

  // --- Allocation --------------------------------------------------
  // Everything not admitted is charged hold current; the admitted share what
  // is left. The share is a hard per-motor ceiling, so the sum can never
  // exceed the budget no matter what the governor's scale does.
  uint16_t share = 0;
  if (admitted > 0) {
    uint32_t heldTotal = (uint32_t)(numMotors_ - admitted) * holdCurrentMa_;
    uint32_t available = (heldTotal >= totalCurrentBudgetMa_)
                           ? 0 : (totalCurrentBudgetMa_ - heldTotal);
    share = (uint16_t)(available / admitted);
  }

  for (int i = 0; i < numMotors_; ++i) {
    uint16_t target;
    if (!motorAdmitted_[i]) {
      target = holdCurrentMa_;
    } else {
      uint16_t ceiling = min(currentLimits_[i], share);
      target = (uint16_t)(ceiling * currentBudgetScale_);
      // Floor at what it takes to actually move. Safe by construction: the
      // admission limit above guarantees `admitted * MIN_MOVE_CURRENT_MA` plus
      // the held motors still fits the budget, so this floor cannot breach it.
      uint16_t floor_mA = min((uint16_t)MIN_MOVE_CURRENT_MA, ceiling);
      target = max(target, floor_mA);
    }
    applyGoalCurrent(i, target);
  }
}
void NMLHandExo::noteGoalCommanded(int index, float goalAngle) {
  if (index < 0 || index >= numMotors_) return;
  unsigned long now = millis();
  motorMoving_[index] = true;
  goalIssuedMs_[index] = now;
  stallSinceMs_[index] = 0;      // a fresh goal clears any stall demotion
  lastGoalIssuedMs_ = now;
  goalAngle_[index] = goalAngle; // kept so the verdict can measure the error
  verdictPending_[index] = true;

  // Clamp NOW, from the static worst case, rather than waiting for the first
  // sample. A whole-hand posture writes every goal in one pass, so measurement
  // would arrive milliseconds after the inrush that causes the brownout.
  //
  // The new scale is computed here but NOT written: a whole-hand gesture calls
  // this once per motor, and refreshing inside each call would rewrite the
  // whole fleet every time -- O(N^2) register writes, about 171 of them for 18
  // motors, tens of milliseconds of bus time for one command. Instead the batch
  // is marked dirty and flushed once from update(), which loop() calls
  // immediately after parseMessage. By then every motor in the batch has been
  // counted, so each one gets its final allocation in a single write.
  float feedForward = staticBudgetScale();
  if (feedForward < currentBudgetScale_) {
    currentBudgetScale_ = feedForward;
  }
  allocationDirty_ = true;
}
void NMLHandExo::concludeMove(int index, uint8_t verdict) {
  // Single exit for a move, so every path that ends one -- arrival, stall,
  // load shedding, timeout -- records a verdict and frees its admission slot.
  motorMoving_[index] = false;
  motorAdmitted_[index] = false;
  stallSinceMs_[index] = 0;
  allocationDirty_ = true;       // a freed slot lets the next wave in

  if (!verdictPending_[index]) return;
  verdictPending_[index] = false;

  if (verdict == MOVE_VERDICT_REACHED) {
    // "Settled" only means it stopped pulling; confirm it actually arrived.
    // This is the one extra position read per move, not per sample.
    float present = dxl_.getPresentPosition(motorIds_[index], UNIT_DEGREE);
    float error = present - goalAngle_[index];
    if (error < 0) error = -error;
    if (error > GESTURE_REACH_TOLERANCE_DEG) verdict = MOVE_VERDICT_SHORT;
  }
  lastVerdict_[index] = verdict;
  if (verdict != MOVE_VERDICT_REACHED) verdictFailures_++;
  verdictsCollected_++;
}
void NMLHandExo::reportMoveVerdicts() {
  // Emitted once per batch, never once per motor: each telemetryPrintln is a
  // blocking USB-CDC write, and the gesture path was explicitly optimised down
  // to one write per command. A summary keeps that property.
  if (verdictsCollected_ == 0) return;
  for (int i = 0; i < numMotors_; ++i) {
    if (verdictPending_[i]) return;          // batch still finishing
  }

  uint8_t reached = 0, stalled = 0, shortfall = 0, starved = 0;
  String detail;
  for (int i = 0; i < numMotors_; ++i) {
    switch (lastVerdict_[i]) {
      case MOVE_VERDICT_REACHED: reached++; continue;
      case MOVE_VERDICT_STALLED: stalled++; break;
      case MOVE_VERDICT_SHORT:   shortfall++; break;
      case MOVE_VERDICT_STARVED: starved++; break;
      default: continue;
    }
    if (detail.length() < 80) {
      if (detail.length()) detail += ",";
      detail += String(motorIds_[i]) + ":";
      detail += (lastVerdict_[i] == MOVE_VERDICT_STALLED) ? "stalled"
              : (lastVerdict_[i] == MOVE_VERDICT_STARVED) ? "starved" : "short";
    }
  }

  String line = "GESTURE_RESULT: reached=" + String(reached) +
                " stalled=" + String(stalled) +
                " short=" + String(shortfall) +
                " starved=" + String(starved) +
                " budget_scale=" + String(currentBudgetScale_, 2);
  if (detail.length()) line += " detail=" + detail;
  telemetryPrintln(line);

  verdictsCollected_ = 0;
  verdictFailures_ = 0;
  for (int i = 0; i < numMotors_; ++i) lastVerdict_[i] = MOVE_VERDICT_NONE;
}
int16_t NMLHandExo::readPresentCurrentMa(uint8_t id, bool& ok) {
  // Same defensive shape as the fast-telemetry path: drain any stale bytes
  // first, then check the library error code rather than trusting the value.
  while (DXL_SERIAL.available() > 0) {
    DXL_SERIAL.read();
  }
  // Both arguments are explicitly typed. Dynamixel2Arduino overloads this on
  // (item, id, timeout) and (model_num, item, id, timeout=default), so a bare
  // integer literal for the timeout is ambiguous between the two -- it can be
  // read as this call's timeout or as the model-number form's id.
  const uint8_t item = (uint8_t)PRESENT_CURRENT;
  const uint32_t timeout_ms = 10;
  int32_t raw = dxl_.readControlTableItem(item, id, timeout_ms);
  if (dxl_.getLastLibErrCode() != DXL_LIB_OK) {
    ok = false;
    return 0;
  }
  return (int16_t)raw;
}
void NMLHandExo::serviceCurrentGovernor() {
  if (!currentGovernorEnabled_) return;
  if (motorControlMode_ != "CURRENT_POSITION") return;
  if (numMotors_ == 0) return;

  unsigned long now = millis();

  // Sampling only runs while something is actually happening. An idle exo adds
  // no bus traffic at all, which matters because repeated per-motor telemetry
  // reads have been seen to wedge this bus.
  bool active = (now - lastGoalIssuedMs_) < CURRENT_GOVERNOR_ACTIVE_MS;
  if (!active) {
    for (int i = 0; i < numMotors_; ++i) {
      if (motorMoving_[i]) { active = true; break; }
    }
  }
  if (!active) {
    governorCursor_ = 0;     // start the next burst of activity on a clean sweep
    sweepAccumMa_ = 0;
    return;
  }
  if (now - lastGovernorSampleMs_ < CURRENT_GOVERNOR_SAMPLE_INTERVAL_MS) return;
  lastGovernorSampleMs_ = now;

  // ONE motor per pass. Reading the whole fleet in a burst would block the
  // control loop for milliseconds and put that jitter directly onto command
  // latency; the control law below runs when the cursor completes a sweep.
  bool ok = true;
  int cursor = governorCursor_;
  int16_t current_mA = readPresentCurrentMa(motorIds_[cursor], ok);
  if (ok) {
    uint16_t magnitude = (current_mA < 0) ? (uint16_t)(-current_mA)
                                          : (uint16_t)current_mA;
    sweepAccumMa_ += magnitude;

    if (motorMoving_[cursor]) {
      bool windowElapsed = (now - goalIssuedMs_[cursor]) >= MOVE_WINDOW_MS;
      uint16_t allowance = max(appliedCurrents_[cursor], (uint16_t)1);

      if (!motorAdmitted_[cursor]) {
        // Waiting for a slot. It holds its goal but is only funded at hold
        // current, so it must not be judged on progress it was never given the
        // current to make -- and its clock only starts when it is admitted.
        if ((now - goalIssuedMs_[cursor]) >= MOVE_TIMEOUT_MS) {
          concludeMove(cursor, MOVE_VERDICT_STARVED);
        }
      } else if (windowElapsed && magnitude <= CURRENT_SETTLED_MA) {
        // Stopped pulling. concludeMove confirms it actually arrived, and
        // downgrades to SHORT if it stopped somewhere else. Releasing it also
        // frees its slot for the next wave -- without this nothing ever clears
        // motorMoving_ and the governor would sample forever after one move.
        concludeMove(cursor, MOVE_VERDICT_REACHED);
      } else if ((now - admissionMs_[cursor]) >= MOVE_TIMEOUT_MS) {
        // Funded, given time, still not there. Do not let it hold a slot.
        concludeMove(cursor, MOVE_VERDICT_STALLED);
      } else if (magnitude >= (uint16_t)(allowance * STALL_CURRENT_FRACTION)) {
        // Pushing rather than travelling. This only MARKS the motor as a
        // shedding candidate; see the load-shedding pass below, which fires
        // only under real budget pressure so deliberate grip is not weakened.
        if (stallSinceMs_[cursor] == 0) stallSinceMs_[cursor] = now;
      } else {
        stallSinceMs_[cursor] = 0;
      }
    }

    governorCursor_++;
    if (governorCursor_ < numMotors_) return;   // sweep still in progress
  }

  // Sweep complete (or aborted by a read error): act on what it found.
  uint32_t total = sweepAccumMa_;
  governorCursor_ = 0;
  sweepAccumMa_ = 0;

  if (!ok) {
    // Fail safe: an unreadable bus means we do not know the draw, so fall back
    // to the static worst case rather than assuming there is headroom.
    measuredTotalMa_ = -1;
    if (++governorReadFails_ >= CURRENT_GOVERNOR_MAX_READ_FAILS) {
      if (governorMeasurementTrusted_) {
        governorMeasurementTrusted_ = false;
        debugPrint(F("[Budget] Current reads failing; using static clamp"));
      }
      currentBudgetScale_ = staticBudgetScale();
      refreshCurrentAllocation();
    }
    return;
  }
  governorReadFails_ = 0;
  if (!governorMeasurementTrusted_) {
    governorMeasurementTrusted_ = true;
    debugPrint(F("[Budget] Current reads recovered"));
  }
  measuredTotalMa_ = (int32_t)total;

  // Load shedding, applied ONLY under budget pressure.
  //
  // Demoting every stalled motor unconditionally would be wrong: a motor
  // holding a grasp against a spastic hand is stalled by definition, and that
  // is the device doing its job. So a stall is merely a candidate, and the
  // longest-stalled candidate is shed one per sample, and only while the fleet
  // is over budget. That converges on the smallest set of motors that has to
  // give way, instead of dropping the whole hand to hold current at once.
  //
  // A shed motor stays shed until it is commanded again -- promoting it back
  // while it is still pushing would just stall it a second time and oscillate.
  if (total > totalCurrentBudgetMa_) {
    int worst = -1;
    unsigned long longest = 0;
    for (int i = 0; i < numMotors_; ++i) {
      if (!motorMoving_[i] || stallSinceMs_[i] == 0) continue;
      unsigned long stalledFor = now - stallSinceMs_[i];
      if (stalledFor >= STALL_HOLD_MS && stalledFor >= longest) {
        longest = stalledFor;
        worst = i;
      }
    }
    if (worst >= 0) {
      concludeMove(worst, MOVE_VERDICT_STALLED);
      debugPrint("[Budget] Shedding motor " + String(motorIds_[worst]) +
                 " to " + String(holdCurrentMa_) + " mA (fleet at " +
                 String(total) + " mA over " + String(totalCurrentBudgetMa_) + ")");
    }
  }

  // Multiplicative decrease, additive increase. Clamping down is immediate and
  // proportional to the overshoot; relaxing is gradual and only inside the
  // deadband, so the allocation does not oscillate around the budget.
  if (total > totalCurrentBudgetMa_) {
    float correction = (float)totalCurrentBudgetMa_ / (float)total;
    currentBudgetScale_ = max(staticBudgetScale() * 0.5f,
                              currentBudgetScale_ * correction);
  } else if (total < (uint32_t)(totalCurrentBudgetMa_ * CURRENT_GOVERNOR_RELEASE_FRACTION)) {
    currentBudgetScale_ = min(1.0f, currentBudgetScale_ + CURRENT_GOVERNOR_RECOVERY_STEP);
  }
  // Deadband before re-writing: a refresh is up to N_MOTORS register writes, so
  // pushing every marginal adjustment would cost more bus time than the
  // sampling does. The comparison is against the last APPLIED scale, not the
  // previous iteration's, so a run of sub-deadband corrections accumulates
  // until it is worth a write instead of being discarded one at a time.
  // Reaching a hard 1.0 always applies, so full effort is never left stranded a
  // hair below its ceiling.
  if (fabsf(currentBudgetScale_ - appliedScale_) >= CURRENT_GOVERNOR_APPLY_DEADBAND ||
      (currentBudgetScale_ >= 1.0f && appliedScale_ < 1.0f)) {
    refreshCurrentAllocation();
  }
}
String NMLHandExo::getCurrentBudgetStatus() {
  String out = "Current budget:\n";
  out += "  total_budget_mA: " + String(totalCurrentBudgetMa_) + "\n";
  out += "  hold_current_mA: " + String(holdCurrentMa_) + "\n";
  out += "  governor: " + String(currentGovernorEnabled_ ? "on" : "off") + "\n";
  out += "  measured_total_mA: " +
         (measuredTotalMa_ < 0 ? String("n/a") : String(measuredTotalMa_)) + "\n";
  out += "  scale: " + String(currentBudgetScale_, 3) + "\n";
  out += "  measurement_trusted: " +
         String(governorMeasurementTrusted_ ? "true" : "false") + "\n";
  for (int i = 0; i < numMotors_; ++i) {
    out += "Motor " + String(i) + ": {name: " + getMotorNameByID(motorIds_[i]) +
           ", id: " + String(motorIds_[i]) +
           ", nominal_mA: " + String(currentLimits_[i]) +
           ", applied_mA: " + String(appliedCurrents_[i]) +
           ", state: " + String(motorMoving_[i] ? "moving" : "hold") + "}\n";
  }
  return out;
}

void NMLHandExo::setZeroOffsetValue(uint8_t id, float offset_deg) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID for zero offset: " + String(id));
    return;
  }
  zeroOffsets_[index] = offset_deg;
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Zero offset for motor %d set to %.2f deg", id, offset_deg);
  debugPrint(buffer);
}
void NMLHandExo::setFlipMotor(uint8_t id, bool flip) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID for flip: " + String(id));
    return;
  }
  flipMotor_[index] = flip;
  debugPrint("Motor " + String(id) + " flip set to " + String(flip ? "true" : "false"));
}
bool NMLHandExo::getFlipMotor(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return false;
  return flipMotor_[index];
}
void NMLHandExo::setTorque(uint8_t id, float torque_Nm) {
  int index = getIndexById(id);
  if (index == -1) {
      debugPrint(F("Invalid motor ID"));
      return;
  }

  // Convert Nm to mA
  uint16_t current_mA = (uint16_t)(torque_Nm / XC330_T288_TORQUE_CONSTANT);
  setCurrentLimit(id, current_mA);
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Torque limit for motor %d: set to %.2f N·m", id, torque_Nm);
  debugPrint(buffer);

}

// Velocity commands
void NMLHandExo::setVelocityLimit(uint8_t id, uint32_t vel) {
  dxl_.writeControlTableItem(PROFILE_VELOCITY, id, vel);
  debugPrint("Velocity limit set for motor " + String(id) + ": " + String(vel));
}
uint32_t NMLHandExo::getVelocityLimit(uint8_t id) {
  return dxl_.readControlTableItem(PROFILE_VELOCITY, id);
}
float NMLHandExo::limitDirectVelocity(
    int index, float velocity_rpm, float position) {
  if (velocity_rpm == 0.0f) return 0.0f;

  const int8_t direction = velocity_rpm > 0.0f ? 1 : -1;
  const float lower = jointLimits_[index][0];
  const float upper = jointLimits_[index][1];
  const float halfRange = max(0.0f, (upper - lower) * 0.5f);
  const float softZone = min(
      DIRECT_VELOCITY_SOFT_ZONE_DEG,
      max(DIRECT_LIMIT_MARGIN_DEG, halfRange));

  // A command away from a blocked boundary is always allowed immediately.
  if (directVelocityLimitBlock_[index] != 0 &&
      direction != directVelocityLimitBlock_[index]) {
    directVelocityLimitBlock_[index] = 0;
  }

  const float distanceToLimit =
      direction > 0 ? upper - position : position - lower;
  if (directVelocityLimitBlock_[index] == direction) {
    if (distanceToLimit < softZone) return 0.0f;
    directVelocityLimitBlock_[index] = 0;
  }

  if (distanceToLimit <= DIRECT_LIMIT_MARGIN_DEG) {
    directVelocityLimitBlock_[index] = direction;
    return 0.0f;
  }
  if (distanceToLimit >= softZone ||
      softZone <= DIRECT_LIMIT_MARGIN_DEG) {
    return velocity_rpm;
  }

  const float normalized =
      (distanceToLimit - DIRECT_LIMIT_MARGIN_DEG) /
      (softZone - DIRECT_LIMIT_MARGIN_DEG);
  const float smoothScale =
      normalized * normalized * (3.0f - 2.0f * normalized);
  return velocity_rpm * smoothScale;
}
bool NMLHandExo::setGoalVelocity(uint8_t id, float velocity_rpm) {
  int index = getIndexById(id);
  if (index == -1 || motorControlMode_ != "VELOCITY" ||
      !directVelocityLimitVerified_[index] || positionHoldActive_[index]) return false;

  velocity_rpm = constrain(
      velocity_rpm,
      -DIRECT_VELOCITY_LIMIT_RPM,
      DIRECT_VELOCITY_LIMIT_RPM);
  if (flipMotor_[index]) velocity_rpm *= -1.0f;

  float position = getAbsoluteAngle(id);
  float limited_velocity_rpm =
      limitDirectVelocity(index, velocity_rpm, position);

  int32_t raw = (int32_t)round(limited_velocity_rpm / 0.229f);
  const int32_t requestedRaw = (int32_t)round(velocity_rpm / 0.229f);
  if (raw == 0 && requestedRaw != 0 && limited_velocity_rpm != 0.0f) {
    // The taper fell below one register step. Latch here rather than letting
    // repeated teleop packets alternate between raw velocity 0 and 1.
    directVelocityLimitBlock_[index] = velocity_rpm > 0.0f ? 1 : -1;
  }

  dxl_.writeControlTableItem(GOAL_VELOCITY, id, raw);
  lastDirectCommandMs_[index] = millis();
  directCommandActive_[index] = (raw != 0);
  directCommandDirection_[index] = velocity_rpm;
  return true;
}
float NMLHandExo::getPresentVelocity(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return 0;
  int32_t raw = (int32_t)dxl_.readControlTableItem(PRESENT_VELOCITY, id);
  float rpm = raw * 0.229f;
  return flipMotor_[index] ? -rpm : rpm;
}
void NMLHandExo::stopDirectControl(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return;
  if (positionHoldActive_[index]) {
    directCommandActive_[index] = false;
    directCommandDirection_[index] = 0;
    lastDirectCommandMs_[index] = millis();
    return;
  } else if (motorControlMode_ == "VELOCITY") {
    dxl_.writeControlTableItem(GOAL_VELOCITY, id, 0);
  } else if (motorControlMode_ == "CURRENT") {
    dxl_.writeControlTableItem(GOAL_CURRENT, id, 0);
  }
  directCommandActive_[index] = false;
  directCommandDirection_[index] = 0;
  lastDirectCommandMs_[index] = millis();
}
void NMLHandExo::stopAllDirectControl() {
  for (int i = 0; i < numMotors_; ++i) {
    stopDirectControl(motorIds_[i]);
  }
}
void NMLHandExo::setDirectCommandTimeout(unsigned long timeout_ms) {
  directCommandTimeoutMs_ = constrain(timeout_ms, 50UL, 5000UL);
}
unsigned long NMLHandExo::getDirectCommandTimeout() const {
  return directCommandTimeoutMs_;
}

bool NMLHandExo::holdRelativePosition(
    uint8_t id, float relativeAngle, uint16_t requestedCurrentMa) {
  int index = getIndexById(id);
  if (index == -1) return false;
  if (motorControlMode_ != "VELOCITY" && motorControlMode_ != "CURRENT") {
    return false;
  }

  stopDirectControl(id);
  directVelocityLimitBlock_[index] = 0;
  enableTorque(id, false);
  setMotorControlMode(id, "CURRENT_POSITION");

  // Mixed-mode holds are outside the global current-position allocator.
  // Give the held joint only the configured settled-motor current, bounded by
  // its per-motor limit and the part maximum.
  uint16_t hold_mA = requestedCurrentMa > 0
                       ? requestedCurrentMa
                       : holdCurrentMa_;
  hold_mA = min(hold_mA, currentLimits_[index]);
  hold_mA = min(hold_mA, (uint16_t)MOTOR_CURRENT_LIMIT);
  hold_mA = min(hold_mA, totalCurrentBudgetMa_);
  appliedCurrents_[index] = hold_mA;
  dxl_.writeControlTableItem(GOAL_CURRENT, id, hold_mA);
  setRelativeAngle(id, relativeAngle);  // Existing joint-limit clamp applies.
  positionHoldActive_[index] = true;
  enableTorque(id, true);
  return true;
}

uint16_t NMLHandExo::getPositionHoldCurrent(uint8_t id) const {
  for (int i = 0; i < numMotors_; ++i) {
    if (motorIds_[i] == id) {
      return positionHoldActive_[i] ? appliedCurrents_[i] : 0;
    }
  }
  return 0;
}

bool NMLHandExo::releasePositionHold(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return false;
  enableTorque(id, false);
  positionHoldActive_[index] = false;
  appliedCurrents_[index] = 0;
  directCommandActive_[index] = false;
  directCommandDirection_[index] = 0;
  directVelocityLimitBlock_[index] = 0;
  setMotorControlMode(id, motorControlMode_);
  return true;
}

bool NMLHandExo::isPositionHoldActive(uint8_t id) const {
  for (int i = 0; i < numMotors_; ++i) {
    if (motorIds_[i] == id) return positionHoldActive_[i];
  }
  return false;
}
void NMLHandExo::serviceDirectControlSafety() {
  if (motorControlMode_ != "VELOCITY" && motorControlMode_ != "CURRENT") return;
  unsigned long now = millis();
  for (int i = 0; i < numMotors_; ++i) {
    if (!directCommandActive_[i]) continue;
    uint8_t id = motorIds_[i];
    float position = getAbsoluteAngle(id);
    bool timedOut = now - lastDirectCommandMs_[i] > directCommandTimeoutMs_;
    bool drivingIntoLimit = (
        (directCommandDirection_[i] < 0 &&
         position <= jointLimits_[i][0] + DIRECT_LIMIT_MARGIN_DEG) ||
        (directCommandDirection_[i] > 0 &&
         position >= jointLimits_[i][1] - DIRECT_LIMIT_MARGIN_DEG));
    if (timedOut || drivingIntoLimit) {
      if (drivingIntoLimit && motorControlMode_ == "VELOCITY") {
        directVelocityLimitBlock_[i] =
            directCommandDirection_[i] > 0 ? 1 : -1;
      }
      stopDirectControl(id);
    }
  }
}

// Acceleration commands
void NMLHandExo::setAccelerationLimit(uint8_t id, uint32_t acc) {
  dxl_.writeControlTableItem(PROFILE_ACCELERATION, id, acc);
  debugPrint("Acceleration limit set for motor " + String(id) + ": " + String(acc));
}
uint32_t NMLHandExo::getAccelerationLimit(uint8_t id) {
  return dxl_.readControlTableItem(PROFILE_ACCELERATION, id);
}

// Motor-specific commands
void NMLHandExo::rebootMotor(uint8_t id) {
  dxl_.reboot(id);
  debugPrint("Motor ID:" + String(id) + " rebooted");
}
void NMLHandExo::getMotorInfo(uint8_t id) {
  dxl_.ping(id);  // could be expanded to read Model Number, Version, etc.
  debugPrint("Pinged motor ID: " + String(id));
}
void NMLHandExo::setBaudRate(uint8_t id, uint32_t baudrate) {
  dxl_.writeControlTableItem(BAUD_RATE, id, baudrate);
  debugPrint("Motor ID:" + String(id) + " baudrate set to " + String(baudrate));
}
uint32_t NMLHandExo::getBaudRate(uint8_t id) {
  return dxl_.readControlTableItem(BAUD_RATE, id);
}
void NMLHandExo::setMotorLED(uint8_t id, bool state) {
  // Sets specified motor LED to the specified state
  if (state) {
    dxl_.ledOn(id);
  } else {
    dxl_.ledOff(id);
  }
}
void NMLHandExo::setAllMotorLED(bool state) {
  // Sets the state of all motor LEDs to the specified state
  for (int i = 0; i < numMotors_; i++) {
    uint8_t id = motorIds_[i];
    setMotorLED(id, state);
  }
}
void NMLHandExo::setMotorControlMode(uint8_t id, const String& mode){
  String m = mode;
  m.toUpperCase();

  if (m == "POSITION") {
    dxl_.setOperatingMode(id, OP_POSITION);
    debugPrint("Set motor " + String(id) + " to POSITION mode");
  } else if (m == "CURRENT_POSITION") {
    dxl_.setOperatingMode(id, OP_CURRENT_BASED_POSITION);
    debugPrint("Set motor " + String(id) + " to CURRENT_POSITION mode");
  } else if (m == "VELOCITY") {
    dxl_.setOperatingMode(id, OP_VELOCITY);
    debugPrint("Set motor " + String(id) + " to VELOCITY mode");
  } else if (m == "CURRENT") {
    dxl_.setOperatingMode(id, OP_CURRENT);
    debugPrint("Set motor " + String(id) + " to CURRENT mode");
  } else {
    debugPrint("[ERROR] Unknown operating mode: " + m);
  }
}
bool NMLHandExo::ensureDirectVelocityLimit(uint8_t id) {
  uint32_t current = dxl_.readControlTableItem(VELOCITY_LIMIT, id);
  if (current != DIRECT_VELOCITY_LIMIT_RAW) {
    if (!dxl_.writeControlTableItem(
            VELOCITY_LIMIT, id, DIRECT_VELOCITY_LIMIT_RAW)) {
      debugPrint("[ERROR] Could not write VELOCITY_LIMIT for motor " + String(id));
      return false;
    }
    current = dxl_.readControlTableItem(VELOCITY_LIMIT, id);
  }
  if (current != DIRECT_VELOCITY_LIMIT_RAW) {
    debugPrint("[ERROR] VELOCITY_LIMIT readback mismatch for motor " +
               String(id) + ": " + String(current));
    return false;
  }
  return true;
}

bool NMLHandExo::setMotorControlMode(const String& mode) {
  String m = mode;
  m.toUpperCase();
  for (int i = 0; i < numMotors_; i++) {
    dxl_.torqueOff(motorIds_[i]);
  }
  if (m == "VELOCITY") {
    for (int i = 0; i < numMotors_; i++) {
      directVelocityLimitVerified_[i] = false;
      // Dual firmware can legitimately run with only one hand attached.
      // Missing configured IDs must not block the reachable side, but an ID
      // that was not verified is rejected later by setGoalVelocity().
      if (dxl_.ping(motorIds_[i]) == 0) continue;
      if (!ensureDirectVelocityLimit(motorIds_[i])) {
        motorControlMode_ = "DISABLED";
        stopAllDirectControl();
        return false;
      }
      directVelocityLimitVerified_[i] = true;
    }
  } else {
    for (int i = 0; i < numMotors_; i++) {
      directVelocityLimitVerified_[i] = false;
    }
  }
  for (int i = 0; i < numMotors_; i++) {
    uint8_t id = motorIds_[i];
    NMLHandExo::setMotorControlMode(id, m); // Set the mode for each motor
    directCommandActive_[i] = false;
    directCommandDirection_[i] = 0;
    directVelocityLimitBlock_[i] = 0;
    positionHoldActive_[i] = false;
  }
  motorControlMode_ = m;
  stopAllDirectControl();
  return true;
}
String NMLHandExo::getMotorControlMode() {
  String mode = "UNKNOWN";
  if (numMotors_ > 0) {
    mode = motorControlMode_; // return the internally tracked mode
  }
  return mode;
}

