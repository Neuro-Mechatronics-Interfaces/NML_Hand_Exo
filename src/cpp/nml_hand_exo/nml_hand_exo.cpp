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

/// @brief Flag for mode switching
static volatile bool modeSwitchFlag = false;

/// @brief Checking last interrupt time to check for unwanted presses, control debounce
static unsigned long lastInterruptTime = 0;

Stream* debugStream = &DEBUG_SERIAL;

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
  for (int i = 0; i < numMotors_; ++i) {
    flipMotor_[i] = DEFAULT_FLIPS[i];
    lastDirectCommandMs_[i] = 0;
    directCommandActive_[i] = false;
    directCommandDirection_[i] = 0;
  }
  motorControlMode_ = "CURRENT_POSITION";


  // If jointLimits_, zeroOffsets_, currentLimits_ were dynamically allocated, make sure to add a destructor.
}
// ====================================================================================
// ================================ Utility functions =================================
// ====================================================================================
void NMLHandExo::initializeSerial(int baud) {
  // Initialize serial communication with DYNAMIXEL hardware using the specified baudrate. Has to match hardware
  dxl_.begin(baud);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl_.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}
void NMLHandExo::initializeMotors() {
  // Configure motor operating modes and torque, but DON'T move them yet
  for (int i = 0; i < numMotors_; i++) {
    uint8_t id = motorIds_[i];
    dxl_.torqueOff(id);
    dxl_.setOperatingMode(id, OP_CURRENT_BASED_POSITION);
    dxl_.torqueOn(id);
    setCurrentLimit(id, MOTOR_CURRENT_LIMIT);
    dxl_.writeControlTableItem(GOAL_CURRENT, id, currentLimits_[i]);
  }
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

String NMLHandExo::getDeviceInfo() {

    // Need to return a single string with all the information
    String info = "Name: NMLHandExo\n";
    info += "Version: " + String(VERSION) + "\n";
    info += "Side: " + String(HAND_SIDE) + "\n";
    info += "Number of Motors: " + String(numMotors_) + "\n";
    for (int i = 0; i < numMotors_; ++i) {

      uint8_t id = getMotorIDByIndex(i);
      String name = getMotorNameByID(id);
      float angle = getRelativeAngle(id);
      float abs = getAbsoluteAngle(id);
      float minLimit = jointLimits_[i][0];
      float maxLimit = jointLimits_[i][1];
      float torque = getTorque(id);
      bool isEnabled = getTorqueEnabledStatus(id);

      info += "Motor " + String(i) + ": {name: " + String(name) +
            ", id: " + String(id) +
            ", angle: " + String(angle, 2) + "(abs: " + String(abs) + ")"
            ", limits: [" + String(minLimit, 2) + ";" + String(maxLimit, 2) + "]" +
            ", torque: " + String(torque, 2) +
            ", enabled: " + (isEnabled ? "true" : "false") + "}\n";
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
      DEBUG_SERIAL.println("[Gesture] Timed calibration mode. You have " + String(duration) + " seconds to complete.");
  } else {
      DEBUG_SERIAL.println(F("[Gesture] Step-through calibration mode. Follow prompts to set limits."));
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
    DEBUG_SERIAL.println("[Gesture] Calibration complete.");
    for (uint8_t i = 0; i < this->getMotorCount(); ++i) {
      DEBUG_SERIAL.println("Motor " + String(i) + ": Min = " + String(jointLimits_[i][0]) +
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
  float currentPos = dxl_.getPresentPosition(id, UNIT_DEGREE);
  float diff = abs_goal - currentPos;
  if (diff > 180.0f)       abs_goal -= 360.0f;
  else if (diff < -180.0f) abs_goal += 360.0f;

  // Command the motor to the absolute position
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
  float currentPos = dxl_.getPresentPosition(id, UNIT_DEGREE);
  float diff = clamped - currentPos;
  if (diff > 180.0f)       clamped -= 360.0f;
  else if (diff < -180.0f) clamped += 360.0f;

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
  float currentPos = dxl_.getPresentPosition(id, UNIT_DEGREE);
  float diff = homeAngle - currentPos;
  if (diff > 180.0f)       homeAngle -= 360.0f;
  else if (diff < -180.0f) homeAngle += 360.0f;

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
  float currentPos = dxl_.getPresentPosition(id, UNIT_DEGREE);
  float diff = abs_goal - currentPos;
  if (diff > 180.0f)       abs_goal -= 360.0f;
  else if (diff < -180.0f) abs_goal += 360.0f;

  // Set new goal tick position
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
  debugPrint("Set upper bound for motor " + String(id) + " to " + String(upperBound) + " deg");
}
String NMLHandExo::getMotorLimits(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) {
    return "[ERROR] Invalid motor ID: " + String(id);
  }

  float min = jointLimits_[index][0];
  float max = jointLimits_[index][1];
  return String(min, 2) + ";" + String(max, 2);
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
    dxl_.writeControlTableItem(GOAL_CURRENT, id, current_mA);
  }
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Set current limit for motor %d: %.2f mA", id, current_mA);
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
  if (index == -1 || motorControlMode_ != "CURRENT") return false;

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
bool NMLHandExo::setGoalVelocity(uint8_t id, float velocity_rpm) {
  int index = getIndexById(id);
  if (index == -1 || motorControlMode_ != "VELOCITY") return false;

  velocity_rpm = constrain(
      velocity_rpm,
      -DIRECT_VELOCITY_LIMIT_RPM,
      DIRECT_VELOCITY_LIMIT_RPM);
  if (flipMotor_[index]) velocity_rpm *= -1.0f;

  float position = getAbsoluteAngle(id);
  if ((velocity_rpm > 0 && position >= jointLimits_[index][1] - DIRECT_LIMIT_MARGIN_DEG) ||
      (velocity_rpm < 0 && position <= jointLimits_[index][0] + DIRECT_LIMIT_MARGIN_DEG)) {
    velocity_rpm = 0;
  }

  int32_t raw = (int32_t)round(velocity_rpm / 0.229f);
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
  if (motorControlMode_ == "VELOCITY") {
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
void NMLHandExo::setMotorControlMode(const String& mode) {
  String m = mode;
  m.toUpperCase();
  for (int i = 0; i < numMotors_; i++) {
    dxl_.torqueOff(motorIds_[i]);
  }
  for (int i = 0; i < numMotors_; i++) {
    uint8_t id = motorIds_[i];
    NMLHandExo::setMotorControlMode(id, m); // Set the mode for each motor
    directCommandActive_[i] = false;
    directCommandDirection_[i] = 0;
  }
  motorControlMode_ = m;
  stopAllDirectControl();
}
String NMLHandExo::getMotorControlMode() {
  String mode = "UNKNOWN";
  if (numMotors_ > 0) {
    mode = motorControlMode_; // return the internally tracked mode
  }
  return mode;
}

