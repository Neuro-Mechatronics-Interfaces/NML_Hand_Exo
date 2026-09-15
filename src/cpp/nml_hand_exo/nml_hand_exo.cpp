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
#include "io_profile.h"


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
  goalAngleValid_ = new bool[numMotors_];
  motorReachable_ = new bool[numMotors_];
  verdictPending_ = new bool[numMotors_];
  lastVerdict_ = new uint8_t[numMotors_];
  goalIssuedMs_ = new unsigned long[numMotors_];
  stallSinceMs_ = new unsigned long[numMotors_];
  for (int i = 0; i < numMotors_; ++i) {
    flipMotor_[i] = DEFAULT_FLIPS[i];
    lastDirectCommandMs_[i] = 0;
    torqueEnabled_[i] = false;
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
    goalAngleValid_[i] = false;
    motorReachable_[i] = false;
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
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.torqueOff(id));

    // RETURN_DELAY_TIME is EEPROM-backed, so only write it when needed. Zero
    // removes the factory 500 us pause before every read response without
    // changing which commands produce responses.
    uint32_t returnDelay = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(RETURN_DELAY_TIME, id));
    if (dxl_.getLastLibErrCode() == DXL_LIB_OK &&
        returnDelay != DYNAMIXEL_RETURN_DELAY) {
      EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(RETURN_DELAY_TIME, id,
                                 DYNAMIXEL_RETURN_DELAY));
    }
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.setOperatingMode(id, OP_CURRENT_BASED_POSITION));
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.torqueOn(id));
    // Ceiling stays at the part maximum so set_current_lim can raise effort at
    // runtime, but the COMMANDED effort starts low. Writing GOAL_CURRENT at
    // the ceiling makes every motor push at max whenever it is resisted, and
    // several digits held flexed at once will brown out the supply.
    //
    // The register write goes direct rather than through setCurrentLimit(),
    // which now also owns the NOMINAL effort: the two are different numbers
    // here (part ceiling vs. working effort) and must not be conflated.
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(CURRENT_LIMIT, id, MOTOR_CURRENT_LIMIT));
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
    float currentPos = EXO_PROFILE_CALL(DXL_READ, dxl_.getPresentPosition(id, UNIT_DEGREE));

    // Zero is a valid angle. Seed the model only from a successful read.
    if (dxl_.getLastLibErrCode() != DXL_LIB_OK || !isfinite(currentPos)) {
      debugPrint("[WARN] Motor " + String(id) + " position read failed, "
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
    goalAngle_[i] = currentPos;
    goalAngleValid_[i] = true;
    motorReachable_[i] = true;
    torqueEnabled_[i] = true;
    jointModels_[i].correct(currentPos, 0, 0, false, millis());

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
    float current_angle = EXO_PROFILE_CALL(DXL_READ, dxl_.getPresentPosition(id, UNIT_DEGREE));
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
  uint8_t method;
  getFastTelemetryRecords(&id, 1, &record, method, 10);
  return record.error == 0;
}

uint8_t NMLHandExo::getFastTelemetryRecords(
    const uint8_t* ids, uint8_t count, FastTelemetryRecord* records,
    uint8_t& methodOut, uint32_t timeoutMs) {
  methodOut = FAST_TELEM_METHOD_FAILED;
  if (!ids || !records) return 0;
  count = min(count, (uint8_t)32);
  serviceJointModels();
  const bool estimated = telemetryEstimated();
  for (uint8_t i = 0; i < count; ++i) {
    records[i] = {};
    records[i].id = ids[i];
    records[i].error = 1;
    records[i].sample_ms = millis();
    records[i].utc_ms = utcClock_.now(records[i].sample_ms);
  }
  if (isAssistBusy()) {
    // Reuse control measurements without competing for the Dynamixel bus or
    // relabelling the current limit / last model prediction as measured effort.
    methodOut = FAST_TELEM_METHOD_CONTROL_CACHE;
    for (uint8_t k = 0; k < count; ++k) {
      const int i = getIndexById(ids[k]);
      if (i < 0) continue;
      const auto& j = assistJoints_[i];
      if (!j.selected || !j.measuredValid || millis()-j.lastSample > 500) continue;
      auto& r = records[k];
      r.position_ticks = (int32_t)roundf(j.angle*PULSE_RESOLUTION/360.0f);
      r.absolute_cdeg = (int32_t)roundf(j.angle*100);
      r.relative_cdeg = (int32_t)roundf((j.angle-zeroOffsets_[i])*(flipMotor_[i] ? -100 : 100));
      r.current_mA = (int16_t)roundf(j.measuredCurrent);
      r.velocity_raw = (int32_t)roundf(j.measuredVelocity/(.229f*6));
      r.sources = 0x15; r.error = 0;
      r.sample_ms = j.lastSample; r.utc_ms = j.measuredUtc;
    }
    return count;
  }
  if (estimated) {
    methodOut = FAST_TELEM_METHOD_MODEL | TELEM_SOURCE_ESTIMATED;
    for (uint8_t i = 0; i < count; ++i) {
      const int index = getIndexById(ids[i]);
      if (index >= 0 && motorReachable_[index]) fillModelRecord(index, records[i]);
    }
    return count; // No telemetry register reads, including idle-side requests.
  }

  DYNAMIXEL::XELInfoSyncRead_t xels[32] = {};
  DYNAMIXEL::InfoSyncReadInst_t info = {};
  uint8_t buffers[32][DYNAMIXEL_PRESENT_BLOCK_LENGTH] = {};
  uint8_t packet[256];
  uint8_t slots[32];
  uint8_t packed = 0;
  for (uint8_t i = 0; i < count; ++i) {
    const int index = getIndexById(ids[i]);
    if (index < 0 || !motorReachable_[index]) continue;
    xels[packed].id = ids[i];
    xels[packed].p_recv_buf = buffers[packed];
    slots[packed++] = i;
  }
  if (!packed) return count;
  info.packet.p_buf = packet;
  info.packet.buf_capacity = sizeof(packet);
  info.addr = DYNAMIXEL_PRESENT_BLOCK_ADDRESS;
  info.addr_length = DYNAMIXEL_PRESENT_BLOCK_LENGTH;
  info.p_xels = xels;
  info.xel_count = packed;
  info.is_info_changed = true;
  while (DXL_SERIAL.available() > 0) DXL_SERIAL.read();
  const uint8_t got = EXO_PROFILE_CALL(DXL_READ, dxl_.syncRead(&info, timeoutMs));
  methodOut = FAST_TELEM_METHOD_SYNC_READ;
  // The library does not supply a reliable per-slot completion bitmap.
  // A partial response must never decode unfilled buffers as physical zeros.
  // Fail the whole sample closed; retry on the next poll, not N*3 fallbacks.
  if (got != packed) return count;
  const uint32_t now = millis();
  for (uint8_t p = 0; p < packed; ++p) {
    if (xels[p].error != 0) continue;
    FastTelemetryRecord& r = records[slots[p]];
    const int index = getIndexById(r.id);
    memcpy(&r.current_mA, buffers[p] + DYNAMIXEL_PRESENT_CURRENT_OFFSET, 2);
    memcpy(&r.velocity_raw, buffers[p] + DYNAMIXEL_PRESENT_VELOCITY_OFFSET, 4);
    memcpy(&r.position_ticks, buffers[p] + DYNAMIXEL_PRESENT_POSITION_OFFSET, 4);
    const float angle = r.position_ticks * 360.0f / PULSE_RESOLUTION;
    if (!isfinite(angle) || fabsf(angle) > 36000.0f) continue;
    r.absolute_cdeg = (int32_t)roundf(angle * 100.0f);
    r.relative_cdeg = (int32_t)roundf((angle - zeroOffsets_[index]) *
                                    (flipMotor_[index] ? -100.0f : 100.0f));
    r.error = 0;
    r.sample_ms = now;
    r.utc_ms = utcClock_.now(now);
    r.sources = 0x15; // position/current/velocity measured
    jointModels_[index].correct(angle, r.velocity_raw * 0.229f * 6.0f,
                                r.current_mA, true, now);
  }
  return count;
}

bool NMLHandExo::motionActive() const {
  if (romCalPhase_ != ROM_CAL_IDLE && romCalPhase_ != ROM_CAL_DONE) return true;
  for (int i = 0; i < numMotors_; ++i) {
    if (motorReachable_[i] && torqueEnabled_[i] &&
        (directCommandActive_[i] || motorMoving_[i])) return true;
  }
  return false;
}

bool NMLHandExo::telemetryEstimated() {
  return telemetryGate_.estimated(millis(), motionActive());
}

void NMLHandExo::serviceJointModels() {
  EXO_PROFILE(MODEL);
  const uint32_t now = millis();
  utcClock_.tick(now);
  telemetryGate_.estimated(now, motionActive());
  for (int i = 0; i < numMotors_; ++i) {
    const bool positional = positionHoldActive_[i] || modeControlsPosition();
    const bool currentKnown = positionHoldActive_[i] || modeControlsCurrent();
    const bool moving = torqueEnabled_[i] &&
        (directCommandActive_[i] || (motorMoving_[i] && goalAngleValid_[i] &&
         (motorControlMode_ != "CURRENT_POSITION" || motorAdmitted_[i])));
    float effort = motorControlMode_ == "CURRENT" && !positionHoldActive_[i] ?
        directCommandDirection_[i] : appliedCurrents_[i];
    if (!torqueEnabled_[i]) effort = 0.0f;
    jointModels_[i].advance(now, jointLimits_[i][0], jointLimits_[i][1],
        zeroOffsets_[i], moving, positional, goalAngle_[i], effort,
        currentKnown, directCommandDirection_[i] * 6.0f,
        XC330_T288_TORQUE_CONSTANT);
  }
}

void NMLHandExo::fillModelRecord(int index, FastTelemetryRecord& r) {
  const JointStateModel& m = jointModels_[index];
  if (!m.valid) return;
  r.error = 0;
  r.sources = 0x22; // position and velocity estimated
  if (m.currentValid) r.sources |= 0x08;
  r.current_mA = m.currentValid ? (int16_t)roundf(m.current) : 0;
  r.velocity_raw = (int32_t)roundf(m.velocity / (6.0f * 0.229f));
  r.position_ticks = (int32_t)roundf(m.angle * PULSE_RESOLUTION / 360.0f);
  r.absolute_cdeg = (int32_t)roundf(m.angle * 100.0f);
  r.relative_cdeg = (int32_t)roundf((m.angle - zeroOffsets_[index]) *
                                 (flipMotor_[index] ? -100.0f : 100.0f));
}

bool NMLHandExo::setJointModel(uint8_t id, const JointModelParams& params) {
  const int index = getIndexById(id);
  if (index < 0 || !params.valid()) return false;
  serviceJointModels();
  jointModels_[index].params = params;
  return true;
}
String NMLHandExo::getJointModel(uint8_t id) {
  const int index = getIndexById(id);
  if (index < 0) return "ERROR: unknown joint ID";
  const JointStateModel& m = jointModels_[index];
  String out = "JOINT_MODEL: id=" + String(id) + " gain=" + String(m.params.gain, 6) +
      " time_constant=" + String(m.params.time_constant, 6) +
      " max_velocity=" + String(m.params.max_velocity, 6) +
      " stiffness=" + String(m.params.stiffness, 6) +
      " moment=" + String(m.params.moment, 6) +
      " trigger=" + (isfinite(m.trigger) ? String(m.trigger, 6) : String("nan"));
  for (uint8_t d = 0; d < 2; ++d) {
    const auto& p = m.pulseResponses[d];
    const String prefix = d == 0 ? " pulse_flex_" : " pulse_extend_";
    out += prefix + "slope=" + String(p.slope, 6) + prefix + "samples=" + String(p.movingSamples) +
           prefix + "no_motion=" + String(p.noMotionSamples) + prefix + "gradients=" + String(p.gradientSamples);
  }
  return out;
}
bool NMLHandExo::setEstimateHoldoff(uint32_t ms) {
  if (ms < 50 || ms > 5000) return false;
  telemetryGate_.holdoff = ms;
  return true;
}
bool NMLHandExo::setJointTrigger(uint8_t id, float torque) {
  const int index = getIndexById(id);
  if (index < 0 || !isfinite(torque) || torque < 0 || torque > 1) return false;
  jointModels_[index].trigger = torque;
  return true;
}
bool NMLHandExo::clearJointTrigger(uint8_t id) {
  const int index = getIndexById(id);
  if (index < 0) return false;
  jointModels_[index].trigger = NAN;
  return true;
}

// Position-only feedback for modes/holds that the current governor does not
// own. The model cannot declare physical arrival. One ID per 50 ms pass.
void NMLHandExo::servicePositionMonitor() {
  const uint32_t now = millis();
  if (now - lastPositionMonitorMs_ < 50) return;
  lastPositionMonitorMs_ = now;
  for (int i = 0; i < numMotors_; ++i) {
    if (!motorMoving_[i] || !torqueEnabled_[i] || !motorReachable_[i]) continue;
    if (motorControlMode_ == "CURRENT_POSITION" && currentGovernorEnabled_) continue;
    if (now - goalIssuedMs_[i] < MOVE_WINDOW_MS) continue;
    int32_t ticks;
    if (readPresentPositionTicks(motorIds_[i], ticks) &&
        fabsf(ticks * 360.0f / PULSE_RESOLUTION - goalAngle_[i]) <= GESTURE_REACH_TOLERANCE_DEG) {
      concludeMove(i, MOVE_VERDICT_REACHED);
    } else if (now - goalIssuedMs_[i] >= MOVE_TIMEOUT_MS) {
      concludeMove(i, MOVE_VERDICT_SHORT);
    }
    return;
  }
}

void NMLHandExo::resetAllZeros() {
  for (int i = 0; i < numMotors_; ++i) {
    uint8_t id = motorIds_[i];
    float current_angle = EXO_PROFILE_CALL(DXL_READ, dxl_.getPresentPosition(id, UNIT_DEGREE));
    zeroOffsets_[i] = current_angle;
    debugPrint("[DEBUG] Zero offset set for motor " + String(id) + ": " + String(current_angle, 2) + " deg");
  }
}
const char* NMLHandExo::getSide() const {
  return HAND_SIDE;
}

String NMLHandExo::getDeviceInfo(bool includeLiveTelemetry) {
  if (telemetryEstimated()) includeLiveTelemetry = false;

    // Need to return a single string with all the information
    String info = "Name: NMLHandExo\n";
    info += "Version: " + String(VERSION) + "\n";
#if EXO_USB_PROTOBUF
    info += "USB Protocol: protobuf-v1\nUSB CDC Count: 2\nUSB Text Port: primary\nUSB Binary Port: secondary\n";
    info += "USB Features: joint_model_write batch_motion assist_v1\n";
#elif EXO_AXON_USB
    info += "USB Protocol: axon-ascii\nUSB CDC Count: 1\n";
#elif defined(DUAL_CDC) && DUAL_CDC
    info += "USB Protocol: ascii-nx\nUSB CDC Count: 2\n";
#else
    info += "USB Protocol: ascii-nx\nUSB CDC Count: 1\n";
#endif
    info += "Assist Protocol: assist-v1\n";
    info += "Model Step Ms: " + String(EXO_MODEL_STEP_MS) + "\n";
    info += "I/O Profile: 1\n";
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
// ==================== Impedance range-of-motion calibration =========================
// ====================================================================================
//
// This is a non-blocking state machine, not a loop. beginRomCalibration() only
// arms it; all of the work happens one bounded step at a time in
// serviceRomCalibration(), which update() calls every pass. That is deliberate:
// the control loop also services the direct-control watchdog and the current
// governor, and a blocking ramp here would starve both. Every tuning value it
// reads is a ROM_CAL_* constant from config.h -- see the block header there.
//
// FLEX/EXTEND use the resolved absolute axis from getGestureSpan, compensating
// the current writer's flip once. Measured wrong-way motion permits one bounded
// sign correction after settling; a second mismatch aborts the campaign.

bool NMLHandExo::beginRomCalibration(uint8_t id, RomCalDirection direction) {
  // A sweep drives current directly, so the fleet must already be in CURRENT
  // mode. Refusing here (rather than silently switching) keeps the mode change
  // -- which turns torque off on every motor -- an explicit host decision.
  // A fresh single-joint sweep replaces any queue a prior batch left behind.
  romCalQueueCount_ = 0;
  romCalQueueCursor_ = 0;
  return romCalArmMotor(id, direction);
}

bool NMLHandExo::romCalArmMotor(uint8_t id, RomCalDirection direction) {
  if (motorControlMode_ != "CURRENT") return false;
  const int index = getIndexById(id);
  if (index < 0) return false;
  if (romCalPhase_ != ROM_CAL_IDLE) return false;   // one sweep at a time
  if (positionHoldActive_[index]) return false;      // don't fight a held joint

  // Reject an unreachable motor. A failed Dynamixel read returns 0.0 -- which is
  // finite -- so an offline motor would otherwise arm, ramp to the ceiling
  // against a phantom "0 deg that never moves," and report a bogus endstop. Two
  // guards: motorReachable_ (set at startup by ping) rejects a never-connected
  // ID, and re-reading the position with an explicit error check rejects one
  // that has since dropped off the bus. Only a read that both succeeds AND is
  // finite is trusted as the return-home reference.
  if (!motorReachable_[index]) return false;
  const float startAngle = EXO_PROFILE_CALL(DXL_READ, dxl_.getPresentPosition(id, UNIT_DEGREE));
  if (dxl_.getLastLibErrCode() != DXL_LIB_OK || !isfinite(startAngle)) {
    return false;
  }

  romCalId_ = id;
  romCalIndex_ = index;
  romCalDir_ = direction;
  romCalStatus_ = ROM_CAL_STATUS_NONE;
  romCalHomeAngle_ = startAngle;
  romCalLastAngle_ = startAngle;
  romCalEndstopAngle_ = startAngle;
  romCalCurrentMa_ = ROM_CAL_START_CURRENT_MA;
  romCalDirLocked_ = false;

  // Use the same resolved absolute flexion axis as gestures. That axis can
  // override a flip flag when home leaves only a short stub on its preferred
  // side. Undo the writer's flip here so the physical current follows the axis.
  const float span = getGestureSpan(id);
  if (!isfinite(span) || fabsf(span) < ROM_CAL_ONSET_DEG) return false;
  romCalSign_ = (direction == ROM_CAL_DIR_FLEX ? 1.0f : -1.0f) *
                (span >= 0 ? 1.0f : -1.0f) * (flipMotor_[index] ? -1.0f : 1.0f);

  const unsigned long now = millis();
  romCalStartMs_ = now;
  romCalLastStepMs_ = now;
  romCalPulseCount_ = 0;
  romCalFitSamples_ = 0;
  romCalStallPulses_ = 0;
  romCalCeilingNoMotionPulses_ = 0;
  romCalSignReversed_ = false;
  romCalStopRequested_ = false;
  romCalReason_ = "none";
  romCalPulseOnMs_ = romCalMaxPulseOnMs_ = 0;
  romCalFeedbackAngle_ = startAngle;
  romCalFeedbackVelocity_ = NAN;
  romCalFeedbackVelocityRaw_ = 0;
  romCalSettleCheck_ = "none";
  romCalSettleWindow_.reset();
  romCalRecoveries_ = romCalFeedbackFailures_ = 0;
  romCalMaxExcursionDeg_ = 0;
  romCalPulseMaxExcursion_ = 0;
  romCalPulseEndDelta_ = NAN;
  romCalPulseStop_ = "none";
  romCalFitReason_ = "no_completed_pulse";
  romCalResponseReason_ = "no_completed_pulse";
  romCalPredictedDeg_ = romCalObservedDeg_ = NAN;
  // Begin each independently armed direction conservatively. Retain its final
  // response for inspection until that direction is calibrated again.
  jointModels_[index].pulseResponses[direction] = PulseResponseModel{};

  // Clear a stale goal before torque-on. A pulse starts only with valid feedback
  // and room inside the stored joint limits.
  if (!writeRomSweepCurrent(0)) return false;
  enableTorque(id, true);
  float angle, velocity;
  if (!romCalReadFeedback(angle, velocity)) {
    enableTorque(id, false);
    return false;
  }
  // The prior direction can still be coasting when the host arms this one.
  // Observe a full zero-current rest before the FIRST pulse as well.
  romCalPulseStartAngle_ = romCalPulsePeakAngle_ = angle;
  romCalPulseValid_ = false;
  romCalRestStartMs_ = millis();
  romCalSettleWindow_.reset();
  romCalPhase_ = ROM_CAL_REST;
  debugPrint("[ROM] Calibration armed: id=" + String(id) + " dir=" +
             String(direction == ROM_CAL_DIR_FLEX ? "flex" : "extend"));
  return true;
}

bool NMLHandExo::isRomCalibrating() const {
  return romCalPhase_ != ROM_CAL_IDLE;
}

bool NMLHandExo::canStartRomCalibration() const {
  // Cheap precheck the parser can gate its OK: ack on WITHOUT touching a motor.
  // Keeping it side-effect-free is what lets the ack be emitted before any
  // ROM_CAL_RESULT line: a batch that armed (and reported skips) first would
  // put a result line on the wire ahead of its own ack, and a host reading the
  // first framed reply would see the result instead of the OK. Per-motor
  // reachability is deliberately NOT checked here -- an offline motor is
  // reported as an aborted result in the async stream, after the ack.
  return motorControlMode_ == "CURRENT" && romCalPhase_ == ROM_CAL_IDLE;
}

bool NMLHandExo::beginRomCalibrationBatch(const uint8_t* ids, uint8_t count,
                                          RomCalDirection direction) {
  if (!ids || count == 0 || count > N_MOTORS) return false;
  if (!canStartRomCalibration()) return false;

  // Load the queue and start it. romCalAdvanceQueue arms the first reachable
  // motor and reports any offline motors ahead of it as aborted results. The
  // CALLER (the parser) must already have emitted its OK: ack before calling
  // this, so those result lines follow the ack on the wire rather than racing
  // ahead of it.
  for (uint8_t k = 0; k < count; ++k) romCalQueue_[k] = ids[k];
  romCalQueueCount_ = count;
  romCalQueueCursor_ = 0;
  romCalQueueDir_ = direction;

  romCalAdvanceQueue();
  return true;
}

void NMLHandExo::romCalAdvanceQueue() {
  // Arm the next queued motor. An unreachable queued motor is reported (as
  // aborted) and skipped so the host still sees one result per requested motor
  // and one offline joint does not abort the rest of a gesture. When the queue
  // is exhausted the campaign is over and the machine is left idle.
  while (romCalQueueCursor_ < romCalQueueCount_) {
    const uint8_t id = romCalQueue_[romCalQueueCursor_++];
    if (romCalArmMotor(id, romCalQueueDir_)) return;
    // Report the skip. romCalReport() emits the line but, unlike romCalFinish(),
    // does NOT itself advance the queue -- this loop does, so there is no mutual
    // recursion between the two.
    romCalId_ = id;
    romCalDir_ = romCalQueueDir_;
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalHomeAngle_ = NAN;
    romCalCurrentMa_ = 0.0f;
    romCalPulseCount_ = romCalFitSamples_ = 0;
    romCalIndex_ = getIndexById(id);
    romCalReason_ = "unreachable";
    romCalFeedbackAngle_ = NAN;
    romCalPulseOnMs_ = romCalMaxPulseOnMs_ = romCalRecoveries_ = 0;
    romCalMaxExcursionDeg_ = 0;
    romCalPredictedDeg_ = romCalObservedDeg_ = NAN;
    romCalFitReason_ = romCalResponseReason_ = "unreachable";
    romCalPulseStop_ = "none";
    romCalReport();
  }
  romCalQueueCount_ = 0;
  romCalQueueCursor_ = 0;
  romCalPhase_ = ROM_CAL_IDLE;
}

void NMLHandExo::romCalBeginReturnHome() {
  // A fault must never turn into another move. Automatic return is disabled
  // by default even after success; the operator may choose Home explicitly.
  if (!ROM_CAL_AUTO_RETURN_HOME || romCalStatus_ == ROM_CAL_STATUS_ABORTED ||
      romCalStatus_ == ROM_CAL_STATUS_TIMEOUT) {
    romCalFinish();
    return;
  }
  // Stop pushing, then hold the joint back to its start angle with a modest,
  // bounded current. holdRelativePosition switches only this ID to
  // CURRENT_POSITION and clamps to the calibrated window, so this cannot drive
  // the joint past its limits.
  stopDirectControl(romCalId_);
  const float homeRelative = romCalHomeAngle_ - zeroOffsets_[romCalIndex_];
  const float relative = flipMotor_[romCalIndex_] ? -homeRelative : homeRelative;
  holdRelativePosition(romCalId_, relative, ROM_CAL_RETURN_CURRENT_MA);
  romCalReturnStartMs_ = millis();
  romCalPhase_ = ROM_CAL_RETURN;
}

void NMLHandExo::romCalFinish() {
  if (romCalStatus_ == ROM_CAL_STATUS_ABORTED || romCalStatus_ == ROM_CAL_STATUS_TIMEOUT) {
    // A stop or fault ends the campaign; it must not arm the next queued ID.
    romCalQueueCount_ = romCalQueueCursor_ = 0;
  }
  // Release the return hold (restores the global CURRENT mode for this ID with
  // torque off) and report the outcome, then advance to the next queued motor
  // (for a gesture/batch sweep) or go idle (single-joint sweep).
  if (isPositionHoldActive(romCalId_)) releasePositionHold(romCalId_);
  stopDirectControl(romCalId_);
  enableTorque(romCalId_, false);
  romCalReport();
  // Return to IDLE BEFORE advancing: romCalArmMotor refuses to arm unless the
  // machine is idle, so the next queued motor could never arm while this one's
  // phase still read RETURN. romCalAdvanceQueue then arms the next motor or, if
  // the queue is empty/spent (a single-joint sweep, or the campaign's end),
  // leaves it idle.
  romCalPhase_ = ROM_CAL_IDLE;
  romCalAdvanceQueue();
}

void NMLHandExo::romCalReport() {
  // Emit one ROM_CAL_RESULT line for the current target. Pure reporting: no
  // motor I/O and no queue advance, so both romCalFinish() and the skip path
  // in romCalAdvanceQueue() can call it without re-entering each other.
  const char* dirStr = (romCalDir_ == ROM_CAL_DIR_FLEX) ? "flex" : "extend";
  const char* statusStr = "aborted";
  switch (romCalStatus_) {
    case ROM_CAL_STATUS_OK:      statusStr = "ok";      break;
    case ROM_CAL_STATUS_CEILING: statusStr = "ceiling"; break;
    case ROM_CAL_STATUS_TIMEOUT: statusStr = "timeout"; break;
    case ROM_CAL_STATUS_LIMIT:   statusStr = "limit";   break;
    default:                     statusStr = "aborted"; break;
  }

  String line = "ROM_CAL_RESULT: id=" + String(romCalId_) +
                " dir=" + String(dirStr) +
                " home=" + String(romCalHomeAngle_, 2);
  if (romCalStatus_ == ROM_CAL_STATUS_OK) {
    line += " endstop=" + String(romCalEndstopAngle_, 2);
  } else {
    line += " endstop=nan";
  }
  line += " current_mA=" + String(romCalCurrentMa_, 0) +
          " status=" + String(statusStr) +
          " pulses=" + String(romCalPulseCount_) +
          " fit_samples=" + String(romCalFitSamples_) +
          " model_gain=" + (romCalIndex_ >= 0 ?
              String(jointModels_[romCalIndex_].params.gain, 6) : String("nan")) +
          " reason=" + String(romCalReason_) +
          " pulse_on_ms=" + String(romCalPulseOnMs_) +
          " max_pulse_on_ms=" + String(romCalMaxPulseOnMs_) +
          " pulse_stop=" + String(romCalPulseStop_) +
          " max_excursion_deg=" + String(romCalMaxExcursionDeg_, 2) +
          " fit_reason=" + String(romCalFitReason_) +
          " predicted_deg=" + String(romCalPredictedDeg_, 3) +
          " observed_deg=" + String(romCalObservedDeg_, 3) +
          " response_reason=" + String(romCalResponseReason_) +
          " recoveries=" + String(romCalRecoveries_) +
          " angle=" + String(romCalFeedbackAngle_, 2) +
          " velocity_deg_s=" + String(romCalFeedbackVelocity_, 3) +
          " velocity_raw=" + String(romCalFeedbackVelocityRaw_) +
          " settle_check=" + String(romCalSettleCheck_) +
          " settle_span_deg=" + String(romCalSettleWindow_.span(), 3) +
          " settle_quiet_ms=" + String(romCalSettleWindow_.quietMs()) +
          " net_travel_deg=" + String(romCalFeedbackAngle_ - romCalHomeAngle_, 3);
  line += " ceiling_no_motion_pulses=" + String(romCalCeilingNoMotionPulses_);
  if (romCalIndex_ >= 0) {
    const auto& response = jointModels_[romCalIndex_].pulseResponses[romCalDir_];
    line += " response_samples=" + String(response.movingSamples) +
            " no_motion_samples=" + String(response.noMotionSamples);
    line += " limit_min=" + String(jointLimits_[romCalIndex_][0], 2) +
            " limit_max=" + String(jointLimits_[romCalIndex_][1], 2);
  }
  // Append the command delimiter so the line self-frames. This result is
  // emitted asynchronously with no following command, so under the dual-CDC
  // host transport it must terminate with the delimiter to be published as its
  // own reply frame -- an unterminated line is held pending until the NEXT
  // delimited reply, which for a standalone sweep result never comes. (The
  // class emits via telemetryPrintln rather than the parser's commandPrint to
  // avoid a dependency on utils.h; the delimiter is what the host frames on.)
  line += COMMAND_DELIMITER;
  telemetryPrintln(line);

  // Clear only the per-sweep status. The PHASE is owned by the caller
  // (romCalAdvanceQueue arms the next motor or idles), so it is not touched
  // here -- that is what keeps report and advance from fighting over it.
  romCalStatus_ = ROM_CAL_STATUS_NONE;
}

void NMLHandExo::cancelRomCalibration() {
  if (romCalPhase_ == ROM_CAL_IDLE) return;
  // Cancel the WHOLE campaign: drop any remaining queued motors so the sweep
  // does not simply advance to the next joint after this one returns home.
  romCalQueueCount_ = 0;
  romCalQueueCursor_ = 0;
  romCalStatus_ = ROM_CAL_STATUS_ABORTED;
  romCalReason_ = "cancelled";
  // Stop and report through the normal finish path. An aborted status never
  // initiates a return, including builds that opt into return after success.
  stopDirectControl(romCalId_);
  romCalBeginReturnHome();
}

void NMLHandExo::romCalReportPulse() {
  const auto& p = jointModels_[romCalIndex_].pulseResponses[romCalDir_];
  String line = "ROM_CAL_PULSE: id=" + String(romCalId_) +
      " dir=" + String(romCalDir_ == ROM_CAL_DIR_FLEX ? "flex" : "extend") +
      " pulse=" + String(romCalPulseCount_) + " current_mA=" + String(romCalCurrentMa_, 0) +
      " shape=" + String(ROM_CAL_SHAPE_STEP_MS == 5 ? "gaussian" : "triangle") +
      " shape_step_ms=" + String(ROM_CAL_SHAPE_STEP_MS) +
      " exposure_mA_ms=" + String(romCalExposure_.impulse * 1000.0f, 2) +
      " on_ms=" + String(romCalPulseOnMs_) + " predicted_deg=" + String(romCalPredictedDeg_, 3) +
      " observed_deg=" + String(romCalObservedDeg_, 3) + " target_deg=" + String(ROM_CAL_TARGET_TRAVEL_DEG, 3) +
      " slope=" + String(p.slope, 6) + " response_samples=" + String(p.movingSamples) +
      " no_motion_samples=" + String(p.noMotionSamples) + " gradients=" + String(p.gradientSamples) +
      " response_reason=" + String(romCalResponseReason_) + " fit_reason=" + String(romCalFitReason_) +
      " pulse_stop=" + String(romCalPulseStop_) +
      " peak_excursion_deg=" + String(romCalPulseMaxExcursion_, 3) +
      " pulse_end_delta_deg=" + String(romCalPulseEndDelta_, 3) +
      " ceiling_no_motion_pulses=" + String(romCalCeilingNoMotionPulses_) + COMMAND_DELIMITER;
  telemetryPrintln(line);
}

bool NMLHandExo::romCalReadPosition(float& angle) {
  const int32_t ticks = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(
      (uint8_t)PRESENT_POSITION, romCalId_, (uint32_t)ROM_CAL_IO_TIMEOUT_MS));
  if (dxl_.getLastLibErrCode() != DXL_LIB_OK) return false;
  angle = ticks * (360.0f / PULSE_RESOLUTION);
  if (!isfinite(angle) || fabsf(angle) > 36000.0f) return false;
  romCalFeedbackAngle_ = angle;
  if (romCalPhase_ == ROM_CAL_REST) {
    romCalSettleWindow_.observe(angle, millis(), ROM_CAL_SETTLED_POSITION_DEG, ROM_CAL_SETTLED_MAX_GAP_MS);
  }
  return true;
}

bool NMLHandExo::romCalReadFeedback(float& angle, float& velocity) {
  if (!romCalReadPosition(angle)) return false;
  const int32_t raw = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(
      (uint8_t)PRESENT_VELOCITY, romCalId_, (uint32_t)ROM_CAL_IO_TIMEOUT_MS));
  velocity = raw * (0.229f * 6.0f);
  if (dxl_.getLastLibErrCode() != DXL_LIB_OK || !isfinite(velocity)) velocity = NAN;
  romCalFeedbackVelocityRaw_ = raw;
  romCalFeedbackVelocity_ = velocity;
  return true; // velocity failure rejects fitting, never fabricates a speed
}

void NMLHandExo::romCalRecoverFeedback() {
  // A missed read never leaves current applied while we retry. Discard the
  // incomplete pulse, rest, and require fresh feedback before retrying at the
  // SAME amplitude. No gain/stall update is based on the failed sample.
  if (!writeRomSweepCurrent(0)) {
    romCalReason_ = "stop_write";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return;
  }
  if (++romCalFeedbackFailures_ > ROM_CAL_MAX_FEEDBACK_RECOVERIES) {
    romCalReason_ = "position_read";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return;
  }
  ++romCalRecoveries_;
  romCalPulseValid_ = false;
  romCalFitReason_ = "position_unavailable";
  romCalRestStartMs_ = millis();
  romCalSettleWindow_.reset();
  romCalPhase_ = ROM_CAL_REST;
}

bool NMLHandExo::romCalStartPulse(float angle, float velocity) {
  if (romCalPhase_ != ROM_CAL_IDLE && millis() - romCalStartMs_ >= ROM_CAL_TIMEOUT_MS) {
    romCalReason_ = "sweep_timeout";
    romCalStatus_ = ROM_CAL_STATUS_TIMEOUT;
    romCalFinish();
    return false;
  }
  // Diagnostic output and fitting occur at zero current. Refresh afterwards
  // so a blocked text endpoint cannot make the next pulse use stale feedback.
  if (!romCalReadFeedback(angle, velocity)) {
    romCalReason_ = "position_read";
    romCalFitReason_ = "position_unavailable";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return false;
  }
  if (romCalPhase_ == ROM_CAL_REST && !romCalCheckTravel(angle)) return false;
  const float physicalSign = romCalSign_ * (flipMotor_[romCalIndex_] ? -1.0f : 1.0f);
  const float lo = jointLimits_[romCalIndex_][0], hi = jointLimits_[romCalIndex_][1];
  if (!isfinite(angle) || !isfinite(lo) || !isfinite(hi) || lo > hi ||
      angle < lo || angle > hi ||
      (physicalSign < 0 && angle <= lo + DIRECT_LIMIT_MARGIN_DEG) ||
      (physicalSign > 0 && angle >= hi - DIRECT_LIMIT_MARGIN_DEG)) {
    romCalStatus_ = angle < lo || angle > hi ? ROM_CAL_STATUS_ABORTED : ROM_CAL_STATUS_LIMIT;
    romCalReason_ = angle < lo || angle > hi ? "outside_limits" : "stored_limit";
    // A rejected start has not driven a pulse. Do not turn it into an implicit
    // return move (whose target would be clamped if the start is out of range).
    if (romCalPhase_ == ROM_CAL_IDLE) romCalFinish();
    else romCalBeginReturnHome();
    return false;
  }
  romCalSettleCheck_ = "pulse_start";
  const bool quietPosition = romCalPhase_ != ROM_CAL_REST ||
      romCalSettleWindow_.quietMs() >= ROM_CAL_SETTLED_WINDOW_MS;
  if (!isfinite(velocity) || fabsf(velocity) > ROM_CAL_SETTLED_SPEED_DEG_S || !quietPosition) {
    if (isfinite(velocity) && romCalPhase_ == ROM_CAL_REST &&
        millis()-romCalRestStartMs_ < ROM_CAL_SETTLE_TIMEOUT_MS) {
      // A completed pulse has already been counted/fitted. Wait at zero without
      // processing that observation again or increasing amplitude again.
      romCalPulseValid_ = false;
      if (fabsf(velocity) > ROM_CAL_SETTLED_SPEED_DEG_S) romCalSettleWindow_.reset();
      return false;
    }
    romCalReason_ = isfinite(velocity) ? "not_settled" : "velocity_read";
    romCalFitReason_ = isfinite(velocity) ? "not_settled" : "velocity_unavailable";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return false;
  }
  romCalPulseStartAngle_ = romCalPulsePeakAngle_ = angle;
  romCalPulseStartVelocity_ = velocity;
  romCalPulseMaxExcursion_ = 0;
  romCalPulseEndDelta_ = NAN;
  romCalPredictedDeg_ = jointModels_[romCalIndex_].pulseResponses[romCalDir_].predict(romCalCurrentMa_);
  // This control feedback seeds the next estimate even though fleet telemetry
  // remains estimated. Zero is the off-phase command, not measured consumption.
  jointModels_[romCalIndex_].correct(angle, velocity, 0, true, millis());
  if (!writeRomSweepCurrent(romCalSign_ * romCalCurrentMa_ * romPulseShape(0, ROM_CAL_SHAPE_STEP_MS))) {
    romCalReason_ = "current_write";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return false;
  }
  romCalPulseStartMs_ = millis();
  romCalSettleWindow_.reset();
  romCalShapeSlot_ = 0;
  romCalExpectedCurrent_ = directCommandDirection_[romCalIndex_];
  romCalExposure_.reset(romCalPulseStartMs_, jointModels_[romCalIndex_].params.time_constant);
  romCalExposure_.command(romCalExpectedCurrent_, romCalPulseStartMs_);
  romCalLastStepMs_ = romCalPulseStartMs_;
  romCalPhase_ = ROM_CAL_RAMP;
  romCalPulseValid_ = true;
  romCalPulseLimited_ = false;
  romCalPulseStop_ = "powered";
  return true;
}

bool NMLHandExo::romCalCheckTravel(float angle) {
  const float excursion = fabsf(angle - romCalPulseStartAngle_);
  romCalMaxExcursionDeg_ = fmaxf(romCalMaxExcursionDeg_, excursion);
  romCalPulseMaxExcursion_ = fmaxf(romCalPulseMaxExcursion_, excursion);
  const float lo = jointLimits_[romCalIndex_][0], hi = jointLimits_[romCalIndex_][1];
  if (angle < lo || angle > hi || excursion >= ROM_CAL_MAX_EXCURSION_DEG) {
    romCalReason_ = angle < lo || angle > hi ? "outside_limits" : "excursion_limit";
    romCalFitReason_ = "excessive_travel";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return false;
  }
  const float sign = romCalSign_ * (flipMotor_[romCalIndex_] ? -1.0f : 1.0f);
  if ((sign < 0 && angle <= lo + DIRECT_LIMIT_MARGIN_DEG) ||
      (sign > 0 && angle >= hi - DIRECT_LIMIT_MARGIN_DEG)) {
    romCalStatus_ = ROM_CAL_STATUS_LIMIT;
    romCalReason_ = "stored_limit";
    romCalFitReason_ = "stored_limit";
    romCalBeginReturnHome();
    return false;
  }
  return true;
}

void NMLHandExo::romCalEndPulse(const char* reason) {
  // Stop current before any read or reporting, including early travel cutoffs.
  romCalPulseStop_ = reason;
  if (!writeRomSweepCurrent(0)) {
    romCalReason_ = "stop_write";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return;
  }
  romCalPulseOnMs_ = millis() - romCalPulseStartMs_;
  if (romCalPulseOnMs_ > ROM_CAL_PULSE_ON_MS + ROM_CAL_STEP_INTERVAL_MS) {
    romCalReason_ = "pulse_overrun";
    romCalFitReason_ = "pulse_overrun";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return;
  }
  if (strcmp(reason, "time") == 0 && romCalShapeSlot_ + 1 < ROM_CAL_PULSE_ON_MS / ROM_CAL_SHAPE_STEP_MS) {
    romCalReason_ = romCalFitReason_ = "shape_overrun";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return;
  }
  romCalRestStartMs_ = millis();
  romCalSettleWindow_.reset();
  romCalPhase_ = ROM_CAL_REST;
  float angle, velocity;
  if (!romCalReadFeedback(angle, velocity)) {
    romCalRecoverFeedback();
    return;
  }
  romCalPulsePeakAngle_ = angle;
  romCalPulseEndDelta_ = angle - romCalPulseStartAngle_;
  if (!romCalCheckTravel(angle)) return;
  jointModels_[romCalIndex_].correct(angle, velocity, 0, true, millis());
}

void NMLHandExo::romCalCompletePulse(float angle, float velocity) {
  romCalFeedbackFailures_ = 0;
  ++romCalPulseCount_;
  const float span = getGestureSpan(romCalId_);
  const float wantSign = (romCalDir_ == ROM_CAL_DIR_FLEX ? 1.0f : -1.0f) *
                         (span >= 0 ? 1.0f : -1.0f);
  romCalObservedDeg_ = (angle - romCalPulseStartAngle_) * wantSign;
  // Short pulses may travel mostly during coast. Count the measured response
  // of a powered pulse, not a timer tick at zero command, as progress.
  if ((angle - romCalPulsePeakAngle_) * wantSign > 0) romCalPulsePeakAngle_ = angle;
  const float delta = romCalPulsePeakAngle_ - romCalPulseStartAngle_;
  jointModels_[romCalIndex_].correct(angle, velocity, 0, true, millis());
  if (romCalObservedDeg_ <= -ROM_CAL_ONSET_DEG) {
    romCalFitReason_ = "wrong_direction";
    // One correction only, always after zero current and the rest interval.
    // Never learn from the wrong-way probe or oscillate the sign indefinitely.
    if (romCalSignReversed_ || romCalDirLocked_) {
      romCalReason_ = "wrong_direction";
      romCalStatus_ = ROM_CAL_STATUS_ABORTED;
      romCalFinish();
      return;
    }
    romCalResponseReason_ = "wrong_direction";
    romCalReportPulse();
    romCalSignReversed_ = true;
    romCalSign_ = -romCalSign_;
    romCalCurrentMa_ = ROM_CAL_START_CURRENT_MA;
    romCalStartPulse(angle, velocity);
    return;
  }
  if (delta * wantSign >= ROM_CAL_ONSET_DEG) romCalDirLocked_ = true;
  const float progress = (romCalPulsePeakAngle_ - romCalEndstopAngle_) * wantSign;
  if (progress >= ROM_CAL_ONSET_DEG) {
    romCalEndstopAngle_ = romCalPulsePeakAngle_;
    romCalStallPulses_ = 0;
    // Fit from the complete powered + unpowered response, compensating the
    // measured starting velocity. Stalls, recoil and saturation are excluded.
    JointModelParams& params = jointModels_[romCalIndex_].params;
    const float gain = romShapedPulseGain(params, angle - romCalPulseStartAngle_,
        romCalPulseStartVelocity_, velocity, romCalExposure_,
        (millis() - romCalRestStartMs_) * 0.001f, ROM_CAL_ONSET_DEG, &romCalFitReason_);
    if (isfinite(gain) && ROM_CAL_UPDATE_MODEL_GAIN) {
      const float bounded = JointStateModel::bound(gain,
          params.gain * (1.0f - ROM_CAL_GAIN_CHANGE_FRAC),
          params.gain * (1.0f + ROM_CAL_GAIN_CHANGE_FRAC));
      params.gain += ROM_CAL_GAIN_BLEND * (bounded - params.gain);
      ++romCalFitSamples_;
    }
  } else {
    romCalFitReason_ = "insufficient_progress";
  }
  // Keep the actual furthest measured endpoint even when its final approach
  // is smaller than the motion/noise threshold used for stall evidence.
  if (progress > 0) romCalEndstopAngle_ = romCalPulsePeakAngle_;
  auto& response = jointModels_[romCalIndex_].pulseResponses[romCalDir_];
  // The displacement map is specific to the nominal 40-ms shaped input. Do not learn
  // a false amplitude slope from an early-cutoff pulse of a different duration.
  if (romCalPulseLimited_ || romCalPulseOnMs_ < ROM_CAL_PULSE_ON_MS) {
    romCalResponseReason_ = "pulse_limited";
  } else if (response.observe(romCalCurrentMa_, romCalObservedDeg_, ROM_CAL_ONSET_DEG)) {
    romCalResponseReason_ = romCalObservedDeg_ < ROM_CAL_ONSET_DEG ? "no_motion" :
                           response.gradientSamples ? "local_gradient" : "ratio_seed";
  } else {
    romCalResponseReason_ = "invalid_response";
  }
  // A small pulse can fail to overcome friction away from an endstop. Require
  // repeated full-ceiling pulses at a stable endpoint, not merely <0.3 deg per
  // pulse or a few accumulated encoder counts since starting the campaign.
  const float low = fminf(angle, fminf(romCalPulseStartAngle_, romCalPulsePeakAngle_));
  const float high = fmaxf(angle, fmaxf(romCalPulseStartAngle_, romCalPulsePeakAngle_));
  if (romCalCurrentMa_ < ROM_CAL_MAX_CURRENT_MA || romCalPulseLimited_ ||
      high - low > ROM_CAL_SETTLED_POSITION_DEG) {
    romCalStallPulses_ = 0;
  } else {
    if (!romCalStallPulses_ ||
        fmaxf(romCalStallHigh_, high) - fminf(romCalStallLow_, low) > ROM_CAL_SETTLED_POSITION_DEG) {
      romCalStallLow_ = low; romCalStallHigh_ = high; romCalStallPulses_ = 0;
    }
    romCalStallLow_ = fminf(romCalStallLow_, low);
    romCalStallHigh_ = fmaxf(romCalStallHigh_, high);
    if (romCalStallPulses_ < 255) ++romCalStallPulses_;
  }
  if (!romCalPulseLimited_ && romCalPulseOnMs_ >= ROM_CAL_PULSE_ON_MS &&
      romCalCurrentMa_ >= ROM_CAL_MAX_CURRENT_MA &&
      fabsf(romCalObservedDeg_) < ROM_CAL_ONSET_DEG) {
    if (romCalCeilingNoMotionPulses_ < 255) ++romCalCeilingNoMotionPulses_;
  } else {
    romCalCeilingNoMotionPulses_ = 0;
  }
  romCalReportPulse(); // zero current throughout this diagnostic write
  if (romCalCurrentMa_ <= ROM_CAL_MIN_CURRENT_MA &&
      (romCalPulseLimited_ || romCalObservedDeg_ >= ROM_CAL_PULSE_TRAVEL_DEG)) {
    romCalReason_ = "pulse_too_large_at_min_current";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return;
  }
  const bool travelled = (romCalEndstopAngle_ - romCalHomeAngle_) * wantSign >= ROM_CAL_PROGRESS_DEG;
  const bool movingEvidence = response.movingSamples >= ROM_CAL_MIN_MOVING_PULSES;
  if (romCalStallPulses_ >= ROM_CAL_STALL_PULSES && travelled && movingEvidence) {
    romCalStatus_ = ROM_CAL_STATUS_OK;
    romCalReason_ = "endstop";
    romCalBeginReturnHome();
    return;
  }
  if (romCalCurrentMa_ >= ROM_CAL_MAX_CURRENT_MA && (!travelled || !movingEvidence) &&
      romCalStallPulses_ >= ROM_CAL_STALL_PULSES) {
    romCalStatus_ = ROM_CAL_STATUS_CEILING;
    romCalReason_ = "current_ceiling";
    romCalBeginReturnHome();
    return;
  }
  if (romCalCeilingNoMotionPulses_ >= ROM_CAL_CEILING_NO_MOTION_PULSES) {
    romCalStatus_ = ROM_CAL_STATUS_CEILING;
    romCalReason_ = "response_saturated";
    romCalBeginReturnHome();
    return;
  }
  if (romCalPulseLimited_ || fabsf(angle - romCalPulseStartAngle_) >= ROM_CAL_PULSE_TRAVEL_DEG) {
    romCalCurrentMa_ = fmaxf(ROM_CAL_MIN_CURRENT_MA, romCalCurrentMa_ - ROM_CAL_CURRENT_STEP_MA);
  } else {
    romCalCurrentMa_ = roundf(response.nextCurrent(romCalCurrentMa_, romCalObservedDeg_,
        ROM_CAL_TARGET_TRAVEL_DEG, ROM_CAL_ONSET_DEG, ROM_CAL_CURRENT_STEP_MA,
        ROM_CAL_MIN_CURRENT_MA, ROM_CAL_MAX_CURRENT_MA));
  }
  romCalLastAngle_ = angle;
  romCalStartPulse(angle, velocity);
}

void NMLHandExo::serviceRomCalibration() {
  EXO_PROFILE(CONTROL);
  if (romCalPhase_ == ROM_CAL_IDLE) return;
  const unsigned long now = millis();
  if (romCalPhase_ == ROM_CAL_RETURN) {
    float angle, velocity;
    if (now - romCalLastStepMs_ < ROM_CAL_STEP_INTERVAL_MS) return;
    romCalLastStepMs_ = now;
    const bool settled = romCalReadFeedback(angle, velocity) &&
        fabsf(angle - romCalHomeAngle_) <= GESTURE_REACH_TOLERANCE_DEG;
    if (settled || now - romCalReturnStartMs_ >= ROM_CAL_RETURN_TIMEOUT_MS) romCalFinish();
    return;
  }
  // An explicit STOP, disable, mode change or watchdog stop must not be undone.
  const float expectedCurrent = romCalExpectedCurrent_;
  if (motorControlMode_ != "CURRENT" || romCalStopRequested_ || !torqueEnabled_[romCalIndex_] ||
      (romCalPhase_ == ROM_CAL_RAMP && ((expectedCurrent != 0 && !directCommandActive_[romCalIndex_]) ||
          fabsf(directCommandDirection_[romCalIndex_] - expectedCurrent) > 0.5f)) ||
      (romCalPhase_ == ROM_CAL_REST && directCommandActive_[romCalIndex_])) {
    if (motorControlMode_ != "CURRENT") romCalReason_ = "mode_changed";
    else if (!torqueEnabled_[romCalIndex_]) romCalReason_ = "disabled";
    else if (!romCalStopRequested_) romCalReason_ = "command_changed";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return;
  }
  if (now - romCalStartMs_ >= ROM_CAL_TIMEOUT_MS) {
    romCalStatus_ = ROM_CAL_STATUS_TIMEOUT;
    romCalReason_ = "sweep_timeout";
    romCalBeginReturnHome();
    return;
  }
  if (romCalPhase_ == ROM_CAL_RAMP && now - romCalPulseStartMs_ >= ROM_CAL_PULSE_ON_MS) {
    romCalEndPulse("time");
    return;
  }
  // Monitor real travel during BOTH power and coast. Zero current cannot brake
  // inertia; a large excursion must end the campaign instead of re-arming.
  if (now - romCalLastStepMs_ < ROM_CAL_STEP_INTERVAL_MS) return;
  romCalLastStepMs_ = now;
  float angle;
  if (!romCalReadPosition(angle)) {
    romCalRecoverFeedback();
    return;
  }
  if (!romCalCheckTravel(angle)) return;
  if (romCalPhase_ == ROM_CAL_REST) {
    if (millis() - romCalRestStartMs_ < ROM_CAL_PULSE_OFF_MS) return;
    float velocity;
    if (!romCalReadFeedback(angle, velocity)) {
      romCalRecoverFeedback();
      return;
    }
    if (!romCalCheckTravel(angle)) return;
    romCalSettleCheck_ = "rest";
    if (!isfinite(velocity) || fabsf(velocity) > ROM_CAL_SETTLED_SPEED_DEG_S ||
        romCalSettleWindow_.quietMs() < ROM_CAL_SETTLED_WINDOW_MS) {
      if (isfinite(velocity) && millis() - romCalRestStartMs_ < ROM_CAL_SETTLE_TIMEOUT_MS) {
        if (fabsf(velocity) > ROM_CAL_SETTLED_SPEED_DEG_S) romCalSettleWindow_.reset();
        return;
      }
      romCalReason_ = isfinite(velocity) ? "not_settled" : "velocity_read";
      romCalFitReason_ = isfinite(velocity) ? "not_settled" : "velocity_unavailable";
      romCalStatus_ = ROM_CAL_STATUS_ABORTED;
      romCalFinish();
      return;
    }
    if (!romCalPulseValid_) {
      romCalStartPulse(angle, velocity);
      return;
    }
    romCalCompletePulse(angle, velocity);
    return;
  }
  const float travel = fabsf(angle - romCalPulseStartAngle_);
  const unsigned long elapsed = millis() - romCalPulseStartMs_;
  const bool fast = elapsed && travel * 1000.0f / elapsed >= ROM_CAL_PULSE_SPEED_DEG_S;
  if (travel >= ROM_CAL_PULSE_TRAVEL_DEG || fast) {
    romCalPulseLimited_ = true;
    romCalEndPulse(fast ? "speed" : "travel");
    return;
  }
  // A read can cross the pulse deadline. Never write a late nonzero sample.
  if (elapsed >= ROM_CAL_PULSE_ON_MS) { romCalEndPulse("time"); return; }
  const uint8_t slot = elapsed / ROM_CAL_SHAPE_STEP_MS;
  if (slot > romCalShapeSlot_ + 1) {
    romCalReason_ = romCalFitReason_ = "shape_overrun";
    romCalStatus_ = ROM_CAL_STATUS_ABORTED;
    romCalFinish();
    return;
  }
  if (slot != romCalShapeSlot_) {
    if (!writeRomSweepCurrent(romCalSign_ * romCalCurrentMa_ * romPulseShape(slot, ROM_CAL_SHAPE_STEP_MS))) {
      romCalReason_ = "current_write";
      romCalStatus_ = ROM_CAL_STATUS_ABORTED;
      romCalFinish();
      return;
    }
    romCalShapeSlot_ = slot;
    romCalExpectedCurrent_ = directCommandDirection_[romCalIndex_];
    // The write itself can block, too. Cut off before yielding to other work.
    if (millis() - romCalPulseStartMs_ >= ROM_CAL_PULSE_ON_MS) romCalEndPulse("time");
  }

}

// ====================================================================================
// ================================= Mode commands ====================================
// ====================================================================================
void NMLHandExo::update() {
  EXO_PROFILE(CONTROL);
  if (isAssistBusy()) { serviceJointModels(); serviceAssist(); return; }
    serviceJointModels();
    servicePositionMonitor();

    serviceDirectControlSafety();

    // Read-only Phase-1 instrumentation. This is disabled by default and
    // self-pauses outside VELOCITY mode, so it cannot compete with the
    // current-position governor or influence a motor command.
    serviceShadowTelemetry();

    // Flush any allocation batched up by this pass's commands before the
    // governor gets a say, so the feed-forward clamp reaches the motors within
    // the same loop iteration that issued their goals.
    if (allocationDirty_) refreshCurrentAllocation();
    serviceCurrentGovernor();
    reportMoveVerdicts();

    // Impedance ROM calibration is a non-blocking state machine that drives one
    // joint under current control. It runs alongside the safety services above
    // (which is why they precede it) and is independent of the button-driven
    // GESTURE_CALIBRATION handled below.
    serviceRomCalibration();

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

  float abs_angle = EXO_PROFILE_CALL(DXL_READ, dxl_.getPresentPosition(id, UNIT_DEGREE));
  float rel_angle = abs_angle - zeroOffsets_[index];

  // Flip if necessary
  if (flipMotor_[index]) {
    rel_angle *= -1;
  }

  return rel_angle;
}
void NMLHandExo::setRelativeAngle(uint8_t id, float relativeAngle) {
  if (isAssistBusy()) return;
  if (!isfinite(relativeAngle)) return;
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
  goalAngleValid_[index] = EXO_PROFILE_CALL(DXL_WRITE, dxl_.setGoalPosition(id, abs_goal, UNIT_DEGREE));

  char buffer[128];
  snprintf(buffer, sizeof(buffer), "Motor %d set to relative angle %.2f deg (absolute: %.2f deg)", id, relativeAngle, abs_goal);
  debugPrint(buffer);
}
float NMLHandExo::getAbsoluteAngle(uint8_t id) {
  if (getIndexById(id) < 0) return NAN;
  int32_t ticks;
  return readPresentPositionTicks(id, ticks) ? ticks * 360.0f / PULSE_RESOLUTION : NAN;
}
void NMLHandExo::setAbsoluteAngle(uint8_t id, float absoluteAngle) {
  if (isAssistBusy()) return;
  if (!isfinite(absoluteAngle)) return;
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
  goalAngleValid_[index] = EXO_PROFILE_CALL(DXL_WRITE, dxl_.setGoalPosition(id, clamped, UNIT_DEGREE));
  debugPrint("[NMLHandExo] Setting motor " + String(id) + " to absolute angle " + String(clamped, 2));
}

bool NMLHandExo::setAbsoluteAnglesSync(const MotorAngleTarget* targets,
                                       uint8_t count, uint8_t* writtenOut,
                                       uint8_t* skippedOut,
                                       int16_t* libErrorOut) {
  if (isAssistBusy()) return false;
  if (writtenOut) *writtenOut = 0;
  if (skippedOut) *skippedOut = 0;
  if (libErrorOut) *libErrorOut = DXL_LIB_OK;
  if (!targets || count == 0 || count > numMotors_ || count > N_MOTORS) {
    if (libErrorOut) *libErrorOut = DXL_LIB_ERROR_LENGTH;
    return false;
  }

  DYNAMIXEL::XELInfoSyncWrite_t xels[N_MOTORS];
  DYNAMIXEL::InfoSyncWriteInst_t syncInfo = {};
  uint8_t packetBuffer[192];
  int32_t goalTicks[N_MOTORS];
  float resolvedGoals[N_MOTORS];
  int targetIndices[N_MOTORS];
  uint8_t packedCount = 0;
  uint8_t skippedCount = 0;

  // Resolve the complete batch before transmitting anything. In steady state
  // the last accepted goal is the reference, so continuous frames require no
  // read round trips. A motor with no trustworthy cache gets one live read.
  for (uint8_t k = 0; k < count; ++k) {
    const int index = getIndexById(targets[k].id);
    if (index < 0 || !isfinite(targets[k].angleDeg)) {
      if (libErrorOut) *libErrorOut = DXL_LIB_ERROR_INVAILD_ID;
      return false;
    }
    for (uint8_t prior = 0; prior < k; ++prior) {
      if (targets[prior].id == targets[k].id) {
        if (libErrorOut) *libErrorOut = DXL_LIB_ERROR_INVAILD_ID;
        return false;
      }
    }

    // Dual firmware contains all 18 configured IDs even when only one hand is
    // plugged in. Initialization already attempted a position read from every
    // ID; omit known-offline motors instead of letting one missing hand veto
    // the complete frame or adding repeated timeout delays to every update.
    if (!motorReachable_[index]) {
      ++skippedCount;
      continue;
    }

    float reference;
    if (goalAngleValid_[index]) {
      reference = goalAngle_[index];
    } else {
      reference = EXO_PROFILE_CALL(DXL_READ, dxl_.getPresentPosition(targets[k].id, UNIT_DEGREE));
      if (dxl_.getLastLibErrCode() != DXL_LIB_OK || !isfinite(reference) ||
          reference < -36000.0f || reference > 36000.0f) {
        // Skip this frame but keep retrying a motor that was online at boot;
        // a transient read error must not disable that joint until reset.
        ++skippedCount;
        continue;
      }
    }

    float goal = constrain(targets[k].angleDeg,
                           jointLimits_[index][0], jointLimits_[index][1]);
    goal = applyShortestPathFromReference(index, goal, reference);

    targetIndices[packedCount] = index;
    resolvedGoals[packedCount] = goal;
    // Match Dynamixel2Arduino's Protocol-2 position conversion (0.088 deg/tick).
    goalTicks[packedCount] = (int32_t)roundf(goal / 0.088f);
    xels[packedCount].id = targets[k].id;
    xels[packedCount].p_data =
        reinterpret_cast<uint8_t*>(&goalTicks[packedCount]);
    ++packedCount;
  }

  if (skippedOut) *skippedOut = skippedCount;
  if (packedCount == 0) {
    if (libErrorOut) *libErrorOut = DXL_LIB_ERROR_TIMEOUT;
    return false;
  }

  // Use a dedicated buffer so this packet is independent of any prior
  // telemetry/read instruction cached in Dynamixel2Arduino's internal buffer.
  syncInfo.packet.p_buf = packetBuffer;
  syncInfo.packet.buf_capacity = sizeof(packetBuffer);
  syncInfo.packet.is_completed = false;
  syncInfo.addr = DYNAMIXEL_GOAL_POSITION_ADDRESS;
  syncInfo.addr_length = DYNAMIXEL_GOAL_POSITION_LENGTH;
  syncInfo.p_xels = xels;
  syncInfo.xel_count = packedCount;
  syncInfo.is_info_changed = true;

  if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.syncWrite(&syncInfo))) {
    if (libErrorOut) *libErrorOut = (int16_t)dxl_.getLastLibErrCode();
    return false;
  }

  for (uint8_t k = 0; k < packedCount; ++k) {
    noteGoalCommanded(targetIndices[k], resolvedGoals[k]);
    goalAngleValid_[targetIndices[k]] = true;
  }
  if (writtenOut) *writtenOut = packedCount;
  return true;
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
  float homeAngle = constrain(zeroOffsets_[index], jointLimits_[index][0], jointLimits_[index][1]);

  // Shortest-path guard: avoid 360° rotations in extended position mode
  homeAngle = applyShortestPath(index, homeAngle);

  noteGoalCommanded(index, homeAngle);
  goalAngleValid_[index] = EXO_PROFILE_CALL(DXL_WRITE, dxl_.setGoalPosition(id, homeAngle, UNIT_DEGREE));
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
  if (!isfinite(angle_deg)) return;
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
  goalAngleValid_[index] = EXO_PROFILE_CALL(DXL_WRITE, dxl_.setGoalPosition(id, abs_goal, UNIT_DEGREE));
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
  const float present = EXO_PROFILE_CALL(DXL_READ, dxl_.getPresentPosition(motorIds_[index], UNIT_DEGREE));
  return applyShortestPathFromReference(index, goal, present);
}
float NMLHandExo::applyShortestPathFromReference(int index, float goal,
                                                 float reference) const {
  if (index < 0 || index >= (int)numMotors_) return goal;
  const float diff = goal - reference;
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
  return EXO_PROFILE_CALL(DXL_READ, dxl_.getTorqueEnableStat(id));
}

void NMLHandExo::enableTorque(uint8_t id, bool enable) {
  if (isAssistBusy()) { if (enable) return; stopAssist("disabled"); }
  const int index = getIndexById(id);
  if (index < 0) return;
  serviceJointModels();
  telemetryGate_.note(millis());
  if (enable) {
    if (EXO_PROFILE_CALL(DXL_WRITE, dxl_.torqueOn(id))) torqueEnabled_[index] = true;
  } else {
    stopDirectControl(id);
    if (EXO_PROFILE_CALL(DXL_WRITE, dxl_.torqueOff(id))) {
      torqueEnabled_[index] = false;
      motorMoving_[index] = false;
      motorAdmitted_[index] = false;
      directCommandActive_[index] = false;
      directCommandDirection_[index] = 0;
      verdictPending_[index] = false;
      allocationDirty_ = true;
    }
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
  if (wasEnabled) EXO_PROFILE_CALL(DXL_WRITE, dxl_.torqueOff(id));
  EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(CURRENT_LIMIT, id, current_mA));
  if (wasEnabled) EXO_PROFILE_CALL(DXL_WRITE, dxl_.torqueOn(id));
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
  return EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(PRESENT_CURRENT, id));
}
float NMLHandExo::getTorque(uint8_t id) {
  float current_mA = NMLHandExo::getCurrent(id);
  float torque_Nm = current_mA * XC330_T288_TORQUE_CONSTANT;
  return torque_Nm;  // in N·m
}
bool NMLHandExo::setGoalCurrents(const uint8_t* ids, const float* currents, uint8_t count) {
  if (!ids || !currents || count == 0 || count > N_MOTORS || motorControlMode_ != "CURRENT") return false;
  for (uint8_t k = 0; k < count; ++k) {
    const int index = getIndexById(ids[k]);
    if (index < 0 || !isfinite(currents[k]) || positionHoldActive_[index]) return false;
    for (uint8_t j = 0; j < k; ++j) if (ids[j] == ids[k]) return false;
  }
  for (uint8_t k = 0; k < count; ++k) {
    if (!setGoalCurrent(ids[k], currents[k])) return false;
  }
  return true;
}
bool NMLHandExo::setGoalCurrent(uint8_t id, float current_mA) {
  if (isAssistBusy()) return false;
  int index = getIndexById(id);
  if (index == -1 || !isfinite(current_mA) || motorControlMode_ != "CURRENT" || positionHoldActive_[index]) return false;

  current_mA = constrain(
      current_mA,
      -(float)DIRECT_CURRENT_LIMIT_MA,
      (float)DIRECT_CURRENT_LIMIT_MA);
  if (flipMotor_[index]) current_mA *= -1.0f;

  float position = getAbsoluteAngle(id);
  if (!isfinite(position) || (current_mA > 0 && position >= jointLimits_[index][1] - DIRECT_LIMIT_MARGIN_DEG) ||
      (current_mA < 0 && position <= jointLimits_[index][0] + DIRECT_LIMIT_MARGIN_DEG)) {
    current_mA = 0;
  }

  serviceJointModels();
  if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(GOAL_CURRENT, id, (int16_t)round(current_mA)))) return false;
  telemetryGate_.note(millis());
  lastDirectCommandMs_[index] = millis();
  directCommandActive_[index] = (current_mA != 0);
  directCommandDirection_[index] = current_mA;
  return true;
}
bool NMLHandExo::writeRomSweepCurrent(float current_mA) {
  const int index = romCalIndex_;
  if (index < 0 || !isfinite(current_mA) || motorControlMode_ != "CURRENT") return false;

  // Pulse service checks the stored joint window using measured feedback.
  // This final write independently enforces the ROM-specific current ceiling.
  const float ceiling = min(ROM_CAL_MAX_CURRENT_MA, (float)DIRECT_CURRENT_LIMIT_MA);
  current_mA = constrain(current_mA, -ceiling, ceiling);
  if (flipMotor_[index]) current_mA *= -1.0f;

  current_mA = roundf(current_mA);
  serviceJointModels();
  if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem((uint8_t)GOAL_CURRENT, romCalId_,
        (int32_t)round(current_mA), (uint32_t)ROM_CAL_IO_TIMEOUT_MS))) return false;
  if (romCalPhase_ == ROM_CAL_RAMP) romCalExposure_.command(current_mA, millis());
  if (current_mA == 0 && romCalPhase_ == ROM_CAL_RAMP) {
    romCalPulseOnMs_ = millis() - romCalPulseStartMs_;
    if (romCalPulseOnMs_ > romCalMaxPulseOnMs_) romCalMaxPulseOnMs_ = romCalPulseOnMs_;
  }
  telemetryGate_.note(millis());
  lastDirectCommandMs_[index] = millis();
  directCommandActive_[index] = (current_mA != 0);
  directCommandDirection_[index] = current_mA;
  return true;
}
float NMLHandExo::getGoalCurrent(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return 0;
  float current_mA = (int16_t)EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(GOAL_CURRENT, id));
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
  if (EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(GOAL_CURRENT, motorIds_[index], current_mA))) {
    appliedCurrents_[index] = current_mA;
  }
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
  serviceJointModels();
  unsigned long now = millis();
  telemetryGate_.note(now);
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
  telemetryGate_.note(millis());
  motorMoving_[index] = false;
  motorAdmitted_[index] = false;
  stallSinceMs_[index] = 0;
  allocationDirty_ = true;       // a freed slot lets the next wave in

  if (!verdictPending_[index]) return;
  verdictPending_[index] = false;

  if (verdict == MOVE_VERDICT_REACHED) {
    // "Settled" only means it stopped pulling; confirm it actually arrived.
    // This is the one extra position read per move, not per sample.
    float present = EXO_PROFILE_CALL(DXL_READ, dxl_.getPresentPosition(motorIds_[index], UNIT_DEGREE));
    if (dxl_.getLastLibErrCode() == DXL_LIB_OK && isfinite(present)) {
      float error = present - goalAngle_[index];
      if (error < 0) error = -error;
      if (error > GESTURE_REACH_TOLERANCE_DEG) verdict = MOVE_VERDICT_SHORT;
      // Refresh the shortest-path cache with physical evidence whenever a move
      // settles, including a SHORT verdict where goal and reality diverged.
      goalAngle_[index] = present;
      goalAngleValid_[index] = true;
    } else {
      verdict = MOVE_VERDICT_SHORT;
      goalAngleValid_[index] = false;
    }
  } else {
    // A stall/starvation means the accepted goal is no longer a trustworthy
    // proxy for position. The next batch performs one recovery read for it.
    goalAngleValid_[index] = false;
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
  int32_t raw = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(item, id, timeout_ms));
  if (dxl_.getLastLibErrCode() != DXL_LIB_OK) {
    ok = false;
    return 0;
  }
  return (int16_t)raw;
}

bool NMLHandExo::readAxonAngle(uint8_t id, int16_t& angle) {
  angle = INT16_MIN;
  if (getIndexById(id) < 0) return false;
  const int32_t ticks = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem((uint8_t)PRESENT_POSITION, id, 2));
  if (dxl_.getLastLibErrCode() != DXL_LIB_OK) return false;
  const float value = round(ticks * 3600.0f / (float)PULSE_RESOLUTION);
  if (!isfinite(value) || value <= INT16_MIN || value > INT16_MAX) return false;
  angle = (int16_t)value;
  return true;
}

bool NMLHandExo::readPresentPositionTicks(uint8_t id, int32_t& ticks) {
  while (DXL_SERIAL.available() > 0) {
    DXL_SERIAL.read();
  }
  const uint8_t item = (uint8_t)PRESENT_POSITION;
  const uint32_t timeout_ms = 10;
  ticks = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(item, id, timeout_ms));
  return dxl_.getLastLibErrCode() == DXL_LIB_OK;
}

bool NMLHandExo::configureShadowTelemetry(
  const uint8_t* ids, uint8_t count, unsigned long intervalMs
) {
  if (ids == nullptr || count == 0 || count > SHADOW_TELEMETRY_MAX_MOTORS) {
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) {
    if (getIndexById(ids[i]) < 0) return false;
    for (uint8_t j = 0; j < i; ++j) {
      if (ids[j] == ids[i]) return false;
    }
  }

  shadowTelemetryEnabled_ = false;
  shadowTelemetryCount_ = count;
  shadowTelemetryCursor_ = 0;
  shadowTelemetryReadCurrent_ = true;
  shadowTelemetryIntervalMs_ = constrain(
    intervalMs,
    SHADOW_TELEMETRY_MIN_INTERVAL_MS,
    SHADOW_TELEMETRY_MAX_INTERVAL_MS
  );
  shadowTelemetryLastReadMs_ = 0;
  shadowTelemetrySequence_ = 0;
  shadowTelemetryReadErrors_ = 0;
  for (uint8_t i = 0; i < SHADOW_TELEMETRY_MAX_MOTORS; ++i) {
    shadowRecords_[i] = ShadowTelemetryRecord();
    if (i < count) shadowRecords_[i].id = ids[i];
  }
  return true;
}

bool NMLHandExo::startShadowTelemetry() {
  if (shadowTelemetryCount_ == 0 || motorControlMode_ != "VELOCITY") {
    return false;
  }
  shadowTelemetryEnabled_ = true;
  shadowTelemetryLastReadMs_ = 0;
  return true;
}

void NMLHandExo::stopShadowTelemetry() {
  shadowTelemetryEnabled_ = false;
}

bool NMLHandExo::isShadowTelemetryEnabled() const {
  return shadowTelemetryEnabled_;
}

uint8_t NMLHandExo::getShadowTelemetryCount() const {
  return shadowTelemetryCount_;
}

unsigned long NMLHandExo::getShadowTelemetryIntervalMs() const {
  return shadowTelemetryIntervalMs_;
}

uint32_t NMLHandExo::getShadowTelemetrySequence() const {
  return shadowTelemetrySequence_;
}

uint32_t NMLHandExo::getShadowTelemetryReadErrors() const {
  return shadowTelemetryReadErrors_;
}

uint8_t NMLHandExo::copyShadowTelemetryRecords(
  ShadowTelemetryRecord* records, uint8_t capacity
) const {
  if (records == nullptr) return 0;
  uint8_t count = min(shadowTelemetryCount_, capacity);
  for (uint8_t i = 0; i < count; ++i) records[i] = shadowRecords_[i];
  return count;
}

void NMLHandExo::serviceShadowTelemetry() {
  if (telemetryEstimated()) return;
  if (!shadowTelemetryEnabled_) return;
  if (motorControlMode_ != "VELOCITY") {
    // Preserve the configuration for diagnostics, but stop bus traffic if a
    // mode change could activate the current-position governor.
    shadowTelemetryEnabled_ = false;
    return;
  }
  if (shadowTelemetryCount_ == 0) return;

  unsigned long now = millis();
  if (now - shadowTelemetryLastReadMs_ < shadowTelemetryIntervalMs_) return;
  shadowTelemetryLastReadMs_ = now;

  ShadowTelemetryRecord& record = shadowRecords_[shadowTelemetryCursor_];
  bool ok = true;
  if (shadowTelemetryReadCurrent_) {
    record.current_mA = readPresentCurrentMa(record.id, ok);
    if (ok) record.current_sample_ms = now;
  } else {
    int32_t ticks = 0;
    ok = readPresentPositionTicks(record.id, ticks);
    if (ok) {
      int index = getIndexById(record.id);
      float absoluteDeg = ticks * 360.0f / (float)PULSE_RESOLUTION;
      float relativeDeg = absoluteDeg - zeroOffsets_[index];
      if (flipMotor_[index]) relativeDeg *= -1.0f;
      int32_t relativeCdeg = (int32_t)round(relativeDeg * 100.0f);
      if (record.position_sample_ms != 0 && now > record.position_sample_ms) {
        uint32_t dtMs = now - record.position_sample_ms;
        int32_t deltaCdeg = relativeCdeg - record.relative_cdeg;
        record.velocity_cdeg_s = (int32_t)(((int64_t)deltaCdeg * 1000) / dtMs);
      } else {
        record.velocity_cdeg_s = 0;
      }
      record.position_ticks = ticks;
      record.absolute_cdeg = (int32_t)round(absoluteDeg * 100.0f);
      record.relative_cdeg = relativeCdeg;
      record.position_sample_ms = now;
      shadowTelemetrySequence_++;
    }
  }

  const uint8_t errorMask = shadowTelemetryReadCurrent_ ? 0x01 : 0x02;
  if (!ok) {
    record.error |= errorMask;
    shadowTelemetryReadErrors_++;
  } else {
    record.error &= (uint8_t)~errorMask;
  }

  shadowTelemetryReadCurrent_ = !shadowTelemetryReadCurrent_;
  if (shadowTelemetryReadCurrent_) {
    shadowTelemetryCursor_++;
    if (shadowTelemetryCursor_ >= shadowTelemetryCount_) {
      shadowTelemetryCursor_ = 0;
    }
  }
}
void NMLHandExo::serviceCurrentGovernor() {
  if (!currentGovernorEnabled_) return;
  if (motorControlMode_ != "CURRENT_POSITION") return;
  if (numMotors_ == 0) return;

  unsigned long now = millis();
  for (int i = 0; i < numMotors_; ++i) {
    if (motorMoving_[i] && now - goalIssuedMs_[i] >= MOVE_TIMEOUT_MS) {
      concludeMove(i, motorAdmitted_[i] ? MOVE_VERDICT_STALLED : MOVE_VERDICT_STARVED);
    }
  }

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
  EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(PROFILE_VELOCITY, id, vel));
  debugPrint("Velocity limit set for motor " + String(id) + ": " + String(vel));
}
uint32_t NMLHandExo::getVelocityLimit(uint8_t id) {
  return EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(PROFILE_VELOCITY, id));
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
  if (isAssistBusy()) return false;
  int index = getIndexById(id);
  if (index == -1 || !isfinite(velocity_rpm) || motorControlMode_ != "VELOCITY" ||
      !directVelocityLimitVerified_[index] || positionHoldActive_[index]) return false;

  velocity_rpm = constrain(
      velocity_rpm,
      -DIRECT_VELOCITY_LIMIT_RPM,
      DIRECT_VELOCITY_LIMIT_RPM);
  if (flipMotor_[index]) velocity_rpm *= -1.0f;

  float position = getAbsoluteAngle(id);
  float limited_velocity_rpm =
      isfinite(position) ? limitDirectVelocity(index, velocity_rpm, position) : 0.0f;

  int32_t raw = (int32_t)round(limited_velocity_rpm / 0.229f);
  const int32_t requestedRaw = (int32_t)round(velocity_rpm / 0.229f);
  if (raw == 0 && requestedRaw != 0 && limited_velocity_rpm != 0.0f) {
    // The taper fell below one register step. Latch here rather than letting
    // repeated teleop packets alternate between raw velocity 0 and 1.
    directVelocityLimitBlock_[index] = velocity_rpm > 0.0f ? 1 : -1;
  }

  serviceJointModels();
  if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(GOAL_VELOCITY, id, raw))) return false;
  telemetryGate_.note(millis());
  lastDirectCommandMs_[index] = millis();
  directCommandActive_[index] = (raw != 0);
  directCommandDirection_[index] = raw * 0.229f;
  return true;
}
float NMLHandExo::getPresentVelocity(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return 0;
  int32_t raw = (int32_t)EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(PRESENT_VELOCITY, id));
  float rpm = raw * 0.229f;
  return flipMotor_[index] ? -rpm : rpm;
}
void NMLHandExo::stopDirectControl(uint8_t id) {
  if (isAssistBusy()) stopAssist("external_stop");
  if (id == romCalId_ && (romCalPhase_ == ROM_CAL_RAMP || romCalPhase_ == ROM_CAL_REST)) {
    romCalStopRequested_ = true;
    if (strcmp(romCalReason_, "none") == 0) romCalReason_ = "stop_requested";
    if (romCalPhase_ == ROM_CAL_RAMP) {
      romCalPulseOnMs_ = millis() - romCalPulseStartMs_;
      if (romCalPulseOnMs_ > romCalMaxPulseOnMs_) romCalMaxPulseOnMs_ = romCalPulseOnMs_;
    }
  }
  serviceJointModels();
  telemetryGate_.note(millis());
  int index = getIndexById(id);
  if (index == -1) return;
  if (positionHoldActive_[index]) {
    directCommandActive_[index] = false;
    directCommandDirection_[index] = 0;
    lastDirectCommandMs_[index] = millis();
    return;
  } else if (motorControlMode_ == "VELOCITY") {
    if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(GOAL_VELOCITY, id, 0))) return;
  } else if (motorControlMode_ == "CURRENT") {
    if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(GOAL_CURRENT, id, 0))) return;
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
  if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(GOAL_CURRENT, id, hold_mA))) return false;
  appliedCurrents_[index] = hold_mA;
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
    // ROM owns paced, error-checked position limits and pulse deadlines.
    // Keep the independent command watchdog, without duplicate unpaced reads.
    const bool isActiveRomTarget =
        (romCalPhase_ == ROM_CAL_RAMP) && (id == romCalId_);
    if (isActiveRomTarget) {
      if (now - lastDirectCommandMs_[i] > directCommandTimeoutMs_) {
        romCalReason_ = "command_watchdog";
        stopDirectControl(id);
      }
      continue;
    }
    float position = getAbsoluteAngle(id);
    bool timedOut = now - lastDirectCommandMs_[i] > directCommandTimeoutMs_;
    bool drivingIntoLimit = (
        (directCommandDirection_[i] < 0 &&
         position <= jointLimits_[i][0] + DIRECT_LIMIT_MARGIN_DEG) ||
        (directCommandDirection_[i] > 0 &&
         position >= jointLimits_[i][1] - DIRECT_LIMIT_MARGIN_DEG));
    if (timedOut || drivingIntoLimit || !isfinite(position)) {
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
  EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(PROFILE_ACCELERATION, id, acc));
  debugPrint("Acceleration limit set for motor " + String(id) + ": " + String(acc));
}
uint32_t NMLHandExo::getAccelerationLimit(uint8_t id) {
  return EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(PROFILE_ACCELERATION, id));
}

// Motor-specific commands
void NMLHandExo::rebootMotor(uint8_t id) {
  EXO_PROFILE_CALL(DXL_WRITE, dxl_.reboot(id));
  debugPrint("Motor ID:" + String(id) + " rebooted");
}
void NMLHandExo::getMotorInfo(uint8_t id) {
  EXO_PROFILE_CALL(DXL_READ, dxl_.ping(id));  // could be expanded to read Model Number, Version, etc.
  debugPrint("Pinged motor ID: " + String(id));
}
bool NMLHandExo::setBaudRate(uint8_t id, uint32_t baudrate) {
  // BAUD_RATE stores a model-specific index, not the literal bits-per-second
  // value. Let the library perform that mapping and reject unsupported rates.
  const bool ok = EXO_PROFILE_CALL(DXL_WRITE, dxl_.setBaudrate(id, baudrate));
  debugPrint("Motor ID:" + String(id) + " baudrate " +
             (ok ? "set to " : "failed: ") + String(baudrate));
  return ok;
}
uint32_t NMLHandExo::getBaudRate(uint8_t id) {
  const uint32_t baudIndex = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(BAUD_RATE, id));
  if (dxl_.getLastLibErrCode() != DXL_LIB_OK) return 0;
  // XL330/XC330 Protocol-2 baud indices. The old implementation exposed this
  // raw index even though the API and serial response both promise bit/s.
  switch (baudIndex) {
    case 0: return 9600;
    case 1: return 57600;
    case 2: return 115200;
    case 3: return 1000000;
    case 4: return 2000000;
    case 5: return 3000000;
    case 6: return 4000000;
    default: return 0;
  }
}
void NMLHandExo::setMotorLED(uint8_t id, bool state) {
  // Sets specified motor LED to the specified state
  if (state) {
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.ledOn(id));
  } else {
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.ledOff(id));
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
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.setOperatingMode(id, OP_POSITION));
    debugPrint("Set motor " + String(id) + " to POSITION mode");
  } else if (m == "CURRENT_POSITION") {
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.setOperatingMode(id, OP_CURRENT_BASED_POSITION));
    debugPrint("Set motor " + String(id) + " to CURRENT_POSITION mode");
  } else if (m == "VELOCITY") {
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.setOperatingMode(id, OP_VELOCITY));
    debugPrint("Set motor " + String(id) + " to VELOCITY mode");
  } else if (m == "CURRENT") {
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.setOperatingMode(id, OP_CURRENT));
    debugPrint("Set motor " + String(id) + " to CURRENT mode");
  } else {
    debugPrint("[ERROR] Unknown operating mode: " + m);
  }
}
bool NMLHandExo::ensureDirectVelocityLimit(uint8_t id) {
  uint32_t current = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(VELOCITY_LIMIT, id));
  if (current != DIRECT_VELOCITY_LIMIT_RAW) {
    if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(
            VELOCITY_LIMIT, id, DIRECT_VELOCITY_LIMIT_RAW))) {
      debugPrint("[ERROR] Could not write VELOCITY_LIMIT for motor " + String(id));
      return false;
    }
    current = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(VELOCITY_LIMIT, id));
  }
  if (current != DIRECT_VELOCITY_LIMIT_RAW) {
    debugPrint("[ERROR] VELOCITY_LIMIT readback mismatch for motor " +
               String(id) + ": " + String(current));
    return false;
  }
  return true;
}

bool NMLHandExo::setMotorControlMode(const String& mode) {
  if (isAssistBusy()) { stopAssist("mode_changed"); if (isAssistBusy()) return false; }
  String m = mode;
  m.toUpperCase();
  for (int i = 0; i < numMotors_; i++) {
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.torqueOff(motorIds_[i]));
    torqueEnabled_[i] = false;
    motorMoving_[i] = false;
    verdictPending_[i] = false;
  }
  telemetryGate_.note(millis());
  if (m == "VELOCITY") {
    for (int i = 0; i < numMotors_; i++) {
      directVelocityLimitVerified_[i] = false;
      // Dual firmware can legitimately run with only one hand attached.
      // Missing configured IDs must not block the reachable side, but an ID
      // that was not verified is rejected later by setGoalVelocity().
      if (EXO_PROFILE_CALL(DXL_READ, dxl_.ping(motorIds_[i])) == 0) continue;
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
    torqueEnabled_[i] = false;
    motorMoving_[i] = false;
    verdictPending_[i] = false;
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

// Field-validity by control mode for Axon telemetry. The mode strings mirror
// setMotorControlMode()/setMotorMode(): "POSITION", "CURRENT_POSITION",
// "VELOCITY", "CURRENT" (and "DISABLED"). Position is a controlled quantity in
// the two position-holding modes; current (and its derived torque) is a
// controlled quantity in the two current-driving modes. The Axon telemetry
// callback ANDs these into the per-field validity so an uncontrolled-but-
// readable register is reported as unavailable rather than as a measurement.
// Both false for "DISABLED"/"UNKNOWN": no field is a controlled quantity then.
bool NMLHandExo::modeControlsPosition() const {
  return motorControlMode_ == "POSITION" || motorControlMode_ == "CURRENT_POSITION";
}
bool NMLHandExo::modeControlsCurrent() const {
  return motorControlMode_ == "CURRENT" || motorControlMode_ == "CURRENT_POSITION";
}

