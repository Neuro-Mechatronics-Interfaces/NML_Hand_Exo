#include "config.h"
#include "nml_hand_exo.h"
#include "io_profile.h"
#include <string.h>

bool NMLHandExo::configureAssist(uint8_t id, float thresholdMa, float gainDegSPerMa, float capMa) {
  const int i = getIndexById(id);
  if (isAssistBusy() || i < 0 || !isfinite(thresholdMa) || thresholdMa < 5 || thresholdMa > 200 ||
      !isfinite(gainDegSPerMa) || gainDegSPerMa < .001f || gainDegSPerMa > .5f ||
      !isfinite(capMa) || capMa < 10 || capMa > 910 || thresholdMa >= capMa ||
      thresholdMa >= assistJoints_[i].intentCap) return false;
  auto& j = assistJoints_[i];
  j.threshold = thresholdMa; j.gain = gainDegSPerMa; j.currentCap = floorf(capMa);
  j.configured = true; j.calibrated = false;
  return true;
}

bool NMLHandExo::configureAssistObserver(uint8_t id, float inertia, float viscous, float coulomb) {
  const int i = getIndexById(id);
  if (isAssistBusy() || i < 0 || !isfinite(inertia) || inertia < 0 || inertia > 1 ||
      !isfinite(viscous) || viscous < 0 || viscous > 1 ||
      !isfinite(coulomb) || coulomb < 0 || coulomb > 1) return false;
  auto& j = assistJoints_[i];
  j.inertia = inertia; j.viscous = viscous; j.coulomb = coulomb;
  j.resetTelemetryObserver();
  j.calibrated = false;
  return true;
}

bool NMLHandExo::configureAssistSettling(float velocityLimitDegS, float poseDriftDeg) {
  // These limits are startup qualification tolerances, not motion or current
  // safety limits. Keep a bounded range so a UI/API mistake cannot disable the
  // stationary-start gate altogether.
  if (isAssistBusy() || !isfinite(velocityLimitDegS) || velocityLimitDegS < .1f ||
      velocityLimitDegS > 10.0f || !isfinite(poseDriftDeg) || poseDriftDeg < .01f ||
      poseDriftDeg > 5.0f) return false;
  assistSettleVelocityLimitDegS_ = velocityLimitDegS;
  assistSettlePoseDriftDeg_ = poseDriftDeg;
  return true;
}

// XC330 control-table addresses are wire addresses, not ControlTableItem enums.
// No per-ID bus operations or sequential fallback are used in assist.
bool NMLHandExo::readAssistRegisters(const bool* targets, uint16_t address,
                                    uint16_t length, AssistRegisterBlock* data) {
  if (!targets || !data || !length || length > sizeof(data[0].bytes)) return false;
  DYNAMIXEL::XELInfoSyncRead_t xels[N_MOTORS] = {};
  DYNAMIXEL::InfoSyncReadInst_t info = {};
  uint8_t packet[64];
  uint8_t count = 0;
  for (int i = 0; i < numMotors_; ++i) if (targets[i]) {
    data[i] = AssistRegisterBlock{};
    xels[count].id = motorIds_[i];
    xels[count].error = 0xff;
    xels[count++].p_recv_buf = data[i].bytes;
  }
  if (!count) return true;
  info.packet.p_buf = packet; info.packet.buf_capacity = sizeof(packet);
  info.addr = address; info.addr_length = length;
  info.p_xels = xels; info.xel_count = count; info.is_info_changed = true;
  while (DXL_SERIAL.available() > 0) DXL_SERIAL.read();
  const uint8_t got = EXO_PROFILE_CALL(DXL_READ, dxl_.syncRead(&info, (uint32_t)10));
  // The library has no completion bitmap. A partial response cannot certify
  // any member of this control observation, even if some buffers were filled.
  if (got != count || dxl_.getLastLibErrCode() != DXL_LIB_OK) return false;
  for (uint8_t k = 0; k < count; ++k) if (xels[k].error != 0) return false;
  return true;
}

bool NMLHandExo::writeAssistRegisters(const bool* targets, uint16_t address,
                                     uint16_t length, AssistRegisterBlock* data, bool verify) {
  if (!targets || !data || !length || length > sizeof(data[0].bytes)) return false;
  DYNAMIXEL::XELInfoSyncWrite_t xels[N_MOTORS] = {};
  DYNAMIXEL::InfoSyncWriteInst_t info = {};
  // Enough for 18 IDs x 12 bytes, including worst-case Protocol-2 byte stuffing.
  uint8_t packet[16 + N_MOTORS*18];
  uint8_t count = 0;
  for (int i = 0; i < numMotors_; ++i) if (targets[i]) {
    xels[count].id = motorIds_[i]; xels[count++].p_data = data[i].bytes;
  }
  if (!count) return true;
  info.packet.p_buf = packet; info.packet.buf_capacity = sizeof(packet);
  info.addr = address; info.addr_length = length;
  info.p_xels = xels; info.xel_count = count; info.is_info_changed = true;
  if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.syncWrite(&info))) return false;
  if (verify) {
    AssistRegisterBlock actual[N_MOTORS];
    if (!readAssistRegisters(targets, address, length, actual)) return false;
    for (int i = 0; i < numMotors_; ++i) if (targets[i] && memcmp(data[i].bytes, actual[i].bytes, length)) return false;
  }
  return true;
}

bool NMLHandExo::readAssistFeedback(const bool* targets, AssistFeedback* feedback) {
  AssistRegisterBlock data[N_MOTORS];
  if (!readAssistRegisters(targets, 126, 10, data)) return false;
  for (int i = 0; i < numMotors_; ++i) if (targets[i]) {
    int16_t current; int32_t speed, ticks;
    memcpy(&current, data[i].bytes, 2);
    memcpy(&speed, data[i].bytes+2, 4);
    memcpy(&ticks, data[i].bytes+6, 4);
    feedback[i].angle = ticks*(360.0f/PULSE_RESOLUTION);
    feedback[i].velocity = speed*(.229f*6); feedback[i].current = current;
    if (!isfinite(feedback[i].angle) || fabsf(feedback[i].angle) > 36000 ||
        !isfinite(feedback[i].velocity)) return false;
  }
  return true;
}

bool NMLHandExo::writeAssistGoals(const bool* targets) {
  AssistRegisterBlock data[N_MOTORS];
  // Validate every requested goal before transmitting any of them.
  for (int i = 0; i < numMotors_; ++i) if (targets[i]) {
    const float q = assistJoints_[i].goal;
    if (!isfinite(q) || fabsf(q) > 36000) return false;
    const int32_t ticks = (int32_t)roundf(q*PULSE_RESOLUTION/360.0f);
    const float quantized = ticks*(360.0f/PULSE_RESOLUTION);
    if (q < jointLimits_[i][0]+DIRECT_LIMIT_MARGIN_DEG || q > jointLimits_[i][1]-DIRECT_LIMIT_MARGIN_DEG ||
        quantized < jointLimits_[i][0]+DIRECT_LIMIT_MARGIN_DEG ||
        quantized > jointLimits_[i][1]-DIRECT_LIMIT_MARGIN_DEG) return false;
    memcpy(data[i].bytes, &ticks, 4);
  }
  if (!writeAssistRegisters(targets, 116, 4, data)) return false;
  for (int i = 0; i < numMotors_; ++i) if (targets[i]) {
    goalAngle_[i] = assistJoints_[i].goal; goalAngleValid_[i] = true;
    motorMoving_[i] = assistJoints_[i].moving; motorAdmitted_[i] = true;
  }
  telemetryGate_.note(millis());
  return true;
}

bool NMLHandExo::enableTorqueAssist(const uint8_t* ids, uint8_t count, float thresholdMa,
                                  float gainDegSPerMa, float capMa, float inertia,
                                  float viscous, float coulomb) {
  if (isAssistBusy()) return false;
  assistAutoStart_ = false;
  assistRejectId_ = -1; assistRejectAngle_ = assistRejectVelocity_ = NAN;
  if (!ids || !count || count > numMotors_ || !isfinite(thresholdMa) || thresholdMa < 5 ||
      thresholdMa > 200 || !isfinite(gainDegSPerMa) || gainDegSPerMa < .001f || gainDegSPerMa > .5f ||
      !isfinite(capMa) || capMa < 10 || capMa > 910 || thresholdMa >= capMa ||
      !isfinite(inertia) || inertia < 0 || inertia > 1 ||
      !isfinite(viscous) || viscous < 0 || viscous > 1 ||
      !isfinite(coulomb) || coulomb < 0 || coulomb > 1) {
    assistReason_ = ASSIST_REASON_INVALID_TARGETS; return false;
  }
  bool chosen[N_MOTORS] = {};
  float caps[N_MOTORS] = {}, requestedTotal = 0;
  for (uint8_t k = 0; k < count; ++k) {
    const int i = getIndexById(ids[k]);
    if (i < 0 || chosen[i]) {
      assistRejectId_ = ids[k];
      assistReason_ = i < 0 ? ASSIST_REASON_UNKNOWN_MOTOR : ASSIST_REASON_DUPLICATE_MOTOR;
      return false;
    }
    chosen[i] = true;
    caps[i] = fminf(floorf(capMa), currentLimits_[i]); requestedTotal += caps[i];
  }
  // Share the existing budget without raising it. Each installed cap remains
  // below both the requested cap and the motor's configured safety limit.
  const float scale = requestedTotal > totalCurrentBudgetMa_ ? totalCurrentBudgetMa_/requestedTotal : 1;
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    caps[i] = floorf(caps[i]*scale);
    if (caps[i] < 10 || thresholdMa >= .9f*caps[i]) {
      assistRejectId_ = motorIds_[i]; assistReason_ = ASSIST_REASON_INSUFFICIENT_CURRENT_HEADROOM;
      return false;
    }
  }
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    configureAssist(motorIds_[i], thresholdMa, gainDegSPerMa, caps[i]);
    configureAssistObserver(motorIds_[i], inertia, viscous, coulomb);
  }
  assistAutoStart_ = true;
  const bool ok = calibrateAssist(ids, count);
  if (!ok) assistAutoStart_ = false;
  return ok;
}

bool NMLHandExo::calibrateAssist(const uint8_t* ids, uint8_t count) {
  if (isAssistBusy()) return false;
  assistRejectId_ = -1; assistRejectAngle_ = assistRejectVelocity_ = NAN;
  auto reject = [&](AssistReason reason, int id = -1) {
    assistReason_ = reason; assistRejectId_ = id; return false;
  };
  if (isRomCalibrating() || isExoCalibrating()) return reject(ASSIST_REASON_CALIBRATION_BUSY);
  if (!assistAutoStart_ && motorControlMode_ != "CURRENT_POSITION") return reject(ASSIST_REASON_REQUIRES_CURRENT_POSITION);
  if (!ids || !count || count > numMotors_) return reject(ASSIST_REASON_INVALID_TARGETS);
  bool chosen[N_MOTORS] = {}, reachable[N_MOTORS] = {};
  uint32_t budget = 0;
  for (uint8_t k = 0; k < count; ++k) {
    const int i = getIndexById(ids[k]);
    if (i < 0) return reject(ASSIST_REASON_UNKNOWN_MOTOR, ids[k]);
    if (chosen[i]) return reject(ASSIST_REASON_DUPLICATE_MOTOR, ids[k]);
    if (!assistJoints_[i].configured) return reject(ASSIST_REASON_NOT_CONFIGURED, ids[k]);
    if (positionHoldActive_[i] && !assistAutoStart_) return reject(ASSIST_REASON_POSITION_HOLD_ACTIVE, ids[k]);
    if (assistJoints_[i].currentCap > currentLimits_[i]) return reject(ASSIST_REASON_MOTOR_CURRENT_LIMIT, ids[k]);
    chosen[i] = true; budget += (uint16_t)assistJoints_[i].currentCap;
  }
  for (int i = 0; i < numMotors_; ++i) {
    if (!chosen[i] && torqueEnabled_[i]) return reject(ASSIST_REASON_UNSELECTED_MOTOR_ENABLED, motorIds_[i]);
    reachable[i] = chosen[i] || motorReachable_[i];
  }
  if (budget > totalCurrentBudgetMa_) return reject(ASSIST_REASON_CURRENT_BUDGET);
  AssistRegisterBlock state[N_MOTORS];
  AssistFeedback feedback[N_MOTORS];
  if (!readAssistFeedback(chosen, feedback)) return reject(ASSIST_REASON_FEEDBACK_OR_TORQUE_READ);
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    const float q = feedback[i].angle, v = feedback[i].velocity;
    assistRejectAngle_ = q; assistRejectVelocity_ = v;
    if (fabsf(v) > assistSettleVelocityLimitDegS_)
      return reject(ASSIST_REASON_NOT_SETTLED, motorIds_[i]);
    if (!isfinite(jointLimits_[i][0]) || !isfinite(jointLimits_[i][1]) ||
        jointLimits_[i][1]-jointLimits_[i][0] <= 2*DIRECT_LIMIT_MARGIN_DEG ||
        q < jointLimits_[i][0]+DIRECT_LIMIT_MARGIN_DEG ||
        q > jointLimits_[i][1]-DIRECT_LIMIT_MARGIN_DEG) return reject(ASSIST_REASON_STORED_LIMIT, motorIds_[i]);
  }
  assistRejectAngle_ = assistRejectVelocity_ = NAN;
  if (!readAssistRegisters(chosen, 10, 2, state)) return reject(ASSIST_REASON_OPERATING_MODE_READ);
  bool changeMode[N_MOTORS] = {};
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    if (state[i].bytes[1] != 5 && !assistAutoStart_) return reject(ASSIST_REASON_MOTOR_REQUIRES_CURRENT_POSITION, motorIds_[i]);
    changeMode[i] = state[i].bytes[1] != 5;
    if (state[i].bytes[0] & 0x0d) return reject(ASSIST_REASON_UNSUPPORTED_DRIVE_MODE, motorIds_[i]);
  }
  // One grouped read includes reachable inactive IDs, but writes never do.
  if (!readAssistRegisters(reachable, 64, 1, state)) return reject(ASSIST_REASON_UNSELECTED_TORQUE_READ);
  for (int i = 0; i < numMotors_; ++i)
    if (!chosen[i] && reachable[i] && state[i].bytes[0] != 0) return reject(ASSIST_REASON_UNSELECTED_MOTOR_ENABLED, motorIds_[i]);

  assistRejectId_ = -1;
  assistState_ = 1; assistReason_ = ASSIST_REASON_CALIBRATING;
  assistHeartbeatMs_ = assistSampleMs_ = assistBeginMs_ = millis();
  for (int i = 0; i < numMotors_; ++i) assistJoints_[i].selected = chosen[i];
  AssistRegisterBlock settings[N_MOTORS];
  if (!writeAssistRegisters(chosen, 64, 1, settings, true)) {
    stopAssist(ASSIST_REASON_START_TORQUE_OFF_FAILED); return false;
  }
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) torqueEnabled_[i] = false;
  if (assistAutoStart_) {
    for (int i = 0; i < numMotors_; ++i) if (changeMode[i]) settings[i].bytes[0] = 5;
    if (!writeAssistRegisters(changeMode, 11, 1, settings, true)) {
      stopAssist(ASSIST_REASON_START_WRITE_FAILED); return false;
    }
    motorControlMode_ = "CURRENT_POSITION";
    for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
      positionHoldActive_[i] = directCommandActive_[i] = false;
    }
  }
  AssistFeedback captured[N_MOTORS];
  if (!readAssistFeedback(chosen, captured)) { stopAssist(ASSIST_REASON_START_POSE_CHANGED); return false; }
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    auto& j = assistJoints_[i];
    const float q = captured[i].angle;
    if (fabsf(captured[i].velocity) > assistSettleVelocityLimitDegS_ ||
        fabsf(q-feedback[i].angle) > assistSettlePoseDriftDeg_) {
      stopAssist(ASSIST_REASON_START_POSE_CHANGED); return false;
    }
    j.hardMin = ceilf((jointLimits_[i][0]+DIRECT_LIMIT_MARGIN_DEG)*PULSE_RESOLUTION/360.0f)*360.0f/PULSE_RESOLUTION;
    j.hardMax = floorf((jointLimits_[i][1]-DIRECT_LIMIT_MARGIN_DEG)*PULSE_RESOLUTION/360.0f)*360.0f/PULSE_RESOLUTION;
    j.begin(q, millis());
    j.collectionSettleMs = assistAutoStart_ ? 0 : 500;
    const uint16_t cap = (uint16_t)j.currentCap;
    memcpy(settings[i].bytes, &cap, 2);
  }
  // Each group is read back before torque-on. Sync Write only confirms TX.
  if (!writeAssistRegisters(chosen, 102, 2, settings, true)) {
    stopAssist(ASSIST_REASON_START_WRITE_FAILED); return false;
  }
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    const auto& j = assistJoints_[i];
    memcpy(settings[i].bytes, &j.positionD, 2);
    memcpy(settings[i].bytes+2, &j.positionI, 2);
    memcpy(settings[i].bytes+4, &j.positionP, 2);
  }
  if (!writeAssistRegisters(chosen, 80, 6, settings, true)) {
    stopAssist(ASSIST_REASON_START_WRITE_FAILED); return false;
  }
  for (int i = 0; i < numMotors_; ++i) settings[i] = AssistRegisterBlock{};
  if (!writeAssistRegisters(chosen, 88, 4, settings, true)) {
    stopAssist(ASSIST_REASON_START_WRITE_FAILED); return false;
  }
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    const int32_t ticks = (int32_t)roundf(assistJoints_[i].goal*PULSE_RESOLUTION/360.0f);
    const float q = ticks*(360.0f/PULSE_RESOLUTION);
    if (q < assistJoints_[i].hardMin || q > assistJoints_[i].hardMax) {
      stopAssist(ASSIST_REASON_START_WRITE_FAILED); return false;
    }
    memcpy(settings[i].bytes+8, &ticks, 4);
  }
  if (!writeAssistRegisters(chosen, 108, 12, settings, true)) {
    stopAssist(ASSIST_REASON_START_WRITE_FAILED); return false;
  }
  // Ensure the lease did not expire while collecting/verifying whole-fleet data.
  if (millis()-assistHeartbeatMs_ > 1000) { stopAssist(ASSIST_REASON_HEARTBEAT_TIMEOUT); return false; }
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) settings[i].bytes[0] = 1;
  if (!writeAssistRegisters(chosen, 64, 1, settings, true)) {
    stopAssist(ASSIST_REASON_START_TORQUE_ON_FAILED); return false;
  }
  const uint32_t now = millis();
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    torqueEnabled_[i] = true; appliedCurrents_[i] = (uint16_t)assistJoints_[i].currentCap;
    goalAngle_[i] = assistJoints_[i].goal; goalAngleValid_[i] = true;
    motorMoving_[i] = false; motorAdmitted_[i] = true;
    assistJoints_[i].phaseStart = assistJoints_[i].lastSample = now;
  }
  telemetryGate_.note(now);
  return true;
}

bool NMLHandExo::startAssist() {
  if (assistState_ != 2 || millis()-assistHeartbeatMs_ > 1000) return false;
  for (int i = 0; i < numMotors_; ++i) if (assistJoints_[i].selected) {
    // READY observations cannot pre-qualify activation dwell.
    assistJoints_[i].intentTimer = assistJoints_[i].releaseTimer = assistJoints_[i].stallTimer = 0;
  }
  assistState_ = 3; assistReason_ = ASSIST_REASON_ACTIVE; assistHeartbeatMs_ = millis();
  return true;
}
bool NMLHandExo::heartbeatAssist() {
  // An expired lease cannot be resurrected by a late queued heartbeat.
  if (!isAssistBusy() || assistState_ == 4) return false;
  if (millis()-assistHeartbeatMs_ > 1000) { stopAssist(ASSIST_REASON_HEARTBEAT_TIMEOUT); return false; }
  assistHeartbeatMs_ = millis(); return true;
}
void NMLHandExo::stopAssist(AssistReason reason) {
  assistAutoStart_ = false;
  if (!isAssistBusy()) return;
  assistState_ = 0; assistReason_ = reason;
  bool chosen[N_MOTORS] = {};
  AssistRegisterBlock zero[N_MOTORS], actual[N_MOTORS];
  for (int i = 0; i < numMotors_; ++i) chosen[i] = assistJoints_[i].selected;
  // Both Sync Writes are attempted independently; a successful TX is not an
  // acknowledgement that any motor disabled. A grouped read certifies stop.
  writeAssistRegisters(chosen, 102, 2, zero);
  writeAssistRegisters(chosen, 64, 1, zero);
  bool stopped = readAssistRegisters(chosen, 64, 1, actual);
  if (stopped) for (int i = 0; i < numMotors_; ++i)
    if (chosen[i] && actual[i].bytes[0] != 0) stopped = false;
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    if (stopped) torqueEnabled_[i] = false;
    directCommandActive_[i] = false;
    positionHoldActive_[i] = false;
    appliedCurrents_[i] = 0; motorMoving_[i] = false; goalAngleValid_[i] = false;
    assistJoints_[i].calibrated = false; assistJoints_[i].moving = false;
    assistJoints_[i].measuredValid = false;
  }
  if (!stopped) { assistState_ = 4; assistReason_ = ASSIST_REASON_TORQUE_OFF_UNCONFIRMED; }
  allocationDirty_ = false;
  telemetryGate_.note(millis());
}
void NMLHandExo::serviceAssist() {
  if (!isAssistBusy()) return;
  EXO_PROFILE(CONTROL);
  const uint32_t now = millis();
  if (assistState_ == 4) {
    if (now-assistSampleMs_ >= 100) { assistSampleMs_ = now; stopAssist(ASSIST_REASON_STOP_RETRIED); }
    return;
  }
  if (now-assistHeartbeatMs_ > 1000) { stopAssist(ASSIST_REASON_HEARTBEAT_TIMEOUT); return; }
  if (motorControlMode_ != "CURRENT_POSITION") { stopAssist(ASSIST_REASON_MODE_CHANGED); return; }
  if (assistState_ == 1 && now-assistBeginMs_ > 30000) { stopAssist(ASSIST_REASON_CALIBRATION_TIMEOUT); return; }
  if (now-assistSampleMs_ < 20) return;
  assistSampleMs_ = now;
  bool chosen[N_MOTORS] = {}, changed[N_MOTORS] = {};
  uint8_t count = 0;
  for (int i = 0; i < numMotors_; ++i) if (assistJoints_[i].selected) {
    chosen[i] = true; ++count;
    if (!torqueEnabled_[i]) { stopAssist(ASSIST_REASON_DISABLED); return; }
  }
  if (!count) { stopAssist(ASSIST_REASON_NO_TARGETS); return; }
  AssistFeedback feedback[N_MOTORS];
  AssistRegisterBlock torque[N_MOTORS];
  if (!readAssistFeedback(chosen, feedback) || !readAssistRegisters(chosen, 64, 1, torque)) {
    stopAssist(ASSIST_REASON_FEEDBACK_READ); return;
  }
  const uint32_t sampleMs = millis();
  if (sampleMs-assistHeartbeatMs_ > 1000) { stopAssist(ASSIST_REASON_HEARTBEAT_TIMEOUT); return; }
  // Validate the complete fleet before publishing feedback or any new goals.
  // All joints share the same acquisition timestamp and are serviced every pass.
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    auto& j = assistJoints_[i];
    const auto& f = feedback[i];
    if (torque[i].bytes[0] != 1) { stopAssist(ASSIST_REASON_DISABLED); return; }
    if (f.angle < jointLimits_[i][0]+DIRECT_LIMIT_MARGIN_DEG ||
        f.angle > jointLimits_[i][1]-DIRECT_LIMIT_MARGIN_DEG) { stopAssist(ASSIST_REASON_STORED_LIMIT); return; }
    const AssistReason failure = j.sample(f.angle, f.velocity, f.current, sampleMs,
                                 assistState_ == 1, assistState_ == 3, changed[i]);
    if (failure) {
      assistRejectId_ = motorIds_[i]; assistRejectAngle_ = f.angle; assistRejectVelocity_ = f.velocity;
      stopAssist(failure); return;
    }
    // Keep the read-only telemetry observer current on the same feedback
    // sample used by Assist. It deliberately does not adapt its bias while
    // Assist owns the joint.
    j.observeTelemetry(f.angle, f.velocity, f.current, true, true, false, sampleMs);
  }
  if (millis()-assistHeartbeatMs_ > 1000) { stopAssist(ASSIST_REASON_HEARTBEAT_TIMEOUT); return; }
  if (!writeAssistGoals(changed)) { stopAssist(ASSIST_REASON_GOAL_WRITE_OR_LIMIT); return; }
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    auto& j = assistJoints_[i]; const auto& f = feedback[i];
    j.measuredUtc = utcClock_.now(sampleMs); motorMoving_[i] = j.moving;
    jointModels_[i].correct(f.angle, f.velocity, f.current, true, sampleMs);
  }
  if (assistState_ == 1) {
    bool ready = true;
    for (int i = 0; i < numMotors_; ++i) if (chosen[i] && !assistJoints_[i].calibrated) ready = false;
    if (ready) {
      assistState_ = 2; assistReason_ = ASSIST_REASON_READY;
      // Automatic completion never renews a host lease or prequalifies dwell.
      if (assistAutoStart_) { assistState_ = 3; assistReason_ = ASSIST_REASON_ACTIVE; }
    }
  }
}
void NMLHandExo::getAssistSnapshot(AssistStatusSnapshot& out) const {
  out = AssistStatusSnapshot{};
  out.state = assistState_; out.reason = assistReason_;
  out.total_current_budget_mA = totalCurrentBudgetMa_;
  out.reject_id = assistRejectId_;
  out.reject_angle_deg = assistRejectAngle_; out.reject_velocity_deg_s = assistRejectVelocity_;
  out.settle_velocity_limit_deg_s = assistSettleVelocityLimitDegS_;
  out.settle_pose_drift_deg = assistSettlePoseDriftDeg_;
  const uint32_t now = millis();
  for (int i = 0; i < numMotors_ && out.count < 18; ++i) if (assistJoints_[i].selected) {
    const auto& j = assistJoints_[i];
    auto& record = out.joints[out.count++];
    record.id = motorIds_[i]; record.samples = j.samples; record.steps = j.steps;
    record.bias_mA = j.bias; record.noise_mA = j.noise; record.deadzone_mA = j.deadzone();
    record.effort_mA = j.effort; record.deflection_deg = j.deflection;
    record.angle = j.angle; record.goal = j.goal; record.moving = j.moving;
    record.secondary_goal = j.secondaryGoal; record.intent_deg_s = j.intentVelocity;
    record.lead_deg = j.lead; record.rearm_blocked = j.rearmBlocked;
    record.age_ms = now-j.lastSample;
    record.current_cap_mA = j.currentCap;
  }
}

String NMLHandExo::assistStatus() const {
  String out = "ASSIST: state=" + String(assistState_) + " reason=" + String(assistReasonName(assistReason_));
  out += " total_current_budget_mA=" + String(totalCurrentBudgetMa_);
  out += " settle_velocity_limit_deg_s=" + String(assistSettleVelocityLimitDegS_, 3) +
      " settle_pose_drift_deg=" + String(assistSettlePoseDriftDeg_, 3);
  if (assistRejectId_ >= 0) out += " reject_id=" + String(assistRejectId_) +
      " reject_angle_deg=" + String(assistRejectAngle_, 3) +
      " reject_velocity_deg_s=" + String(assistRejectVelocity_, 3);
  for (int i = 0; i < numMotors_; ++i) if (assistJoints_[i].selected) {
    const auto& j = assistJoints_[i];
    out += "\nASSIST_JOINT: id=" + String(motorIds_[i]) + " samples=" + String(j.samples) +
        " bias_mA=" + String(j.bias, 2) + " noise_mA=" + String(j.noise, 2) +
        " deadzone_mA=" + String(j.deadzone(), 2) + " effort_mA=" + String(j.effort, 2) +
        " deflection_deg=" + String(j.deflection, 3) + " angle=" + String(j.angle, 3) +
        " goal=" + String(j.goal, 3) + " moving=" + String(j.moving ? 1 : 0) +
        " secondary_goal=" + String(j.secondaryGoal, 3) + " intent_deg_s=" + String(j.intentVelocity, 3) +
        " lead_deg=" + String(j.lead, 3) + " rearm_blocked=" + String(j.rearmBlocked ? 1 : 0) +
        " steps=" + String(j.steps) + " age_ms=" + String(millis()-j.lastSample) +
        " current_cap_mA=" + String(j.currentCap, 0);
  }
  return out;
}
