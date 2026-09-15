#include "config.h"
#include "nml_hand_exo.h"
#include "io_profile.h"

bool NMLHandExo::configureAssist(uint8_t id, float thresholdMa, float gainDegPerMa, float capMa) {
  const int i = getIndexById(id);
  if (isAssistBusy() || i < 0 || !isfinite(thresholdMa) || thresholdMa < 5 || thresholdMa > 60 ||
      !isfinite(gainDegPerMa) || gainDegPerMa < .001f || gainDegPerMa > .05f ||
      !isfinite(capMa) || capMa < 10 || capMa > 80 || thresholdMa >= capMa) return false;
  auto& j = assistJoints_[i];
  j.threshold = thresholdMa; j.gain = gainDegPerMa; j.currentCap = floorf(capMa);
  j.configured = true; j.calibrated = false;
  return true;
}

bool NMLHandExo::readAssistSample(uint8_t id, float& q, float& v, float& current, bool requireEnabled) {
  const int32_t ticks = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem((uint8_t)PRESENT_POSITION, id, (uint32_t)10));
  if (dxl_.getLastLibErrCode() != DXL_LIB_OK) return false;
  const int32_t speed = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem((uint8_t)PRESENT_VELOCITY, id, (uint32_t)10));
  if (dxl_.getLastLibErrCode() != DXL_LIB_OK) return false;
  const int16_t amps = (int16_t)EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem((uint8_t)PRESENT_CURRENT, id, (uint32_t)10));
  if (dxl_.getLastLibErrCode() != DXL_LIB_OK) return false;
  const int enabled = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem((uint8_t)TORQUE_ENABLE, id, (uint32_t)10));
  if (dxl_.getLastLibErrCode() != DXL_LIB_OK || (requireEnabled && enabled != 1)) return false;
  q = ticks*(360.0f/PULSE_RESOLUTION); v = speed*(.229f*6); current = amps;
  return isfinite(q) && fabsf(q) <= 36000 && isfinite(v);
}

bool NMLHandExo::writeAssistGoal(int i) {
  const float q = assistJoints_[i].goal;
  if (!isfinite(q) || q < jointLimits_[i][0]+DIRECT_LIMIT_MARGIN_DEG ||
      q > jointLimits_[i][1]-DIRECT_LIMIT_MARGIN_DEG) return false;
  // No wrapping, model prediction, implicit enable, or allocator is involved.
  const bool ok = EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(
      (uint8_t)GOAL_POSITION, motorIds_[i], (int32_t)roundf(q*PULSE_RESOLUTION/360.0f), (uint32_t)10));
  if (ok) {
    goalAngle_[i] = q; goalAngleValid_[i] = true;
    motorMoving_[i] = assistJoints_[i].moving; motorAdmitted_[i] = true;
    telemetryGate_.note(millis());
  }
  return ok;
}

bool NMLHandExo::calibrateAssist(const uint8_t* ids, uint8_t count) {
  if (isAssistBusy()) return false;
  assistRejectId_ = -1; assistRejectAngle_ = assistRejectVelocity_ = NAN;
  auto reject = [&](const char* reason, int id = -1) {
    assistReason_ = reason; assistRejectId_ = id; return false;
  };
  if (isRomCalibrating() || isExoCalibrating()) return reject("calibration_busy");
  if (motorControlMode_ != "CURRENT_POSITION") return reject("requires_current_position");
  if (!ids || !count || count > numMotors_) return reject("invalid_targets");
  bool chosen[N_MOTORS] = {};
  uint32_t budget = 0;
  float positions[N_MOTORS] = {};
  for (uint8_t k = 0; k < count; ++k) {
    assistRejectAngle_ = assistRejectVelocity_ = NAN;
    const int i = getIndexById(ids[k]);
    if (i < 0) return reject("unknown_motor", ids[k]);
    if (chosen[i]) return reject("duplicate_motor", ids[k]);
    if (!assistJoints_[i].configured) return reject("not_configured", ids[k]);
    if (positionHoldActive_[i]) return reject("position_hold_active", ids[k]);
    if (assistJoints_[i].currentCap > currentLimits_[i]) return reject("motor_current_limit", ids[k]);
    chosen[i] = true; budget += (uint16_t)assistJoints_[i].currentCap;
    float v, current;
    if (!readAssistSample(ids[k], positions[i], v, current, false)) return reject("feedback_or_torque_read", ids[k]);
    assistRejectAngle_ = positions[i]; assistRejectVelocity_ = v;
    if (fabsf(v) > 1) return reject("not_settled", ids[k]);
    if (positions[i] < jointLimits_[i][0]+DIRECT_LIMIT_MARGIN_DEG ||
        positions[i] > jointLimits_[i][1]-DIRECT_LIMIT_MARGIN_DEG) return reject("stored_limit", ids[k]);
    const int mode = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem((uint8_t)OPERATING_MODE, ids[k], (uint32_t)10));
    if (dxl_.getLastLibErrCode() != DXL_LIB_OK) return reject("operating_mode_read", ids[k]);
    if (mode != 5) return reject("motor_requires_current_position", ids[k]);
    const int drive = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem((uint8_t)DRIVE_MODE, ids[k], (uint32_t)10));
    // Require normal direction, velocity-based profile and no implicit torque-on
    // by goal writes. Do not silently change EEPROM or invert a load signal.
    if (dxl_.getLastLibErrCode() != DXL_LIB_OK || (drive & 0x0d)) {
      return reject("unsupported_drive_mode", ids[k]);
    }
  }
  // Exclusive ownership: no enabled unselected joint can retain an old goal
  // while the ordinary position governor is paused.
  assistRejectAngle_ = assistRejectVelocity_ = NAN;
  for (int i = 0; i < numMotors_; ++i) if (!chosen[i]) {
    if (torqueEnabled_[i]) return reject("unselected_motor_enabled", motorIds_[i]);
    if (motorReachable_[i]) {
      const int enabled = EXO_PROFILE_CALL(DXL_READ, dxl_.readControlTableItem(
          (uint8_t)TORQUE_ENABLE, motorIds_[i], (uint32_t)10));
      if (dxl_.getLastLibErrCode() != DXL_LIB_OK) return reject("unselected_torque_read", motorIds_[i]);
      if (enabled != 0) return reject("unselected_motor_enabled", motorIds_[i]);
    }
  }
  if (budget > totalCurrentBudgetMa_) return reject("current_budget");
  assistRejectId_ = -1;
  assistState_ = 1; assistReason_ = "calibrating"; assistCursor_ = 0;
  assistHeartbeatMs_ = assistSampleMs_ = assistBeginMs_ = millis();
  for (int i = 0; i < numMotors_; ++i) assistJoints_[i].selected = chosen[i];
  for (int i = 0; i < numMotors_; ++i) if (chosen[i]) {
    assistJoints_[i].begin(positions[i], millis());
    // Disable before replacing any old goal. The selected torque-on happens
    // only after a capped current and an in-range captured goal are installed.
    if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(
          (uint8_t)TORQUE_ENABLE, motorIds_[i], 0, (uint32_t)10))) {
      stopAssist("start_torque_off_failed"); return false;
    }
    torqueEnabled_[i] = false;
    float q, v, current;
    if (!readAssistSample(motorIds_[i], q, v, current, false) || fabsf(v) > 1 ||
        fabsf(q-positions[i]) > .3f) {
      stopAssist("start_pose_changed"); return false;
    }
    assistJoints_[i].begin(q, millis());
    if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(
          (uint8_t)GOAL_CURRENT, motorIds_[i], (uint16_t)assistJoints_[i].currentCap, (uint32_t)10)) ||
        !EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(
          (uint8_t)PROFILE_VELOCITY, motorIds_[i], 4, (uint32_t)10)) ||
        !EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(
          (uint8_t)PROFILE_ACCELERATION, motorIds_[i], 1, (uint32_t)10)) || !writeAssistGoal(i)) {
      stopAssist("start_write_failed"); return false;
    }
    if (!EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem(
          (uint8_t)TORQUE_ENABLE, motorIds_[i], 1, (uint32_t)10))) {
      stopAssist("start_torque_on_failed"); return false;
    }
    torqueEnabled_[i] = true;
    appliedCurrents_[i] = (uint16_t)assistJoints_[i].currentCap;
  }
  return true;
}

bool NMLHandExo::startAssist() {
  if (assistState_ != 2 || millis()-assistHeartbeatMs_ > 1000) return false;
  assistState_ = 3; assistReason_ = "active"; assistHeartbeatMs_ = millis();
  return true;
}
bool NMLHandExo::heartbeatAssist() {
  // An expired lease cannot be resurrected by a late queued heartbeat.
  if (!isAssistBusy() || assistState_ == 4) return false;
  if (millis()-assistHeartbeatMs_ > 1000) { stopAssist("heartbeat_timeout"); return false; }
  assistHeartbeatMs_ = millis(); return true;
}
void NMLHandExo::stopAssist(const char* reason) {
  if (!isAssistBusy()) return;
  assistState_ = 0; assistReason_ = reason;
  bool stopped = true;
  for (int i = 0; i < numMotors_; ++i) if (assistJoints_[i].selected) {
    // Both zero effort and torque-off are attempted, even if one write fails.
    EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem((uint8_t)GOAL_CURRENT, motorIds_[i], 0, (uint32_t)10));
    const bool off = EXO_PROFILE_CALL(DXL_WRITE, dxl_.writeControlTableItem((uint8_t)TORQUE_ENABLE, motorIds_[i], 0, (uint32_t)10));
    stopped = stopped && off;
    if (off) torqueEnabled_[i] = false; directCommandActive_[i] = false;
    appliedCurrents_[i] = 0; motorMoving_[i] = false; goalAngleValid_[i] = false;
    assistJoints_[i].calibrated = false; assistJoints_[i].moving = false;
    assistJoints_[i].measuredValid = false;
  }
  if (!stopped) { assistState_ = 4; assistReason_ = "torque_off_unconfirmed"; }
  allocationDirty_ = false;
  telemetryGate_.note(millis());
}
void NMLHandExo::serviceAssist() {
  if (!isAssistBusy()) return;
  EXO_PROFILE(CONTROL);
  const unsigned long now = millis();
  if (assistState_ == 4) {
    if (now-assistSampleMs_ >= 100) { assistSampleMs_ = now; stopAssist("stop_retried"); }
    return;
  }
  if (now-assistHeartbeatMs_ > 1000) { stopAssist("heartbeat_timeout"); return; }
  if (motorControlMode_ != "CURRENT_POSITION") { stopAssist("mode_changed"); return; }
  if (assistState_ == 1 && now-assistBeginMs_ > 30000) { stopAssist("calibration_timeout"); return; }
  if (now-assistSampleMs_ < 5) return;
  // Round robin, one joint per pass. Each selected joint gets real feedback
  // at least every 500 ms or the whole session stops; no estimates are allowed.
  int index = -1;
  for (int k = 0; k < numMotors_; ++k) {
    const int i = assistCursor_; assistCursor_ = (assistCursor_+1)%numMotors_;
    if (assistJoints_[i].selected) { index = i; break; }
  }
  if (index < 0) { stopAssist("no_targets"); return; }
  auto& j = assistJoints_[index];
  if (now-j.lastSample < 20) return;
  assistSampleMs_ = now;
  if (!torqueEnabled_[index]) { stopAssist("disabled"); return; }
  float q, v, current;
  if (!readAssistSample(motorIds_[index], q, v, current)) { stopAssist("feedback_read"); return; }
  if (q < jointLimits_[index][0]+DIRECT_LIMIT_MARGIN_DEG ||
      q > jointLimits_[index][1]-DIRECT_LIMIT_MARGIN_DEG) { stopAssist("stored_limit"); return; }
  bool write = false;
  const char* failure = j.sample(q, v, current, millis(), assistState_ == 1, assistState_ == 3, write);
  if (failure) { stopAssist(failure); return; }
  if (millis()-assistHeartbeatMs_ > 1000) { stopAssist("heartbeat_timeout"); return; }
  j.measuredUtc = utcClock_.now(j.lastSample);
  motorMoving_[index] = j.moving;
  jointModels_[index].correct(q, v, current, true, j.lastSample);
  if (write && !writeAssistGoal(index)) { stopAssist("goal_write_or_limit"); return; }
  if (assistState_ == 1) {
    bool ready = true;
    for (int i = 0; i < numMotors_; ++i) if (assistJoints_[i].selected && !assistJoints_[i].calibrated) ready = false;
    if (ready) { assistState_ = 2; assistReason_ = "ready"; }
  }
}
String NMLHandExo::assistStatus() const {
  String out = "ASSIST: state=" + String(assistState_) + " reason=" + String(assistReason_);
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
        " steps=" + String(j.steps) + " age_ms=" + String(millis()-j.lastSample);
  }
  return out;
}
