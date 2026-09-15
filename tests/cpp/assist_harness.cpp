#include "assist_controller.h"
#include "joint_state_model.h"
#include "utc_clock.h"
#include <cassert>
#include <string>
#include <vector>
using std::isfinite;
uint32_t nowMs = 0;
unsigned long millis() { return nowMs; }
constexpr int N_MOTORS = 2, PULSE_RESOLUTION = 4096, DXL_LIB_OK = 0;
constexpr float DIRECT_LIMIT_MARGIN_DEG = 2;
enum { PRESENT_POSITION, PRESENT_VELOCITY, PRESENT_CURRENT, OPERATING_MODE, DRIVE_MODE,
       GOAL_POSITION, GOAL_CURRENT, TORQUE_ENABLE, PROFILE_VELOCITY, PROFILE_ACCELERATION };
struct Bus {
  float q[2] = {100, 100}, current[2] = {5, 5}, velocity[2] = {};
  int error = 0, writes = 0, goals = 0, mode = 5, driveMode = 0;
  bool failRead = false, failOff = false, failGoal = false;
  bool enabled[2] = {true, false};
  int goalTicks[2] = {}, caps[2] = {};
  int32_t readControlTableItem(uint8_t field, uint8_t id, uint32_t timeout) {
    assert(timeout == 10); error = failRead ? 1 : 0;
    const int i = id-16;
    if (field == PRESENT_POSITION) return roundf(q[i]*4096/360);
    if (field == PRESENT_VELOCITY) return roundf(velocity[i]/(.229f*6));
    if (field == PRESENT_CURRENT) return roundf(current[i]);
    if (field == TORQUE_ENABLE) return enabled[i] ? 1 : 0;
    if (field == DRIVE_MODE) return driveMode;
    return mode;
  }
  int getLastLibErrCode() const { return error; }
  bool writeControlTableItem(uint8_t field, uint8_t id, int32_t value, uint32_t timeout) {
    assert(timeout == 10); ++writes; const int i = id-16;
    if (field == TORQUE_ENABLE) {
      if (value == 0 && failOff) return false;
      if (value == 1) { assert(caps[i] > 0 && goalTicks[i] > 0); }
      enabled[i] = value == 1;
    }
    if (field == GOAL_CURRENT) { assert(value >= 0 && value <= 80); caps[i] = value; }
    if (field == GOAL_POSITION) {
      if (failGoal) return false;
      ++goals; goalTicks[i] = value;
    }
    return true;
  }
};
struct NMLHandExo {
  Bus dxl_;
  AssistJoint assistJoints_[N_MOTORS];
  uint8_t assistState_ = 0, assistCursor_ = 0;
  unsigned long assistHeartbeatMs_ = 0, assistSampleMs_ = 0, assistBeginMs_ = 0;
  const char* assistReason_ = "off";
  int assistRejectId_ = -1;
  float assistRejectAngle_ = NAN, assistRejectVelocity_ = NAN;
  const uint8_t motorIds_[2] = {16, 17};
  int numMotors_ = 2;
  bool rom = false, otherCalibration = false;
  bool motorReachable_[2] = {true, true};
  std::string motorControlMode_ = "CURRENT_POSITION";
  bool torqueEnabled_[2] = {true, false}, positionHoldActive_[2] = {};
  bool directCommandActive_[2] = {}, motorMoving_[2] = {}, goalAngleValid_[2] = {};
  bool allocationDirty_ = false;
  uint16_t currentLimits_[2] = {80, 80}, totalCurrentBudgetMa_ = 80, appliedCurrents_[2] = {};
  float jointLimits_[2][2] = {{80, 120}, {80, 120}};
  JointStateModel jointModels_[2];
  ExoUtcClock utcClock_;
  float goalAngle_[2] = {};
  bool motorAdmitted_[2] = {};
  struct { void note(unsigned long) {} } telemetryGate_;
  int getIndexById(uint8_t id) { return id == 16 || id == 17 ? id-16 : -1; }
  bool isAssistBusy() const { return assistState_ != 0; }
  bool isRomCalibrating() const { return rom; }
  bool isExoCalibrating() const { return otherCalibration; }
  bool configureAssist(uint8_t, float, float, float);
  bool calibrateAssist(const uint8_t*, uint8_t);
  bool startAssist(); bool heartbeatAssist();
  void stopAssist(const char* reason = "stopped"); void serviceAssist();
  bool readAssistSample(uint8_t, float&, float&, float&, bool requireEnabled = true);
  bool writeAssistGoal(int);
  void run(uint32_t until, bool heartbeat = true) {
    while (nowMs < until) {
      nowMs += 5;
      if (heartbeat && nowMs%250 == 0 && assistState_ != 4) heartbeatAssist();
      serviceAssist();
    }
  }
  void ready() {
    nowMs = 0; const uint8_t ids[] = {16};
    assert(configureAssist(16, 10, .01f, 40));
    assert(calibrateAssist(ids, 1)); run(2000);
    assert(assistState_ == 2 && assistJoints_[0].samples == 50);
  }
};
// ASSIST_IMPLEMENTATION
int main() {
  {
    NMLHandExo e; const uint8_t id[] = {16};
    e.torqueEnabled_[0] = e.dxl_.enabled[0] = false;
    assert(e.configureAssist(16, 10, .01f, 40));
    assert(e.calibrateAssist(id, 1));
    assert(e.dxl_.enabled[0] && !e.dxl_.enabled[1]);
    assert(e.dxl_.caps[0] == 40 && e.dxl_.goals == 1);
    assert(fabsf(e.dxl_.goalTicks[0]*360.0f/4096 - 100) < .05f);
  }
  {
    NMLHandExo e; const uint8_t id[] = {16};
    e.dxl_.enabled[1] = true; // stale firmware cache must not hide another goal
    assert(e.configureAssist(16, 10, .01f, 40));
    assert(!e.calibrateAssist(id, 1));
    assert(e.assistRejectId_ == 17 && e.dxl_.writes == 0);
  }
  {
    NMLHandExo e; const uint8_t id[] = {16};
    e.torqueEnabled_[0] = e.dxl_.enabled[0] = false; e.dxl_.failGoal = true;
    assert(e.configureAssist(16, 10, .01f, 40));
    assert(!e.calibrateAssist(id, 1));
    assert(!e.dxl_.enabled[0] && e.assistState_ == 0);
  }
  {
    NMLHandExo e; const uint8_t id[] = {16};
    assert(!e.configureAssist(1, 10, .01f, 40));
    assert(!e.configureAssist(16, NAN, .01f, 40));
    assert(!e.configureAssist(16, 10, .01f, 100));
    assert(e.configureAssist(16, 10, .01f, 40));
    e.motorControlMode_ = "CURRENT"; assert(!e.calibrateAssist(id, 1));
    assert(std::string(e.assistReason_) == "requires_current_position" && e.assistRejectId_ == -1);
    e.motorControlMode_ = "CURRENT_POSITION"; e.rom = true; assert(!e.calibrateAssist(id, 1));
    e.rom = false; e.torqueEnabled_[1] = true; assert(!e.calibrateAssist(id, 1));
    assert(std::string(e.assistReason_) == "unselected_motor_enabled" && e.assistRejectId_ == 17);
    e.torqueEnabled_[1] = false; e.totalCurrentBudgetMa_ = 39; assert(!e.calibrateAssist(id, 1));
    assert(std::string(e.assistReason_) == "current_budget");
    e.totalCurrentBudgetMa_ = 80;
    for (int mode : {1, 4, 8}) { e.dxl_.driveMode = mode; assert(!e.calibrateAssist(id, 1)); }
    e.dxl_.driveMode = 0; e.dxl_.velocity[0] = 4.122f;
    assert(!e.calibrateAssist(id, 1));
    assert(std::string(e.assistReason_) == "not_settled" && e.assistRejectId_ == 16);
    assert(fabsf(e.assistRejectVelocity_ - 4.122f) < .001f);
    e.dxl_.velocity[0] = 0; e.dxl_.q[0] = 81;
    assert(!e.calibrateAssist(id, 1));
    assert(std::string(e.assistReason_) == "stored_limit" && e.assistRejectId_ == 16);
    e.torqueEnabled_[0] = false;
    assert(!e.calibrateAssist(id, 1));
    assert(std::string(e.assistReason_) == "stored_limit");
    assert(e.dxl_.writes == 0);
  }
  {
    NMLHandExo e; e.ready(); assert(e.dxl_.goals == 1);
    assert(!e.configureAssist(16, 15, .01f, 40));
    assert(e.startAssist()); e.run(3000);
    assert(e.dxl_.goals == 1); // constant bias alone never triggers
    e.dxl_.current[0] = -25; e.run(3300);
    assert(e.dxl_.goals == 1); // current without agreeing deflection never triggers
    e.dxl_.q[0] = 100.25f; e.run(3550);
    assert(e.dxl_.goals == 2 && e.assistJoints_[0].moving);
    assert(e.assistJoints_[0].goal > 100.25f && e.assistJoints_[0].goal <= 100.75f);
    e.dxl_.q[0] = e.assistJoints_[0].goal;
    e.run(4250); assert(e.dxl_.goals == 2); // our motion/current does not retrigger
    e.dxl_.current[0] = 5; e.run(4600); assert(e.dxl_.goals == 2);
    e.dxl_.q[0] = e.assistJoints_[0].goal-.25f; e.dxl_.current[0] = 35;
    e.run(4900); assert(e.dxl_.goals == 3 && e.assistJoints_[0].goal < e.dxl_.q[0]);
    e.stopAssist(); assert(e.assistState_ == 0 && !e.dxl_.enabled[0]);
    assert(!e.startAssist() && !e.heartbeatAssist());
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); e.run(3010, false);
    assert(e.assistState_ == 0 && std::string(e.assistReason_) == "heartbeat_timeout");
    assert(!e.heartbeatAssist());
  }
  {
    NMLHandExo e; e.ready(); nowMs += 1001;
    assert(!e.heartbeatAssist() && e.assistState_ == 0);
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); e.dxl_.failRead = true; e.run(2020);
    assert(e.assistState_ == 0 && !e.dxl_.enabled[0]);
  }
  {
    NMLHandExo e; e.ready(); e.dxl_.failOff = true; e.stopAssist();
    assert(e.assistState_ == 4 && e.isAssistBusy() && !e.heartbeatAssist());
    assert(e.torqueEnabled_[0]); // never falsely report acknowledged torque off
    e.dxl_.failOff = false; e.run(2200, false);
    assert(e.assistState_ == 0 && !e.dxl_.enabled[0]);
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); e.dxl_.q[0] = 106; e.run(2020);
    assert(e.assistState_ == 0 && std::string(e.assistReason_) == "bias_pose_changed");
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); e.dxl_.velocity[0] = 15; e.run(2020);
    assert(e.assistState_ == 0 && std::string(e.assistReason_) == "speed_limit");
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); e.dxl_.current[0] = 50; e.run(2020);
    assert(e.assistState_ == 0 && std::string(e.assistReason_) == "current_limit");
  }
  {
    NMLHandExo e; e.dxl_.current[0] = 35;
    nowMs = 0; uint8_t ids[] = {16}; e.configureAssist(16, 10, .01f, 40);
    assert(e.calibrateAssist(ids, 1)); e.run(2000);
    assert(e.assistState_ == 0 && std::string(e.assistReason_) == "insufficient_current_headroom");
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); nowMs += 600;
    e.heartbeatAssist(); e.serviceAssist();
    assert(e.assistState_ == 0 && std::string(e.assistReason_) == "feedback_stale");
  }
}
