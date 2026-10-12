#include "assist_controller.h"
#include "joint_state_model.h"
#include "utc_clock.h"
#include <cassert>
#include <string>
#include <vector>
#include <cstring>
using std::isfinite;
uint32_t nowMs = 0;
unsigned long millis() { return nowMs; }
constexpr int N_MOTORS = 18, PULSE_RESOLUTION = 4096, DXL_LIB_OK = 0;
constexpr float DIRECT_LIMIT_MARGIN_DEG = 2;
namespace DYNAMIXEL {
struct Packet { uint8_t* p_buf; unsigned buf_capacity; bool is_completed; };
struct XELInfoSyncRead_t { uint8_t id, error; uint8_t* p_recv_buf; };
struct XELInfoSyncWrite_t { uint8_t id; uint8_t* p_data; };
struct InfoSyncReadInst_t {
  Packet packet; uint16_t addr, addr_length;
  XELInfoSyncRead_t* p_xels; uint8_t xel_count; bool is_info_changed;
};
struct InfoSyncWriteInst_t {
  Packet packet; uint16_t addr, addr_length;
  XELInfoSyncWrite_t* p_xels; uint8_t xel_count; bool is_info_changed;
};
}
struct { int available() { return 0; } void read() { assert(false); } } DXL_SERIAL;
struct Bus {
  struct Operation { bool write; uint16_t address, length; std::vector<int> ids; };
  std::vector<Operation> operations;
  float q[N_MOTORS], current[N_MOTORS], velocity[N_MOTORS] = {};
  int error = 0, writes = 0, goals = 0, mode = 5, driveMode = 0;
  bool failRead = false, failOff = false, failGoal = false;
  bool failZero = false, failOn = false, badStatus = false, corruptCap = false;
  int partialAddress = -1, badStatusId = -1;
  uint32_t readDelay = 0;
  bool enabled[N_MOTORS] = {};
  bool modeWritten[N_MOTORS] = {};
  int goalTicks[N_MOTORS] = {}, caps[N_MOTORS] = {};
  uint8_t registers[N_MOTORS][136] = {};
  int pGain = -1, iGain = -1, dGain = -1, velocityProfile = -1, accelerationProfile = -1;
  Bus() {
    for (int i = 0; i < N_MOTORS; ++i) { q[i] = 100; current[i] = 5; }
    enabled[0] = true;
  }
  int index(uint8_t id) { return id <= 9 ? id-1 : id-11+9; }
  template<class Info> void record(bool write, Info* info) {
    Operation op{write, info->addr, info->addr_length, {}};
    for (int k = 0; k < info->xel_count; ++k) op.ids.push_back(info->p_xels[k].id);
    operations.push_back(op);
  }
  uint8_t syncRead(DYNAMIXEL::InfoSyncReadInst_t* info, uint32_t timeout) {
    assert(timeout == 10); record(false, info); nowMs += readDelay;
    error = failRead ? 1 : 0;
    if (failRead) return 0;
    const int count = info->addr == partialAddress ? info->xel_count-1 : info->xel_count;
    for (int k = 0; k < count; ++k) {
      const int i = index(info->p_xels[k].id);
      const int32_t ticks = roundf(q[i]*4096/360), speed = roundf(velocity[i]/(.229f*6));
      const int16_t amps = roundf(current[i]);
      registers[i][10] = driveMode;
      if (!modeWritten[i]) registers[i][11] = mode;
      registers[i][64] = enabled[i];
      memcpy(registers[i]+126, &amps, 2);
      memcpy(registers[i]+128, &speed, 4); memcpy(registers[i]+132, &ticks, 4);
      memcpy(info->p_xels[k].p_recv_buf, registers[i]+info->addr, info->addr_length);
      info->p_xels[k].error = badStatus || info->p_xels[k].id == badStatusId ? 1 : 0;
    }
    return count;
  }
  int getLastLibErrCode() const { return error; }
  bool syncWrite(DYNAMIXEL::InfoSyncWriteInst_t* info) {
    record(true, info); ++writes;
    if ((info->addr == 116 || info->addr == 108) && failGoal) return false;
    if (info->addr == 102 && failZero && info->p_xels[0].p_data[0] == 0) return false;
    for (int k = 0; k < info->xel_count; ++k) {
      const int i = index(info->p_xels[k].id);
      if (info->addr == 64) {
        const bool on = info->p_xels[k].p_data[0] == 1;
        if ((!on && failOff) || (on && failOn)) continue; // TX succeeds, motor ignores it
        if (on) assert(caps[i] > 0 && goalTicks[i] > 0);
        enabled[i] = on;
      }
      memcpy(registers[i]+info->addr, info->p_xels[k].p_data, info->addr_length);
      if (info->addr == 11) { assert(!enabled[i]); modeWritten[i] = true; }
      if (info->addr == 102) {
        uint16_t cap; memcpy(&cap, registers[i]+102, 2);
        assert(cap <= 910); caps[i] = cap;
        if (corruptCap) registers[i][102] ^= 1;
      }
      if (info->addr == 108 || info->addr == 116) {
        ++goals; memcpy(&goalTicks[i], registers[i]+116, 4);
      }
      if (info->addr == 80) {
        uint16_t p, ig, d; memcpy(&d, registers[i]+80, 2);
        memcpy(&ig, registers[i]+82, 2); memcpy(&p, registers[i]+84, 2);
        pGain = p; iGain = ig; dGain = d;
      }
      if (info->addr == 108) {
        memcpy(&accelerationProfile, registers[i]+108, 4);
        memcpy(&velocityProfile, registers[i]+112, 4);
      }
    }
    return true;
  }
};
struct NMLHandExo {
  Bus dxl_;
  AssistJoint assistJoints_[N_MOTORS];
  uint8_t assistState_ = 0;
  unsigned long assistHeartbeatMs_ = 0, assistSampleMs_ = 0, assistBeginMs_ = 0;
  AssistReason assistReason_ = ASSIST_REASON_OFF;
  bool assistAutoStart_ = false;
  int assistRejectId_ = -1;
  float assistRejectAngle_ = NAN, assistRejectVelocity_ = NAN;
  const uint8_t motorIds_[N_MOTORS] = {1,2,3,4,5,6,7,8,9,11,12,13,14,15,16,17,18,19};
  int numMotors_ = N_MOTORS;
  bool rom = false, otherCalibration = false;
  bool motorReachable_[N_MOTORS] = {};
  std::string motorControlMode_ = "CURRENT_POSITION";
  bool torqueEnabled_[N_MOTORS] = {}, positionHoldActive_[N_MOTORS] = {};
  bool directCommandActive_[N_MOTORS] = {}, motorMoving_[N_MOTORS] = {}, goalAngleValid_[N_MOTORS] = {};
  bool allocationDirty_ = false;
  uint16_t currentLimits_[N_MOTORS] = {}, totalCurrentBudgetMa_ = 80, appliedCurrents_[N_MOTORS] = {};
  float jointLimits_[N_MOTORS][2] = {};
  JointStateModel jointModels_[N_MOTORS];
  ExoUtcClock utcClock_;
  float goalAngle_[N_MOTORS] = {};
  bool motorAdmitted_[N_MOTORS] = {};
  struct { void note(unsigned long) {} } telemetryGate_;
  NMLHandExo() {
    for (int i = 0; i < N_MOTORS; ++i) {
      motorReachable_[i] = true; currentLimits_[i] = 80;
      jointLimits_[i][0] = 80; jointLimits_[i][1] = 120;
    }
    torqueEnabled_[0] = true;
  }
  int getIndexById(uint8_t id) {
    for (int i = 0; i < N_MOTORS; ++i) if (motorIds_[i] == id) return i;
    return -1;
  }
  bool isAssistBusy() const { return assistState_ != 0; }
  bool isRomCalibrating() const { return rom; }
  bool isExoCalibrating() const { return otherCalibration; }
  bool configureAssist(uint8_t, float, float, float);
  bool configureAssistObserver(uint8_t, float, float, float);
  bool calibrateAssist(const uint8_t*, uint8_t);
  bool enableTorqueAssist(const uint8_t*, uint8_t, float, float, float, float, float, float);
  bool startAssist(); bool heartbeatAssist();
  void stopAssist(AssistReason reason = ASSIST_REASON_STOPPED); void serviceAssist();
  bool readAssistRegisters(const bool*, uint16_t, uint16_t, AssistRegisterBlock*);
  bool writeAssistRegisters(const bool*, uint16_t, uint16_t, AssistRegisterBlock*, bool verify = false);
  bool readAssistFeedback(const bool*, AssistFeedback*);
  bool writeAssistGoals(const bool*);
  void getAssistSnapshot(AssistStatusSnapshot&) const;
  void run(uint32_t until, bool heartbeat = true) {
    while (nowMs < until) {
      nowMs += 5;
      if (heartbeat && nowMs%250 == 0 && assistState_ != 4) heartbeatAssist();
      serviceAssist();
    }
  }
  void ready() {
    nowMs = 0; const uint8_t ids[] = {1};
    assert(configureAssist(1, 10, .01f, 40));
    assert(calibrateAssist(ids, 1)); run(2000);
    assert(assistState_ == 2 && assistJoints_[0].samples == 50);
  }
  void readyFleet(int count) {
    nowMs = 0; totalCurrentBudgetMa_ = 40*count;
    for (int k = 0; k < count; ++k) assert(configureAssist(motorIds_[k], 10, .01f, 40));
    assert(calibrateAssist(motorIds_, count));
    assert(dxl_.operations.size() == 16); // fixed startup cost for 1, 9 or 18 motors
    run(2000);
    assert(assistState_ == 2);
    for (int k = 0; k < count; ++k) assert(assistJoints_[k].samples == 50);
  }
};
// ASSIST_IMPLEMENTATION
int main() {
  {
    // A right-only session transitions from CURRENT mode using grouped writes,
    // shares the existing budget, and activates without a separate START RPC.
    NMLHandExo e; nowMs = 0;
    const uint8_t ids[] = {11,12,13,14,15,16,17,18,19};
    e.motorControlMode_ = "CURRENT"; e.dxl_.mode = 0;
    e.torqueEnabled_[0] = e.dxl_.enabled[0] = false;
    e.totalCurrentBudgetMa_ = 800;
    for (int i = 9; i < 18; ++i) e.currentLimits_[i] = 910;
    assert(e.enableTorqueAssist(ids, 9, .08f/.00115f, 80*.00115f, 300, 0, 0, 0));
    assert(e.dxl_.operations.size() == 18); // fixed fleet cost, including verified mode change
    for (const auto& op : e.dxl_.operations) if (op.write)
      for (int id : op.ids) assert(id >= 11 && id <= 19);
    for (int i = 9; i < 18; ++i) assert(e.dxl_.caps[i] == 88 && e.dxl_.enabled[i]);
    assert(e.totalCurrentBudgetMa_ == 800);
    e.run(980); assert(e.assistState_ == 1);
    e.run(1000); assert(e.assistState_ == 3);
    AssistStatusSnapshot snapshot; e.getAssistSnapshot(snapshot);
    assert(snapshot.count == 9 && snapshot.total_current_budget_mA == 800);
    assert(snapshot.joints[0].current_cap_mA == 88);
    for (int i = 9; i < 18; ++i) e.dxl_.current[i] = -80;
    e.run(2100);
    for (int i = 9; i < 18; ++i) {
      assert(e.assistJoints_[i].moving && e.assistJoints_[i].goal > 100);
      assert(e.dxl_.caps[i] == 88); // effort changes goals, never the current cap
    }
    e.stopAssist(); assert(e.assistState_ == 0 && !e.assistAutoStart_);
    for (int i = 0; i < 18; ++i) assert(!e.dxl_.enabled[i]);
  }
  {
    // Insufficient shared headroom is rejected before touching any motor.
    NMLHandExo e; nowMs = 0;
    e.totalCurrentBudgetMa_ = 50;
    const uint8_t ids[] = {1};
    assert(!e.enableTorqueAssist(ids, 1, .08f/.00115f, 80*.00115f, 300, 0, 0, 0));
    assert(e.assistReason_ == ASSIST_REASON_INSUFFICIENT_CURRENT_HEADROOM && e.assistRejectId_ == 1);
    assert(e.dxl_.operations.empty());
  }
  {
    // Mode readback failures leave the selection torque-off and never activate.
    NMLHandExo e; nowMs = 0; const uint8_t ids[] = {1};
    e.totalCurrentBudgetMa_ = e.currentLimits_[0] = 300;
    e.motorControlMode_ = "CURRENT"; e.dxl_.mode = 0; e.dxl_.partialAddress = 11;
    assert(!e.enableTorqueAssist(ids, 1, 10, .01f, 300, 0, 0, 0));
    assert(e.assistState_ == 0 && !e.dxl_.enabled[0] && !e.assistAutoStart_);
  }
  {
    // Automatic startup still requires a lease, and ignores a separate late START.
    NMLHandExo e; nowMs = 0; const uint8_t ids[] = {1};
    e.totalCurrentBudgetMa_ = e.currentLimits_[0] = 300;
    assert(e.enableTorqueAssist(ids, 1, 10, .01f, 300, 0, 0, 0));
    e.run(1200, false);
    assert(e.assistState_ == 0 && !e.dxl_.enabled[0]);
    assert(!e.startAssist());
  }
  {
    NMLHandExo e; const uint8_t id[] = {1};
    e.torqueEnabled_[0] = e.dxl_.enabled[0] = false;
    assert(e.configureAssist(1, 10, .01f, 40));
    assert(e.calibrateAssist(id, 1));
    assert(e.dxl_.enabled[0] && !e.dxl_.enabled[1]);
    assert(e.dxl_.caps[0] == 40 && e.dxl_.goals == 1);
    assert(e.dxl_.pGain == 300 && e.dxl_.iGain == 0 && e.dxl_.dGain == 160);
    assert(e.dxl_.velocityProfile == 0 && e.dxl_.accelerationProfile == 0);
    assert(fabsf(e.dxl_.goalTicks[0]*360.0f/4096 - 100) < .05f);
  }
  {
    NMLHandExo e; const uint8_t id[] = {1};
    e.dxl_.enabled[1] = true; // stale firmware cache must not hide another goal
    assert(e.configureAssist(1, 10, .01f, 40));
    assert(!e.calibrateAssist(id, 1));
    assert(e.assistRejectId_ == 2 && e.dxl_.writes == 0);
  }
  {
    NMLHandExo e; const uint8_t id[] = {1};
    e.torqueEnabled_[0] = e.dxl_.enabled[0] = false; e.dxl_.failGoal = true;
    assert(e.configureAssist(1, 10, .01f, 40));
    assert(!e.calibrateAssist(id, 1));
    assert(!e.dxl_.enabled[0] && e.assistState_ == 0);
  }
  {
    NMLHandExo e; const uint8_t id[] = {1};
    assert(!e.configureAssist(10, 10, .01f, 40));
    assert(!e.configureAssist(1, NAN, .01f, 40));
    assert(!e.configureAssist(1, 10, .01f, 911));
    assert(e.configureAssist(1, 10, .01f, 40));
    e.motorControlMode_ = "CURRENT"; assert(!e.calibrateAssist(id, 1));
    assert(std::string(assistReasonName(e.assistReason_)) == "requires_current_position" && e.assistRejectId_ == -1);
    e.motorControlMode_ = "CURRENT_POSITION"; e.rom = true; assert(!e.calibrateAssist(id, 1));
    e.rom = false; e.torqueEnabled_[1] = true; assert(!e.calibrateAssist(id, 1));
    assert(std::string(assistReasonName(e.assistReason_)) == "unselected_motor_enabled" && e.assistRejectId_ == 2);
    e.torqueEnabled_[1] = false; e.totalCurrentBudgetMa_ = 39; assert(!e.calibrateAssist(id, 1));
    assert(std::string(assistReasonName(e.assistReason_)) == "current_budget");
    e.totalCurrentBudgetMa_ = 80;
    for (int mode : {1, 4, 8}) { e.dxl_.driveMode = mode; assert(!e.calibrateAssist(id, 1)); }
    e.dxl_.driveMode = 0; e.dxl_.velocity[0] = 4.122f;
    assert(!e.calibrateAssist(id, 1));
    assert(std::string(assistReasonName(e.assistReason_)) == "not_settled" && e.assistRejectId_ == 1);
    assert(fabsf(e.assistRejectVelocity_ - 4.122f) < .001f);
    e.dxl_.velocity[0] = 0; e.dxl_.q[0] = 81;
    assert(!e.calibrateAssist(id, 1));
    assert(std::string(assistReasonName(e.assistReason_)) == "stored_limit" && e.assistRejectId_ == 1);
    e.torqueEnabled_[0] = false;
    assert(!e.calibrateAssist(id, 1));
    assert(std::string(assistReasonName(e.assistReason_)) == "stored_limit");
    assert(e.dxl_.writes == 0);
  }
  {
    NMLHandExo e; e.ready(); assert(e.dxl_.goals == 1);
    assert(!e.configureAssist(1, 15, .01f, 40));
    assert(e.startAssist()); e.run(3000);
    assert(e.dxl_.goals == 1); // constant bias alone never triggers
    e.dxl_.current[0] = -25; e.run(3500);
    assert(e.dxl_.goals == 1); // observer + sustained detection dwell
    e.run(3750);
    assert(e.dxl_.goals > 1 && e.assistJoints_[0].moving);
    assert(e.assistJoints_[0].secondaryGoal > 100);
    assert(e.dxl_.caps[0] == 40); // current cap does not track effort
    e.run(4500); // stationary environment releases and latches
    assert(!e.assistJoints_[0].moving && e.assistJoints_[0].rearmBlocked);
    assert(fabsf(e.assistJoints_[0].goal-100) < .05f);
    const int stoppedGoals = e.dxl_.goals;
    e.run(5200); assert(e.dxl_.goals == stoppedGoals);
    e.dxl_.current[0] = 5; e.run(6400);
    assert(!e.assistJoints_[0].rearmBlocked);
    e.dxl_.current[0] = 35; e.run(7200);
    assert(e.assistJoints_[0].moving && e.assistJoints_[0].goal < 100);
    e.stopAssist(); assert(e.assistState_ == 0 && !e.dxl_.enabled[0]);
    assert(!e.startAssist() && !e.heartbeatAssist());
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); e.run(3010, false);
    assert(e.assistState_ == 0 && std::string(assistReasonName(e.assistReason_)) == "heartbeat_timeout");
    assert(!e.heartbeatAssist());
  }
  {
    NMLHandExo e; e.ready(); nowMs += 1001;
    assert(!e.heartbeatAssist() && e.assistState_ == 0);
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); e.dxl_.failRead = true; e.run(2020);
    assert(e.assistState_ == 4 && !e.dxl_.enabled[0]);
  }
  {
    NMLHandExo e; e.ready(); e.dxl_.failOff = true; e.stopAssist();
    assert(e.assistState_ == 4 && e.isAssistBusy() && !e.heartbeatAssist());
    assert(e.torqueEnabled_[0]); // never falsely report acknowledged torque off
    e.dxl_.failOff = false; e.run(2200, false);
    assert(e.assistState_ == 0 && !e.dxl_.enabled[0]);
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); e.dxl_.q[0] = 119; e.run(2020);
    assert(e.assistState_ == 0 && std::string(assistReasonName(e.assistReason_)) == "stored_limit");
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); e.dxl_.velocity[0] = 65; e.run(2020);
    assert(e.assistState_ == 0 && std::string(assistReasonName(e.assistReason_)) == "speed_limit");
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); e.dxl_.current[0] = 50; e.run(2020);
    assert(e.assistState_ == 0 && std::string(assistReasonName(e.assistReason_)) == "current_limit");
  }
  {
    NMLHandExo e; e.dxl_.current[0] = 35;
    nowMs = 0; uint8_t ids[] = {1}; e.configureAssist(1, 10, .01f, 40);
    assert(e.calibrateAssist(ids, 1)); e.run(2000);
    assert(e.assistState_ == 0 && std::string(assistReasonName(e.assistReason_)) == "insufficient_current_headroom");
  }
  {
    NMLHandExo e; e.ready(); e.startAssist(); nowMs += 600;
    e.heartbeatAssist(); e.serviceAssist();
    assert(e.assistState_ == 0 && std::string(assistReasonName(e.assistReason_)) == "feedback_stale");
  }
  {
    NMLHandExo e; e.ready();
    auto& j = e.assistJoints_[0];
    j.secondaryGoal = j.hardMax; j.direction = 1; j.lead = 8;
    j.goal = j.reference();
    bool chosen[N_MOTORS] = {}; chosen[0] = true;
    assert(e.writeAssistGoals(chosen));
    assert(e.dxl_.goalTicks[0]*360.0f/4096 <= 118);
    j.secondaryGoal = j.hardMin; j.direction = -1;
    j.goal = j.reference();
    assert(e.writeAssistGoals(chosen));
    assert(e.dxl_.goalTicks[0]*360.0f/4096 >= 82);
    const int goals = e.dxl_.goals;
    j.goal = NAN; assert(!e.writeAssistGoals(chosen));
    j.goal = 81.99f; assert(!e.writeAssistGoals(chosen));
    assert(e.dxl_.goals == goals);
    assert(!e.configureAssistObserver(1, 0, 0, 0)); // busy setter rejected
  }
  {
    NMLHandExo e;
    assert(e.configureAssistObserver(1, .035f, .018f, .025f));
    assert(!e.configureAssistObserver(10, 0, 0, 0));
    assert(!e.configureAssistObserver(1, NAN, 0, 0));
    assert(e.dxl_.writes == 0); // configuration never actuates
    assert(fabsf(e.assistJoints_[0].inertia-.035f) < 1e-6f);
  }
  for (int count : {1, 9, 18}) {
    NMLHandExo e; e.readyFleet(count); assert(e.startAssist());
    for (int i = 0; i < count; ++i) {
      auto& j = e.assistJoints_[i];
      j.moving = true; j.effort = j.estimate = 30; j.bias = 5;
      e.dxl_.current[i] = -25; e.dxl_.velocity[i] = 2.748f;
    }
    e.dxl_.operations.clear(); nowMs += 20; e.serviceAssist();
    assert(e.assistState_ == 3 && e.dxl_.operations.size() == 3);
    const auto& ops = e.dxl_.operations;
    assert(!ops[0].write && ops[0].address == 126 && ops[0].length == 10);
    assert(!ops[1].write && ops[1].address == 64 && ops[1].length == 1);
    assert(ops[2].write && ops[2].address == 116 && ops[2].length == 4);
    for (const auto& op : ops) {
      assert(op.ids.size() == (unsigned)count);
      for (int i = 0; i < count; ++i) assert(op.ids[i] == e.motorIds_[i]);
    }
    for (int i = 0; i < count; ++i) {
      assert(e.assistJoints_[i].lastSample == nowMs);
      assert(e.assistJoints_[i].measuredValid && e.assistJoints_[i].goal > 100);
    }
    e.dxl_.operations.clear(); e.stopAssist();
    assert(e.assistState_ == 0 && e.dxl_.operations.size() == 3);
    assert(e.dxl_.operations[0].write && e.dxl_.operations[0].address == 102);
    assert(e.dxl_.operations[1].write && e.dxl_.operations[1].address == 64);
    assert(!e.dxl_.operations[2].write && e.dxl_.operations[2].address == 64);
    for (const auto& op : e.dxl_.operations) assert(op.ids.size() == (unsigned)count);
  }
  for (int address : {126, 64}) {
    NMLHandExo e; e.readyFleet(18); e.startAssist();
    e.dxl_.partialAddress = address; e.dxl_.operations.clear();
    nowMs += 20; e.serviceAssist();
    assert(!e.assistJoints_[17].measuredValid);
    assert(e.assistState_ == (address == 64 ? 4 : 0));
    for (const auto& op : e.dxl_.operations) {
      assert(op.ids.size() == 18);
      assert(!(op.write && op.address == 116)); // no partial fleet goal update
    }
    e.dxl_.partialAddress = -1; e.run(nowMs+120, false);
    assert(e.assistState_ == 0);
  }
  {
    NMLHandExo e; e.readyFleet(18); e.startAssist();
    e.dxl_.badStatusId = 19; e.dxl_.operations.clear();
    nowMs += 20; e.serviceAssist();
    assert(e.assistState_ == 4 && !e.assistJoints_[17].measuredValid);
    for (const auto& op : e.dxl_.operations) assert(!(op.write && op.address == 116));
  }
  {
    NMLHandExo e; e.readyFleet(18); e.startAssist();
    e.dxl_.q[17] = 119; e.dxl_.operations.clear();
    nowMs += 20; e.serviceAssist();
    assert(e.assistState_ == 0 && std::string(assistReasonName(e.assistReason_)) == "stored_limit");
    for (const auto& op : e.dxl_.operations) assert(!(op.write && op.address == 116));
  }
  {
    NMLHandExo e; e.readyFleet(18);
    bool chosen[N_MOTORS] = {};
    for (int i = 0; i < N_MOTORS; ++i) { chosen[i] = true; e.assistJoints_[i].goal = 101; }
    e.assistJoints_[17].goal = NAN; e.dxl_.operations.clear();
    assert(!e.writeAssistGoals(chosen) && e.dxl_.operations.empty());
  }
  {
    NMLHandExo e; const uint8_t ids[] = {11,16,19}; nowMs = 0;
    e.torqueEnabled_[0] = e.dxl_.enabled[0] = false;
    e.totalCurrentBudgetMa_ = 120;
    for (auto id : ids) assert(e.configureAssist(id, 10, .01f, 40));
    assert(e.calibrateAssist(ids, 3)); e.run(2000);
    for (const auto& op : e.dxl_.operations) if (op.write)
      assert((op.ids == std::vector<int>{11,16,19})); // inactive side never written
    assert(!e.dxl_.enabled[0] && e.dxl_.enabled[9] && e.dxl_.enabled[14] && e.dxl_.enabled[17]);
  }
  {
    NMLHandExo e; const uint8_t id[] = {1}; nowMs = 0;
    e.configureAssist(1, 10, .01f, 40); e.dxl_.corruptCap = true;
    assert(!e.calibrateAssist(id, 1));
    assert(e.assistState_ == 0 && !e.dxl_.enabled[0]); // do not enable on bad readback
  }
  {
    NMLHandExo e; const uint8_t id[] = {1}; nowMs = 0;
    e.configureAssist(1, 10, .01f, 40); e.dxl_.failOn = true;
    assert(!e.calibrateAssist(id, 1));
    assert(e.assistState_ == 0 && !e.dxl_.enabled[0]);
  }
  {
    NMLHandExo e; e.readyFleet(18); e.dxl_.failZero = true;
    e.stopAssist(); assert(e.assistState_ == 0); // torque-off still attempted independently
    for (int i = 0; i < N_MOTORS; ++i) assert(!e.dxl_.enabled[i]);
  }
  {
    NMLHandExo e; e.readyFleet(18); e.startAssist();
    e.assistJoints_[17].effort = -21; e.assistJoints_[17].secondaryGoal = 101.25f;
    e.dxl_.operations.clear();
    AssistStatusSnapshot snapshot; e.getAssistSnapshot(snapshot);
    assert(e.dxl_.operations.empty()); // status never adds a bus transaction
    assert(snapshot.state == 3 && snapshot.reason == ASSIST_REASON_ACTIVE && snapshot.count == 18);
    assert(snapshot.joints[17].id == 19 && snapshot.joints[17].effort_mA == -21);
    assert(snapshot.joints[17].secondary_goal == 101.25f && snapshot.joints[17].samples == 50);
    assert(snapshot.joints[17].age_ms == nowMs-e.assistJoints_[17].lastSample);
    nowMs += 990; e.dxl_.readDelay = 10; e.dxl_.operations.clear(); e.serviceAssist();
    assert(e.assistState_ == 0 && std::string(assistReasonName(e.assistReason_)) == "heartbeat_timeout");
    for (const auto& op : e.dxl_.operations) assert(!(op.write && op.address == 116));
  }
}
