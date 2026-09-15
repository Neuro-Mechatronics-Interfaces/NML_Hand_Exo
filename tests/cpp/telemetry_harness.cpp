#include "joint_state_model.h"
#include "assist_controller.h"
#include "utc_clock.h"
#include <cassert>
#include <cstring>
#include <algorithm>
#include <string>
using std::min;
uint32_t testNow = 0;
uint32_t millis() { return testNow; }
const int N_MOTORS = 3, PULSE_RESOLUTION = 4096;
const int DYNAMIXEL_PRESENT_BLOCK_LENGTH = 10, DYNAMIXEL_PRESENT_BLOCK_ADDRESS = 126;
const int DYNAMIXEL_PRESENT_CURRENT_OFFSET = 0, DYNAMIXEL_PRESENT_VELOCITY_OFFSET = 2;
const int DYNAMIXEL_PRESENT_POSITION_OFFSET = 6;
const float XC330_T288_TORQUE_CONSTANT = 0.00115f;
enum {FAST_TELEM_METHOD_FAILED, FAST_TELEM_METHOD_FALLBACK_READ,
      FAST_TELEM_METHOD_FAST_SYNC_READ, FAST_TELEM_METHOD_SYNC_READ, FAST_TELEM_METHOD_MODEL, FAST_TELEM_METHOD_CONTROL_CACHE};
enum {ROM_CAL_IDLE, ROM_CAL_RAMP, ROM_CAL_RETURN, ROM_CAL_DONE};
// RECORD_DECLARATIONS
namespace DYNAMIXEL {
struct XELInfoSyncRead_t { uint8_t id, error; uint8_t* p_recv_buf; };
struct InfoSyncReadInst_t {
  struct { uint8_t* p_buf; unsigned buf_capacity; } packet;
  unsigned addr, addr_length;
  XELInfoSyncRead_t* p_xels;
  uint8_t xel_count;
  bool is_info_changed;
};
}
struct { int available() { return 0; } void read() { assert(false); } } DXL_SERIAL;
struct Bus {
  int reads = 0;
  bool partial = false, badStatus = false;
  uint8_t syncRead(DYNAMIXEL::InfoSyncReadInst_t* info, uint32_t) {
    ++reads;
    assert(info->addr == 126 && info->addr_length == 10);
    for (unsigned i = 0; i < info->xel_count; ++i) {
      assert(info->p_xels[i].id != 19); // offline motor never enters packet
      if (partial && i) break;
      const int16_t current = -37;
      const int32_t velocity = -2, position = 2048;
      memcpy(info->p_xels[i].p_recv_buf, &current, 2);
      memcpy(info->p_xels[i].p_recv_buf + 2, &velocity, 4);
      memcpy(info->p_xels[i].p_recv_buf + 6, &position, 4);
      info->p_xels[i].error = badStatus ? 1 : 0;
    }
    return partial ? 1 : info->xel_count;
  }
};
struct NMLHandExo {
  Bus dxl_;
  bool assistBusy = false;
  AssistJoint assistJoints_[3];
  bool isAssistBusy() const { return assistBusy; }
  const int numMotors_ = N_MOTORS;
  uint8_t romCalPhase_ = ROM_CAL_IDLE;
  std::string motorControlMode_ = "CURRENT_POSITION";
  float jointLimits_[3][2] = {{-189,2840},{0,360},{0,360}};
  float zeroOffsets_[3] = {0, 10, 0}, goalAngle_[3] = {300, 200, 0};
  float directCommandDirection_[3] = {};
  uint16_t appliedCurrents_[3] = {100,100,0};
  bool motorReachable_[3] = {true,true,false}, torqueEnabled_[3] = {true,true,false};
  bool motorMoving_[3] = {}, directCommandActive_[3] = {}, positionHoldActive_[3] = {};
  bool goalAngleValid_[3] = {true,true,false};
  bool motorAdmitted_[3] = {true,true,false}, flipMotor_[3] = {false,true,false};
  JointStateModel jointModels_[3];
  JointTelemetryGate telemetryGate_;
  ExoUtcClock utcClock_;
  int getIndexById(uint8_t id) { return id == 1 ? 0 : id == 11 ? 1 : id == 19 ? 2 : -1; }
  bool modeControlsPosition() const { return motorControlMode_ == "CURRENT_POSITION"; }
  bool modeControlsCurrent() const { return motorControlMode_ == "CURRENT_POSITION" || motorControlMode_ == "CURRENT"; }
  bool getFastTelemetryRecord(uint8_t, FastTelemetryRecord&);
  uint8_t getFastTelemetryRecords(const uint8_t*, uint8_t, FastTelemetryRecord*, uint8_t&, uint32_t);
  bool motionActive() const;
  bool telemetryEstimated();
  void serviceJointModels();
  void fillModelRecord(int, FastTelemetryRecord&);
};
// FIRMWARE_IMPLEMENTATION
int main() {
  NMLHandExo exo;
  const uint8_t ids[] = {1,11,19,99};
  FastTelemetryRecord records[4];
  uint8_t method;
  auto sample = [&]() { exo.getFastTelemetryRecords(ids, 4, records, method, 10); };
  sample();
  assert(exo.dxl_.reads == 1 && method == FAST_TELEM_METHOD_SYNC_READ);
  assert(records[0].sources == 0x15 && records[0].utc_ms == 0);
  assert(records[1].relative_cdeg == -17000);
  assert(records[2].error && records[3].error);
  assert(exo.jointModels_[0].angle == 180);
  exo.utcClock_.set(1789473600123ULL, testNow);
  exo.motorMoving_[0] = true;
  exo.goalAngle_[1] = 180; // this joint is already homed; another is travelling
  testNow = 100;
  sample();
  assert(exo.dxl_.reads == 1 && method == (4 | 0x80));
  assert(records[0].sources == 0x2a && records[0].absolute_cdeg > 18000);
  assert(records[1].sources == 0x2a); // idle hand shares the bus gate
  assert(records[1].current_mA == 0 && records[0].current_mA == 100);
  assert(records[0].utc_ms == 1789473600223ULL && records[0].sample_ms == 100);
  exo.motorMoving_[0] = false;
  testNow = 349;
  sample();
  assert(exo.dxl_.reads == 1);
  testNow = 350;
  sample();
  assert(exo.dxl_.reads == 2 && records[0].sources == 0x15);
  assert(records[1].sources == 0x15 && records[1].current_mA == -37);
  assert(exo.jointModels_[0].angle == 180); // fresh correction, never goal snap
  exo.dxl_.partial = true;
  sample();
  assert(exo.dxl_.reads == 3); // no serial fallback storm
  assert(records[0].error && records[1].error && records[1].sources == 0);
  exo.dxl_.partial = false;
  exo.dxl_.badStatus = true;
  sample();
  assert(records[0].error && records[1].error);
  exo.dxl_.badStatus = false;
  exo.motorControlMode_ = "VELOCITY";
  exo.directCommandActive_[1] = true;
  exo.directCommandDirection_[1] = -1;
  testNow += 100; // first-order ramp must exceed the raw velocity quantization
  sample();
  assert(exo.dxl_.reads == 4 && records[1].sources == 0x22);
  assert(records[1].velocity_raw < 0);
  exo.directCommandActive_[1] = false;
  exo.romCalPhase_ = ROM_CAL_RAMP;
  testNow += 1000;
  sample();
  assert(exo.dxl_.reads == 4);
  exo.assistBusy = true;
  auto& assist = exo.assistJoints_[1];
  assist.begin(150, testNow);
  bool write = false;
  assist.sample(150, 0, -11, testNow, true, false, write);
  assist.measuredUtc = exo.utcClock_.now(testNow);
  const auto stamp = testNow;
  const auto reads = exo.dxl_.reads;
  testNow += 100;
  sample();
  assert(exo.dxl_.reads == reads && method == FAST_TELEM_METHOD_CONTROL_CACHE);
  assert(records[1].current_mA == -11 && records[1].absolute_cdeg == 15000);
  assert(records[1].sources == 0x15 && records[1].sample_ms == stamp);
  assert(records[1].utc_ms == assist.measuredUtc);
  assert(records[0].error); // unselected IDs are unavailable, not fabricated
  testNow += 501; sample(); assert(records[1].error);

}
