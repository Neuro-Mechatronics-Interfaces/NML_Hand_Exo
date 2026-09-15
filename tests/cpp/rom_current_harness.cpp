#include "rom_pulse_fit.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
using std::min;
using std::isfinite;
uint32_t testNow = 0;
unsigned long millis() { return testNow; }
float constrain(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }
constexpr float DIRECT_CURRENT_LIMIT_MA = 910;
constexpr float DIRECT_LIMIT_MARGIN_DEG = 2;
constexpr float GESTURE_REACH_TOLERANCE_DEG = 2;
constexpr float XC330_T288_TORQUE_CONSTANT = 0.00115f;
constexpr int PULSE_RESOLUTION = 4096;
enum { DXL_LIB_OK, UNIT_DEGREE, UNIT_RPM, GOAL_CURRENT, GOAL_VELOCITY, PRESENT_POSITION, PRESENT_VELOCITY };
enum { ROM_CAL_IDLE, ROM_CAL_RAMP, ROM_CAL_REST, ROM_CAL_RETURN, ROM_CAL_DONE };
enum { ROM_CAL_DIR_FLEX, ROM_CAL_DIR_EXTEND };
enum { ROM_CAL_STATUS_NONE, ROM_CAL_STATUS_OK, ROM_CAL_STATUS_CEILING,
       ROM_CAL_STATUS_TIMEOUT, ROM_CAL_STATUS_ABORTED, ROM_CAL_STATUS_LIMIT };
// ROM_CONFIG
struct Bus {
  float position = 100, velocity = 0, plantGain = 0, endstop = 1000;
  int error = DXL_LIB_OK, command = 0;
  bool failVelocity = false, failWrite = false;
  std::vector<int> writes;
  std::vector<uint32_t> times;
  float getPresentPosition(uint8_t, int) { return position; }
  float getPresentVelocity(uint8_t, int) { return failVelocity ? NAN : velocity / 6; }
  int getLastLibErrCode() { return error; }
  int32_t readControlTableItem(uint8_t item, uint8_t, uint32_t timeout) {
    assert(timeout == 10);
    if (item == PRESENT_POSITION) return (int32_t)round(position * 4096 / 360);
    if (failVelocity) { error = 1; return 0; }
    return (int32_t)round(velocity / (0.229f * 6.0f));
  }
  bool writeControlTableItem(int, uint8_t, int32_t current, uint32_t timeout = 100) {
    assert(timeout == 10 || timeout == 100);
    assert(std::abs(current) <= ROM_CAL_MAX_CURRENT_MA);
    if (failWrite) return false;
    writes.push_back(current); times.push_back(testNow); command = current;
    return true;
  }
  void step(float dt) {
    const float desired = plantGain * command;
    const float decay = expf(-dt / 0.15f);
    position += desired * dt + (velocity - desired) * 0.15f * (1 - decay);
    velocity = desired + (velocity - desired) * decay;
    if (position >= endstop) { position = endstop; velocity = 0; }
  }
};
struct NMLHandExo {
  Bus dxl_;
  int romCalIndex_ = 0, romCalPhase_ = ROM_CAL_IDLE, romCalStatus_ = ROM_CAL_STATUS_NONE;
  uint8_t romCalId_ = 16, romCalDir_ = ROM_CAL_DIR_FLEX;
  std::string motorControlMode_ = "CURRENT";
  unsigned long romCalReturnStartMs_ = 0, romCalStartMs_ = 0, romCalLastStepMs_ = 0,
                romCalPulseStartMs_ = 0, romCalRestStartMs_ = 0, romCalPulseOnMs_ = 0;
  float romCalHomeAngle_ = 100, romCalLastAngle_ = 100, romCalEndstopAngle_ = 100,
        romCalCurrentMa_ = ROM_CAL_START_CURRENT_MA, romCalSign_ = 1,
        romCalPulseStartAngle_ = 100, romCalPulseStartVelocity_ = 0,
        romCalPulsePeakAngle_ = 100;
  uint16_t romCalPulseCount_ = 0, romCalFitSamples_ = 0;
  uint8_t romCalStallPulses_ = 0;
  bool romCalDirLocked_ = false, romCalSignReversed_ = false, romCalStopRequested_ = false;
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
  float romCalPredictedDeg_ = NAN, romCalObservedDeg_ = NAN;
  const char* romCalResponseReason_ = "no_completed_pulse";
  void romCalReportPulse() { assert(dxl_.command == 0); }
  float zeroOffsets_[1] = {0};
  bool flipMotor_[1] = {false}, directCommandActive_[1] = {false}, torqueEnabled_[1] = {true};
  bool positionHoldActive_[1] = {false};
  int numMotors_ = 1, directVelocityLimitBlock_[1] = {0};
  uint8_t motorIds_[1] = {16};
  unsigned long directCommandTimeoutMs_ = 250;
  unsigned long lastDirectCommandMs_[1] = {0};
  float directCommandDirection_[1] = {0}, jointLimits_[1][2] = {{-500, 500}};
  JointStateModel jointModels_[1];
  struct { void note(unsigned long) {} } telemetryGate_;
  void serviceJointModels() {}
  int getIndexById(uint8_t id) { return id == 16 ? 0 : -1; }
  float getAbsoluteAngle(uint8_t) { return dxl_.position; }
  void stopDirectControl(uint8_t);
  void serviceDirectControlSafety();
  float getGestureSpan(uint8_t) { return 100; }
  void romCalFinish() {
    writeRomSweepCurrent(0); torqueEnabled_[0] = false; romCalPhase_ = ROM_CAL_IDLE;
  }
  void romCalBeginReturnHome();
  void holdRelativePosition(uint8_t, float, uint16_t) { assert(false && "unexpected return motion"); }
  bool romCalCheckTravel(float);
  void romCalEndPulse(const char*);
  void serviceRomCalibration();
  bool writeRomSweepCurrent(float);
  bool romCalReadFeedback(float&, float&);
  bool romCalReadPosition(float&);
  void romCalRecoverFeedback();
  bool romCalStartPulse(float, float);
  void romCalCompletePulse(float, float);
  void arm() { testNow = 0; romCalStartPulse(dxl_.position, dxl_.velocity); }
  void run(unsigned long until) {
    while (testNow < until && (romCalPhase_ == ROM_CAL_RAMP || romCalPhase_ == ROM_CAL_REST)) {
      dxl_.step(0.005f); testNow += 5;
      serviceDirectControlSafety(); serviceRomCalibration();
    }
  }
};
// ROM_IMPLEMENTATION
// ROM_WRITER
// ROM_STOP
// ROM_SAFETY
int main() {
  assert(ROM_CAL_START_CURRENT_MA == 20 && ROM_CAL_MAX_CURRENT_MA == 80);
  assert(ROM_CAL_PULSE_ON_MS == 20 && ROM_CAL_PULSE_OFF_MS == 400);
  {
    NMLHandExo exo; exo.arm(); exo.run(20000);
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_CEILING && exo.romCalPulseCount_ == 31);
    assert(exo.romCalFitSamples_ == 0 && exo.dxl_.command == 0);
    int lastAmplitude = 18;
    for (size_t i = 0; i < exo.dxl_.writes.size(); ++i) {
      if (exo.dxl_.writes[i] == 0) continue;
      assert(exo.dxl_.writes[i] == lastAmplitude + 2);
      lastAmplitude = exo.dxl_.writes[i];
      assert(exo.dxl_.writes[i+1] == 0);
      assert(exo.dxl_.times[i+1] - exo.dxl_.times[i] == 20);
      if (i) assert(exo.dxl_.times[i] - exo.dxl_.times[i-1] >= 400);
    }
  }
  {
    NMLHandExo exo; exo.dxl_.plantGain = 1.0f; exo.dxl_.endstop = 115;
    exo.arm(); exo.run(40000);
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_OK);
    assert(fabsf(exo.romCalEndstopAngle_ - 115) < 0.1f && exo.romCalFitSamples_ >= 3);
    assert(exo.jointModels_[0].params.gain > 0.1f && exo.jointModels_[0].params.gain <= 1.1f);
    assert(exo.dxl_.command == 0);
  }
  {
    NMLHandExo exo; exo.arm(); exo.dxl_.error = 1; exo.run(2000);
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED && exo.dxl_.command == 0);
    assert(exo.romCalFitSamples_ == 0);
    assert(exo.romCalRecoveries_ == 3 && std::string(exo.romCalReason_) == "position_read");
  }
  {
    NMLHandExo exo; exo.arm(); exo.dxl_.error = 1; exo.run(5);
    assert(exo.romCalPhase_ == ROM_CAL_REST && exo.dxl_.command == 0);
    exo.dxl_.error = 0; exo.run(400);
    assert(exo.dxl_.command == 0 && exo.romCalPulseCount_ == 0 && exo.romCalFitSamples_ == 0);
    exo.run(405);
    assert(exo.dxl_.command == 20 && exo.romCalCurrentMa_ == 20);
    exo.run(20000);
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_CEILING && exo.romCalRecoveries_ == 1);
  }
  {
    NMLHandExo exo; exo.arm(); exo.run(20);
    assert(exo.romCalPhase_ == ROM_CAL_REST && exo.dxl_.command == 0);
    exo.stopDirectControl(16); exo.run(1000);
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED && exo.dxl_.command == 0);
  }
  {
    NMLHandExo exo; exo.arm(); exo.torqueEnabled_[0] = false; exo.run(1000);
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED && exo.dxl_.command == 0);
  }
  {
    NMLHandExo exo; exo.arm(); exo.run(20);
    exo.writeRomSweepCurrent(60); // competing current command during rest
    exo.run(1000);
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED && exo.dxl_.command == 0);
  }
  {
    NMLHandExo exo; exo.arm(); exo.dxl_.position = 499; exo.run(5);
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED && exo.dxl_.command == 0);
    assert(exo.romCalFitSamples_ == 0);
  }
  {
    NMLHandExo exo; exo.dxl_.position = 499; exo.arm();
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_LIMIT && exo.dxl_.command == 0);
  }
  {
    NMLHandExo exo; exo.arm(); testNow = 2000; exo.serviceRomCalibration();
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED && exo.dxl_.command == 0);
    exo.writeRomSweepCurrent(900); assert(exo.dxl_.command == 80);
    exo.writeRomSweepCurrent(-900); assert(exo.dxl_.command == -80);
    exo.flipMotor_[0] = true; exo.writeRomSweepCurrent(900); assert(exo.dxl_.command == -80);
    assert(!exo.writeRomSweepCurrent(NAN));
  }
  {
    NMLHandExo exo; exo.arm(); testNow = ROM_CAL_TIMEOUT_MS; exo.serviceRomCalibration();
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_TIMEOUT && exo.dxl_.command == 0);
  }
  {
    NMLHandExo exo; exo.arm(); testNow = 260;
    exo.serviceDirectControlSafety(); exo.serviceRomCalibration();
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED && exo.dxl_.command == 0);
  }
  {
    JointModelParams p;
    const float on = .15f, off = .2f, current = 80, gain = .2f, v0 = 2;
    const float a = expf(-on/p.time_constant), b = expf(-off/p.time_constant);
    const float peak = v0*a + gain*current*(1-a);
    const float delta = v0*p.time_constant*(1-a*b) + gain*current*(on-p.time_constant*(1-a)*b);
    assert(fabsf(romPulseGain(p, delta, v0, peak*b, current, on, off, .3f)-gain) < .0001f);
    assert(fabsf(romPulseGain(p, -delta, -v0, -peak*b, -current, on, off, .3f)-gain) < .0001f);
    assert(isnan(romPulseGain(p, 0, 0, 0, current, on, off, .3f)));
    assert(isnan(romPulseGain(p, -delta, 0, 0, current, on, off, .3f)));
    assert(isnan(romPulseGain(p, delta, NAN, 0, current, on, off, .3f)));
    p.moment = .1f;
    assert(isnan(romPulseGain(p, delta, v0, peak*b, current, on, off, .3f)));
    p.moment = 0; p.max_velocity = 10;
    assert(isnan(romPulseGain(p, delta, v0, peak*b, current, on, off, .3f)));
    const char* reason = nullptr;
    assert(isnan(romPulseGain(p, delta, v0, peak*b, current, on, off, .3f, &reason)));
    assert(std::string(reason) == "velocity_limit");
    p.max_velocity = 60;
    assert(isnan(romPulseGain(p, delta, NAN, 0, current, on, off, .3f, &reason)));
    assert(std::string(reason) == "velocity_unavailable");
  }
  {
    // Early measured-travel cutoff and smaller next pulse, without learning a
    // false fixed-duration response from the truncated input.
    NMLHandExo exo; exo.arm(); exo.dxl_.position = 100.6f;
    testNow = 5; exo.serviceRomCalibration();
    assert(exo.dxl_.command == 0 && exo.romCalPhase_ == ROM_CAL_REST);
    assert(exo.romCalPulseOnMs_ == 5 && exo.romCalPulseLimited_);
    exo.run(405);
    assert(exo.romCalCurrentMa_ == 18);
    assert(exo.jointModels_[0].pulseResponses[0].movingSamples == 0);
  }
  {
    NMLHandExo exo; exo.romCalCurrentMa_ = ROM_CAL_MIN_CURRENT_MA;
    exo.arm(); exo.dxl_.position = 100.6f; testNow = 5; exo.serviceRomCalibration();
    exo.run(405);
    assert(exo.romCalPhase_ == ROM_CAL_IDLE && exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED);
    assert(std::string(exo.romCalReason_) == "pulse_too_large_at_min_current");
  }
  {
    // Replay the reported excursion during the unpowered interval. Never
    // start a second pulse or an automatic return from outside stored limits.
    NMLHandExo exo; exo.dxl_.position = exo.romCalHomeAngle_ = 117.74f;
    exo.jointLimits_[0][0] = 70.54f; exo.jointLimits_[0][1] = 119.52f;
    exo.romCalDir_ = ROM_CAL_DIR_EXTEND; exo.romCalSign_ = -1;
    exo.arm(); exo.run(20); exo.dxl_.position = 58.45f; exo.run(25);
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED && exo.romCalPhase_ == ROM_CAL_IDLE);
    assert(std::string(exo.romCalReason_) == "outside_limits");
    assert(exo.dxl_.command == 0 && exo.romCalFitSamples_ == 0);
  }
  {
    NMLHandExo exo; exo.arm(); exo.run(20);
    exo.dxl_.position = 103.2f; exo.run(25);
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED);
    assert(std::string(exo.romCalReason_) == "excursion_limit");
  }
  {
    NMLHandExo exo; exo.dxl_.velocity = 20; exo.arm();
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED && exo.dxl_.command == 0);
    assert(std::string(exo.romCalReason_) == "not_settled");
  }
  {
    // Physical response changes after initialization: do not trust prediction
    // to certify safe travel, and keep post-pulse coast under observation.
    NMLHandExo exo; exo.dxl_.plantGain = 25; exo.arm(); exo.run(1000);
    assert(exo.romCalStatus_ == ROM_CAL_STATUS_ABORTED && exo.dxl_.command == 0);
    assert(exo.romCalPulseOnMs_ <= 20 && exo.romCalFitSamples_ == 0);
  }
  {
    PulseResponseModel p;
    assert(isnan(p.predict(20)));
    float current = 20, previous = current;
    // A joint with a 30-mA no-motion region and a linear moving response.
    for (int k = 0; k < 40; ++k) {
      const float travel = fmaxf(0, (current - 30) * .05f);
      assert(p.observe(current, travel, .3f));
      current = roundf(p.nextCurrent(current, travel, .5f, .3f, 2, 5, 80));
      assert(fabsf(current - previous) <= 2);
      previous = current;
    }
    assert(fabsf((current - 30) * .05f - .5f) <= .151f);
    assert(p.movingSamples > 0 && p.noMotionSamples > 0);
    assert(p.observe(current + 4, p.anchorTravel + .4f, .3f));
    assert(p.gradientSamples > 0);
    const float slope = p.slope;
    assert(p.observe(current, 0, .3f));
    assert(p.slope == slope); // no-motion data must not erase the moving gain
    assert(p.nextCurrent(current, 0, .5f, .3f, 2, 5, 80) == current + 2);
    assert(!p.observe(current, NAN, .3f) && !p.observe(current, -2, .3f));
  }
}
