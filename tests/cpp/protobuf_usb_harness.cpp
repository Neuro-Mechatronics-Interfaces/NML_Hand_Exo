#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>
#include <string>
#include "protobuf_frame.h"
#include "joint_state_model.h"
#include "protocol/exo_usb.pb.h"
#include "protocol/exo_usb.pb.c"
#include <pb_decode.h>
#include <pb_encode.h>
using std::isfinite;
uint32_t millis() { return 100; }
struct Stream {
  std::vector<uint8_t> input, output;
  size_t offset = 0;
  int available() { return input.size() - offset; }
  uint8_t read() { return input[offset++]; }
  void write(const uint8_t* bytes, size_t count) { output.insert(output.end(), bytes, bytes+count); }
} usb;
#define TELEM_SERIAL usb
// RECORD_DECLARATIONS
struct MotorAngleTarget { uint8_t id; float angleDeg; };
constexpr int N_MOTORS = 18;
struct NMLHandExo {
  int writes = 0, reads = 0, stops = 0;
  float value = 0;
  bool accept = true, assistBusy = false;
  bool isAssistBusy() const { return assistBusy; }
  bool configureAssist(uint8_t, float t, float g, float cap) { value = cap; return accept && isfinite(t) && isfinite(g) && isfinite(cap); }
  bool calibrateAssist(const uint8_t*, uint8_t) { ++writes; return accept; }
  bool startAssist() { ++writes; return accept; }
  bool heartbeatAssist() { return accept; }
  void stopAssist(const char* = "stopped") { ++stops; assistBusy = false; }
  std::string motorControlMode_ = "CURRENT";
  bool positionHoldActive_[9] = {};
  bool setGoalCurrents(const uint8_t*, const float*, uint8_t);
  int getIndexById(uint32_t id) { return id>=11 && id<=19 ? id-11 : -1; }
  bool setGoalCurrent(uint8_t, float v) { ++writes; value=v; return accept; }
  bool setGoalVelocity(uint8_t, float v) { ++writes; value=v; return accept; }
  bool setJointModel(uint8_t, const JointModelParams& params) {
    if (!accept) return false;
    value = params.gain;
    return true; // no actuator writes or reads
  }
  bool modeControlsPosition() { return accept; }
  float getZeroAngle(uint8_t) { return 100; }
  bool getFlipMotor(uint8_t) { return true; }
  bool setAbsoluteAnglesSync(const MotorAngleTarget* targets, uint8_t count,
                            uint8_t* written=nullptr, uint8_t* skipped=nullptr) {
    if (written) *written=count;
    if (skipped) *skipped=0;
    ++writes; value=targets[0].angleDeg; return accept;
  }
  void stopAllDirectControl() { ++stops; }
  void stopDirectControl(uint8_t) { ++stops; }
  void serviceRomCalibration() {}
  uint8_t getFastTelemetryRecords(uint8_t* ids, uint8_t count, FastTelemetryRecord* out,
                                  uint8_t& method, int) {
    ++reads; method = 3;
    for (uint8_t i=0;i<count;++i) {
      out[i] = FastTelemetryRecord{};
      out[i].id=ids[i]; out[i].sources=0x15; out[i].sample_ms=100;
      out[i].absolute_cdeg=12345; out[i].current_mA=-80;
    }
    return count;
  }
};
// CURRENT_BATCH_IMPLEMENTATION
const int gestureLibrary[] = {0};
int findGestureIndex(const char* name) { return std::string(name)=="index" ? 0 : -1; }
int findStateIndex(int, const char* pose) { return std::string(pose)=="flex" ? 0 : -1; }
struct GestureController {
  NMLHandExo& exo;
  explicit GestureController(NMLHandExo& e): exo(e) {}
  void executeGesture(const char*, const char*) { ++exo.writes; exo.value=100; }
  bool setGestureAngle(const char* name, float value) {
    if (findGestureIndex(name)<0) return false;
    ++exo.writes; exo.value=value; return true;
  }
  bool resolveGestureSignedTargets(const char* name, float value, MotorAngleTarget* targets,
                                   uint8_t, uint8_t& count, uint8_t* stuck) {
    *stuck=0;
    count=1;
    targets[0] = {uint8_t(std::string(name)=="index" ? 16 : 17), value};
    return true;
  }
};
// FIRMWARE_IMPLEMENTATION
int main(int argc, char** argv) {
  std::string hex = argv[1];
  for (size_t i=0;i<hex.size();i+=2) usb.input.push_back(std::stoul(hex.substr(i,2),nullptr,16));
  NMLHandExo exo;
  if (argc > 2 && std::string(argv[2]) == "assist") exo.assistBusy = true;
  GestureController gc(exo);
  exo.accept = argc < 3 || std::string(argv[2]) != "reject";
  if (argc >= 3 && std::string(argv[2]) == "hold") exo.positionHoldActive_[6] = true; // ID 17
  if (argc >= 3 && std::string(argv[2]) == "mode") exo.motorControlMode_ = "POSITION";
  while (usb.available()) pollProtobufUsb(exo, gc);
  std::cout << exo.writes << " " << exo.reads << " " << exo.stops << " " << exo.value << "\n";
  for (auto byte: usb.output) std::cout << std::hex << std::setw(2) << std::setfill('0') << int(byte);
  std::cout << "\n";
}
