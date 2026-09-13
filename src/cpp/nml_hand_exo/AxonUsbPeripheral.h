#pragma once
#include "axon_config.h"
#if EXO_AXON_USB
#include <Arduino.h>
#include "USB/USBAPI.h"
#include "USB/USBCore.h"
#include "api/PluggableUSB.h"
#include "AxonEnumProtocol.h"

namespace axon_exo {
// One motor's read-only measurements for one Axon sample. Every field is filled
// independently; a failed sub-read leaves its value at the unavailable sentinel
// and sets the matching *_ok flag false, so the host can tell exactly which
// measurement failed rather than discarding the whole motor.
struct MotorSample {
  int16_t angle = INT16_MIN;   // absolute encoder angle, tenths of a degree
  int16_t current_mA = 0;      // PRESENT_CURRENT, milliamps (signed)
  float torque_Nm = 0.0f;      // derived torque, Newton-metres
  bool angle_ok = false;
  bool current_ok = false;
  bool torque_ok = false;
};

// Read-only discovery and angle/current/torque polling. Commands remain ASCII
// on CDC 1/2. This peripheral never enables torque, homes, or commands motion;
// it only reads. The reader fills the requested field of one MotorSample per
// call and returns false on a bus error for that field.
class AxonUsbPeripheral : public arduino::PluggableUSBModule {
 public:
  AxonUsbPeripheral();
  // Which single measurement the poll loop wants this pass; kept to one bounded
  // Dynamixel transaction per pass, staggered across passes for one motor.
  enum class Field : uint8_t { kAngle = 0, kCurrent = 1, kTorque = 2 };
  using SampleReader = bool (*)(void*, uint8_t, Field, MotorSample&);
  void poll(SampleReader read, void* context);
 protected:
  bool setup(arduino::USBSetup&) override { return false; }
  int getInterface(uint8_t* count) override;
  int getDescriptor(arduino::USBSetup&) override { return 0; }
  uint8_t getShortName(char* name) override;
 private:
  unsigned int epType_[2];
  uint8_t rx_[112]{}, tx_[512]{};
  unsigned used_ = 0, expected_ = 24, discard_ = 0;
  unsigned tx_size_ = 0, tx_offset_ = 0;
  uint8_t ids_[18]{};
  MotorSample samples_[18]{};
  uint64_t sample_ms_[18]{};
  uint8_t count_ = 0, cursor_ = 0;
  // Which field of samples_[cursor_] is read next. Advances angle->current->
  // torque, one bounded read per poll pass; on kTorque completion the motor is
  // done and cursor_ advances.
  Field field_ = Field::kAngle;
  bool sampling_ = false;
  uint32_t token_ = 0, address_ = 0, sequence_ = 0;
  uint32_t last_ms_ = 0;
  uint64_t uptime_ms_ = 0;
  void accept();
  void finish();
  void beginReply(uint16_t type, unsigned payload_size, uint32_t source);
  uint32_t last_rx_ = 0;
};
} // namespace axon_exo
#endif
