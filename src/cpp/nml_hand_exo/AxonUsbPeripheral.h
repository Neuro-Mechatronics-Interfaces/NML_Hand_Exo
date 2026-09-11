#pragma once
#include "axon_config.h"
#if EXO_AXON_USB
#include <Arduino.h>
#include "USB/USBAPI.h"
#include "USB/USBCore.h"
#include "api/PluggableUSB.h"
#include "AxonEnumProtocol.h"

namespace axon_exo {
// Discovery only. Motor commands remain ASCII on CDC interfaces 1/2.
class AxonUsbPeripheral : public arduino::PluggableUSBModule {
 public:
  AxonUsbPeripheral();
  void poll();
 protected:
  bool setup(arduino::USBSetup&) override { return false; }
  int getInterface(uint8_t* count) override;
  int getDescriptor(arduino::USBSetup&) override { return 0; }
  uint8_t getShortName(char* name) override;
 private:
  unsigned int epType_[2];
  EnumProtocol parser_;
  uint32_t last_rx_ = 0;
};
} // namespace axon_exo
#endif
