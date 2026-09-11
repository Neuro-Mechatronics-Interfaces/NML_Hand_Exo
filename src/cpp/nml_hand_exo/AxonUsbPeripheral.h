#pragma once
#include "axon_config.h"
#if EXO_AXON_USB
#include <Arduino.h>
#include "USB/USBAPI.h"
#include "USB/USBCore.h"
#include "api/PluggableUSB.h"
#include "AxonEnumProtocol.h"

namespace axon_exo {
// Discovery and read-only angle polling. Commands remain ASCII on CDC 1/2.
class AxonUsbPeripheral : public arduino::PluggableUSBModule {
 public:
  AxonUsbPeripheral();
  using AngleReader = bool (*)(void*, uint8_t, int16_t&);
  void poll(AngleReader read, void* context);
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
  int16_t angles_[18]{};
  uint64_t sample_ms_[18]{};
  uint8_t count_ = 0, cursor_ = 0;
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
