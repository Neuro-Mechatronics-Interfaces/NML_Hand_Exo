#include "AxonUsbPeripheral.h"
#if EXO_AXON_USB
namespace axon_exo {
AxonUsbPeripheral::AxonUsbPeripheral() : PluggableUSBModule(2, 1, epType_) {
  epType_[0] = USB_ENDPOINT_TYPE_BULK | USB_ENDPOINT_OUT(0);
  epType_[1] = USB_ENDPOINT_TYPE_BULK | USB_ENDPOINT_IN(0);
  PluggableUSB().plug(this);
}
int AxonUsbPeripheral::getInterface(uint8_t* count) {
  ++*count;
  struct Descriptor { InterfaceDescriptor iface; EndpointDescriptor out, in; };
  const Descriptor desc = {
    D_INTERFACE(pluggedInterface, 2, 0xFF, 0, 0),
    D_ENDPOINT(USB_ENDPOINT_OUT(pluggedEndpoint), USB_ENDPOINT_TYPE_BULK, EPX_SIZE, 0),
    D_ENDPOINT(USB_ENDPOINT_IN(pluggedEndpoint+1), USB_ENDPOINT_TYPE_BULK, EPX_SIZE, 0)
  };
  return USBDevice.sendControl(&desc, sizeof(desc));
}
uint8_t AxonUsbPeripheral::getShortName(char* name) {
  memcpy(name, "EXO", 3); return 3;
}
void AxonUsbPeripheral::poll() {
  if (!USBDevice.configured() || pluggedInterface != 0) { parser_.reset(); return; }
  const uint32_t now = millis();
  if (now - last_rx_ > 250) parser_.reset();
  // Bound each pass to one USB packet so discovery cannot starve exo.update().
  uint8_t bytes[EPX_SIZE], reply[32];
  const uint32_t avail = USBDevice.available(pluggedEndpoint);
  if (!avail) return;
  const uint32_t got = USBDevice.recv(pluggedEndpoint, bytes,
                                     avail < sizeof(bytes) ? avail : sizeof(bytes));
  if (got > sizeof(bytes)) { parser_.reset(); return; }
  last_rx_ = now;
  for (uint32_t i = 0; i < got; ++i) {
    if (parser_.feed(bytes[i], reply)) {
      // Core copies this 32-byte response and bounds an outstanding TX wait
      // to 70 ms. Never forward Axon payloads to the motor command parser.
      USBDevice.send(pluggedEndpoint+1, reply, sizeof(reply));
    }
  }
}
} // namespace axon_exo
#endif
