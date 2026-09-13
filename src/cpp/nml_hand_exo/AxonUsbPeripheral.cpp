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
static uint32_t get32(const uint8_t* p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
static void put32(uint8_t* p, uint32_t v) {
  for (unsigned i=0; i<4; ++i) p[i] = uint8_t(v >> (8*i));
}
void AxonUsbPeripheral::beginReply(uint16_t type, unsigned size, uint32_t source) {
  tx_size_ = size + 28; tx_offset_ = 0;
  memset(tx_, 0, tx_size_);
  put32(tx_, 0xC0FFEE00); put32(tx_+4, source);
  put32(tx_+12, sequence_++);
  put32(tx_+20, size | uint32_t(type)<<16);
  put32(tx_+24+size, 0x20); // inspected host's CRC stub
}
void AxonUsbPeripheral::accept() {
  const unsigned len = get32(rx_+20) & 0xFFFF;
  const unsigned type = get32(rx_+20) >> 16;
  if (type == 3 && len == 0) {
    beginReply(4, 4, 0);
    put32(tx_+8, get32(rx_+8)); put32(tx_+24, EXO_AXON_TYPE_ID);
  } else if (type == 0xF210 && len >= 12 && get32(rx_+24) == 1) {
    const unsigned n = get32(rx_+32);
    if (!n || n > 18 || len != 12+4*n) return;
    for (unsigned i=0; i<n; ++i) {
      const auto id = get32(rx_+36+4*i);
      if (id == 0 || id > 252) return;
      for (unsigned j=0; j<i; ++j) if (ids_[j] == id) return;
      ids_[i] = uint8_t(id);
    }
    token_ = get32(rx_+28); address_ = get32(rx_+8);
    count_ = uint8_t(n); cursor_ = 0; field_ = Field::kAngle; sampling_ = true;
  }
}
void AxonUsbPeripheral::finish() {
  // Eight 16-bit channels per motor (schema 0xF212):
  //   0 angle          absolute encoder angle, tenths of a degree
  //   1 age_lo         low 16 bits of source sample age in ms
  //   2 age_hi         high 16 bits of source sample age in ms
  //   3 angle_status   0 measured, 1 unavailable/error/out of range
  //   4 current_mA     PRESENT_CURRENT in mA (signed), or 0 if unavailable
  //   5 current_status 0 measured, 1 unavailable/error
  //   6 torque_lo      low 16 bits of float32 torque (N*m), little-endian
  //   7 torque_hi      high 16 bits of that float32; all-ones-payload if invalid
  // Source timestamps recover exactly: snapshot timestamp minus the 32-bit age.
  // The schema id is deliberately distinct from the legacy 4-field 0xF211 so an
  // old driver rejects this layout at the frame-type check instead of
  // misinterpreting current/torque words as a second motor's angle/age.
  beginReply(0xF212, 20+16*count_, address_);
  put32(tx_+24, 1); put32(tx_+28, token_);
  put32(tx_+32, uint32_t(uptime_ms_)); put32(tx_+36, uint32_t(uptime_ms_>>32));
  put32(tx_+40, count_);
  for (unsigned i=0; i<count_; ++i) {
    const MotorSample& s = samples_[i];
    const uint32_t age = uint32_t(uptime_ms_ - sample_ms_[i]);
    // angle_status is authoritative from the reader, but a sentinel angle also
    // forces unavailable so a stale buffer can never read as a valid zero.
    const uint32_t angle_bad = (!s.angle_ok || s.angle == INT16_MIN) ? 1u : 0u;
    uint32_t torque_bits;
    if (s.torque_ok) {
      memcpy(&torque_bits, &s.torque_Nm, sizeof(torque_bits));
    } else {
      torque_bits = 0xFFFFFFFFu;  // NaN payload marks torque unavailable
    }
    auto* p = tx_+44+16*i;
    put32(p,    uint16_t(s.angle) | ((age & 0xFFFF) << 16));
    put32(p+4,  (age >> 16) | (angle_bad << 16));
    put32(p+8,  uint16_t(s.current_mA) | (uint32_t(s.current_ok ? 0u : 1u) << 16));
    put32(p+12, torque_bits);
  }
  sampling_ = false;
}
void AxonUsbPeripheral::poll(SampleReader read, void* context) {
  const uint32_t now = millis();
  uptime_ms_ += uint32_t(now-last_ms_); last_ms_ = now;
  if (!USBDevice.configured() || pluggedInterface != 0) {
    used_=discard_=tx_size_=tx_offset_=0; expected_=24; sampling_=false; return;
  }
  // Never enter the core's 70-ms send wait. One <=64-byte TX chunk per pass.
  const unsigned ep = pluggedEndpoint+1;
  if (tx_offset_ < tx_size_) {
    if (!USB->DEVICE.DeviceEndpoint[ep].EPSTATUS.bit.BK1RDY) {
      const unsigned n = min(unsigned(EPX_SIZE), tx_size_-tx_offset_);
      if (USBDevice.send(ep, tx_+tx_offset_, n) == n) tx_offset_ += n;
    }
    return;
  }
  if (sampling_) {
    // One bounded read per pass; never issue a whole-hand blocking loop. Each
    // motor needs three register reads (angle, current, torque); they are
    // staggered across passes so a single pass still issues at most one
    // Dynamixel transaction. The whole-motor sample timestamp is stamped on the
    // first (angle) field, so age reflects the start of that motor's read set.
    MotorSample& s = samples_[cursor_];
    if (field_ == Field::kAngle) {
      s = MotorSample{};              // clear stale values at the motor's start
      sample_ms_[cursor_] = uptime_ms_;
      s.angle_ok = read(context, ids_[cursor_], Field::kAngle, s);
      if (!s.angle_ok) s.angle = INT16_MIN;
      field_ = Field::kCurrent;
    } else if (field_ == Field::kCurrent) {
      s.current_ok = read(context, ids_[cursor_], Field::kCurrent, s);
      if (!s.current_ok) s.current_mA = 0;
      field_ = Field::kTorque;
    } else {
      s.torque_ok = read(context, ids_[cursor_], Field::kTorque, s);
      if (!s.torque_ok) s.torque_Nm = 0.0f;
      field_ = Field::kAngle;
      if (++cursor_ == count_) finish();
    }
    return;
  }
  if (now-last_rx_ > 250) { used_=discard_=0; expected_=24; }
  // Consume one byte at a time so a completed request leaves following bytes
  // queued while its reply is pending. Bound work to one USB packet per pass.
  for (unsigned budget=0; budget<EPX_SIZE && USBDevice.available(pluggedEndpoint); ++budget) {
    const int b = USBDevice.recv(pluggedEndpoint);
    if (b < 0) break;
    last_rx_ = now;
    if (discard_) { --discard_; continue; }
    rx_[used_++] = uint8_t(b);
    if (used_ <= 4) {
      const uint8_t magic[] = {0,0xEE,0xFF,0xC0};
      if (rx_[used_-1] != magic[used_-1]) { used_ = b == 0 ? 1 : 0; rx_[0]=0; }
    }
    if (used_ == 24) {
      const unsigned len = get32(rx_+20) & 0xFFFF;
      if (len > sizeof(rx_)-28) { discard_=len+4; used_=0; expected_=24; continue; }
      expected_=28+len;
    }
    if (used_ == expected_) {
      accept(); used_=0; expected_=24;
      if (sampling_ || tx_offset_ < tx_size_) break;
    }
  }
}
} // namespace axon_exo
#endif
