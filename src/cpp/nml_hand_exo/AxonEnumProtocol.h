#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "axon_config.h"

namespace axon_exo {
// Incremental discovery parser. Unknown payloads are consumed by length, never
// scanned for embedded ENUMERATE commands. At most 24 bytes of parser storage.
class EnumProtocol {
 public:
  void reset() { used_ = remaining_ = 0; enumerate_ = false; }
  bool feed(uint8_t byte, uint8_t (&reply)[32]) {
    if (remaining_) {
      if (--remaining_ || !enumerate_) return false;
      memset(reply, 0, sizeof(reply));
      put32(reply, 0xC0FFEE00u);
      reply[8] = header_[8]; reply[9] = header_[9];
      put32(reply + 20, (4u << 16) | 4u);
      put32(reply + 24, EXO_AXON_TYPE_ID);
      put32(reply + 28, 0x20u); // inspected server's CRC stub, not a real CRC
      enumerate_ = false;
      return true;
    }
    header_[used_++] = byte;
    if (used_ <= 4) {
      const uint8_t magic[] = {0, 0xEE, 0xFF, 0xC0};
      if (byte != magic[used_ - 1]) {
        used_ = byte == 0 ? 1 : 0;
        header_[0] = 0;
      }
      return false;
    }
    if (used_ < sizeof(header_)) return false;
    const unsigned len = header_[20] | (unsigned(header_[21]) << 8);
    const unsigned type = header_[22] | (unsigned(header_[23]) << 8);
    used_ = 0;
    if (len > 4068) { reset(); return false; }
    remaining_ = len + 4; // consume trailer even for a zero-payload request
    enumerate_ = type == 3 && len == 0;
    return false;
  }
 private:
  static void put32(uint8_t* p, uint32_t v) {
    for (unsigned i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8*i));
  }
  uint8_t header_[24]{};
  unsigned used_ = 0, remaining_ = 0;
  bool enumerate_ = false;
};
} // namespace axon_exo
