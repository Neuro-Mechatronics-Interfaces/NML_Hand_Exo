#pragma once
#include <stdint.h>
#include <stddef.h>

namespace exo_pb {
static constexpr size_t MAX_RESPONSE = 2048;
// Envelope: 'PB', version, little-endian length and CRC16-CCITT-FALSE.
// CRC covers version + length + payload; it excludes magic and CRC itself.
inline uint16_t crc(const uint8_t* data, size_t size, uint16_t value = 0xffff) {
  while (size--) {
    value ^= uint16_t(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      value = (value & 0x8000) ? (value << 1) ^ 0x1021 : value << 1;
  }
  return value;
}
struct Reader {
  static constexpr size_t MAX_REQUEST = 512;
  uint8_t bytes[MAX_REQUEST + 7] = {};
  size_t used = 0, size = 0;
  uint32_t last = 0;
  uint8_t expected_version = 1;
  explicit Reader(uint8_t version = 1) : expected_version(version) {}
  bool feed(uint8_t byte, uint32_t now) {
    if (used && uint32_t(now - last) > 50) used = 0;
    last = now;
    if (!used && byte != 'P') return false;
    if (used == 1 && byte != 'B') { used = byte == 'P' ? 1 : 0; return false; }
    bytes[used++] = byte;
    if (used == 7) {
      size = bytes[3] | uint16_t(bytes[4]) << 8;
      if (bytes[2] != expected_version || size == 0 || size > MAX_REQUEST) { used = 0; return false; }
    }
    if (used >= 7 && used == size + 7) {
      used = 0;
      return crc(bytes + 7, size, crc(bytes + 2, 3)) ==
          (bytes[5] | uint16_t(bytes[6]) << 8);
    }
    return false;
  }
};
} // namespace exo_pb
