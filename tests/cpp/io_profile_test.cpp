#include "io_profile.h"
#include "protobuf_frame.h"
#include <cassert>
#include <cstring>

int main() {
  using namespace exo_io;
  char buffer[21];
  assert(strcmp(decimal(0, buffer), "0") == 0);
  assert(strcmp(decimal(UINT64_MAX, buffer), "18446744073709551615") == 0);
  Profile p;
  p.reset(0xfffffff0);
  auto parent = p.enter(COMMAND, 0xfffffff5); // 5 us other
  auto nested = p.enter(DXL_READ, 0xfffffffa); // 5 us command
  p.leave(nested, 0xfffffffa, 4, p.generation()); // 10 us bus, across rollover
  p.leave(parent, 0xfffffff5, 9, p.generation()); // 5 more command; span 20
  auto s = p.snapshot(14);
  assert(s.window_us == 30);
  assert(s.stages[OTHER].exclusive_us == 10);
  assert(s.stages[COMMAND].exclusive_us == 10 && s.stages[COMMAND].max_span_us == 20);
  assert(s.stages[DXL_READ].exclusive_us == 10 && s.stages[DXL_READ].calls == 1);
  auto generation = p.generation();
  parent = p.enter(COMMAND, 20);
  p.reset(25);
  p.leave(parent, 20, 30, generation);
  s = p.snapshot(35);
  assert(s.window_us == 10 && s.stages[COMMAND].exclusive_us == 5);
  assert(s.stages[COMMAND].max_span_us == 0);
  assert(exo_pb::crc((const uint8_t*)"123456789", 9) == 0x29b1);
  exo_pb::Reader reader;
  uint8_t packet[] = {'P','B',1,1,0,0,0,42};
  uint16_t crc = exo_pb::crc(packet+7, 1, exo_pb::crc(packet+2,3));
  packet[5] = crc; packet[6] = crc >> 8;
  for (int i=0; i<8; ++i) assert(reader.feed(packet[i], i) == (i==7));
  packet[7] ^= 1;
  for (int i=0; i<8; ++i) assert(!reader.feed(packet[i], i+10));
  packet[7] ^= 1;
  // Timed-out partial message never becomes a motion request.
  for (int i=0; i<4; ++i) assert(!reader.feed(packet[i], 30+i));
  for (int i=4; i<8; ++i) assert(!reader.feed(packet[i], 100+i));
  for (int i=0; i<8; ++i) assert(reader.feed(packet[i], 120+i) == (i==7));
  // Bad lengths are discarded without growing the fixed buffer.
  packet[3] = 255;
  for (int i=0; i<8; ++i) assert(!reader.feed(packet[i], 140+i));
}
