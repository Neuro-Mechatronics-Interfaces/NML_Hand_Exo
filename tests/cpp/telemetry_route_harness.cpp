#include <cassert>
#include <cstdint>
#include <string>
#include <vector>
using String = std::string;
#define DUAL_CDC 1
#define REPLY_ROUTE_BOTH 0
#define REPLY_ROUTE_TELEM 1
#define REPLY_ROUTE_CMD 2
int gReplyRoute = REPLY_ROUTE_TELEM;
uint32_t nowMs = 0;
uint32_t millis() { return nowMs; }
uint32_t micros() { return nowMs * 1000; }
struct Stream {
  std::vector<uint8_t> bytes;
  int flushes = 0;
  void write(const uint8_t* p, unsigned n) { bytes.insert(bytes.end(), p, p+n); }
  void flush() { ++flushes; }
  void clear() { bytes.clear(); flushes = 0; }
} commandPort, telemetryPort, bluetoothPort;
#define DEBUG_SERIAL commandPort
#define DEBUG_SERIAL commandPort
#define CMD_SERIAL commandPort
#define TELEM_SERIAL telemetryPort
#define COMMAND_SERIAL bluetoothPort
struct FastTelemetryRecord { uint8_t data[33] = {}; };
enum { FAST_TELEM_METHOD_FAILED };
struct NMLHandExo {
  uint8_t getFastTelemetryRecords(const uint8_t*, uint8_t count, FastTelemetryRecord*,
                                 uint8_t& method, uint32_t) { method = 0x84; return count; }
};
uint8_t collectFastTelemetryIDs(NMLHandExo&, const String&, uint8_t*, uint8_t) { return 9; }
void commandPrint(String) { assert(false); }
// FRAME_DECLARATIONS
// SEND_TELEMETRY
int main() {
  NMLHandExo exo;
  for (int route = 0; route <= 2; ++route) {
    gReplyRoute = route;
    commandPort.clear(); telemetryPort.clear(); bluetoothPort.clear();
    sendFastTelemetry(exo, "get_telemetry_fast", false);
    assert(commandPort.bytes.size() == 310);
    assert(telemetryPort.bytes.size() == 0);
    assert(bluetoothPort.bytes.empty() && bluetoothPort.flushes == 0);
  }
  sendFastTelemetry(exo, "get_telemetry_fast", true);
  assert(bluetoothPort.bytes.size() == 310 && bluetoothPort.flushes == 1);
  assert(bluetoothPort.bytes[0] == 'N' && bluetoothPort.bytes[1] == 'X');
}
