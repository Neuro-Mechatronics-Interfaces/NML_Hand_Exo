#include <cassert>
#include <cstdint>
#include <string>
using String = std::string;
#define DUAL_CDC 1
#define EXO_AXON_USB 0
uint32_t nowMs = 0;
int commandStream, telemetryStream, bluetoothStream;
#define CMD_SERIAL commandStream
#define TELEM_SERIAL telemetryStream
#define COMMAND_SERIAL bluetoothStream
struct Reader {
  int pending = 1000;
  bool poll(int&, String& line) {
    if (!pending) return false;
    --pending; line = "get_telemetry_fast:13:14:15"; return true;
  }
} gCmdLines, gTelemLines, gBtLines;
struct Exo {
  bool rom = true, powered = true;
  uint32_t cutoff = 0;
  bool isAssistBusy() const { return false; }
  void serviceAssist() {}
  void serviceRomCalibration() {
    if (powered && nowMs >= 150) { powered = false; cutoff = nowMs; }
  }
  void update() { serviceRomCalibration(); }
  bool isRomCalibrating() { return rom; }
} exo;
struct Gestures { int calls = 0; void update() { ++calls; } } gc;
int bno055, oledCalls = 0, commands = 0, bluetoothCommands = 0;
void loopStatsTick() {}
void debugPrint(String) {}
void oledTick() { ++oledCalls; }
void parseMessage(Exo&, Gestures&, int&, String, bool bluetooth = false) {
  ++commands;
  if (bluetooth) ++bluetoothCommands;
  nowMs += 20; // deliberately slow reply, sustained queues on all interfaces
}
// FIRMWARE_LOOP
int main() {
  loop();
  assert(commands == 3 && bluetoothCommands == 1); // bounded work per pass
  while (exo.powered) loop();
  assert(exo.cutoff >= 150 && exo.cutoff <= 170);
  assert(!gc.calls && !oledCalls); // no LED flash / display work during ROM
  exo.rom = false;
  loop();
  assert(gc.calls == 1 && oledCalls == 1);
}
