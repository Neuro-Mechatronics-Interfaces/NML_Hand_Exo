#include "oled.h"

// ignoring oled
bool oledInit() { return false; }
void oledSetEnabled(bool) {}
bool oledEnabled() { return false; }
void oledTick() {}
void oledSetState(ExoState) {}
ExoState oledGetState() { return EXO_READY; }
const __FlashStringHelper* oledGetStateText(ExoState) { return nullptr; }
void oledStartupAnimation() {}
