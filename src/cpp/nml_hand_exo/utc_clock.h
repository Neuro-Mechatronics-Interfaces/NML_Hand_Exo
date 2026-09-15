#ifndef NML_UTC_CLOCK_H
#define NML_UTC_CLOCK_H
#include <stdint.h>

// UTC is an offset from extended uptime. Control/watchdog timing never uses it.
// tick() must run at least once per millis() rollover (49.7 days).
struct ExoUtcClock {
  static constexpr uint64_t MAX_EPOCH_MS = 253402300799999ULL; // 9999-12-31
  uint32_t lastRaw = 0;
  uint64_t uptime = 0;
  uint64_t anchorUptime = 0;
  uint64_t anchorUtc = 0;
  bool isSynchronized = false;

  void tick(uint32_t raw) {
    uptime += uint32_t(raw - lastRaw);
    lastRaw = raw;
  }
  bool set(uint64_t epochMs, uint32_t raw) {
    if (!epochMs || epochMs > MAX_EPOCH_MS) return false;
    tick(raw);
    anchorUtc = epochMs;
    anchorUptime = uptime;
    isSynchronized = true;
    return true;
  }
  uint64_t now(uint32_t raw) {
    tick(raw);
    if (!isSynchronized) return 0;
    const uint64_t elapsed = uptime - anchorUptime;
    if (elapsed > MAX_EPOCH_MS - anchorUtc) { isSynchronized = false; return 0; }
    return anchorUtc + elapsed;
  }
};
#endif
