#ifndef NML_ROM_SETTLE_WINDOW_H
#define NML_ROM_SETTLE_WINDOW_H
#include <math.h>
#include <stdint.h>

// Bounded-memory quiet interval. Range (not endpoint displacement) detects
// oscillation; a gap invalidates old evidence after blocked I/O or reporting.
struct RomSettleWindow {
  bool valid = false;
  float low = 0, high = 0;
  uint32_t since = 0, last = 0;
  void reset() { valid = false; }
  void observe(float angle, uint32_t now, float tolerance, uint32_t maxGap) {
    if (!isfinite(angle)) { reset(); return; }
    if (!valid || uint32_t(now-last) > maxGap ||
        fmaxf(high, angle)-fminf(low, angle) > tolerance) {
      valid = true; low = high = angle; since = now;
    } else {
      low = fminf(low, angle); high = fmaxf(high, angle);
    }
    last = now;
  }
  uint32_t quietMs() const { return valid ? uint32_t(last-since) : 0; }
  float span() const { return valid ? high-low : NAN; }
};
#endif
