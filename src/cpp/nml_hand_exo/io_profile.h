#pragma once
#include <stdint.h>

// Exclusive wall time: nested bus/output calls are subtracted from their parent.
// Main-loop use only; never enter from an ISR. Charge at least once per micros()
// rollover (~71 minutes). No allocation, serial output, or extra motor traffic.
namespace exo_io {
// Some embedded printf variants do not implement %llu. Only used for queries.
inline const char* decimal(uint64_t value, char (&buffer)[21]) {
  char* end = buffer + 20;
  *end = '\0';
  do { *--end = char('0' + value % 10); value /= 10; } while (value);
  return end;
}
enum Stage : uint8_t { OTHER, COMMAND, CONTROL, MODEL, DXL_READ, DXL_WRITE,
                       NX_PACK, USB_TX, BT_TX, PERIPHERAL, COUNT };
struct Counter {
  uint64_t exclusive_us = 0;
  uint32_t calls = 0;
  uint32_t max_span_us = 0; // inclusive duration of one call, not additive
};
struct Snapshot {
  Counter stages[COUNT];
  uint64_t window_us = 0;
};
class Profile {
 public:
  void charge(uint32_t now) {
    if (started_) counters_[active_].exclusive_us += uint32_t(now - last_);
    started_ = true;
    last_ = now;
  }
  Stage enter(Stage next, uint32_t now) {
    charge(now);
    Stage parent = active_;
    active_ = next;
    ++counters_[next].calls;
    return parent;
  }
  void leave(Stage parent, uint32_t begin, uint32_t now, uint32_t generation) {
    charge(now);
    // A reset inside a command must not reintroduce its pre-reset span.
    if (generation == generation_) {
      uint32_t span = now - begin;
      if (span > counters_[active_].max_span_us) counters_[active_].max_span_us = span;
    }
    active_ = parent;
  }
  void reset(uint32_t now) {
    for (uint8_t i = 0; i < COUNT; ++i) counters_[i] = Counter{};
    ++generation_;
    started_ = true;
    last_ = now; // preserve active scope; its destructor will restore the parent
  }
  Snapshot snapshot(uint32_t now) {
    charge(now);
    Snapshot result;
    for (uint8_t i = 0; i < COUNT; ++i) {
      result.stages[i] = counters_[i];
      result.window_us += counters_[i].exclusive_us;
    }
    return result;
  }
  uint32_t generation() const { return generation_; }
 private:
  Counter counters_[COUNT];
  Stage active_ = OTHER;
  uint32_t last_ = 0, generation_ = 0;
  bool started_ = false;
};
inline Profile& profile() { static Profile value; return value; }
} // namespace exo_io

#ifdef ARDUINO
namespace exo_io {
class Scope {
 public:
  explicit Scope(Stage stage) : begin_(micros()), generation_(profile().generation()),
                                parent_(profile().enter(stage, begin_)) {}
  ~Scope() { profile().leave(parent_, begin_, micros(), generation_); }
 private:
  uint32_t begin_, generation_;
  Stage parent_;
};
}
#define EXO_IO_JOIN_(a,b) a##b
#define EXO_IO_JOIN(a,b) EXO_IO_JOIN_(a,b)
#define EXO_PROFILE(stage) exo_io::Scope EXO_IO_JOIN(ioScope_, __LINE__)(exo_io::stage)
#define EXO_PROFILE_CALL(stage, ...) ([&]() { EXO_PROFILE(stage); return (__VA_ARGS__); }())
#else
// Native extracted-method harnesses have no Arduino clock.
#define EXO_PROFILE(stage) ((void)0)
#define EXO_PROFILE_CALL(stage, ...) (__VA_ARGS__)
#endif
