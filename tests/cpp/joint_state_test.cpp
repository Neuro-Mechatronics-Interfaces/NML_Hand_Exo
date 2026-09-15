#include "joint_state_model.h"
#include "utc_clock.h"
#include <cassert>
#include <limits>

int main() {
  // A 10-ms service gap is subdivided into the same prediction as ten 1-ms
  // services. Constant-input displacement agrees with the analytic plant.
  JointStateModel coarse, fine;
  coarse.correct(0, 0, 0, true, 0);
  fine.correct(0, 0, 0, true, 0);
  coarse.advance(10, -100, 100, 0, true, false, 0, 100, true, 0, .00115f);
  for (uint32_t t=1; t<=10; ++t)
    fine.advance(t, -100, 100, 0, true, false, 0, 100, true, 0, .00115f);
  assert(fabsf(coarse.angle-fine.angle) < 1e-6f);
  assert(fabsf(coarse.velocity-fine.velocity) < 1e-6f);
  assert(fabsf(coarse.angle - 10*(.01f-.15f*(1-expf(-.01f/.15f)))) < 1e-5f);
  assert(fine.lastTick == 10 && fine.angle > 0);
  JointStateModel m;
  assert(m.params.valid());
  m.params.gain = NAN;
  assert(!m.params.valid());
  m.params = JointModelParams();
  // Multi-turn goal: continuous, monotonic, bounded, not a modulo-360 snap.
  m.correct(-189, 0, 0, true, 0);
  float previous = m.angle;
  for (uint32_t t = 10; t < 400000; t += 10) {
    m.advance(t, -189, 2840, -189, true, true, 2840, 910, true, 0, 0.00115f);
    assert(m.angle >= previous && m.angle <= 2840);
    assert(m.angle - previous <= 0.601f);
    previous = m.angle;
  }
  assert(fabsf(m.angle - 2840) <= JointStateModel::POSITION_SETTLE_DEG);
  assert(m.current == 0 && m.velocity == 0);
  m.advance(400000, -189, 2840, -189, true, true, 100, 910, true, 0, 0.00115f);
  assert(m.angle < 2840 && m.angle > 100);
  // A resisting moment reduces travel; never drives a position estimate away.
  JointStateModel loaded, free;
  loaded.correct(0, 0, 0, true, 0);
  free.correct(0, 0, 0, true, 0);
  loaded.params.moment = 0.1f;
  for (uint32_t t = 10; t <= 1000; t += 10) {
    loaded.advance(t, -100, 100, 0, true, true, 90, 100, true, 0, 0.00115f);
    free.advance(t, -100, 100, 0, true, true, 90, 100, true, 0, 0.00115f);
  }
  assert(loaded.angle > 0 && loaded.angle < free.angle);
  loaded.params.moment = 1;
  loaded.velocity = 0;
  previous = loaded.angle;
  loaded.advance(1010, -100, 100, 0, true, true, 90, 100, true, 0, 0.00115f);
  assert(loaded.angle == previous);
  // Unknown-current velocity control never fabricates measured effort.
  m.correct(0, 0, 0, true, 0xfffffff0);
  m.advance(14, -10, 10, 0, true, false, 0, 0, false, -60, 0.00115f);
  assert(m.angle < 0 && !m.currentValid);
  m.advance(0x100000, -10, 10, 0, true, false, 0, 0, false, -60, 0.00115f);
  assert(m.angle >= -6.1f); // stalled scheduler must not integrate hours
  m.correct(7, 3, -19, true, 0x100001);
  assert(m.angle == 7 && m.velocity == 3 && m.current == -19);

  // Quiet position holding uses load, not the current allowance. Even a tick
  // below the integration period must not retain a previous travel current.
  JointStateModel quiet;
  quiet.correct(30, 0, 83, true, 0);
  quiet.advance(1, -100, 100, 30, false, true, 30, 300, true, 0, 0.00115f);
  assert(quiet.currentValid && quiet.current == 0 && quiet.velocity == 0);
  quiet.params.moment = 0.023f; // 20 mA of signed holding effort
  quiet.advance(10, -100, 100, 30, true, true, 29.95f, 300, true, 0, 0.00115f);
  assert(fabsf(quiet.current - 20) < 0.001f && quiet.angle == 30);
  quiet.params.stiffness = 0.00115f;
  quiet.advance(20, -100, 100, 20, false, true, 30, 300, true, 0, 0.00115f);
  assert(fabsf(quiet.current - 30) < 0.001f);
  quiet.params.moment = -1;
  quiet.advance(30, -100, 100, 30, false, true, 30, 100, true, 0, 0.00115f);
  assert(quiet.current == -100); // cap modeled loads in either direction
  quiet.params.moment = 1;
  quiet.advance(40, -100, 100, 30, false, true, 30, 100, true, 0, 0.00115f);
  assert(quiet.current == 100);
  quiet.advance(50, -100, 100, 30, false, true, 30, 0, true, 0, 0.00115f);
  assert(quiet.current == 0); // disabled / zero allowance cannot imply effort
  quiet.advance(60, -100, 100, 30, false, true, 30, 100, false, 0, 0.00115f);
  assert(!quiet.currentValid); // position-only control remains unknown
  quiet.params = JointModelParams();
  quiet.advance(70, -100, 100, 30, false, true, 60, 100, true, 0, 0.00115f);
  assert(quiet.current == 100); // a stopped-short joint is not at rest at goal
  quiet.advance(80, -100, 100, 30, true, false, 30, -120, true, 0, 0.00115f);
  assert(quiet.current == -120); // direct current is not quieted by position
  quiet.correct(30, 0, 17, true, 81);
  assert(quiet.current == 17); // real readings always remain real

  // A modeled arrival quiets on the same tick; a new goal restores effort.
  quiet.params.max_velocity = 300;
  quiet.correct(0, 100, 100, true, 100);
  quiet.advance(200, -100, 100, 0, true, true, 1, 100, true, 0, 0.00115f);
  assert(quiet.atPositionGoal(1) && quiet.current == 0 && quiet.velocity == 0);
  quiet.advance(201, -100, 100, 0, true, true, -20, 100, true, 0, 0.00115f);
  assert(quiet.current == -100);

  JointTelemetryGate gate;
  assert(!gate.estimated(0, false));
  assert(gate.estimated(0xfffffff0, true));
  assert(gate.estimated(100, false));
  assert(!gate.estimated(234, false));
  gate.note(235); // even a zero-length command burst starts the quiet window
  assert(gate.estimated(240, false));

  ExoUtcClock clock;
  assert(clock.now(10) == 0);
  assert(!clock.set(0, 20));
  assert(!clock.set(ExoUtcClock::MAX_EPOCH_MS + 1, 20));
  const uint64_t epoch = 1789473600123ULL;
  assert(clock.set(epoch, 0xfffffff0));
  assert(clock.now(14) == epoch + 30);
  const uint64_t uptime = clock.uptime;
  assert(clock.set(epoch - 1000, 14)); // backward wall-clock correction
  assert(clock.uptime == uptime);      // cannot reset watchdog time
  assert(clock.now(24) == epoch - 990);
  assert(clock.set(ExoUtcClock::MAX_EPOCH_MS, 24));
  assert(clock.now(25) == 0 && !clock.isSynchronized);
}
