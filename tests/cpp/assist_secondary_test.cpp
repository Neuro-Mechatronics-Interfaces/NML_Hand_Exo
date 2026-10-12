#include "assist_controller.h"
#include <cassert>
#include <cstdio>
#include <cstring>

AssistJoint joint() {
  AssistJoint j;
  j.hardMin = -200; j.hardMax = 3000; // multi-turn, no wrapping
  j.begin(100, 0); j.calibrated = true;
  return j;
}

int main(int argc, char**) {
  if (argc > 1) {
    // Recorded motor-coordinate inputs for the Python reference comparison.
    auto j = joint();
    for (int k = 0; k < 150; ++k) {
      const float torque = k < 75 ? .2f : -.15f;
      j.effort = torque/AssistJoint::kTorqueConstant;
      j.updateIntent(100, k < 75 ? 5 : -5, .02f);
      std::printf("%.6f %.6f %.6f %d\n", j.secondaryGoal, j.intentVelocity, j.lead, j.moving);
    }
    return 0;
  }
  auto j = joint();
  assert(fabsf(j.threshold*AssistJoint::kTorqueConstant-.08f) < 1e-6f);
  assert(j.currentCap == 300);
  assert(j.positionP == AssistJoint::kAssistPositionP && j.positionP == 300);
  assert(j.positionI == 0 && j.positionD == AssistJoint::kAssistPositionD);
  j.effort = .2f/AssistJoint::kTorqueConstant;
  for (int k = 0; k < 20; ++k) j.updateIntent(100, 5, .02f);
  assert(!j.moving && j.secondaryGoal == 100);
  j.effort = 0; j.updateIntent(100, 5, .02f); // reset interrupted dwell
  j.effort = .2f/AssistJoint::kTorqueConstant;
  for (int k = 0; k < 20; ++k) j.updateIntent(100, 5, .02f);
  assert(!j.moving);
  for (int k = 0; k < 40; ++k) j.updateIntent(100, 5, .02f);
  assert(j.moving && j.secondaryGoal > 110 && j.lead == 8);
  assert(fabsf(j.intentVelocity-16) < 1e-4f && j.currentCap == 300);
  // A measured position jump does not redefine the integrated secondary goal.
  const float old = j.secondaryGoal;
  j.updateIntent(200, 5, .02f);
  assert(fabsf(j.secondaryGoal-old-.32f) < 1e-4f);
  for (int k = 0; k < 26; ++k) j.updateIntent(200, 0, .02f);
  assert(!j.moving && j.rearmBlocked && j.reference() == 200);
  for (int k = 0; k < 100; ++k) j.updateIntent(200, 0, .02f);
  assert(j.rearmBlocked && j.reference() == 200 && j.lead == 0);
  j.effort = -200; // a reversal cannot bypass relaxation after a stall
  for (int k = 0; k < 30; ++k) j.updateIntent(200, -5, .02f);
  assert(j.rearmBlocked);
  j.effort = 0;
  for (int k = 0; k < 26; ++k) j.updateIntent(200, 0, .02f);
  assert(!j.rearmBlocked);
  j.effort = -200;
  for (int k = 0; k < 40; ++k) j.updateIntent(200, -5, .02f);
  assert(j.moving && j.reference() < 200);
  j.effort = 0;
  for (int k = 0; k < 26; ++k) j.updateIntent(180, 0, .02f);
  assert(!j.moving && j.reference() == 180); // achieved-pose release

  j = joint(); j.hardMin = 99; j.hardMax = 3000;
  j.direction = 1; j.lead = 8; j.releaseAt(99);
  assert(j.reference() == 99); // capture cancels lead near a hard boundary too
  j = joint(); j.secondaryGoal = 2840; j.lead = 8; j.direction = 1;
  assert(j.reference() == 2848); // wrist remains multi-turn
  j.secondaryGoal = 3000; assert(j.reference() == 3000);

  // Identified dynamics cancel our own motor torque; residual is external only.
  j = joint(); j.inertia = .035f; j.viscous = .018f; j.coulomb = .025f;
  j.measuredValid = true; j.measuredVelocity = 2; j.lastSample = 0;
  const float motorCurrent = (j.inertia*10+j.viscous*2.2f+j.coulomb)/AssistJoint::kTorqueConstant;
  j.currentCap = 910;
  bool write = false;
  assert(!j.sample(100, 2.2f, motorCurrent, 20, false, true, write));
  assert(fabsf(j.effort) < .001f && !write);
  j = joint(); j.bias = 5;
  assert(!j.sample(100, 0, -95, 20, false, true, write));
  assert(fabsf(j.effort-100.0f/11) < .001f); // alpha=dt/(tau+dt)
  assert(j.sample(100, 0, 0, 521, false, true, write) == ASSIST_REASON_FEEDBACK_STALE);
  assert(j.sample(NAN, 0, 0, 540, false, true, write) == ASSIST_REASON_FEEDBACK_INVALID);
  j = joint();
  assert(j.sample(133, 0, 0, 20, false, true, write) == ASSIST_REASON_TRACKING_ERROR);
}
