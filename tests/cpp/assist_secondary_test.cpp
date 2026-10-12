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
  assert(fabsf(j.threshold*AssistJoint::kTorqueConstant-.002f) < 1e-6f);
  assert(j.currentCap == 300);
  assert(j.positionP == AssistJoint::kAssistPositionP);
  assert(j.positionI == 0 && j.positionD == AssistJoint::kAssistPositionD);
  assert(j.targetSlewDegS == AssistJoint::kAssistTargetSlewDegS);
  assert(j.leadMax == AssistJoint::kAssistLeadMaxDeg);
  assert(j.detectDelay == AssistJoint::kAssistIntentDetectDelayS);
  j.effort = .2f/AssistJoint::kTorqueConstant;
  for (int k = 0; k < 1; ++k) j.updateIntent(100, 5, .02f);
  assert(!j.moving && j.secondaryGoal == 100);
  j.effort = 0; j.updateIntent(100, 5, .02f); // reset interrupted dwell
  j.effort = .2f/AssistJoint::kTorqueConstant;
  for (int k = 0; k < 1; ++k) j.updateIntent(100, 5, .02f);
  assert(!j.moving);
  for (int k = 0; k < 2; ++k) j.updateIntent(100, 5, .02f);
  assert(j.moving);
  for (int k = 0; k < 40; ++k) j.updateIntent(100, 5, .02f);
  assert(j.moving && j.secondaryGoal > 110 && j.lead == AssistJoint::kAssistLeadMaxDeg);
  assert(fabsf(j.intentVelocity-(100*AssistJoint::kTorqueConstant)*
               (.2f/AssistJoint::kTorqueConstant)) < 1e-4f && j.currentCap == 300);
  // A measured position jump does not redefine the integrated secondary goal.
  const float old = j.secondaryGoal;
  j.updateIntent(200, 5, .02f);
  assert(fabsf(j.secondaryGoal-old-
               (100*AssistJoint::kTorqueConstant)*
               (.2f/AssistJoint::kTorqueConstant)*.02f) < 1e-4f);
  for (int k = 0; k < 26; ++k) j.updateIntent(200, 0, .02f);
  const float releasedReference = 200 + AssistJoint::kAssistReleaseLeadScale *
      AssistJoint::kAssistLeadMaxDeg;
  assert(!j.moving && j.rearmBlocked &&
         fabsf(j.reference()-releasedReference) < 1e-5f);
  j.effort = 0;
  j.updateIntent(200, 0, .02f);
  assert(j.rearmBlocked && fabsf(j.reference()-releasedReference) < 1e-5f);
  j.updateIntent(200, 0, .02f);
  assert(!j.rearmBlocked && fabsf(j.reference()-releasedReference) < 1e-5f && j.lead == 0);
  j.effort = -200; // a reversal is an intentional new command
  j.updateIntent(200, -5, .02f);
  assert(!j.rearmBlocked && !j.moving && j.secondaryGoal == 200 &&
         j.direction == -1);
  for (int k = 1; k < 10; ++k) j.updateIntent(200, -5, .02f);
  assert(j.moving && j.reference() < 200);
  j.effort = 0;
  for (int k = 0; k < 2; ++k) j.updateIntent(180, 0, .02f);
  assert(!j.moving && j.rearmBlocked && fabsf(j.reference()-180) <=
         AssistJoint::kAssistLeadMaxDeg*AssistJoint::kAssistReleaseLeadScale + .001f);
  // Reversing an active trajectory must not flip the accumulated lead in
  // place, which would create a large one-sample reference jump.
  j = joint(); j.moving = true; j.direction = 1; j.secondaryGoal = 110; j.lead = 8;
  j.effort = -200;
  j.updateIntent(110, 0, .02f);
  assert(j.direction == -1 && j.secondaryGoal == 110 && j.lead == 0 &&
         j.reference() == 110 && j.goalReanchorPending);
  // A reversal during re-arm bypasses the old quiet-only lockout but keeps
  // the normal activation dwell before movement begins.
  j = joint(); j.rearmBlocked = true; j.direction = 1; j.capture = 100;
  j.releaseLead = 2; j.secondaryGoal = 102; j.effort = -200;
  j.updateIntent(105, 0, .02f);
  assert(!j.rearmBlocked && !j.moving && j.secondaryGoal == 105 &&
         j.direction == -1);
  j.effort = 0;
  for (int k = 0; k < 26; ++k) j.updateIntent(180, 0, .02f);
  assert(!j.moving && fabsf(j.reference()-105) < .001f);

  // The telemetry observer remains usable outside Assist.  A disabled motor
  // ignores its residual current at rest, while motion can still contribute
  // model-inferred effort.
  j = joint();
  assert(j.observeTelemetry(100, 0, 50, true, false, false, 0));
  assert(j.observeTelemetry(100, 0, 50, true, false, false, 20));
  assert(fabsf(j.telemetryEffort) < .001f);
  j = joint(); j.inertia = .035f;
  assert(j.observeTelemetry(100, 0, 0, true, false, false, 0));
  assert(j.observeTelemetry(100, 10, 0, false, false, false, 20));
  assert(j.telemetryEffort > 1.0f);

  // Calibration motion must use the limits supplied by the firmware's
  // startup-settling configuration, not the historical 1 deg/s / .3 deg
  // defaults. Keep the defaults for direct callers that do not provide a
  // configuration, while proving the configured 10 deg/s / 5 deg limits.
  bool calibrationWrite = false;
  j = joint(); j.calibrated = false; j.collectionSettleMs = 0;
  assert(j.sample(100.8f, 4.0f, 0, 20, true, false, calibrationWrite) ==
         ASSIST_REASON_CALIBRATION_MOVING);
  j = joint(); j.calibrated = false; j.collectionSettleMs = 0;
  assert(j.sample(100.8f, 4.0f, 0, 20, true, false, calibrationWrite, 10.0f, 5.0f) ==
         ASSIST_REASON_NONE);

  j = joint(); j.hardMin = 99; j.hardMax = 3000;
  j.direction = 1; j.lead = 8; j.releaseAt(99);
  assert(j.reference() == 101); // captured pose plus scaled lead
  j = joint(); j.secondaryGoal = 2840; j.lead = 8; j.direction = 1;
  assert(j.reference() == 2848); // wrist remains multi-turn
  j.secondaryGoal = 3000; assert(j.reference() == 3000);

  // The emitted position target is deliberately rate-limited even when the
  // effort and lead reference change sharply in one feedback sample.
  j = joint(); j.observerTau = 0; j.measuredValid = true;
  j.measuredVelocity = 0; j.lastSample = 0; j.goal = j.secondaryGoal = 100;
  j.moving = true; j.direction = 1;
  bool write = false;
  assert(!j.sample(100, 0, -100, 20, false, true, write));
  assert(write && fabsf(j.goal - (100 + AssistJoint::kAssistTargetAccelDegS2*.02f*.02f)) < 1e-5f);

  // Identified dynamics cancel our own motor torque; residual is external only.
  j = joint(); j.inertia = .035f; j.viscous = .018f; j.coulomb = .025f;
  j.measuredValid = true; j.measuredVelocity = 2; j.lastSample = 0;
  const float motorCurrent = (j.inertia*10+j.viscous*2.2f+j.coulomb)/AssistJoint::kTorqueConstant;
  j.currentCap = 910;
  write = false;
  assert(!j.sample(100, 2.2f, motorCurrent, 20, false, true, write));
  assert(fabsf(j.effort) < .001f && !write);
  j = joint(); j.bias = 5;
  assert(!j.sample(100, 0, -95, 20, false, true, write));
  assert(fabsf(j.effort-100.0f/11) < .001f); // alpha=dt/(tau+dt)
  assert(j.sample(100, 0, 0, 521, false, true, write) == ASSIST_REASON_FEEDBACK_STALE);
  assert(j.sample(NAN, 0, 0, 540, false, true, write) == ASSIST_REASON_FEEDBACK_INVALID);
  j = joint();
  assert(j.sample(170, 0, 0, 20, false, true, write) == ASSIST_REASON_TRACKING_ERROR);
}
