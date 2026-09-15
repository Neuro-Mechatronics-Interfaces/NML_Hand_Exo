#ifndef NML_ASSIST_CONTROLLER_H
#define NML_ASSIST_CONTROLLER_H
#include <math.h>
#include <stdint.h>

// Measured, position-holding load proxy. Units are raw motor encoder degrees
// and signed PRESENT_CURRENT mA, never estimated telemetry or anatomical torque.
struct AssistJoint {
  float threshold = 10, gain = .01f, currentCap = 40;
  bool configured = false, selected = false, calibrated = false, moving = false;
  float bias = 0, m2 = 0, noise = 0, biasAngle = 0, biasError = 0;
  float goal = 0, angle = 0, effort = 0, deflection = 0, moveStart = 0;
  bool measuredValid = false;
  float measuredCurrent = 0, measuredVelocity = 0;
  uint64_t measuredUtc = 0;
  uint16_t samples = 0, steps = 0;
  uint32_t lastSample = 0, phaseStart = 0, evidenceStart = 0;
  int8_t evidenceDirection = 0;
  static constexpr unsigned kSamples = 50;
  static constexpr float kPoseWindow = 5; // one-pose bias calibration validity
  static constexpr float kMaxStep = .5f;

  void begin(float position, uint32_t now) {
    selected = true; calibrated = moving = measuredValid = false;
    bias = m2 = noise = biasError = effort = deflection = 0;
    samples = steps = 0; goal = angle = biasAngle = position;
    phaseStart = lastSample = now; evidenceStart = 0; evidenceDirection = 0;
  }
  float deadzone() const { return fmaxf(threshold, fmaxf(5, 3*noise)); }
  // Return a reason on failure, nullptr while valid. A new goal is emitted only
  // after sustained agreeing current residual and encoder deflection.
  const char* sample(float q, float velocity, float current, uint32_t now,
                     bool calibrating, bool active, bool& writeGoal) {
    writeGoal = false;
    if (!isfinite(q) || !isfinite(velocity) || !isfinite(current)) return "feedback_invalid";
    if ((uint32_t)(now-lastSample) > 500) return "feedback_stale";
    lastSample = now; angle = q;
    measuredCurrent = current; measuredVelocity = velocity; measuredValid = true;
    if (fabsf(q-biasAngle) > kPoseWindow) return "bias_pose_changed";
    if (fabsf(velocity) > 10) return "speed_limit";
    if (fabsf(q-goal) > 2) return "tracking_error";
    if (fabsf(current) > currentCap+5) return "current_limit";
    if (calibrating && !calibrated) {
      // Let the explicitly captured hold settle before collecting a baseline.
      if (now-phaseStart < 500) return nullptr;
      if (fabsf(velocity) > 1 || fabsf(q-biasAngle) > .3f) return "calibration_moving";
      ++samples;
      const float d = current-bias;
      bias += d/samples; m2 += d*(current-bias);
      biasError += ((q-goal)-biasError)/samples;
      if (samples == kSamples) {
        noise = sqrtf(fmaxf(0, m2/(samples-1)));
        if (fabsf(bias)+deadzone() >= .9f*currentCap) return "insufficient_current_headroom";
        calibrated = true;
      }
      return nullptr;
    }
    effort = -(current-bias); // holding motor opposes applied external effort
    deflection = q-goal-biasError;
    if (!active || !calibrated) return nullptr;
    if (moving) {
      if (fabsf(q-moveStart) > 2) return "step_excursion";
      if (now-phaseStart >= 2000) return "settle_timeout";
      if (now-phaseStart >= 400 && fabsf(velocity) <= 1 && fabsf(q-goal-biasError) <= .3f) {
        moving = false; evidenceDirection = 0; evidenceStart = 0;
      }
      return nullptr; // our own movement never qualifies as user effort
    }
    int8_t direction = effort > deadzone() ? 1 : effort < -deadzone() ? -1 : 0;
    if (fabsf(velocity) > 1 || direction*deflection < .15f) direction = 0;
    if (!direction) { evidenceDirection = 0; evidenceStart = 0; return nullptr; }
    if (direction != evidenceDirection) {
      evidenceDirection = direction; evidenceStart = now; return nullptr;
    }
    if (now-evidenceStart < 150) return nullptr;
    const float step = fminf(kMaxStep, gain*(fabsf(effort)-deadzone()));
    if (step < .1f) return nullptr; // below ~one encoder tick
    // Start beyond measured position, not an old goal behind the user's hand.
    goal = q + direction*step;
    if (fabsf(goal-biasAngle) > kPoseWindow) return "bias_pose_changed";
    moving = true; moveStart = q; phaseStart = now; ++steps;
    evidenceDirection = 0; evidenceStart = 0; writeGoal = true;
    return nullptr;
  }
};
#endif
