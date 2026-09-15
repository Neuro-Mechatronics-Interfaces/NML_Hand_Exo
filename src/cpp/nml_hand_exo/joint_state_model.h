#ifndef NML_JOINT_STATE_MODEL_H
#define NML_JOINT_STATE_MODEL_H

#include <math.h>
#include <stdint.h>
#include "pulse_response_model.h"

// Integration resolution is independent of the host polling/reporting rate.
#ifndef EXO_MODEL_STEP_MS
#define EXO_MODEL_STEP_MS 1
#endif
static_assert(EXO_MODEL_STEP_MS >= 1 && EXO_MODEL_STEP_MS <= 10,
              "EXO_MODEL_STEP_MS must be 1..10");

// Telemetry-only dynamics. No bus access and no authority over motor goals.
struct JointModelParams {
  float gain = 0.1f;             // deg/s per mA
  float time_constant = 0.15f;   // s
  float max_velocity = 60.0f;    // deg/s
  float stiffness = 0.0f;       // Nm/deg relative to calibrated home
  float moment = 0.0f;          // Nm, signed resisting load

  bool valid() const {
    return isfinite(gain) && gain >= 0.001f && gain <= 10.0f &&
        isfinite(time_constant) && time_constant >= 0.01f && time_constant <= 10.0f &&
        isfinite(max_velocity) && max_velocity >= 0.1f && max_velocity <= 300.0f &&
        isfinite(stiffness) && stiffness >= 0.0f && stiffness <= 1.0f &&
        isfinite(moment) && moment >= -1.0f && moment <= 1.0f;
  }
};

struct JointStateModel {
  // Telemetry-only deadband, approximately one 4096-tick encoder count.
  // Never used by the physical controller to certify arrival.
  static constexpr float POSITION_SETTLE_DEG = 0.1f;
  JointModelParams params;
  // Only explicitly armed ROM uses these to choose current; advance() never actuates.
  PulseResponseModel pulseResponses[2]; // flex / extend
  float angle = 0.0f;
  float velocity = 0.0f;         // encoder deg/s
  float current = 0.0f;          // signed mA
  float trigger = NAN;          // interface only, no automatic actuation
  bool valid = false;
  bool currentValid = false;
  uint32_t lastTick = 0;

  static float bound(float value, float lo, float hi) {
    return fminf(hi, fmaxf(lo, value));
  }

  bool atPositionGoal(float goal) const {
    return isfinite(goal) && fabsf(goal - angle) <= POSITION_SETTLE_DEG;
  }

  void estimatePositionCurrent(float goal, float home, float effort,
                               float torqueConstant) {
    if (!currentValid) return;
    if (!atPositionGoal(goal)) {
      current = goal < angle ? -fabsf(effort) : fabsf(effort);
      return;
    }
    // A stationary joint needs the modeled holding load, not its full current
    // allowance. Keep the sign of the load even when approaching from above.
    const float load = params.moment + params.stiffness * (angle - home);
    if (!isfinite(load) || !isfinite(torqueConstant) || torqueConstant <= 0.0f) {
      currentValid = false;
      current = 0.0f;
      return;
    }
    current = bound(load / torqueConstant, -fabsf(effort), fabsf(effort));
  }

  void correct(float position, float speed, float effort, bool effortValid,
               uint32_t now) {
    valid = isfinite(position);
    if (!valid) return;
    angle = position;
    velocity = isfinite(speed) ? speed : 0.0f;
    currentValid = effortValid && isfinite(effort);
    current = currentValid ? effort : 0.0f;
    lastTick = now;
  }

  void advance(uint32_t now, float lo, float hi, float home,
               bool moving, bool positional, float goal,
               float effort, bool effortValid, float commandedVelocity,
               float torqueConstant) {
    if (!valid || !isfinite(lo) || !isfinite(hi) || lo > hi) return;
    currentValid = effortValid && isfinite(effort);
    current = currentValid ? effort : 0.0f;
    if (positional) estimatePositionCurrent(goal, home, effort, torqueConstant);
    const bool settled = positional && atPositionGoal(goal);
    if (!moving || settled) velocity = 0.0f;
    const uint32_t elapsed = now - lastTick; // rollover-safe
    if (elapsed < EXO_MODEL_STEP_MS) return;
    lastTick = now;
    const uint32_t boundedMs = elapsed < 100 ? elapsed : 100;
    angle = bound(angle, lo, hi);
    if (!moving || settled) return;

    const uint32_t steps = (boundedMs + EXO_MODEL_STEP_MS - 1) / EXO_MODEL_STEP_MS;
    const float dt = boundedMs * 0.001f / steps;
    const float alpha = 1.0f - expf(-dt / params.time_constant);
    for (uint32_t i = 0; i < steps; ++i) {
      advanceStep(dt, alpha, lo, hi, home, positional, goal, effort,
                  commandedVelocity, torqueConstant);
      if (positional && atPositionGoal(goal)) break;
    }
  }

  void advanceStep(float dt, float alpha, float lo, float hi, float home,
                   bool positional, float goal, float effort,
                   float commandedVelocity, float torqueConstant) {
    float desired;
    const float error = goal - angle;
    const float load = params.moment + params.stiffness * (angle - home);
    if (positional) {
      // A current ceiling is not a current measurement; use its magnitude
      // as the available effort, directed toward the commanded target.
      const float sign = error >= 0.0f ? 1.0f : -1.0f;
      if (currentValid) current = sign * fabsf(effort);
      const float available = currentValid ?
          fmaxf(0.0f, fabsf(effort) - sign * load / torqueConstant) * params.gain :
          params.max_velocity;
      desired = sign * fminf(available, params.max_velocity);
      if (velocity * error < 0.0f) velocity = 0.0f;
    } else if (currentValid) {
      desired = (effort - load / torqueConstant) * params.gain;
    } else {
      desired = commandedVelocity;
    }
    desired = bound(desired, -params.max_velocity, params.max_velocity);
    // Exact first-order velocity and displacement for constant input during
    // this substep. Re-evaluate position-dependent load on the next substep.
    float step = desired * dt + (velocity - desired) * params.time_constant * alpha;
    velocity += (desired - velocity) * alpha;
    if (positional) {
      if (step * error < 0.0f) step = 0.0f;
      if (fabsf(step) >= fabsf(error)) { step = error; velocity = 0.0f; }
    }
    const float next = bound(angle + step, lo, hi);
    if (next == lo || next == hi) velocity = 0.0f;
    angle = next;
    if (positional) {
      estimatePositionCurrent(goal, home, effort, torqueConstant);
      if (atPositionGoal(goal)) velocity = 0.0f;
    }
  }
};

// A shared-bus debounce, separate from per-joint provenance. No value is
// labelled measured merely because this gate opens: a read must succeed.
struct JointTelemetryGate {
  uint32_t lastActivity = 0;
  uint32_t holdoff = 250;
  bool seenActivity = false;
  void note(uint32_t now) { lastActivity = now; seenActivity = true; }
  bool estimated(uint32_t now, bool active) {
    if (active) note(now);
    return active || (seenActivity && uint32_t(now - lastActivity) < holdoff);
  }
};

#endif
