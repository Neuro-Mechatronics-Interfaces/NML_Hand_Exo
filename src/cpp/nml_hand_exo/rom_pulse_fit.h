#ifndef NML_ROM_PULSE_FIT_H
#define NML_ROM_PULSE_FIT_H

#include "joint_state_model.h"

// Identify only gain for v'=(gain*I-v)/tau with a rectangular current pulse.
// Starting velocity is measured; zero-current decay remains in the integral.
// This is a local command-to-motion fit, not a torque or gravity calibration.
inline float romPulseGain(const JointModelParams& params, float delta,
                          float initialVelocity, float finalVelocity,
                          float signedCurrent, float onSeconds, float offSeconds,
                          float minTravel, const char** reason = nullptr) {
  const auto reject = [reason](const char* why) -> float {
    if (reason) *reason = why;
    return NAN;
  };
  if (!params.valid()) return reject("invalid_model");
  if (params.stiffness != 0 || params.moment != 0) return reject("configured_load");
  if (!isfinite(initialVelocity) || !isfinite(finalVelocity)) return reject("velocity_unavailable");
  if (!isfinite(delta) || !isfinite(signedCurrent) || signedCurrent == 0 ||
      !isfinite(onSeconds) || !isfinite(offSeconds) || onSeconds <= 0 || offSeconds <= 0)
    return reject("invalid_measurement");
  if (fabsf(delta) < minTravel) return reject("insufficient_travel");
  if (delta * signedCurrent <= 0) return reject("wrong_direction");
  const float tau = params.time_constant;
  const float decayOn = expf(-onSeconds / tau);
  const float decayOff = expf(-offSeconds / tau);
  const float carry = initialVelocity * tau * (1.0f - decayOn * decayOff);
  const float exposure = signedCurrent * (onSeconds - tau * (1.0f - decayOn) * decayOff);
  if (fabsf(exposure) < 0.000001f) return reject("insufficient_exposure");
  const float gain = (delta - carry) / exposure;
  const float peak = initialVelocity * decayOn + gain * signedCurrent * (1.0f - decayOn);
  if (!isfinite(gain) || gain < 0.001f || gain > 10.0f) return reject("gain_out_of_range");
  if (fabsf(gain * signedCurrent) >= params.max_velocity ||
      fabsf(initialVelocity) >= params.max_velocity || fabsf(finalVelocity) >= params.max_velocity ||
      fabsf(peak) >= params.max_velocity) return reject("velocity_limit");
  // A large mismatch to the measured tail velocity indicates recoil, contact,
  // or dynamics this one-parameter fit cannot explain.
  if (fabsf(finalVelocity - peak * decayOff) > fmaxf(3.0f, 0.5f * fabsf(peak))) return reject("tail_mismatch");
  if (reason) *reason = "accepted";
  return gain;
}

#endif
