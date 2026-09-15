#ifndef NML_ROM_PULSE_FIT_H
#define NML_ROM_PULSE_FIT_H

#include "joint_state_model.h"

// Midpoint samples held over eight 5-ms bins (Gaussian sigma=8 ms), or four
// 10-ms bins (triangle). Peak normalized, never area normalized: shaping
// reduces exposure. The caller writes zero at 40 ms, including early cutoffs.
inline float romPulseShape(unsigned slot, unsigned stepMs) {
  static const float gaussian[] = {0.095967f, 0.309786f, 0.676634f, 1.0f,
                                  1.0f, 0.676634f, 0.309786f, 0.095967f};
  static const float triangle[] = {1.0f/3, 1.0f, 1.0f, 1.0f/3};
  return stepMs == 5 ? (slot < 8 ? gaussian[slot] : 0) :
         stepMs == 10 ? (slot < 4 ? triangle[slot] : 0) : 0;
}

// Exact first-order integration for gain=1, using completed current-write
// timestamps and rounded register commands. No growing pulse-history buffer.
struct RomPulseExposure {
  float tau = .15f, current = 0, velocity = 0, displacement = 0;
  float seconds = 0, impulse = 0, peakCurrent = 0, peakVelocity = 0;
  uint32_t lastMs = 0;
  void reset(uint32_t now, float timeConstant) {
    *this = RomPulseExposure(); lastMs = now; tau = timeConstant;
  }
  void command(float next, uint32_t now) {
    const float dt = (uint32_t)(now - lastMs) * .001f;
    const float decay = expf(-dt / tau);
    displacement += current * dt + (velocity-current) * tau * (1-decay);
    velocity = current + (velocity-current) * decay;
    peakVelocity = fmaxf(peakVelocity, fabsf(velocity));
    impulse += current * dt; seconds += dt;
    current = next; peakCurrent = fmaxf(peakCurrent, fabsf(next)); lastMs = now;
  }
};

inline float romShapedPulseGain(const JointModelParams& params, float delta,
    float initialVelocity, float finalVelocity, const RomPulseExposure& pulse,
    float offSeconds, float minTravel, const char** reason = nullptr) {
  const auto reject = [reason](const char* why) -> float {
    if (reason) *reason = why;
    return NAN;
  };
  if (!params.valid() || pulse.tau != params.time_constant) return reject("invalid_model");
  if (params.stiffness != 0 || params.moment != 0) return reject("configured_load");
  if (!isfinite(initialVelocity) || !isfinite(finalVelocity)) return reject("velocity_unavailable");
  if (!isfinite(delta) || !isfinite(pulse.displacement) || !isfinite(pulse.velocity) ||
      !isfinite(offSeconds) || offSeconds <= 0 || pulse.seconds <= 0 || pulse.current != 0)
    return reject("invalid_measurement");
  if (fabsf(delta) < minTravel) return reject("insufficient_travel");
  if (delta * pulse.impulse <= 0) return reject("wrong_direction");
  const float tau = params.time_constant, decayOff = expf(-offSeconds/tau);
  const float decayTotal = expf(-(pulse.seconds+offSeconds)/tau);
  const float exposure = pulse.displacement + pulse.velocity*tau*(1-decayOff);
  if (fabsf(exposure) < .000001f) return reject("insufficient_exposure");
  const float gain = (delta - initialVelocity*tau*(1-decayTotal)) / exposure;
  if (!isfinite(gain) || gain < .001f || gain > 10) return reject("gain_out_of_range");
  const float peakBound = fabsf(initialVelocity) + gain*pulse.peakVelocity;
  if (gain*pulse.peakCurrent >= params.max_velocity || peakBound >= params.max_velocity ||
      fabsf(finalVelocity) >= params.max_velocity) return reject("velocity_limit");
  const float tail = initialVelocity*decayTotal + gain*pulse.velocity*decayOff;
  if (fabsf(finalVelocity-tail) > fmaxf(3, .5f*peakBound)) return reject("tail_mismatch");
  if (reason) *reason = "accepted";
  return gain;
}

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
