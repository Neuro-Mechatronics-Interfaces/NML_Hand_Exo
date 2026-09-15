#ifndef NML_PULSE_RESPONSE_MODEL_H
#define NML_PULSE_RESPONSE_MODEL_H
#include <math.h>
#include <stdint.h>

// Local, directional displacement response to a fixed-duration current pulse.
// Empirical sensitivity, not physical joint stiffness. No bus access.
struct PulseResponseModel {
  float slope = NAN; // motor degrees per mA at the calibrated pulse duration
  float anchorCurrent = NAN, anchorTravel = NAN;
  float previousCurrent = NAN, previousTravel = NAN;
  float lastNoMotionCurrent = NAN;
  uint16_t movingSamples = 0, noMotionSamples = 0, gradientSamples = 0;
  static float bound(float v, float lo, float hi) { return fminf(hi, fmaxf(lo, v)); }

  bool observe(float current, float travel, float noiseFloor) {
    if (!isfinite(current) || current <= 0 || !isfinite(travel) || travel < -noiseFloor) return false;
    if (travel < noiseFloor) {
      // Censored observation: insufficient motion, not zero gain/infinite stiffness.
      if (noMotionSamples < 65535) ++noMotionSamples;
      lastNoMotionCurrent = current;
      // Do not use a censored zero as an exact point in the secant gradient.
      previousCurrent = previousTravel = NAN;
      return true;
    }
    if (movingSamples < 65535) ++movingSamples;
    if (isfinite(previousCurrent) && fabsf(current - previousCurrent) >= 1.9f) {
      const float dy = travel - previousTravel;
      const float gradient = dy / (current - previousCurrent);
      if (fabsf(dy) >= noiseFloor * 0.5f && gradient >= 0.001f && gradient <= 1.0f) {
        // A noisy secant cannot abruptly replace the established sensitivity.
        slope = isfinite(slope) ? slope + 0.25f *
            (bound(gradient, slope * 0.75f, slope * 1.25f) - slope) : gradient;
        if (gradientSamples < 65535) ++gradientSamples;
      }
    }
    if (!isfinite(slope)) slope = bound(travel / current, 0.001f, 1.0f);
    anchorCurrent = previousCurrent = current;
    anchorTravel = previousTravel = travel;
    return true;
  }
  float predict(float current) const {
    if (!isfinite(slope) || !isfinite(current)) return NAN;
    return fmaxf(0.0f, anchorTravel + slope * (current - anchorCurrent));
  }
  float nextCurrent(float current, float observed, float target, float noiseFloor,
                    float step, float minimum, float maximum) const {
    float candidate = current;
    if (observed < noiseFloor) candidate += step;
    else if (fabsf(target - observed) > noiseFloor * 0.5f) {
      candidate += isfinite(slope) ? (target - observed) / slope :
                   (target > observed ? step : -step);
    }
    return bound(bound(candidate, current - step, current + step), minimum, maximum);
  }
};
#endif
