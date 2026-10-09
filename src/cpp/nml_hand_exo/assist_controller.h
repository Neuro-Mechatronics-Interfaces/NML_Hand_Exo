#ifndef NML_ASSIST_CONTROLLER_H
#define NML_ASSIST_CONTROLLER_H
#include <math.h>
#include <stdint.h>

enum AssistReason : uint8_t {
  ASSIST_REASON_NONE = 0,
  ASSIST_REASON_OFF = 1,
  ASSIST_REASON_STOPPED = 2,
  ASSIST_REASON_FEEDBACK_INVALID = 3,
  ASSIST_REASON_FEEDBACK_STALE = 4,
  ASSIST_REASON_SPEED_LIMIT = 5,
  ASSIST_REASON_TRACKING_ERROR = 6,
  ASSIST_REASON_CURRENT_LIMIT = 7,
  ASSIST_REASON_CALIBRATION_MOVING = 8,
  ASSIST_REASON_INSUFFICIENT_CURRENT_HEADROOM = 9,
  ASSIST_REASON_CALIBRATION_BUSY = 10,
  ASSIST_REASON_REQUIRES_CURRENT_POSITION = 11,
  ASSIST_REASON_INVALID_TARGETS = 12,
  ASSIST_REASON_UNKNOWN_MOTOR = 13,
  ASSIST_REASON_DUPLICATE_MOTOR = 14,
  ASSIST_REASON_NOT_CONFIGURED = 15,
  ASSIST_REASON_POSITION_HOLD_ACTIVE = 16,
  ASSIST_REASON_MOTOR_CURRENT_LIMIT = 17,
  ASSIST_REASON_UNSELECTED_MOTOR_ENABLED = 18,
  ASSIST_REASON_CURRENT_BUDGET = 19,
  ASSIST_REASON_FEEDBACK_OR_TORQUE_READ = 20,
  ASSIST_REASON_NOT_SETTLED = 21,
  ASSIST_REASON_STORED_LIMIT = 22,
  ASSIST_REASON_OPERATING_MODE_READ = 23,
  ASSIST_REASON_MOTOR_REQUIRES_CURRENT_POSITION = 24,
  ASSIST_REASON_UNSUPPORTED_DRIVE_MODE = 25,
  ASSIST_REASON_UNSELECTED_TORQUE_READ = 26,
  ASSIST_REASON_CALIBRATING = 27,
  ASSIST_REASON_START_TORQUE_OFF_FAILED = 28,
  ASSIST_REASON_START_POSE_CHANGED = 29,
  ASSIST_REASON_START_WRITE_FAILED = 30,
  ASSIST_REASON_HEARTBEAT_TIMEOUT = 31,
  ASSIST_REASON_START_TORQUE_ON_FAILED = 32,
  ASSIST_REASON_ACTIVE = 33,
  ASSIST_REASON_TORQUE_OFF_UNCONFIRMED = 34,
  ASSIST_REASON_STOP_RETRIED = 35,
  ASSIST_REASON_MODE_CHANGED = 36,
  ASSIST_REASON_CALIBRATION_TIMEOUT = 37,
  ASSIST_REASON_DISABLED = 38,
  ASSIST_REASON_NO_TARGETS = 39,
  ASSIST_REASON_FEEDBACK_READ = 40,
  ASSIST_REASON_GOAL_WRITE_OR_LIMIT = 41,
  ASSIST_REASON_READY = 42,
  ASSIST_REASON_UNKNOWN = 43,
  ASSIST_REASON_EXTERNAL_STOP = 44,
};
inline const char* assistReasonName(AssistReason reason) {
  switch (reason) {
    case ASSIST_REASON_NONE: return "none";
    case ASSIST_REASON_OFF: return "off";
    case ASSIST_REASON_STOPPED: return "stopped";
    case ASSIST_REASON_FEEDBACK_INVALID: return "feedback_invalid";
    case ASSIST_REASON_FEEDBACK_STALE: return "feedback_stale";
    case ASSIST_REASON_SPEED_LIMIT: return "speed_limit";
    case ASSIST_REASON_TRACKING_ERROR: return "tracking_error";
    case ASSIST_REASON_CURRENT_LIMIT: return "current_limit";
    case ASSIST_REASON_CALIBRATION_MOVING: return "calibration_moving";
    case ASSIST_REASON_INSUFFICIENT_CURRENT_HEADROOM: return "insufficient_current_headroom";
    case ASSIST_REASON_CALIBRATION_BUSY: return "calibration_busy";
    case ASSIST_REASON_REQUIRES_CURRENT_POSITION: return "requires_current_position";
    case ASSIST_REASON_INVALID_TARGETS: return "invalid_targets";
    case ASSIST_REASON_UNKNOWN_MOTOR: return "unknown_motor";
    case ASSIST_REASON_DUPLICATE_MOTOR: return "duplicate_motor";
    case ASSIST_REASON_NOT_CONFIGURED: return "not_configured";
    case ASSIST_REASON_POSITION_HOLD_ACTIVE: return "position_hold_active";
    case ASSIST_REASON_MOTOR_CURRENT_LIMIT: return "motor_current_limit";
    case ASSIST_REASON_UNSELECTED_MOTOR_ENABLED: return "unselected_motor_enabled";
    case ASSIST_REASON_CURRENT_BUDGET: return "current_budget";
    case ASSIST_REASON_FEEDBACK_OR_TORQUE_READ: return "feedback_or_torque_read";
    case ASSIST_REASON_NOT_SETTLED: return "not_settled";
    case ASSIST_REASON_STORED_LIMIT: return "stored_limit";
    case ASSIST_REASON_OPERATING_MODE_READ: return "operating_mode_read";
    case ASSIST_REASON_MOTOR_REQUIRES_CURRENT_POSITION: return "motor_requires_current_position";
    case ASSIST_REASON_UNSUPPORTED_DRIVE_MODE: return "unsupported_drive_mode";
    case ASSIST_REASON_UNSELECTED_TORQUE_READ: return "unselected_torque_read";
    case ASSIST_REASON_CALIBRATING: return "calibrating";
    case ASSIST_REASON_START_TORQUE_OFF_FAILED: return "start_torque_off_failed";
    case ASSIST_REASON_START_POSE_CHANGED: return "start_pose_changed";
    case ASSIST_REASON_START_WRITE_FAILED: return "start_write_failed";
    case ASSIST_REASON_HEARTBEAT_TIMEOUT: return "heartbeat_timeout";
    case ASSIST_REASON_START_TORQUE_ON_FAILED: return "start_torque_on_failed";
    case ASSIST_REASON_ACTIVE: return "active";
    case ASSIST_REASON_TORQUE_OFF_UNCONFIRMED: return "torque_off_unconfirmed";
    case ASSIST_REASON_STOP_RETRIED: return "stop_retried";
    case ASSIST_REASON_MODE_CHANGED: return "mode_changed";
    case ASSIST_REASON_CALIBRATION_TIMEOUT: return "calibration_timeout";
    case ASSIST_REASON_DISABLED: return "disabled";
    case ASSIST_REASON_NO_TARGETS: return "no_targets";
    case ASSIST_REASON_FEEDBACK_READ: return "feedback_read";
    case ASSIST_REASON_GOAL_WRITE_OR_LIMIT: return "goal_write_or_limit";
    case ASSIST_REASON_READY: return "ready";
    case ASSIST_REASON_UNKNOWN: return "unknown";
    case ASSIST_REASON_EXTERNAL_STOP: return "external_stop";
  }
  return "unknown";
}

// Transport-independent numeric snapshot; serializing it never touches the bus.
struct AssistJointStatus {
  uint8_t id = 0;
  uint16_t samples = 0, steps = 0;
  float bias_mA = 0, noise_mA = 0, deadzone_mA = 0, effort_mA = 0;
  float deflection_deg = 0, angle = 0, goal = 0;
  bool moving = false;
  float secondary_goal = 0, intent_deg_s = 0, lead_deg = 0;
  bool rearm_blocked = false;
  uint32_t age_ms = 0;
  float current_cap_mA = 0;
};
struct AssistStatusSnapshot {
  uint8_t state = 0, count = 0;
  AssistReason reason = ASSIST_REASON_OFF;
  int reject_id = -1;
  float reject_angle_deg = NAN, reject_velocity_deg_s = NAN;
  uint16_t total_current_budget_mA = 0;
  AssistJointStatus joints[18];
};

// Largest contiguous block used by assist: profiles + goal (108..119).
struct AssistRegisterBlock { uint8_t bytes[12] = {}; };
struct AssistFeedback { float angle = 0, velocity = 0, current = 0; };

// Raw multi-turn encoder degrees and measured mA. Secondary-goal defaults
// mirror the simulator; physical observer terms require hardware identification.
struct AssistJoint {
  static constexpr float kTorqueConstant = .00115f;
  static constexpr unsigned kSamples = 50;
  float threshold = .08f/kTorqueConstant, gain = 80*kTorqueConstant, currentCap = 300;
  float observerTau = .20f, inertia = 0, viscous = 0, coulomb = 0;
  float intentCap = .25f/kTorqueConstant;
  float leadMax = 8, leadRate = 24, detectDelay = .50f;
  float stallVelocity = 1, stallDelay = .50f;
  float speedLimit = 60, trackingLimit = 32;
  uint16_t positionP = 350, positionI = 0, positionD = 160;
  float hardMin = 0, hardMax = 0;
  bool configured = false, selected = false, calibrated = false, moving = false;
  bool rearmBlocked = false;
  float bias = 0, m2 = 0, noise = 0, biasAngle = 0, biasError = 0;
  float goal = 0, secondaryGoal = 0, intentVelocity = 0, lead = 0, capture = 0;
  float angle = 0, effort = 0, deflection = 0, estimate = 0;
  float intentTimer = 0, releaseTimer = 0, stallTimer = 0;
  int8_t direction = 0;
  bool measuredValid = false;
  float measuredCurrent = 0, measuredVelocity = 0;
  uint64_t measuredUtc = 0;
  uint16_t samples = 0, steps = 0; // steps counts activation episodes in v2
  uint32_t lastSample = 0, phaseStart = 0;
  uint16_t collectionSettleMs = 500;

  static float bound(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
  static int8_t sign(float x) { return x > 0 ? 1 : x < 0 ? -1 : 0; }
  void begin(float position, uint32_t now) {
    selected = true; calibrated = moving = measuredValid = rearmBlocked = false;
    bias = m2 = noise = biasError = effort = deflection = estimate = 0;
    intentVelocity = lead = intentTimer = releaseTimer = stallTimer = 0; direction = 0;
    measuredVelocity = 0;
    samples = steps = 0; goal = secondaryGoal = capture = angle = biasAngle = position;
    phaseStart = lastSample = now;
  }
  float deadzone() const { return fmaxf(threshold, fmaxf(5, 3*noise)); }
  void releaseAt(float q) {
    moving = false; rearmBlocked = true; capture = q;
    intentVelocity = intentTimer = releaseTimer = stallTimer = 0;
    // Cancel lead in the same observation; avoid a final outward kick.
    secondaryGoal = bound(q-direction*lead, hardMin, hardMax);
  }
  void updateIntent(float q, float velocity, float dt) {
    const float intent = bound(effort, -intentCap, intentCap);
    const float magnitude = fabsf(intent), thresholdMa = deadzone();
    if (rearmBlocked) {
      releaseTimer = magnitude <= thresholdMa*.5f ? releaseTimer+dt : 0;
      moving = false; intentVelocity = stallTimer = 0;
      lead = fmaxf(0, lead-leadRate*dt);
      secondaryGoal = bound(capture-direction*lead, hardMin, hardMax);
      if (releaseTimer >= detectDelay) { rearmBlocked = false; releaseTimer = 0; }
      return;
    }
    if (magnitude >= thresholdMa) { intentTimer += dt; releaseTimer = 0; }
    else { intentTimer = 0; releaseTimer = moving ? releaseTimer+dt : 0; }
    if (!moving && intentTimer >= detectDelay) { moving = true; ++steps; }
    if (moving && releaseTimer >= detectDelay) { releaseAt(q); return; }
    if (moving) {
      intentVelocity = gain*intent;
      const int8_t nextDirection = sign(intentVelocity);
      if (nextDirection) {
        direction = nextDirection;
        stallTimer = direction*velocity < stallVelocity ? stallTimer+dt : 0;
        secondaryGoal = bound(secondaryGoal+intentVelocity*dt, hardMin, hardMax);
        lead = fminf(leadMax, lead+leadRate*dt);
        if (stallTimer >= stallDelay) releaseAt(q);
      }
    } else {
      intentVelocity = stallTimer = 0;
      lead = fmaxf(0, lead-leadRate*dt);
    }
  }
  float reference() const {
    if (rearmBlocked) return bound(capture, hardMin, hardMax);
    return bound(secondaryGoal+direction*lead, hardMin, hardMax);
  }
  AssistReason sample(float q, float velocity, float current, uint32_t now,
                     bool calibrating, bool active, bool& writeGoal) {
    writeGoal = false;
    if (!isfinite(q) || !isfinite(velocity) || !isfinite(current)) return ASSIST_REASON_FEEDBACK_INVALID;
    const uint32_t elapsed = now-lastSample;
    if (elapsed > 500) return ASSIST_REASON_FEEDBACK_STALE;
    const float dt = elapsed*.001f;
    const float previousVelocity = measuredVelocity;
    const bool hadMeasurement = measuredValid;
    lastSample = now; angle = q;
    measuredCurrent = current; measuredVelocity = velocity; measuredValid = true;
    if (fabsf(velocity) > speedLimit) return ASSIST_REASON_SPEED_LIMIT;
    if (fabsf(q-goal) > trackingLimit) return ASSIST_REASON_TRACKING_ERROR;
    if (fabsf(current) > currentCap+5) return ASSIST_REASON_CURRENT_LIMIT;
    if (calibrating && !calibrated) {
      if (now-phaseStart < collectionSettleMs) return ASSIST_REASON_NONE;
      if (fabsf(velocity) > 1 || fabsf(q-biasAngle) > .3f) return ASSIST_REASON_CALIBRATION_MOVING;
      ++samples;
      const float d = current-bias;
      bias += d/samples; m2 += d*(current-bias);
      biasError += ((q-goal)-biasError)/samples;
      if (samples == kSamples) {
        noise = sqrtf(fmaxf(0, m2/(samples-1)));
        if (fabsf(bias)+deadzone() >= .9f*currentCap) return ASSIST_REASON_INSUFFICIENT_CURRENT_HEADROOM;
        calibrated = true;
      }
      return ASSIST_REASON_NONE;
    }
    if (!calibrated || dt <= 0) return ASSIST_REASON_NONE;
    // External contacts belong to the environment. This observer models only
    // motor/load dynamics; never consume predicted telemetry as effort.
    const float acceleration = hadMeasurement ? (velocity-previousVelocity)/dt : 0;
    const float raw = (inertia*acceleration+viscous*velocity+coulomb*sign(velocity))/kTorqueConstant-current;
    estimate += dt/(fmaxf(0, observerTau)+dt)*((raw+bias)-estimate);
    effort = estimate;
    deflection = q-goal-biasError;
    if (!active) return ASSIST_REASON_NONE;
    updateIntent(q, velocity, dt);
    const float next = reference();
    if (fabsf(next-goal) >= .05f) { goal = next; writeGoal = true; }
    return ASSIST_REASON_NONE;
  }
};
#endif
