#include "config.h"
#if EXO_USB_PROTOBUF
#include "protobuf_usb.h"
#include "protobuf_frame.h"
#include "nml_hand_exo.h"
#include "gesture_controller.h"
#include "gesture_library.h"
#include "io_profile.h"
#include "utils.h"
#include <pb_decode.h>
#include <pb_encode.h>
#include "protocol/exo_usb.pb.h"
#include "protocol/exo_usb.pb.c"
#include <string.h>

static exo_pb::Reader reader;
static exo_pb::Reader readerV2(2);

static void packAssistStatus(NMLHandExo& exo, exo_usb_Response& response) {
  AssistStatusSnapshot snapshot;
  exo.getAssistSnapshot(snapshot);
  response.has_assist = true;
  auto& status = response.assist;
  status.state = static_cast<exo_usb_AssistState>(snapshot.state);
  status.reason = static_cast<exo_usb_AssistReason>(snapshot.reason);
  status.has_reject_id = snapshot.reject_id >= 0;
  if (status.has_reject_id) status.reject_id = snapshot.reject_id;
  status.has_reject_angle_deg = status.has_reject_id && isfinite(snapshot.reject_angle_deg);
  status.has_reject_velocity_deg_s = status.has_reject_id && isfinite(snapshot.reject_velocity_deg_s);
  status.reject_angle_deg = snapshot.reject_angle_deg;
  status.reject_velocity_deg_s = snapshot.reject_velocity_deg_s;
  status.joints_count = snapshot.count;
  status.has_total_current_budget_mA = true;
  status.total_current_budget_mA = snapshot.total_current_budget_mA;
  for (uint8_t i = 0; i < snapshot.count; ++i) {
    const auto& source = snapshot.joints[i]; auto& target = status.joints[i];
    target.id = source.id;
    target.samples = source.samples;
    target.bias_mA = source.bias_mA;
    target.noise_mA = source.noise_mA;
    target.deadzone_mA = source.deadzone_mA;
    target.effort_mA = source.effort_mA;
    target.deflection_deg = source.deflection_deg;
    target.angle = source.angle;
    target.goal = source.goal;
    target.moving = source.moving;
    target.secondary_goal = source.secondary_goal;
    target.intent_deg_s = source.intent_deg_s;
    target.lead_deg = source.lead_deg;
    target.rearm_blocked = source.rearm_blocked;
    target.steps = source.steps;
    target.age_ms = source.age_ms;
    target.has_current_cap_mA = true;
    target.current_cap_mA = source.current_cap_mA;
  }
}
static_assert(exo_usb_AssistStatus_size + 16 <= exo_pb::MAX_RESPONSE,
              "Assist status must fit the bounded response envelope");


// Fixed-size wire messages, no ASCII serialization or parseMessage dispatch.
// Gesture resolution retains the existing controller's String-based name API.
static void executeRequest(NMLHandExo& exo, GestureController& gc, const exo_usb_Request& request,
                           exo_usb_Response& response) {
  response.sequence = request.sequence;
  response.status = exo_usb_Status_INVALID_REQUEST;
  const bool assist = request.command >= exo_usb_Command_ASSIST_CONFIG && request.command <= exo_usb_Command_ASSIST_ENABLE;
  if (assist) {
    if (request.has_value || request.has_joint_model || request.has_finger_angles || request.has_gesture) return;
    for (uint8_t k = 0; k < request.motor_ids_count; ++k) {
      if (request.motor_ids[k] > 253 || exo.getIndexById(request.motor_ids[k]) < 0) return;
      for (uint8_t j = 0; j < k; ++j) if (request.motor_ids[k] == request.motor_ids[j]) return;
    }
    bool ok = false;
    if (request.command == exo_usb_Command_ASSIST_CONFIG || request.command == exo_usb_Command_ASSIST_OBSERVER) {
      if (request.motor_ids_count != 1 || request.values_count != 3) return;
      ok = request.command == exo_usb_Command_ASSIST_CONFIG
          ? exo.configureAssist(request.motor_ids[0], request.values[0], request.values[1], request.values[2])
          : exo.configureAssistObserver(request.motor_ids[0], request.values[0], request.values[1], request.values[2]);
    } else if (request.command == exo_usb_Command_ASSIST_CALIBRATE || request.command == exo_usb_Command_ASSIST_ENABLE) {
      const bool enable = request.command == exo_usb_Command_ASSIST_ENABLE;
      if (!request.motor_ids_count || request.values_count != (enable ? 6 : 0)) return;
      uint8_t ids[N_MOTORS];
      if (request.motor_ids_count > N_MOTORS) return;
      for (uint8_t k = 0; k < request.motor_ids_count; ++k) ids[k] = request.motor_ids[k];
      ok = enable ? exo.enableTorqueAssist(ids, request.motor_ids_count,
          request.values[0], request.values[1], request.values[2], request.values[3], request.values[4], request.values[5])
          : exo.calibrateAssist(ids, request.motor_ids_count);
    } else {
      if (request.motor_ids_count || request.values_count) return;
      if (request.command == exo_usb_Command_ASSIST_START) ok = exo.startAssist();
      else if (request.command == exo_usb_Command_ASSIST_HEARTBEAT) ok = exo.heartbeatAssist();
      else if (request.command == exo_usb_Command_ASSIST_STOP) { exo.stopAssist(); ok = true; }
      else if (request.command == exo_usb_Command_ASSIST_STATUS) ok = true;
    }
    response.status = ok ? exo_usb_Status_OK : exo_usb_Status_REJECTED;
    packAssistStatus(exo, response);
    return;
  }
  if (exo.isAssistBusy() && request.command != exo_usb_Command_GET_TELEMETRY_FAST &&
      request.command != exo_usb_Command_STOP) { response.status = exo_usb_Status_REJECTED; return; }
  const bool telemetry = request.command == exo_usb_Command_GET_TELEMETRY_FAST;
  const bool stop = request.command == exo_usb_Command_STOP;
  const bool model = request.command == exo_usb_Command_SET_JOINT_MODEL;
  const bool batch = request.command == exo_usb_Command_SET_ANGLES ||
      request.command == exo_usb_Command_SET_ABSOLUTE_ANGLES || request.command == exo_usb_Command_SET_CURRENTS;
  const bool currentLimits = request.command == exo_usb_Command_SET_CURRENT_LIMITS;
  const bool fingers = request.command == exo_usb_Command_SET_FINGER_ANGLES;
  const bool gesture = request.command == exo_usb_Command_SET_GESTURE;
  const bool gestureAngle = request.command == exo_usb_Command_SET_GESTURE_ANGLE;
  // Reject conflicting fields before any resolution or motor traffic.
  if (request.has_joint_model != model || request.has_finger_angles != fingers ||
      request.has_gesture != (gesture || gestureAngle) || (!batch && request.values_count)) return;
  if (model) {
    if (!request.has_joint_model || request.has_value || request.motor_ids_count != 1) return;
  } else if (batch || currentLimits) {
    if (request.has_value || !request.motor_ids_count || request.values_count != request.motor_ids_count) return;
    for (uint8_t k = 0; k < request.values_count; ++k) if (!isfinite(request.values[k])) return;
  } else if (fingers || gesture || gestureAngle) {
    if (request.motor_ids_count || request.has_value != gestureAngle) return;
    if (gestureAngle && (!isfinite(request.value) || request.gesture.has_pose)) return;
    if (gesture && (!request.gesture.has_pose || !request.gesture.pose[0])) return;
    if ((gesture || gestureAngle) && !request.gesture.name[0]) return;
  } else if (telemetry || stop) {
    if (request.has_value || (telemetry && request.motor_ids_count == 0)) return;
  } else if (request.motor_ids_count != 1 || !request.has_value || !isfinite(request.value)) return;
  uint8_t ids[18];
  for (uint8_t i = 0; i < request.motor_ids_count; ++i) {
    const uint32_t id = request.motor_ids[i];
    if (id < 1 || id > 253 || exo.getIndexById(id) < 0) return;
    for (uint8_t j = 0; j < i; ++j) if (ids[j] == id) return;
    ids[i] = id;
  }
  bool ok = false;
  switch (request.command) {
#if EXO_USB_PROTOBUF_V2
    case exo_usb_Command_SET_CURRENT_LIMITS:
      ok = exo.setCurrentLimits(ids, request.values, request.values_count);
      response.has_batch = true;
      response.batch.has_motors = true;
      response.batch.motors = request.values_count;
      break;
#endif
    case exo_usb_Command_SET_CURRENTS:
      ok = exo.setGoalCurrents(ids, request.values, request.values_count);
      break;
    case exo_usb_Command_SET_ANGLES:
    case exo_usb_Command_SET_ABSOLUTE_ANGLES: {
      if (!exo.modeControlsPosition()) break;
      MotorAngleTarget targets[18];
      for (uint8_t k = 0; k < request.values_count; ++k) {
        float angle = request.values[k];
        if (request.command == exo_usb_Command_SET_ANGLES)
          angle = exo.getZeroAngle(ids[k]) + (exo.getFlipMotor(ids[k]) ? -angle : angle);
        if (!isfinite(angle)) return;
        targets[k] = {ids[k], angle};
      }
      uint8_t written = 0, skipped = 0;
      ok = exo.setAbsoluteAnglesSync(targets, request.values_count, &written, &skipped);
      response.has_batch = true;
      response.batch.has_motors = response.batch.has_skipped_offline = true;
      response.batch.motors = written;
      response.batch.skipped_offline = skipped;
      break;
    }
    case exo_usb_Command_SET_FINGER_ANGLES: {
      const auto& f = request.finger_angles;
      const bool present[] = {f.has_thumb, f.has_index, f.has_middle, f.has_ring, f.has_pinky, f.has_wrist};
      const int32_t values[] = {f.thumb, f.index, f.middle, f.ring, f.pinky, f.wrist};
      static const char* const names[] = {"thumb", "index", "middle", "ring", "pinky", "wrist"};
      bool any = false;
      for (uint8_t k = 0; k < 6; ++k) {
        if (present[k] && (values[k] < -100 || values[k] > 100)) return;
        any |= present[k];
      }
      if (!any) return;
      if (!exo.modeControlsPosition()) break;
      MotorAngleTarget targets[18];
      uint8_t count = 0, commanded = 0, held = 0, unknown = 0, zero = 0;
      for (uint8_t k = 0; k < 6; ++k) {
        if (!present[k]) { ++held; continue; }
        MotorAngleTarget resolved[18];
        uint8_t n = 0, stuck = 0;
        if (!gc.resolveGestureSignedTargets(names[k], values[k], resolved, 18, n, &stuck)) {
          ++unknown; continue;
        }
        ++commanded; zero += stuck;
        for (uint8_t j = 0; j < n; ++j) {
          uint8_t slot = 0;
          while (slot < count && targets[slot].id != resolved[j].id) ++slot;
          if (slot == 18) return;
          targets[slot] = resolved[j];
          if (slot == count) ++count;
        }
      }
      uint8_t written = 0, skipped = 0;
      ok = !count || exo.setAbsoluteAnglesSync(targets, count, &written, &skipped);
      response.has_batch = true;
      auto& b = response.batch;
      b.has_motors = b.has_skipped_offline = b.has_commanded = b.has_held = b.has_unknown = b.has_zero_travel = true;
      b.motors = written; b.skipped_offline = skipped; b.commanded = commanded;
      b.held = held; b.unknown = unknown; b.zero_travel = zero;
      break;
    }
    case exo_usb_Command_SET_GESTURE: {
      // Resolve names before invoking the existing gesture controller; never
      // reconstruct or dispatch an ASCII command. Scope is unchanged.
      const int g = findGestureIndex(request.gesture.name);
      if (g < 0 || findStateIndex(gestureLibrary[g], request.gesture.pose) < 0) return;
      if (!exo.modeControlsPosition()) break;
      gc.executeGesture(request.gesture.name, request.gesture.pose);
      ok = true; // accepted gesture, not an acknowledgement from each motor
      break;
    }
    case exo_usb_Command_SET_GESTURE_ANGLE:
      if (exo.modeControlsPosition()) ok = gc.setGestureAngle(request.gesture.name, request.value);
      break;
    case exo_usb_Command_SET_JOINT_MODEL: {
      JointModelParams params;
      params.gain = request.joint_model.gain;
      params.time_constant = request.joint_model.time_constant;
      params.max_velocity = request.joint_model.max_velocity;
      params.stiffness = request.joint_model.stiffness;
      params.moment = request.joint_model.moment;
      if (!params.valid()) return;
      ok = exo.setJointModel(ids[0], params);
      break;
    }
    case exo_usb_Command_GET_TELEMETRY_FAST: {
      EXO_PROFILE(NX_PACK);
      FastTelemetryRecord records[18];
      uint8_t method = 0;
      uint8_t count = exo.getFastTelemetryRecords(ids, request.motor_ids_count, records, method, 10);
      // NX v2 header (13 bytes); records already have the packed wire layout.
      uint8_t* out = response.nx_frame.bytes;
      const uint16_t length = count * sizeof(FastTelemetryRecord);
      out[0] = 'N'; out[1] = 'X'; out[2] = 2; out[3] = method; out[4] = count;
      out[5] = length; out[6] = length >> 8;
      uint32_t now = millis();
      for (uint8_t i = 0; i < 4; ++i) out[7+i] = now >> (8*i);
      memcpy(out + 13, records, length);
      uint16_t sum = 0;
      for (uint16_t i = 0; i < 11; ++i) sum += out[i];
      for (uint16_t i = 13; i < 13 + length; ++i) sum += out[i];
      out[11] = sum; out[12] = sum >> 8;
      response.nx_frame.size = 13 + length;
      response.has_nx_frame = true;
      ok = true;
      break;
    }
    case exo_usb_Command_SET_CURRENT: ok = exo.setGoalCurrent(ids[0], request.value); break;
    case exo_usb_Command_SET_VELOCITY: ok = exo.setGoalVelocity(ids[0], request.value); break;
    case exo_usb_Command_SET_ANGLE:
    case exo_usb_Command_SET_ABSOLUTE_ANGLE: {
      if (!exo.modeControlsPosition()) break;
      float angle = request.value;
      if (request.command == exo_usb_Command_SET_ANGLE)
        angle = exo.getZeroAngle(ids[0]) + (exo.getFlipMotor(ids[0]) ? -angle : angle);
      const MotorAngleTarget target = {ids[0], angle};
      ok = exo.setAbsoluteAnglesSync(&target, 1);
      break;
    }
    case exo_usb_Command_STOP:
      if (!request.motor_ids_count) exo.stopAllDirectControl();
      else for (uint8_t i = 0; i < request.motor_ids_count; ++i) exo.stopDirectControl(ids[i]);
      ok = true; // accepted by the existing stop routine, not proof of physical rest
      break;
    default: return;
  }
  response.status = ok ? exo_usb_Status_OK : exo_usb_Status_REJECTED;
}

void pollProtobufUsb(NMLHandExo& exo, GestureController& gc) {
  EXO_PROFILE(COMMAND);
  // At most one request and 128 bytes per loop; partial frames never block ROM.
  for (uint16_t budget = 0; budget < 128 && TELEM_SERIAL.available(); ++budget) {
    if (!reader.feed(TELEM_SERIAL.read(), millis())) continue;
    exo_usb_Request request = exo_usb_Request_init_zero;
    pb_istream_t input = pb_istream_from_buffer(reader.bytes + 7, reader.size);
    if (!pb_decode(&input, exo_usb_Request_fields, &request)) return;
    exo_usb_Response response = exo_usb_Response_init_zero;
    executeRequest(exo, gc, request, response);
    exo.serviceRomCalibration();
    uint8_t output[exo_pb::MAX_RESPONSE + 7]; // bounded 18-joint status or NX response
    pb_ostream_t stream = pb_ostream_from_buffer(output + 7, sizeof(output) - 7);
    if (!pb_encode(&stream, exo_usb_Response_fields, &response)) return;
    uint16_t size = stream.bytes_written;
    output[0] = 'P'; output[1] = 'B'; output[2] = 1;
    output[3] = size; output[4] = size >> 8;
    uint16_t crc = exo_pb::crc(output + 7, size, exo_pb::crc(output + 2, 3));
    output[5] = crc; output[6] = crc >> 8;
    EXO_PROFILE_CALL(USB_TX, TELEM_SERIAL.write(output, size + 7));
    // CDC write queues the packet; no extra UART copy or Stream::flush().
    return;
  }
}

#if EXO_USB_PROTOBUF_V2
static void sendProtobufV2Response(const exo_usb_Response& response) {
  uint8_t output[exo_pb::MAX_RESPONSE + 7];
  pb_ostream_t stream = pb_ostream_from_buffer(output + 7, sizeof(output) - 7);
  if (!pb_encode(&stream, exo_usb_Response_fields, &response)) return;
  const uint16_t size = stream.bytes_written;
  output[0] = 'P'; output[1] = 'B'; output[2] = 2;
  output[3] = size; output[4] = size >> 8;
  const uint16_t crc = exo_pb::crc(output + 7, size, exo_pb::crc(output + 2, 3));
  output[5] = crc; output[6] = crc >> 8;
  EXO_PROFILE_CALL(USB_TX, TELEM_SERIAL.write(output, size + 7));
}

void protobufV2EmitEvent(const String& message) {
  exo_usb_Response response = exo_usb_Response_init_zero;
  response.sequence = 0;
  response.status = exo_usb_Status_OK;
  response.has_text_reply = true;
  message.substring(0, sizeof(response.text_reply) - 1).toCharArray(
      response.text_reply, sizeof(response.text_reply));
  sendProtobufV2Response(response);
}

static void executeRequestV2(NMLHandExo& exo, GestureController& gc, Adafruit_BNO055& imu,
                             const exo_usb_Request& request, exo_usb_Response& response) {
  response.sequence = request.sequence;
  response.status = exo_usb_Status_INVALID_REQUEST;
  if (request.command == exo_usb_Command_HELLO) {
    if (request.motor_ids_count || request.has_value || request.has_joint_model ||
        request.values_count || request.has_finger_angles || request.has_gesture ||
        request.has_text_command) return;
    response.status = exo_usb_Status_OK;
    response.has_text_reply = true;
    strncpy(response.text_reply, "protobuf-v2", sizeof(response.text_reply) - 1);
    return;
  }
  if (request.command == exo_usb_Command_TEXT_COMMAND) {
    if (!request.has_text_command || !request.text_command[0] || request.motor_ids_count ||
        request.has_value || request.has_joint_model || request.values_count ||
        request.has_finger_angles || request.has_gesture) return;
    String capture;
    protobufReplyCaptureBegin(capture);
    parseMessage(exo, gc, imu, String(request.text_command));
    protobufReplyCaptureEnd();
    response.has_text_reply = true;
    capture.substring(0, sizeof(response.text_reply) - 1).toCharArray(
        response.text_reply, sizeof(response.text_reply));
    response.status = capture.startsWith("ERROR:") ? exo_usb_Status_REJECTED : exo_usb_Status_OK;
    return;
  }
  executeRequest(exo, gc, request, response);
}

void pollProtobufUsbV2(NMLHandExo& exo, GestureController& gc, Adafruit_BNO055& imu) {
  EXO_PROFILE(COMMAND);
  for (uint16_t budget = 0; budget < 512 && CMD_SERIAL.available(); ++budget) {
    if (!readerV2.feed(CMD_SERIAL.read(), millis())) continue;
    exo_usb_Request request = exo_usb_Request_init_zero;
    pb_istream_t input = pb_istream_from_buffer(readerV2.bytes + 7, readerV2.size);
    if (!pb_decode(&input, exo_usb_Request_fields, &request)) return;
    exo_usb_Response response = exo_usb_Response_init_zero;
    executeRequestV2(exo, gc, imu, request, response);
    exo.serviceRomCalibration();
    sendProtobufV2Response(response);
    return;
  }
}
#endif
#endif
