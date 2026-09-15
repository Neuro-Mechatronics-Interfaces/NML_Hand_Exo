#include "config.h"
#if EXO_USB_PROTOBUF
#include "protobuf_usb.h"
#include "protobuf_frame.h"
#include "nml_hand_exo.h"
#include "gesture_controller.h"
#include "gesture_library.h"
#include "io_profile.h"
#include <pb_decode.h>
#include <pb_encode.h>
#include "protocol/exo_usb.pb.h"
#include "protocol/exo_usb.pb.c"
#include <string.h>

static exo_pb::Reader reader;

// Fixed-size wire messages, no ASCII serialization or parseMessage dispatch.
// Gesture resolution retains the existing controller's String-based name API.
static void executeRequest(NMLHandExo& exo, GestureController& gc, const exo_usb_Request& request,
                           exo_usb_Response& response) {
  response.sequence = request.sequence;
  response.status = exo_usb_Status_INVALID_REQUEST;
  const bool assist = request.command >= exo_usb_Command_ASSIST_CONFIG && request.command <= exo_usb_Command_ASSIST_STOP;
  if (assist) {
    if (request.has_value || request.has_joint_model || request.has_finger_angles || request.has_gesture) return;
    for (uint8_t k = 0; k < request.motor_ids_count; ++k) {
      if (request.motor_ids[k] > 253 || exo.getIndexById(request.motor_ids[k]) < 0) return;
      for (uint8_t j = 0; j < k; ++j) if (request.motor_ids[k] == request.motor_ids[j]) return;
    }
    bool ok = false;
    if (request.command == exo_usb_Command_ASSIST_CONFIG) {
      if (request.motor_ids_count != 1 || request.values_count != 3) return;
      ok = exo.configureAssist(request.motor_ids[0], request.values[0], request.values[1], request.values[2]);
    } else if (request.command == exo_usb_Command_ASSIST_CALIBRATE) {
      if (!request.motor_ids_count || request.values_count) return;
      uint8_t ids[N_MOTORS];
      if (request.motor_ids_count > N_MOTORS) return;
      for (uint8_t k = 0; k < request.motor_ids_count; ++k) ids[k] = request.motor_ids[k];
      ok = exo.calibrateAssist(ids, request.motor_ids_count);
    } else {
      if (request.motor_ids_count || request.values_count) return;
      if (request.command == exo_usb_Command_ASSIST_START) ok = exo.startAssist();
      else if (request.command == exo_usb_Command_ASSIST_HEARTBEAT) ok = exo.heartbeatAssist();
      else { exo.stopAssist(); ok = true; }
    }
    response.status = ok ? exo_usb_Status_OK : exo_usb_Status_REJECTED;
    return;
  }
  if (exo.isAssistBusy() && request.command != exo_usb_Command_GET_TELEMETRY_FAST &&
      request.command != exo_usb_Command_STOP) { response.status = exo_usb_Status_REJECTED; return; }
  const bool telemetry = request.command == exo_usb_Command_GET_TELEMETRY_FAST;
  const bool stop = request.command == exo_usb_Command_STOP;
  const bool model = request.command == exo_usb_Command_SET_JOINT_MODEL;
  const bool batch = request.command == exo_usb_Command_SET_ANGLES ||
      request.command == exo_usb_Command_SET_ABSOLUTE_ANGLES || request.command == exo_usb_Command_SET_CURRENTS;
  const bool fingers = request.command == exo_usb_Command_SET_FINGER_ANGLES;
  const bool gesture = request.command == exo_usb_Command_SET_GESTURE;
  const bool gestureAngle = request.command == exo_usb_Command_SET_GESTURE_ANGLE;
  // Reject conflicting fields before any resolution or motor traffic.
  if (request.has_joint_model != model || request.has_finger_angles != fingers ||
      request.has_gesture != (gesture || gestureAngle) || (!batch && request.values_count)) return;
  if (model) {
    if (!request.has_joint_model || request.has_value || request.motor_ids_count != 1) return;
  } else if (batch) {
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
    uint8_t output[640 + 7]; // same bounded response envelope, including batch metadata
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
#endif
