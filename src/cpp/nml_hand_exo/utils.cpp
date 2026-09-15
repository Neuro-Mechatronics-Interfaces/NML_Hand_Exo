/**
 * @file utils.cpp
 * @brief A cpp file for supporting functions
 *
 */
#include "utils.h"
#include "oled.h"
#include <Arduino.h>
#include "nml_hand_exo.h"
#include "gesture_controller.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <chrono>
#include <thread>
#include "io_profile.h"


// IMU variables (print delay logic currently disabled)
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

sensors_event_t orientationData, linearAccelData;



// ---- Loop-period instrumentation -------------------------------------------
static uint32_t sLoopLastUs = 0;
static uint32_t sLoopCount  = 0;
static uint32_t sLoopMaxUs  = 0;
static uint64_t sLoopTotalUs = 0;

void loopStatsTick() {
  exo_io::profile().charge(micros());
  uint32_t now = micros();
  if (sLoopLastUs != 0) {
    uint32_t delta = now - sLoopLastUs;   // unsigned math survives rollover
    sLoopCount++;
    sLoopTotalUs += delta;
    if (delta > sLoopMaxUs) sLoopMaxUs = delta;
  }
  sLoopLastUs = now;
}

void loopStatsReset() {
  sLoopCount = 0;
  sLoopMaxUs = 0;
  sLoopTotalUs = 0;
  sLoopLastUs = 0;
}

uint32_t loopStatsCount() { return sLoopCount; }
uint32_t loopStatsMaxUs() { return sLoopMaxUs; }

uint32_t loopStatsMeanUs() {
  if (sLoopCount == 0) return 0;
  return (uint32_t)(sLoopTotalUs / sLoopCount);
}

void telemetryPrintln(const String& msg) {
  EXO_PROFILE(USB_TX);
  // The Protobuf build reserves the second CDC exclusively for binary data.
#if EXO_USB_PROTOBUF
  CMD_SERIAL.println(msg);
#elif defined(DUAL_CDC) && DUAL_CDC
  // Legacy-safe: default route BOTH mirrors to the command CDC so a single-port
  // host still sees replies; a dual-port host can switch to TELEM to decouple.
  if (gReplyRoute != REPLY_ROUTE_TELEM) CMD_SERIAL.println(msg);
  if (gReplyRoute != REPLY_ROUTE_CMD)   TELEM_SERIAL.println(msg);
#else
  // Single-CDC: CMD_SERIAL == TELEM_SERIAL, so emit once to avoid duplicates.
  TELEM_SERIAL.println(msg);
#endif
}

void debugPrint(const String& msg) {
  // Prints verbose diagnostics to the reply/telemetry CDC(s) if VERBOSE is set.
  // Honors gReplyRoute so a legacy single-port host still sees debug output.
  if (VERBOSE) {
    telemetryPrintln(msg);
  }
}

void commandPrint(const String& msg) {
  // Append delimiter if it's not already present
  String cmdMsg = msg;  // Make a copy to modify
  String cmdDelimiter = COMMAND_DELIMITER;
  if (!cmdMsg.endsWith(cmdDelimiter)) {
    cmdMsg += cmdDelimiter;
  }

  // Emit command replies / sensor-request responses regardless of VERBOSE mode.
  // The Bluetooth command path (COMMAND_SERIAL) always mirrors the reply; the
  // USB CDC target(s) are chosen by telemetryPrintln() per the runtime route.
  #if defined(COMMAND_SERIAL)
    EXO_PROFILE_CALL(BT_TX, COMMAND_SERIAL.println(cmdMsg));
  #else
    EXO_PROFILE_CALL(BT_TX, Serial2.println(cmdMsg));  // fallback
  #endif
  telemetryPrintln(cmdMsg);
}

// void initializeIMU(Adafruit_ISM330DHCX& imu) {
//   if (!imu.begin_I2C()) {
//     // if (!ism330dhcx.begin_SPI(LSM_CS)) {
//     // if (!ism330dhcx.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
//     debugPrint("Failed to find ISM330DHCX chip");
//   } else {
//     debugPrint(F("ISM330DHCX Found!"));
//     imu.configInt1(false, false, true); // accelerometer DRDY on INT1
//     imu.configInt2(false, true, false); // gyro DRDY on INT2

//   }
// }

bool initializeIMU(Adafruit_BNO055& bno) {
  if (!bno.begin()) {
    Serial.println("No BNO055 detected");
    return false;
  }
  delay(1000);
  return true;
}


// void getIMUData(Adafruit_ISM330DHCX& imu) {
//   sensors_event_t accel, gyro, temp;
//   if (!imu.getEvent(&accel, &gyro, &temp)) {
//     debugPrint("IMU read failed");
//     return;
//   }
//   char buffer[128];
//   snprintf(buffer, sizeof(buffer),
//            "Temp:%.2f C; Accel: [%.2f, %.2f, %.2f] m/s^2; Gyro: [%.2f, %.2f, %.2f] rad/s;",
//            temp.temperature,
//            accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
//            gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
//   commandPrint(buffer);
// }

// void getIMUData(Adafruit_BNO055& imu) {
//   sensors_event_t orientationData, accelData, magData, gyroData;

//   imu.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//   imu.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
//   imu.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
//   imu.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);

//   int temperature = imu.getTemp();

//     // Now use orientationData, accelData, etc., and temperature as needed

//   char buffer[128];
//   snprintf(buffer, sizeof(buffer),
//          "Temp: %.2f C; Euler: [%.2f, %.2f, %.2f] deg; Accel: [%.2f, %.2f, %.2f] m/s^2;",
//          (float)temperature,
//          euler.x(), euler.y(), euler.z(),
//          accel.x(), accel.y(), accel.z());

//   commandPrint(buffer);
// }

void updateIMU(Adafruit_BNO055& bno) {
  static unsigned long lastRead = 0;
  const unsigned long IMU_INTERVAL = 10 * 1000; // 10 ms in microseconds (100 Hz)

  unsigned long tStart = micros();
  if (tStart - lastRead < IMU_INTERVAL) return;
  lastRead = tStart;

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);


  // bool ok1 = bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // bool ok2 = bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // if (!ok1 || !ok2) {
  //   Serial.println("IMU read failed, reinitializing...");
  //   bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF);
  //   bno.setExtCrystalUse(true);
  // }




  // if (orientationData.orientation.x == 0 &&
  //   orientationData.orientation.y == 0 &&
  //   orientationData.orientation.z == 0) {
  //   Serial.println("IMU returned zeros - resetting");
  //   bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF);
  //   bno.setExtCrystalUse(true);
  // }

}

String getIMUData(Adafruit_BNO055& bno) {
  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    printCount = 0;
  } else {
    printCount++;
  }

  String imu_data = "Heading: " + String(orientationData.orientation.x) +
                    ", Pitch: "   + String(orientationData.orientation.y) +
                    ", Roll: "    + String(orientationData.orientation.z); 

  commandPrint(imu_data);
  return imu_data;
}

float getIMUYaw(Adafruit_BNO055& bno) {
  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    printCount = 0;
  } else {
    printCount++;
  }

  unsigned long timestamp = millis();

  updateIMU(bno);

  String imu_heading_string = "Heading: " + String(orientationData.orientation.x) + "Timestamp: " + String(timestamp);
  float imu_heading_val = orientationData.orientation.x;

  commandPrint(String(imu_heading_string));
  return imu_heading_val;
}


void flashPin(int pin, int durationMs, int repetitions) {
  for (int i = 0; i < repetitions; ++i) {
    digitalWrite(pin, HIGH);
    delay(durationMs);
    digitalWrite(pin, LOW);
    delay(durationMs);
  }
}

String getArg(const String line, const int index, char delimiter = ':') {
  int fromIndex = 0;
  for (int i = 0; i < index; ++i) {
    fromIndex = line.indexOf(delimiter, fromIndex);
    if (fromIndex == -1) return "";
    fromIndex += 1;
  }
  int toIndex = line.indexOf(delimiter, fromIndex);
  if (toIndex == -1) toIndex = line.length();
  return line.substring(fromIndex, toIndex);
}

int getArgMotorID(NMLHandExo& exo, const String& line, const int index) {
  return exo.getMotorID(getArg(line, index));
}

int applyTorqueToMotorIDList(NMLHandExo& exo, const String& token, bool enable) {
  int updated = 0;
  for (int argIndex = 1;; ++argIndex) {
    String arg = getArg(token, argIndex);
    arg.trim();
    if (arg.length() == 0) {
      break;
    }
    int requestedID = arg.toInt();
    if (requestedID <= 0) {
      continue;
    }
    int resolvedID = exo.getMotorID(String(requestedID));
    if (resolvedID == -1) {
      continue;
    }
    exo.enableTorque(resolvedID, enable);
    updated++;
  }
  return updated;
}

struct __attribute__((packed)) FastTelemetryHeader {
  char magic[2];
  uint8_t version;
  uint8_t flags;
  uint8_t count;
  uint16_t payload_len;
  uint32_t timestamp_ms;
  uint16_t checksum;
};

uint16_t checksumBytes(const uint8_t* data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum = (uint16_t)(sum + data[i]);
  }
  return sum;
}

void writeFastTelemetryFrame(Stream& stream, FastTelemetryRecord* records, uint8_t count, uint8_t flags) {
  FastTelemetryHeader header;
  header.magic[0] = 'N';
  header.magic[1] = 'X';
  header.version = 2;
  header.flags = flags;
  header.count = count;
  header.payload_len = count * sizeof(FastTelemetryRecord);
  header.timestamp_ms = millis();
  header.checksum = 0;

  uint16_t checksum = checksumBytes((const uint8_t*)&header, sizeof(header) - sizeof(header.checksum));
  checksum = (uint16_t)(checksum + checksumBytes((const uint8_t*)records, header.payload_len));
  header.checksum = checksum;

  stream.write((const uint8_t*)&header, sizeof(header));
  if (header.payload_len > 0) {
    stream.write((const uint8_t*)records, header.payload_len);
  }
  stream.flush();
}

uint8_t collectFastTelemetryIDs(NMLHandExo& exo, const String& token, uint8_t* ids, uint8_t maxIds) {
  String firstArg = getArg(token, 1);
  firstArg.trim();
  if (firstArg.length() == 0 || firstArg.equalsIgnoreCase("ALL")) {
    uint8_t count = min((uint8_t)exo.getMotorCount(), maxIds);
    for (uint8_t i = 0; i < count; ++i) {
      ids[i] = exo.getMotorIDByIndex(i);
    }
    return count;
  }

  uint8_t count = 0;
  for (int argIndex = 1; count < maxIds; ++argIndex) {
    String arg = getArg(token, argIndex);
    arg.trim();
    if (arg.length() == 0) {
      break;
    }
    int resolvedID = exo.getMotorID(arg);
    if (resolvedID != -1) {
      ids[count++] = (uint8_t)resolvedID;
    }
  }
  return count;
}

const char* fastTelemetryMethodName(uint8_t method) {
  switch (method & 0x0f) {
    case FAST_TELEM_METHOD_MODEL:
      return "model";
    case FAST_TELEM_METHOD_FAST_SYNC_READ:
      return "fastSyncRead";
    case FAST_TELEM_METHOD_SYNC_READ:
      return "syncRead";
    case FAST_TELEM_METHOD_FALLBACK_READ:
      return "fallbackRead";
    default:
      return "failed";
  }
}

void sendFastTelemetry(NMLHandExo& exo, const String& token, bool fromBluetooth) {
  EXO_PROFILE(NX_PACK);
#if EXO_USB_PROTOBUF
  if (!fromBluetooth) {
    commandPrint(F("ERROR: use Protobuf GET_TELEMETRY_FAST on the binary CDC"));
    return;
  }
#endif
  uint8_t ids[32];
  FastTelemetryRecord records[32];
  uint8_t idCount = collectFastTelemetryIDs(exo, token, ids, 32);
  if (idCount == 0) {
    commandPrint("ERROR: get_telemetry_fast requires valid motor IDs or all");
    return;
  }

  uint8_t method = FAST_TELEM_METHOD_FAILED;
  uint32_t startMicros = micros();
  uint8_t recordCount = exo.getFastTelemetryRecords(ids, idCount, records, method, 10);
  (void)startMicros;

  // Binary NX stays on the primary CDC; DualSerialComm reads raw bytes there.
  // reply_route controls text only (the other CDC has a text reader).
  if (!fromBluetooth) EXO_PROFILE_CALL(USB_TX, writeFastTelemetryFrame(DEBUG_SERIAL, records, recordCount, method));
#if defined(COMMAND_SERIAL)
  // A 310-byte nine-motor frame takes ~27 ms at 115200 baud. Mirroring every
  // USB poll here blocked the control loop even with no Bluetooth client.
  if (fromBluetooth) EXO_PROFILE_CALL(BT_TX, writeFastTelemetryFrame(COMMAND_SERIAL, records, recordCount, method));
#endif
}

void sendFastTelemetryDiag(NMLHandExo& exo, const String& token) {
  uint8_t ids[32];
  FastTelemetryRecord records[32];
  uint8_t idCount = collectFastTelemetryIDs(exo, token, ids, 32);
  if (idCount == 0) {
    commandPrint("ERROR: telemetry_diag requires valid motor IDs or all");
    return;
  }

  uint8_t method = FAST_TELEM_METHOD_FAILED;
  uint32_t startMicros = micros();
  uint8_t recordCount = exo.getFastTelemetryRecords(ids, idCount, records, method, 10);
  uint32_t elapsedMicros = micros() - startMicros;

  String info = "Telemetry diag: {requested: " + String(idCount) +
                ", records: " + String(recordCount) +
                ", method: " + String(fastTelemetryMethodName(method)) +
                ", elapsed_us: " + String(elapsedMicros) + "}\n";
  for (uint8_t i = 0; i < recordCount; ++i) {
    info += "Motor " + String(i) + ": {id: " + String(records[i].id) +
            ", error: " + String(records[i].error) +
            ", sources: " + String(records[i].sources) +
            ", current_mA: " + String(records[i].current_mA) +
            ", velocity_raw: " + String(records[i].velocity_raw) +
            ", position_ticks: " + String(records[i].position_ticks) +
            ", absolute_angle: " + String(records[i].absolute_cdeg / 100.0f, 2) +
            ", angle: " + String(records[i].relative_cdeg / 100.0f, 2) +
            "}\n";
  }
  commandPrint(info);
}

void sendShadowTelemetryStatus(NMLHandExo& exo) {
  ShadowTelemetryRecord records[SHADOW_TELEMETRY_MAX_MOTORS];
  uint8_t count = exo.copyShadowTelemetryRecords(
    records, SHADOW_TELEMETRY_MAX_MOTORS
  );
  String info = "SHADOW: {enabled: " +
                String(exo.isShadowTelemetryEnabled() ? "true" : "false") +
                ", mode: " + exo.getMotorControlMode() +
                ", count: " + String(count) +
                ", interval_ms: " + String(exo.getShadowTelemetryIntervalMs()) +
                ", timestamp_ms: " + String(millis()) +
                ", sequence: " + String(exo.getShadowTelemetrySequence()) +
                ", read_errors: " + String(exo.getShadowTelemetryReadErrors()) +
                "}\n";
  for (uint8_t i = 0; i < count; ++i) {
    info += "Motor " + String(i) + ": {id: " + String(records[i].id) +
            ", error: " + String(records[i].error) +
            ", current: " + String(records[i].current_mA) +
            ", position_ticks: " + String(records[i].position_ticks) +
            ", absolute_angle: " + String(records[i].absolute_cdeg / 100.0f, 2) +
            ", angle: " + String(records[i].relative_cdeg / 100.0f, 2) +
            ", velocity_deg_s: " + String(records[i].velocity_cdeg_s / 100.0f, 2) +
            ", current_sample_ms: " + String(records[i].current_sample_ms) +
            ", position_sample_ms: " + String(records[i].position_sample_ms) +
            "}\n";
  }
  commandPrint(info);
}

void parseMessage(NMLHandExo& exo, GestureController& gc, Adafruit_BNO055& imu, String token,
                  bool fromBluetooth) {
  EXO_PROFILE(COMMAND);

  token.trim();        // Remove any trailing white space
  token.toLowerCase(); // Set all characters to lowercase

  String cmd = getArg(token, 0, ':'); // Get the command part before the first colon
  int id = -1; // Default to -1 if not found
  int val = 0; // Default value for commands that require a value

  // ========== Supported high-level commands ==========
  if (cmd == "set_utc_time" || cmd == "get_utc_time") {
    if (cmd == "set_utc_time") {
      const String value = getArg(token, 1);
      bool valid = value.length() > 0 && value.length() <= 15;
      uint64_t epoch = 0;
      for (unsigned int k = 0; k < value.length() && valid; ++k) {
        if (!isDigit(value[k])) { valid = false; break; }
        epoch = epoch * 10 + (value[k] - '0');
      }
      int delimiters = 0;
      for (unsigned int k = 0; k < token.length(); ++k) if (token[k] == ':') ++delimiters;
      if (!valid || delimiters != 1 || !exo.setUtcTime(epoch)) {
        commandPrint("ERROR: set_utc_time requires Unix UTC milliseconds (1..253402300799999)");
        return;
      }
    } else if (token != "get_utc_time") {
      commandPrint("ERROR: get_utc_time takes no arguments");
      return;
    }
    const uint64_t utc = exo.utcTime();
    char stamp[24];
    // Avoid AVR/newlib printf variants whose integer formatter lacks %llu.
    uint64_t remainder = utc;
    uint8_t length = 0;
    do { stamp[length++] = '0' + remainder % 10; remainder /= 10; } while (remainder);
    for (uint8_t k = 0; k < length / 2; ++k) {
      const char c = stamp[k]; stamp[k] = stamp[length - 1 - k]; stamp[length - 1 - k] = c;
    }
    stamp[length] = '\0';
    commandPrint(String(cmd == "set_utc_time" ? "OK: set_utc_time" : "UTC_TIME:") +
        " utc_ms=" + String(stamp) + " synchronized=" + String(utc != 0 ? 1 : 0) +
        " uptime_ms=" + String(millis()));
    return;
  } else if (cmd == "set_joint_model" || cmd == "get_joint_model" ||
      cmd == "set_joint_trigger" || cmd == "clear_joint_trigger" ||
      cmd == "set_estimate_holdoff") {
    const int expected = cmd == "set_joint_model" ? 6 :
        (cmd == "set_joint_trigger" ? 2 : 1);
    int actual = 0;
    for (unsigned int k = 0; k < token.length(); ++k) if (token[k] == ':') ++actual;
    auto unsignedToken = [](const String& v) {
      if (!v.length() || v.length() > 5) return false;
      for (unsigned int k = 0; k < v.length(); ++k) if (!isDigit(v[k])) return false;
      return true;
    };
    auto number = [](const String& v, float& out) {
      if (!v.length()) return false;
      char* end;
      out = strtof(v.c_str(), &end);
      return end != v.c_str() && *end == '\0' && isfinite(out);
    };
    const String idArg = getArg(token, 1);
    if (actual != expected || !unsignedToken(idArg)) {
      commandPrint("ERROR: " + cmd + " invalid arguments");
      return;
    }
    if (cmd == "set_estimate_holdoff") {
      const uint32_t ms = idArg.toInt();
      commandPrint(exo.setEstimateHoldoff(ms) ?
          "OK: set_estimate_holdoff ms=" + String(ms) :
          "ERROR: set_estimate_holdoff requires 50..5000 ms");
      return;
    }
    const long mid = idArg.toInt();
    if (mid < 1 || mid > 253 || exo.getIndexById((uint8_t)mid) < 0) {
      commandPrint("ERROR: " + cmd + " requires a configured explicit DXL ID");
      return;
    }
    if (cmd == "get_joint_model") {
      commandPrint(exo.getJointModel((uint8_t)mid));
      return;
    }
    bool ok = false;
    if (cmd == "set_joint_model") {
      JointModelParams p;
      ok = number(getArg(token, 2), p.gain) &&
           number(getArg(token, 3), p.time_constant) &&
           number(getArg(token, 4), p.max_velocity) &&
           number(getArg(token, 5), p.stiffness) &&
           number(getArg(token, 6), p.moment) && exo.setJointModel((uint8_t)mid, p);
    } else if (cmd == "set_joint_trigger") {
      float tau;
      ok = number(getArg(token, 2), tau) && exo.setJointTrigger((uint8_t)mid, tau);
    } else {
      ok = exo.clearJointTrigger((uint8_t)mid);
    }
    commandPrint(ok ? "OK: " + cmd + " id=" + String(mid) :
                     "ERROR: " + cmd + " parameter out of range");
    return;
  } else if (cmd == "get_angle" || cmd == "get_absolute_angle" ||
             cmd == "get_current" || cmd == "get_torque" || cmd == "get_velocity") {
    uint8_t ids[32];
    FastTelemetryRecord records[32];
    const uint8_t count = collectFastTelemetryIDs(exo, token, ids, 32);
    uint8_t method;
    exo.getFastTelemetryRecords(ids, count, records, method, 10);
    const String key = cmd.substring(4);
    String reply = "Motor telemetry:\n";
    for (uint8_t i = 0; i < count; ++i) {
      const FastTelemetryRecord& r = records[i];
      const uint8_t shift = (cmd == "get_current" || cmd == "get_torque") ? 2 :
                            (cmd == "get_velocity" ? 4 : 0);
      const uint8_t source = r.error ? 0 : ((r.sources >> shift) & 3);
      float value = NAN;
      if (source) {
        if (cmd == "get_current") value = r.current_mA;
        else if (cmd == "get_torque") value = r.current_mA * XC330_T288_TORQUE_CONSTANT;
        else if (cmd == "get_velocity") value = r.velocity_raw * 0.229f *
            (exo.getFlipMotor(r.id) ? -1.0f : 1.0f);
        else if (cmd == "get_absolute_angle") value = r.absolute_cdeg / 100.0f;
        else value = r.relative_cdeg / 100.0f;
      }
      reply += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(r.id) +
          ", id: " + String(r.id) + ", " + key + ": " + String(value, 4) +
          ", source: " + String(source == 2 ? "estimated" : source == 1 ? "measured" : "unavailable") + "}\n";
    }
    commandPrint(reply);
  } else if (cmd == "get_telemetry_fast") {
    sendFastTelemetry(exo, token, fromBluetooth);

  } else if (cmd == "telemetry_diag") {
    sendFastTelemetryDiag(exo, token);

  } else if (cmd == "shadow_config") {
    String intervalArg = getArg(token, 1);
    intervalArg.trim();
    auto isUnsignedInteger = [](const String& value) -> bool {
      if (value.length() == 0) return false;
      for (unsigned int i = 0; i < value.length(); ++i) {
        if (!isDigit(value.charAt(i))) return false;
      }
      return true;
    };
    long requestedInterval = intervalArg.toInt();
    uint8_t ids[SHADOW_TELEMETRY_MAX_MOTORS];
    uint8_t count = 0;
    bool valid = isUnsignedInteger(intervalArg) && requestedInterval > 0;
    for (int argIndex = 2; valid; ++argIndex) {
      String idArg = getArg(token, argIndex);
      idArg.trim();
      if (idArg.length() == 0) break;
      if (count >= SHADOW_TELEMETRY_MAX_MOTORS || !isUnsignedInteger(idArg)) {
        valid = false;
        break;
      }
      long explicitID = idArg.toInt();
      if (explicitID <= 0 || explicitID > 253 ||
          exo.getIndexById((uint8_t)explicitID) < 0) {
        valid = false;
        break;
      }
      ids[count++] = (uint8_t)explicitID;
    }
    if (!valid || count == 0 ||
        !exo.configureShadowTelemetry(ids, count, (unsigned long)requestedInterval)) {
      commandPrint(F("ERROR: shadow_config requires INTERVAL_MS and unique explicit IDs"));
    } else {
      commandPrint("OK: shadow_config count=" + String(count) +
                   " interval_ms=" + String(exo.getShadowTelemetryIntervalMs()));
    }

  } else if (cmd == "shadow_start") {
    if (exo.startShadowTelemetry()) {
      commandPrint(F("OK: shadow_start read-only instrumentation active"));
    } else {
      commandPrint(F("ERROR: shadow_start requires a configuration and VELOCITY mode"));
    }

  } else if (cmd == "shadow_stop") {
    exo.stopShadowTelemetry();
    commandPrint(F("OK: shadow_stop"));

  } else if (cmd == "shadow_status") {
    sendShadowTelemetryStatus(exo);

  } else if (cmd == "enable") {
    String arg = getArg(token, 1);  // local copy
    arg.trim();
    if (arg.equalsIgnoreCase("ALL")) {
      for (int i = 0; i < exo.getMotorCount(); i++) {
        uint8_t id = exo.getMotorIDByIndex(i);
        exo.enableTorque(id, true);
      }
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) exo.enableTorque(id, true);
    }

  } else if (cmd == "enable_ids") {
    int updated = applyTorqueToMotorIDList(exo, token, true);
    if (updated > 0) {
      commandPrint("OK: enable_ids " + String(updated));
    } else {
      commandPrint("ERROR: enable_ids requires one or more valid DXL IDs");
    }

  } else if (cmd == "disable") {
    // TO-DO: allow support of csv type arguments "1,3"
    String arg = getArg(token, 1);
    arg.trim();
    if (arg.equalsIgnoreCase("ALL")) {
      for (int i = 0; i < exo.getMotorCount(); i++) {
        uint8_t id = exo.getMotorIDByIndex(i);
        exo.enableTorque(id, false);
      }
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) exo.enableTorque(id, false);
    }

  } else if (cmd == "disable_ids") {
    int updated = applyTorqueToMotorIDList(exo, token, false);
    if (updated > 0) {
      commandPrint("OK: disable_ids " + String(updated));
    } else {
      commandPrint("ERROR: disable_ids requires one or more valid DXL IDs");
    }

  } else if (cmd == "get_enabled") {
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    bool status;
    if (arg == "ALL") {
      String info = "Motor Torque Status: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        status = exo.getTorqueEnabledStatus(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", enabled: " + (status ? "true" : "false") + "}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
          status = exo.getTorqueEnabledStatus(id);
          commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", enabled: " + (status ? "true" : "false") + "}");
      }
    }

  } else if (cmd == "get_baud") {
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    bool status;
    if (arg == "ALL") {
      String info = "Motor Baudrate: \n";
      for (int i = 0; i < exo.getMotorCount(); i++) {
        uint8_t id = exo.getMotorIDByIndex(i);
        uint32_t baud = exo.getBaudRate(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", baudrate: " + String(baud) + "}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        uint32_t baud = exo.getBaudRate(id);  
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", baudrate: " + String(baud) + "}");

      }
    }

  } else if (cmd == "set_baud") {
    id = getArgMotorID(exo, token, 1);
    String baudArg = getArg(token, 2);
    baudArg.trim();
    bool numeric = baudArg.length() > 0;
    for (uint16_t i = 0; i < baudArg.length(); ++i) {
      if (!isDigit(baudArg[i])) {
        numeric = false;
        break;
      }
    }
    if (id != -1 && numeric) {
      uint32_t baud = (uint32_t)baudArg.toInt();
      if (exo.setBaudRate(id, baud)) {
        commandPrint("OK: baud id=" + String(id) + " value=" + String(baud));
      } else {
        commandPrint("ERROR: unsupported baud or Dynamixel write failed: " +
                     String(baud));
      }
    } else if (id != -1) {
      commandPrint("ERROR: set_baud requires a numeric baud rate");
    }

  } else if (cmd == "get_goal_velocity") {
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor velocity: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        uint32_t vel = exo.getVelocityLimit(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", velocity: " + String(vel) + "}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        uint32_t vel = exo.getVelocityLimit(id);
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", velocity: " + String(vel) + "}");
      }
    }

  } else if (cmd == "set_goal_velocity") {
    // Set velocity limit for a motor or all motors, Fast == 300 rpm, slow = 10rpm
    String arg = getArg(token, 1);  // local copy
    arg.trim();
    arg.toUpperCase();
    if (arg == "ALL") {
        for (int i = 0; i < exo.getMotorCount(); i++) {
            id = exo.getMotorIDByIndex(i);
            val = getArg(token, 2).toInt();
            exo.setVelocityLimit(id, val);
        }
        debugPrint("Set velocity limit for all motors to " + String(val));
    } else {
        id = getArgMotorID(exo, token, 1);
        val = getArg(token, 2).toInt();
        if (id != -1) exo.setVelocityLimit(id, val);
        debugPrint("Set velocity limit for motor " + String(id) + " to " + String(val));
    }

  } else if (cmd == "set_velocity") {
    id = getArgMotorID(exo, token, 1);
    float rpm = getArg(token, 2).toFloat();
    if (id != -1) {
      bool ok = exo.setGoalVelocity(id, rpm);
      commandPrint(
          ok ? "OK: set_velocity" :
               "ERROR: set_velocity requires velocity mode, a reachable ID, and a verified hardware velocity limit");
    }

  } else if (cmd == "get_goal_acceleration") {
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor acceleration: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        int acc = exo.getAccelerationLimit(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", Acceleration: " + String(acc) + "}\n";
      }
      commandPrint(info);
    } else {
      id = exo.getMotorID(getArg(token, 1));
      if (id != -1) {
        int acc = exo.getAccelerationLimit(id);
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", acceleration: " + String(acc) + "}");
      }
    }

  } else if (cmd == "set_goal_acceleration") {
    String arg = getArg(token, 1);  // local copy
    arg.trim();
    arg.toUpperCase();
    if (arg == "ALL") {
      for (int i = 0; i < exo.getMotorCount(); i++) {
        id = exo.getMotorIDByIndex(i);
        val = getArg(token, 2).toInt();
        exo.setAccelerationLimit(id, val);
      }
      debugPrint("Set acceleration limit for all motors to " + String(val));
    } else {
      id = getArgMotorID(exo, token, 1);
      val = getArg(token, 2).toInt();
      if (id != -1) exo.setAccelerationLimit(id, val);
    }

  } else if (cmd == "set_angle") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setRelativeAngle(id, val);

  } else if (cmd == "set_yaw_angle") {
    id = getArgMotorID(exo, token, 1);
    int target_yaw = getArg(token, 2).toInt();
    char direction = getArg(token, 3)[0];
    // commandPrint("direction value" + String(direction));
    float step_angle = 1.1;
    float current_motor_angle = exo.getRelativeAngle(id);
    int attempts = 0;
    float newAngle;
    float current_wrist_angle;
    bool moving = true;
    if(direction == 'f') {
      newAngle = current_motor_angle - step_angle;
    } else {
      newAngle = current_motor_angle + step_angle;
    }
    while (moving) {
      exo.setRelativeAngle(id, newAngle);
      delay(10);
      current_wrist_angle = getIMUYaw(imu);
      double wrist_diff = abs(current_wrist_angle - target_yaw);
      if ((wrist_diff <= 0.5) || (attempts > 150)) {  //
        moving = false;
      } else {
        if(direction == 'f') { //(directionality is for left hand)
          newAngle = newAngle - step_angle;
          // commandPrint("flexing");
        } else {
          newAngle = newAngle - step_angle;
          // commandPrint("extending");
        }
        attempts++;
      }
    }

    delay(1000);

  } else if (cmd == "set_absolute_angle") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setAbsoluteAngle(id, val);

  } else if (cmd == "get_home") {
    String arg = getArg(token, 1);  // local copy
    arg.trim();
    arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Home Status: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        float home_angle = exo.getZeroAngle(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", home: " + String(home_angle, 2) + "}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        float zero = exo.getZeroAngle(id);
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", home: " + String(zero, 2) + "}");
      }
    }

  } else if (cmd == "set_home") {
    String target = getArg(token, 1);
    if (target == "all") {
      exo.resetAllZeros();
    } else {
      id = exo.getMotorID(target);
      if (id != -1) exo.setZeroOffset(id);
    }

  } else if (cmd == "set_current_lim") {
    // This is the knob that governs gesture effort: in current-based position
    // mode setCurrentLimit() also writes GOAL_CURRENT. (set_current below is
    // direct-control only and errors out in gesture mode.)
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    val = getArg(token, 2).toInt();
    if (arg == "ALL") {
      for (int i = 0; i < exo.getMotorCount(); i++) {
        exo.setCurrentLimit(exo.getMotorIDByIndex(i), val);
      }
      commandPrint("OK: set_current_lim all " + String(val));
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        exo.setCurrentLimit(id, val);
        commandPrint("OK: set_current_lim " + String(id) + " " + String(val));
      } else {
        commandPrint("ERROR: set_current_lim needs a valid motor ID or ALL");
      }
    }

  } else if (cmd == "set_total_current_lim") {
    // Fleet-wide companion to set_current_lim: caps what ALL motors may draw
    // together, which is the quantity the power supply cares about.
    String arg = getArg(token, 1);
    arg.trim();
    if (arg.length() == 0) {
      commandPrint(F("ERROR: usage set_total_current_lim:<mA>"));
      return;
    }
    long requested = arg.toInt();
    if (requested <= 0) {
      commandPrint("ERROR: set_total_current_lim needs a positive mA value, got " + arg);
      return;
    }
    exo.setTotalCurrentBudget((uint16_t)min(requested, 65535L));
    commandPrint("OK: total_current_lim " + String(exo.getTotalCurrentBudget()));

  } else if (cmd == "get_total_current_lim") {
    commandPrint("Total current limit: " + String(exo.getTotalCurrentBudget()) + " mA");

  } else if (cmd == "set_hold_current") {
    String arg = getArg(token, 1);
    arg.trim();
    if (arg.length() == 0) {
      commandPrint(F("ERROR: usage set_hold_current:<mA>"));
      return;
    }
    long requested = arg.toInt();
    if (requested < 0) {
      commandPrint("ERROR: set_hold_current needs a non-negative mA value, got " + arg);
      return;
    }
    exo.setHoldCurrent((uint16_t)min(requested, 65535L));
    commandPrint("OK: hold_current " + String(exo.getHoldCurrent()));

  } else if (cmd == "get_hold_current") {
    commandPrint("Hold current: " + String(exo.getHoldCurrent()) + " mA");

  } else if (cmd == "set_current_governor") {
    String arg = getArg(token, 1);
    arg.trim(); arg.toLowerCase();
    if (arg == "on" || arg == "off") {
      exo.setCurrentGovernorEnabled(arg == "on");
      commandPrint("OK: current_governor " + arg);
    } else {
      commandPrint(F("ERROR: set_current_governor takes on or off"));
    }

  } else if (cmd == "current_status") {
    commandPrint(exo.getCurrentBudgetStatus());

  } else if (cmd == "set_current") {
    id = getArgMotorID(exo, token, 1);
    float current_mA = getArg(token, 2).toFloat();
    if (id != -1) {
      bool ok = exo.setGoalCurrent(id, current_mA);
      commandPrint(ok ? "OK: set_current" : "ERROR: set_current requires current mode and a valid motor ID");
    }

  } else if (cmd == "get_goal_current") {
    String arg = getArg(token, 1);
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor goal current: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        float current_mA = exo.getGoalCurrent(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", goal_current: " + String(current_mA, 3) + " mA}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        float current_mA = exo.getGoalCurrent(id);
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", goal_current: " + String(current_mA, 3) + " mA}");
      }
    }

  } else if (cmd == "get_current_lim") {
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor absolute angles: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        float current_mA = exo.getCurrentLimit(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", current_limit: " + String(current_mA, 3) + "mA}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        float current_mA = exo.getCurrentLimit(id);
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", current_limit: " + String(current_mA, 3) + "mA}");
      }
    }

  } else if (cmd == "get_motor_limits") {
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor Limits: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", Limits: " + exo.getMotorLimits(id) + "}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", limits: " + exo.getMotorLimits(id));
      }
    }

  } else if (cmd == "set_upper_limit") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setMotorUpperBound(id, val);

  } else if (cmd == "set_lower_limit") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setMotorLowerBound(id, val);

  } else if (cmd == "set_motor_limits") {
    id = getArgMotorID(exo, token, 1);
    float lowerLimit = getArg(token, 2).toFloat();
    float upperLimit = getArg(token, 3).toFloat();
    if (id != -1) {
      exo.setMotorLimits(id, lowerLimit, upperLimit);
    } else {
      commandPrint("[Error] Invalid motor ID for set_motor_limits");
    }

  } else if (cmd == "led") {
    String target = getArg(token, 1);
    String stateStr = getArg(token, 2);
    bool state = (stateStr == "on" || stateStr == "1");

    target.trim(); target.toUpperCase();
    if (target == "ALL") {
      exo.setAllMotorLED(state);
      digitalWrite(STATUS_LED_PIN, state);
    } else if (target == "STATUS") {
      digitalWrite(STATUS_LED_PIN, state);
    } else {
      id = exo.getMotorID(target);
      if (id != -1) exo.setMotorLED(id, state);
    }

  } else if (cmd == "debug") {
    String stateStr = getArg(token, 1);
    VERBOSE = (stateStr == "on" || stateStr == "1");
    commandPrint(" Debug state: " + String(VERBOSE ? "true" : "false"));

  } else if (cmd == "set_reply_route") {
    // Choose which USB CDC(s) carry replies/telemetry (dual-CDC transport).
    // both  = legacy-compatible mirror (default); telem = command port becomes
    // input-only (decoupled); cmd = command port only. No-op on single-CDC builds.
    String arg = getArg(token, 1);
    arg.trim(); arg.toLowerCase();
    if (arg == "telem" || arg == "telemetry") {
      gReplyRoute = REPLY_ROUTE_TELEM;
    } else if (arg == "cmd" || arg == "command") {
      gReplyRoute = REPLY_ROUTE_CMD;
    } else if (arg == "both") {
      gReplyRoute = REPLY_ROUTE_BOTH;
    } else {
      commandPrint("ERROR: set_reply_route expects both|telem|cmd");
      return;
    }
    const char* routeName = (gReplyRoute == REPLY_ROUTE_TELEM) ? "telem" :
                            (gReplyRoute == REPLY_ROUTE_CMD)   ? "cmd"   : "both";
    commandPrint("OK: reply_route " + String(routeName));

  } else if (cmd == "check_limits") {
    // Report, per motor, whether a gesture can actually move it.
    //
    // setAbsoluteAngle() clamps every target to [min, max], so a joint with no
    // room to travel accepts and acks every command while holding perfectly
    // still. `span` is the signed distance from home to the flexion endstop --
    // the entire range any gesture percentage can address -- so it is the
    // number that says whether this joint can move at all, and how far.
    //
    // Reads live values, so this reflects whatever apply_calibration and the
    // multi-turn epoch snap have done.
    String out = "Limit check:\n";
    for (int i = 0; i < exo.getMotorCount(); ++i) {
      uint8_t id = exo.getMotorIDByIndex(i);
      float home = exo.getZeroAngle(id);
      float lo = exo.getMotorLimitMin(id);
      float hi = exo.getMotorLimitMax(id);
      float span = exo.getGestureSpan(id);
      String status;
      if (lo > hi) status += "LIMITS_INVERTED ";
      if (home < min(lo, hi) || home > max(lo, hi)) status += "HOME_OUTSIDE ";
      if (fabsf(span) < GESTURE_MIN_TRAVEL_DEG) status += "NO_TRAVEL ";
      // The resolved direction disagreeing with the flip flag means home is
      // not where calibration assumed: the flag's side is a stub. Gestures
      // still work (the long side is used), but the calibration is worth a
      // second look, and this is the only place that says so.
      else if ((span < 0.0f) != exo.isMotorFlipped(id)) status += "SPAN_REVERSED ";
      if (status.length() == 0) status = "ok";
      out += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) +
             ", id: " + String(id) +
             ", home: " + String(home, 2) +
             ", min: " + String(lo, 2) +
             ", max: " + String(hi, 2) +
             ", span: " + String(span, 2) +
             ", status: " + status + "}\n";
    }
    commandPrint(out);

  } else if (cmd == "io_profile") {
    const exo_io::Snapshot snap = exo_io::profile().snapshot(micros());
    const char* names[] = {"other", "command", "control", "model", "dxl_read",
                          "dxl_write", "nx_pack", "usb_tx", "bt_tx", "peripheral"};
    char line[144];
    char decimal[21];
    snprintf(line, sizeof(line), "IO_PROFILE: version=1 window_us=%s",
             exo_io::decimal(snap.window_us, decimal));
    String result(line);
    for (uint8_t i = 0; i < exo_io::COUNT; ++i) {
      snprintf(line, sizeof(line), "\nIO_STAGE: name=%s calls=%lu exclusive_us=%s max_span_us=%lu",
               names[i], (unsigned long)snap.stages[i].calls,
               exo_io::decimal(snap.stages[i].exclusive_us, decimal),
               (unsigned long)snap.stages[i].max_span_us);
      result += line;
    }
    commandPrint(result);

  } else if (cmd == "reset_io_profile") {
    exo_io::profile().reset(micros());
    loopStatsReset();
    commandPrint("OK: io profile reset");

  } else if (cmd == "loop_stats") {
    // Loop period is the floor on command latency: a byte sitting in the USB
    // buffer is not seen until the loop comes back around to poll it.
    commandPrint("Loop: n=" + String(loopStatsCount()) +
                 " mean=" + String(loopStatsMeanUs()) + "us" +
                 " max=" + String(loopStatsMaxUs()) + "us");

  } else if (cmd == "reset_loop_stats") {
    loopStatsReset();
    commandPrint("OK: loop stats reset");

  } else if (cmd == "get_reply_route") {
    const char* routeName = (gReplyRoute == REPLY_ROUTE_TELEM) ? "telem" :
                            (gReplyRoute == REPLY_ROUTE_CMD)   ? "cmd"   : "both";
    commandPrint("Reply route: " + String(routeName));

  } else if (cmd == "reboot") {
    String arg = getArg(token, 1);
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      for (int i = 0; i < exo.getMotorCount(); i++) {
        uint8_t id = exo.getMotorIDByIndex(i);
        exo.rebootMotor(id);
      }
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) exo.rebootMotor(id);
    }

  } else if (cmd == "home") {
    // Case-insensitive ALL, matching every other multi-motor command. The old
    // lowercase-only compare made "home:ALL" fall through to a name lookup,
    // which silently did nothing.
    String target = getArg(token, 1);
    target.trim();
    String upper = target; upper.toUpperCase();
    if (upper == "ALL") {
      exo.homeAllMotors();
      commandPrint(F("OK: home all"));
    } else {
      id = exo.getMotorID(target);
      if (id != -1) {
        exo.setHome(id);
        commandPrint("OK: home " + String(id));
      } else {
        commandPrint("ERROR: home needs a valid motor ID/name or ALL");
      }
    }

  } else if (cmd == "hold_position") {
    // Atomic mixed-mode hold for one explicit DXL ID. Bare names are rejected
    // because they are ambiguous in dual firmware.
    String targetToken = getArg(token, 1);
    String angleToken = getArg(token, 2);
    String currentToken = getArg(token, 3);
    targetToken.trim();
    angleToken.trim();
    currentToken.trim();
    int holdId = targetToken.toInt();
    if (holdId <= 0 || holdId > 255 ||
        exo.getIndexById((uint8_t)holdId) == -1 || angleToken.length() == 0) {
      commandPrint(F("ERROR: usage hold_position:<explicit ID>:<relative angle>"));
    } else {
      float holdAngle = angleToken.toFloat();
      long requestedCurrent = 0;
      if (currentToken.length() > 0) {
        requestedCurrent = currentToken.toInt();
        if (requestedCurrent <= 0) {
          commandPrint(F("ERROR: hold current must be a positive mA value"));
          return;
        }
        requestedCurrent = constrain(requestedCurrent, 1L, 65535L);
      }
      if (exo.holdRelativePosition(
              (uint8_t)holdId, holdAngle, (uint16_t)requestedCurrent)) {
        commandPrint("OK: hold_position id=" + String(holdId) +
                     " angle=" + String(holdAngle, 3) +
                     " current_mA=" +
                     String(exo.getPositionHoldCurrent((uint8_t)holdId)));
      } else {
        commandPrint(F("ERROR: position hold requires global velocity/current mode"));
      }
    }

  } else if (cmd == "release_hold") {
    String targetToken = getArg(token, 1);
    targetToken.trim();
    int holdId = targetToken.toInt();
    if (holdId <= 0 || holdId > 255 ||
        exo.getIndexById((uint8_t)holdId) == -1) {
      commandPrint(F("ERROR: usage release_hold:<explicit ID>"));
    } else if (exo.releasePositionHold((uint8_t)holdId)) {
      commandPrint("OK: release_hold id=" + String(holdId));
    } else {
      commandPrint(F("ERROR: release_hold failed"));
    }

  } else if (cmd == "stop") {
    String target = getArg(token, 1);
    target.trim(); target.toUpperCase();
    if (target == "ALL") {
      exo.stopAllDirectControl();
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) exo.stopDirectControl(id);
    }
    commandPrint("OK: stop");

  } else if (cmd == "calibrate_rom") {
    // Impedance range-of-motion calibration for ONE joint in ONE direction.
    // Usage: calibrate_rom:<id>:<flex|extend>
    // Requires the global mode to already be CURRENT (set_control_mode:all:current).
    // The endstop is reported asynchronously on a ROM_CAL_RESULT line; this ack
    // only says the sweep was accepted and armed.
    id = getArgMotorID(exo, token, 1);
    String dirArg = getArg(token, 2);
    dirArg.trim(); dirArg.toLowerCase();
    if (id == -1) {
      commandPrint(F("ERROR: calibrate_rom requires a valid motor ID"));
    } else if (dirArg != "flex" && dirArg != "extend") {
      commandPrint(F("ERROR: calibrate_rom direction must be flex or extend"));
    } else {
      RomCalDirection dir = (dirArg == "flex") ? ROM_CAL_DIR_FLEX
                                               : ROM_CAL_DIR_EXTEND;
      if (exo.beginRomCalibration((uint8_t)id, dir)) {
        commandPrint("OK: calibrate_rom id=" + String(id) + " dir=" + dirArg);
      } else {
        commandPrint(F("ERROR: calibrate_rom needs CURRENT mode, a reachable "
                       "idle joint, and no sweep already running"));
      }
    }

  } else if (cmd == "calibrate_rom_gesture") {
    // Impedance ROM calibration for a whole GESTURE axis in one direction.
    // Usage: calibrate_rom_gesture:<gesture>:<flex|extend>
    // The gesture (thumb/index/middle/ring/pinky/wrist, or any angle-addressable
    // gesture) is decomposed into the motors its flex posture drives -- the same
    // motors set_finger_angles moves -- and each is swept in turn. In a dual
    // build a name that exists on both sides sweeps both. One ROM_CAL_RESULT
    // line is emitted per motor. Requires the global mode to be CURRENT.
    String gestureArg = getArg(token, 1);
    gestureArg.trim();
    String dirArg = getArg(token, 2);
    dirArg.trim(); dirArg.toLowerCase();
    if (gestureArg.length() == 0) {
      commandPrint(F("ERROR: calibrate_rom_gesture requires a gesture name"));
    } else if (dirArg != "flex" && dirArg != "extend") {
      commandPrint(F("ERROR: calibrate_rom_gesture direction must be flex or extend"));
    } else {
      uint8_t gestureIds[N_MOTORS];
      uint8_t gestureCount = gc.resolveGestureMotorIds(gestureArg, gestureIds, N_MOTORS);
      RomCalDirection dir = (dirArg == "flex") ? ROM_CAL_DIR_FLEX
                                               : ROM_CAL_DIR_EXTEND;
      if (gestureCount == 0) {
        commandPrint("ERROR: calibrate_rom_gesture unknown or non-addressable "
                     "gesture: " + gestureArg);
      } else if (!exo.canStartRomCalibration()) {
        commandPrint(F("ERROR: calibrate_rom_gesture needs CURRENT mode and no "
                       "sweep already running"));
      } else {
        // ACK FIRST, then start. beginRomCalibrationBatch emits one
        // ROM_CAL_RESULT per motor (including aborted lines for offline motors)
        // as it runs; the host reads exactly `motors` of those AFTER this ack,
        // so the ack must reach the wire before the first result line. Ordering
        // it after the batch would let an offline motor's result frame arrive
        // before the ack and be misread as the reply.
        commandPrint("OK: calibrate_rom_gesture " + gestureArg + " dir=" +
                     dirArg + " motors=" + String(gestureCount));
        exo.beginRomCalibrationBatch(gestureIds, gestureCount, dir);
      }
    }

  } else if (cmd == "cancel_rom") {
    exo.cancelRomCalibration();
    commandPrint(F("OK: cancel_rom"));

  } else if (cmd == "set_command_timeout") {
    unsigned long timeoutMs = (unsigned long)getArg(token, 1).toInt();
    exo.setDirectCommandTimeout(timeoutMs);
    commandPrint("Direct command timeout: " + String(exo.getDirectCommandTimeout()) + " ms");

  } else if (cmd == "get_command_timeout") {
    commandPrint("Direct command timeout: " + String(exo.getDirectCommandTimeout()) + " ms");

  } else if (cmd == "get_motor_mode") {
    String mode = exo.getMotorControlMode();
    commandPrint("Motor control mode: " + mode);

  } else if (cmd == "set_motor_mode" || cmd == "set_control_mode") {
    String modeStr = getArg(token, cmd == "set_control_mode" ? 2 : 1);
    if (cmd == "set_control_mode") {
      String target = getArg(token, 1);
      target.trim(); target.toUpperCase();
      if (target != "ALL") {
        commandPrint("Invalid target. Direct control mode changes currently require 'all'.");
        return;
      }
    }
    if (modeStr == "position" || modeStr == "current_position" ||
        modeStr == "velocity" || modeStr == "current") {
      if (exo.setMotorControlMode(modeStr)) {
        commandPrint("Motor control mode: " + modeStr + " (torque remains off until explicitly enabled)");
      } else {
        commandPrint(F("ERROR: motor mode change failed safety verification; torque remains off"));
      }
    } else {
      commandPrint("Invalid motor mode. Use position, current_position, velocity, or current.");
    }

  } else if (cmd == "get_exo_mode") {
    String mode = exo.getExoOperatingMode();
    commandPrint("Exo device mode: " + mode);

  } else if (cmd == "set_exo_mode") {
    String modeStr = getArg(token, 1);
    exo.setExoOperatingMode(modeStr);
    
  } else if (cmd == "get_gesture") {
    String current_gesture = gc.getCurrentGesture();
    commandPrint("Current gesture: " + current_gesture);

  } else if (cmd == "gesture_list") {
      String out = "Gestures:\n";
      for (int i = 0; i < N_GESTURES; ++i) {
        out += "  - ";
        out += gestureLibrary[i].name;
        out += "\n";
      }
      commandPrint(out);

  } else if (cmd == "set_gesture") {
    String gestureStr = getArg(token, 1);
    String stateStr = getArg(token, 2);
    gc.executeGesture(gestureStr, stateStr);
    // Acknowledge so the host has a delimited reply to wait on; without this a
    // request/response transaction on this command can only ever time out.
    commandPrint("OK: gesture " + gestureStr + ":" + stateStr);

  } else if (cmd == "cycle_gesture") {
    debugPrint(F("[GestureController] cycle gesture button pressed"));
    String exo_mode = exo.getExoOperatingMode();
    if (exo_mode == "GESTURE_FIXED" || exo_mode == "GESTURE_CONTINUOUS") {
      gc.cycleGesture();
    } else {
      debugPrint(F("Current mode FREE. Change mode to cycle gestures"));
    }

  } else if (cmd == "get_gesture_state") {
    String current_gesture = gc.getCurrentGestureState();
    commandPrint("Current gesture state: " + current_gesture);

  } else if (cmd == "cycle_gesture_state") {
    debugPrint(F("[GestureController] cycle gesture button pressed"));
    String exo_mode = exo.getExoOperatingMode();
    if (exo_mode == "GESTURE_FIXED" || exo_mode == "GESTURE_CONTINUOUS") {
      gc.cycleGestureState();
    } else {
      debugPrint(F("Current mode FREE. Change mode to cycle gesture states"));
    }

  } else if (cmd == "set_gesture_state") {
    String stateStr = getArg(token, 1);
    gc.executeCurrentGestureNewState(stateStr);
    commandPrint("OK: gesture_state " + stateStr);

  } else if (cmd == "set_gesture_angle") {
    String gestureStr = getArg(token, 1);
    String pctStr = getArg(token, 2);
    gestureStr.trim();
    pctStr.trim();
    if (gestureStr.length() == 0 || pctStr.length() == 0) {
      commandPrint(F("ERROR: usage set_gesture_angle:<gesture>:<0-100>"));
      return;
    }
    // String::toFloat() returns 0.0 for non-numeric input, and 0.0 is a
    // meaningful percentage here (full extension), so a typo would silently
    // command a real move. Validate the token before trusting it.
    bool numeric = true;
    for (uint16_t i = 0; i < pctStr.length(); ++i) {
      char c = pctStr[i];
      if (isDigit(c) || c == '.' || ((c == '-' || c == '+') && i == 0)) continue;
      numeric = false;
      break;
    }
    if (!numeric) {
      commandPrint("ERROR: set_gesture_angle percent not numeric: " + pctStr);
      return;
    }
    float pct = pctStr.toFloat();
    uint8_t moved = 0;
    uint8_t stuck = 0;
    if (gc.setGestureAngle(gestureStr, pct, &moved, &stuck)) {
      // The bare "OK" only ever meant the command parsed. Joints with no
      // calibrated travel are the failure mode this command actually has, and
      // they are invisible from the reply unless it says so -- the suffix is
      // appended only when there is something to report, so the leading
      // "OK: gesture_angle <name>:<pct>" form older hosts match on is intact.
      String reply = "OK: gesture_angle " + gestureStr + ":" +
                     String(constrain(pct, 0.0f, 100.0f), 1);
      if (stuck) {
        reply += " moved=" + String(moved) + " zero_travel=" + String(stuck);
      }
      commandPrint(reply);
    } else {
      commandPrint("ERROR: set_gesture_angle unknown or non-addressable gesture: " + gestureStr);
    }

  } else if (cmd == "set_finger_angles") {
    // Positional batch form of set_gesture_angle. One command positions every
    // named joint, in the fixed order below, so the continuous UDP path sends
    // one write and gets one reply per frame instead of one of each per joint.
    //
    //   set_finger_angles:<thumb>:<index>:<middle>:<ring>:<pinky>[:<wrist>]
    //
    // Each field is a SIGNED INTEGER in [-100, 100] anchored at the gesture's
    // calibrated REST posture: -100 is its extend posture, 0 is its rest
    // posture, +100 is its flex posture. This matches the continuous decoder
    // convention (positive flex, negative extend, zero rest) directly, so the
    // host scales its [-1, 1] value by 100, rounds, and sends it as-is. An
    // EMPTY field HOLDS that joint unchanged -- so a host driving only some
    // fingers, or one whose decoder has fewer channels than the hand has
    // joints, leaves the rest where they are. Trailing joints may simply be
    // omitted (fewer colons), which holds them the same way an empty field
    // does.
    //
    // Signed integers, not percentages, on purpose: the continuous path is the
    // only caller, the -100..100 range carries direction in the sign, and an
    // integer is shorter on the wire than a 0-100.0 percentage. Fixed order
    // matches UDP_GESTURE_JOINTS in the SDK so the receiver and the firmware
    // share one contract. `wrist` is optional and last because not every
    // build/host drives it.
    static const char* const kFingerOrder[] = {
      "thumb", "index", "middle", "ring", "pinky", "wrist"
    };
    const uint8_t kFingerCount = sizeof(kFingerOrder) / sizeof(kFingerOrder[0]);

    uint8_t commanded = 0;   // joints given a signed target
    uint8_t held = 0;        // joints with an empty/omitted field
    uint8_t unknown = 0;     // targets that named no addressable gesture
    uint8_t zeroTravel = 0;  // motors skipped for having no calibrated travel
    MotorAngleTarget targets[N_MOTORS];
    uint8_t targetCount = 0;
    String bad;              // first malformed field, for the error reply

    for (uint8_t k = 0; k < kFingerCount; ++k) {
      // getArg returns "" both for an explicitly empty field ("::") and for a
      // field past the end of the token, so an omitted trailing joint is held
      // exactly like an empty one -- no separate length check needed.
      String field = getArg(token, k + 1);
      field.trim();
      if (field.length() == 0) {
        ++held;
        continue;
      }
      // Signed integer only: toInt() reads a bad token as 0, which is a real
      // move (to rest), so reject anything that is not [+-]?digits up front. No
      // '.' here -- the wire contract is integers.
      bool numeric = true;
      for (uint16_t i = 0; i < field.length(); ++i) {
        char c = field[i];
        if (isDigit(c) || ((c == '-' || c == '+') && i == 0)) continue;
        numeric = false;
        break;
      }
      if (!numeric || (field.length() == 1 && (field[0] == '-' || field[0] == '+'))) {
        if (bad.length() == 0) bad = String(kFingerOrder[k]) + "=" + field;
        continue;
      }
      long signedValue = field.toInt();
      MotorAngleTarget gestureTargets[N_MOTORS];
      uint8_t gestureTargetCount = 0;
      uint8_t stuck = 0;
      if (gc.resolveGestureSignedTargets(String(kFingerOrder[k]),
                                         (float)signedValue,
                                         gestureTargets, N_MOTORS,
                                         gestureTargetCount, &stuck)) {
        ++commanded;
        zeroTravel += stuck;
        for (uint8_t j = 0; j < gestureTargetCount; ++j) {
          // Gesture definitions should not overlap in this fixed command, but
          // replacing a duplicate keeps the final target unambiguous if they
          // ever do. Never transmit until every field has parsed successfully.
          int existing = -1;
          for (uint8_t n = 0; n < targetCount; ++n) {
            if (targets[n].id == gestureTargets[j].id) {
              existing = n;
              break;
            }
          }
          if (existing >= 0) {
            targets[existing] = gestureTargets[j];
          } else if (targetCount < N_MOTORS) {
            targets[targetCount++] = gestureTargets[j];
          } else if (bad.length() == 0) {
            bad = "too_many_motor_targets";
          }
        }
      } else {
        // A joint the firmware does not define (e.g. no wrist on this build)
        // is counted, not fatal: the rest of the array still applies.
        ++unknown;
      }
    }

    uint8_t written = 0;
    uint8_t skippedOffline = 0;
    int16_t syncLibError = DXL_LIB_OK;
    bool syncOk = true;
    if (bad.length() == 0 && targetCount > 0) {
      syncOk = exo.setAbsoluteAnglesSync(targets, targetCount, &written,
                                         &skippedOffline, &syncLibError);
    }

    if (bad.length()) {
      commandPrint("ERROR: set_finger_angles field not a signed integer: " + bad);
    } else if (!syncOk) {
      commandPrint("ERROR: set_finger_angles no reachable targets or Sync Write failed"
                   " requested=" + String(targetCount) +
                   " skipped_offline=" + String(skippedOffline) +
                   " lib_error=" + String(syncLibError));
    } else {
      // Leading "OK: finger_angles" keeps a stable prefix for host matching;
      // the counts follow so a caller can see holds, unknown joints, and
      // zero-travel motors without a separate query.
      String reply = "OK: finger_angles commanded=" + String(commanded) +
                     " held=" + String(held) +
                     " motors=" + String(written) +
                     " transport=sync_write";
      if (skippedOffline) {
        reply += " skipped_offline=" + String(skippedOffline);
      }
      if (unknown) reply += " unknown=" + String(unknown);
      if (zeroTravel) reply += " zero_travel=" + String(zeroTravel);
      commandPrint(reply);
    }

  } else if (cmd == "get_gesture_angle" ||
             cmd == "get_gesture_sang" ||
             cmd == "get_gesture_angles") {
    // Three views of the same batched position sample:
    //   get_gesture_angle  -> legacy 0-100 percentage/status code
    //   get_gesture_sang  -> signed physical delta from rest, in degrees
    //   get_gesture_angles -> both as <code>,<signed-degrees>
    //
    // The percentage is the read-back half of set_gesture_angle, on the same
    // axis: 0 is the gesture's extend posture and 100 is its flex posture, so a
    // gesture commanded to 40 reports 40 once it arrives.
    //
    // 101 and 102 mean it sits below or above those two postures -- reachable
    // by hand, by a set_angle off the axis, or simply by sitting at home, which
    // is BELOW extend whenever EXTEND_* is non-zero. 255 means no position is
    // available (no calibrated travel, or the read failed); `check_limits` says
    // which.
    //
    // Signed degrees use the first motor named by the gesture as the calibrated
    // physical scale. Rest is 0, toward flex is positive, and toward extend is
    // negative. `nan` means that physical angle is unavailable.
    //
    // Emitted with commandPrint, so on a dual-CDC build it follows the active
    // reply route: with `set_reply_route:telem` it lands on the telemetry CDC
    // only and never shares the command port.
    String target = getArg(token, 1);
    target.trim();
    if (target.equalsIgnoreCase("all")) target = "";

    GestureAngleRecord records[N_GESTURES];
    uint8_t count = gc.readGestureAngles(records, N_GESTURES, target);
    if (count == 0) {
      String named = target.length() ? target : String("all");
      commandPrint("ERROR: " + cmd + " unknown or non-addressable gesture: " + named);
      return;
    }
    const bool signedOnly = cmd == "get_gesture_sang";
    const bool combined = cmd == "get_gesture_angles";
    String out = signedOnly ? "GESTURE_SANG:" :
                 (combined ? "GESTURE_ANGLES:" : "GESTURE_ANGLE:");
    for (uint8_t i = 0; i < count; ++i) {
      out += " ";
      out += gestureLibrary[records[i].gesture].name;
      out += "=";
      if (!signedOnly) {
        out += String(records[i].code);
        if (combined) out += ",";
      }
      if (signedOnly || combined) {
        if (isnan(records[i].signedAngleDeg)) {
          out += "nan";
        } else {
          out += String(records[i].signedAngleDeg, 2);
        }
      }
    }
    commandPrint(out);

  } else if (cmd == "set_zero_offset") {
    String arg = getArg(token, 1);
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      float val = getArg(token, 2).toFloat();
      for (int i = 0; i < exo.getMotorCount(); i++) {
        uint8_t mid = exo.getMotorIDByIndex(i);
        exo.setZeroOffsetValue(mid, val);
      }
    } else {
      id = getArgMotorID(exo, token, 1);
      float offset = getArg(token, 2).toFloat();
      if (id != -1) exo.setZeroOffsetValue(id, offset);
    }

  } else if (cmd == "set_flip") {
    id = getArgMotorID(exo, token, 1);
    String flipStr = getArg(token, 2);
    flipStr.trim();
    bool flip = (flipStr == "1" || flipStr == "true");
    if (id != -1) exo.setFlipMotor(id, flip);

  } else if (cmd == "get_flip") {
    String arg = getArg(token, 1);
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor Flip Status:\n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t mid = exo.getMotorIDByIndex(i);
        bool flip = exo.getFlipMotor(mid);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(mid) + ", id: " + String(mid) +
            ", flip: " + (flip ? "true" : "false") + "}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        bool flip = exo.getFlipMotor(id);
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", flip: " + (flip ? "true" : "false") + "}");
      }
    }

  } else if (cmd == "calibrate_exo") {
    debugPrint(F("Command not supported yet"));
    //String mode = getArg(token, 1);
    //bool timed = (mode == "timed");
    //float duration = getArg(token, 2).toFloat();
    //if (duration <=0 ) duration = 10;
    //exo.beginCalibration(timed, duration);

  } else if (cmd == "version") {
    // Print the current version of the exo device
    commandPrint("Exo Device Version: " + String(NMLHandExo::VERSION));

  } else if (cmd == "info") {
    debugPrint(F("Device Info: "));
    commandPrint(exo.getDeviceInfo(false));

  } else if (cmd == "info_verbose") {
    debugPrint(F("Verbose Device Info: "));
    commandPrint(exo.getDeviceInfo(true));

  } else if (cmd == "get_imu") {
    getIMUData(imu);

  } else if (token == "oled:on") {
    oledSetEnabled(true);
    if (oledEnabled()) commandPrint("OLED enabled.");
    else               commandPrint("OLED init failed; disabled.");

  } else if (token == "oled:off") {
    oledSetEnabled(false);
    commandPrint("OLED disabled.");

  } else if (token == "oled:status") {
    commandPrint(oledEnabled() ? "OLED ENABLED" : "OLED DISABLED");
  
  } else if (token == "help") {
    commandPrint(F(" ================================== List of commands ======================================"));
    commandPrint(F(" led                   |  ID/NAME/ALL:ON/OFF  | // Turn motor or system LED on/off"));
    commandPrint(F(" help                  |                      | // Display available commands"));
    commandPrint(F(" home                  |  ID/NAME/ALL         | // Set specific motor (or all) to home position"));
    commandPrint(F(" info                  |                      | // Gets information about exo device. Returns string of metadata with comma delimiters"));
    commandPrint(F(" info_verbose          |                      | // Gets info plus live motor telemetry (slow if motors are offline)"));
    commandPrint(F(" debug                 |  ON/OFF              | // Set verbose output on/off"));
    commandPrint(F(" set_reply_route       |  BOTH/TELEM/CMD      | // Route replies: both(legacy)/telem(decoupled)/cmd (dual-CDC)"));
    commandPrint(F(" get_reply_route       |                      | // Get current reply/telemetry CDC route"));
    commandPrint(F(" check_limits          |                      | // Per-motor gesture span; flags NO_TRAVEL / HOME_OUTSIDE joints"));
    commandPrint(F(" io_profile            |                      | // Exclusive stage times and call maxima"));
    commandPrint(F(" reset_io_profile      |                      | // Reset I/O and loop statistics"));
    commandPrint(F(" loop_stats            |                      | // Loop period: n / mean / max microseconds"));
    commandPrint(F(" reset_loop_stats      |                      | // Clear loop statistics"));
    commandPrint(F(" reboot                |  ID/NAME/ALL         | // Reboot motor"));
    commandPrint(F(" version               |                      | // Get current software version"));
    commandPrint(F(" enable                |  ID/NAME             | // Enable torque for motor"));
    commandPrint(F(" disable               |  ID/NAME             | // Disable torque for motor"));
    commandPrint(F(" enable_ids            |  ID:ID:ID...         | // Enable torque for a list of DXL IDs"));
    commandPrint(F(" disable_ids           |  ID:ID:ID...         | // Disable torque for a list of DXL IDs"));
    commandPrint(F(" get_enable            |  ID/NAME             | // Get the torque enable status of the motor"));
    commandPrint(F(" get_baud              |  ID/NAME             | // Get baud rate for motor"));
    commandPrint(F(" set_baud              |  ID/NAME:VALUE       | // Set baud rate for motor"));
    commandPrint(F(" get_goal_velocity     |  ID/NAME             | // Get current velocity profile for motor"));
    commandPrint(F(" set_goal_velocity     |  ID/NAME/ALL:VALUE   | // Set velocity profile for motor"));
    commandPrint(F(" get_velocity          |  ID/NAME/ALL         | // Get present velocity in rpm"));
    commandPrint(F(" set_velocity          |  ID:SIGNED_RPM       | // Direct velocity command (velocity mode)"));
    commandPrint(F(" get_goal_acceleration |  ID/NAME             | // Get current acceleration profile for motor"));
    commandPrint(F(" set_goal_acceleration |  ID/NAME/ALL:VALUE   | // Set acceleration limit for motor"));
    commandPrint(F(" get_home              |  ID/NAME             | // Get stored zero position"));
    commandPrint(F(" set_home              |  ID/NAME:VALUE       | // Set current position as new zero angle"));
    commandPrint(F(" get_angle             |  ID/NAME             | // Get relative motor angle"));
    commandPrint(F(" set_angle             |  ID/NAME:ANGLE       | // Set motor angle"));
    commandPrint(F(" get_absolute_angle    |  ID/NAME/ALL         | // Get absolute motor angle"));
    commandPrint(F(" set_absolute_angle    |  ID/NAME:ANGLE       | // Set absolute motor angle"));
    commandPrint(F(" get_torque            |  ID/NAME             | // Get torque output reading from motor"));
    commandPrint(F(" get_current           |  ID/NAME             | // Get current draw from motor"));
    commandPrint(F(" set_current_lim       |  ID/NAME:VAL         | // Set per-motor current draw limit"));
    commandPrint(F(" set_total_current_lim |  MA                  | // Set COMBINED current budget across all motors"));
    commandPrint(F(" get_total_current_lim |                      | // Get combined current budget"));
    commandPrint(F(" set_hold_current      |  MA                  | // Current allowed to a settled/shed motor"));
    commandPrint(F(" get_hold_current      |                      | // Get hold current"));
    commandPrint(F(" set_current_governor  |  ON/OFF              | // Closed-loop budget enforcement (off = static clamp)"));
    commandPrint(F(" current_status        |                      | // Budget, measured aggregate draw, per-motor allocation"));
    commandPrint(F(" get_goal_current      |  ID/NAME/ALL         | // Get direct current goal in mA"));
    commandPrint(F(" set_current           |  ID:SIGNED_MA        | // Direct current command (current mode)"));
    commandPrint(F(" stop                  |  ID/ALL              | // Zero direct velocity/current goals"));
    commandPrint(F(" hold_position         |  ID:ANGLE[:MA]       | // Hold one explicit ID with optional per-hold current"));
    commandPrint(F(" release_hold          |  ID                  | // Disable held ID and restore global mode"));
    commandPrint(F(" set_command_timeout   |  MILLISECONDS        | // Set direct-control watchdog (50-5000 ms)"));
    commandPrint(F(" get_telemetry_fast    |  ID:ID:ID.../ALL     | // Binary current/velocity/position telemetry frame"));
    commandPrint(F(" set_utc_time          | UNIX_MS | // Synchronize UTC; no effect on control timing"));
    commandPrint(F(" get_utc_time          |         | // UTC ms, synchronization flag and uptime"));
    commandPrint(F(" set_joint_model       | ID:GAIN:TAU:VMAX:STIFFNESS:MOMENT | // Tune telemetry model only"));
    commandPrint(F(" get_joint_model       | ID | // Query model and trigger threshold"));
    commandPrint(F(" set_estimate_holdoff  | MS | // Telemetry quiet period (50..5000)"));
    commandPrint(F(" set_joint_trigger     | ID:NM | // Store future assistance threshold; no motion"));
    commandPrint(F(" clear_joint_trigger   | ID | // Clear threshold"));
    commandPrint(F(" telemetry_diag        |  ID:ID:ID.../ALL     | // Text diagnostics for fast telemetry reads"));
    commandPrint(F(" shadow_config         |  INTERVAL:ID:ID...   | // Configure read-only contact evidence sampling"));
    commandPrint(F(" shadow_start          |  None                 | // Start sampling (VELOCITY mode only)"));
    commandPrint(F(" shadow_stop           |  None                 | // Stop sampling; no motor state changes"));
    commandPrint(F(" shadow_status         |  None                 | // Buffered current/position/derived-velocity snapshot"));
    commandPrint(F(" get_motor_limits      |  ID/NAME             | // Get motor limits (upper and lower bounds)"));
    commandPrint(F(" set_motor_limits      |  ID/NAME:VAL:VAL     | // Set motor limits (upper and lower bounds)"));
    commandPrint(F(" set_upper_limit       |  ID/NAME:ANGLE       | // Set the absolute upper bound position limit for the motor"));
    commandPrint(F(" set_lower_limit       |  ID/NAME:ANGLE       | // Set the absolute lower bound position limit for the motor"));
    commandPrint(F(" get_motor_mode        |                      | // Get motor control mode"));
    commandPrint(F(" set_motor_mode        |  VALUE               | // Set POSITION, CURRENT_POSITION, VELOCITY, or CURRENT"));
    commandPrint(F(" set_control_mode      |  ALL:VALUE           | // Safe direct-mode alias; torque remains off"));
    commandPrint(F(" get_exo_mode          |                      | // Get exo device operation mode"));
    commandPrint(F(" set_exo_mode          |  VALUE               | // Set exo device operation mode (FREE', 'GESTURE_FIXED', 'GESTURE_CONTINUOUS')"));
    commandPrint(F(" gesture_list          |                      | // Get gestures in library"));
    commandPrint(F(" set_gesture           |  NAME:VALUE          | // Set exo gesture"));
    commandPrint(F(" get_gesture           |                      | // Get exo gesture"));
    commandPrint(F(" set_gesture_state     |  NAME:VALUE          | // Set exo gesture state"));
    commandPrint(F(" set_gesture_angle     |  NAME:0-100          | // Interpolate a gesture: 0=its extend state, 100=its flex state"));
    commandPrint(F(" set_finger_angles     |  T:I:M:R:P[:W]       | // Batch signed -100..100 (thumb:index:middle:ring:pinky[:wrist]): -100 extend, 0 rest, +100 flex; empty holds"));
    commandPrint(F(" get_gesture_angle     |  NAME/ALL            | // Read positions as 0-100 (101/102 out of range, 255 no travel)"));
    commandPrint(F(" get_gesture_sang      |  NAME/ALL            | // Read signed degrees from rest: flex positive, extend negative"));
    commandPrint(F(" get_gesture_angles    |  NAME/ALL            | // Read <percent-code>,<signed-degrees> pairs"));
    commandPrint(F(" get_gesture_state     |                      | // Get exo gesture state"));
    commandPrint(F(" cycle_gesture         |                      | // Executes the next gesture in the library"));
    commandPrint(F(" cycle_gesture_state   |                      | // Cycles the next gesture state"));
    commandPrint(F(" set_zero_offset       |  ID/NAME/ALL:VALUE   | // Set the zero offset for a motor to an arbitrary angle"));
    commandPrint(F(" set_flip              |  ID/NAME:0/1         | // Set motor direction flip (1=inverted, 0=normal)"));
    commandPrint(F(" get_flip              |  ID/NAME/ALL         | // Get motor direction flip status"));
    commandPrint(F(" calibrate_exo         |  VALUE:VALUE         | // start the calibration routine for the exo"));
    commandPrint(F(" calibrate_rom         |  ID:FLEX/EXTEND      | // Impedance ROM sweep of one joint (needs CURRENT mode); reports ROM_CAL_RESULT"));
    commandPrint(F(" calibrate_rom_gesture |  NAME:FLEX/EXTEND    | // Impedance ROM sweep of every motor a gesture drives (thumb/wrist span several)"));
    commandPrint(F(" cancel_rom            |                      | // Abort a running impedance ROM sweep and return the joint home"));
    commandPrint(F(" get_imu               |                      | // Returns list of accel & gyro values"));
    commandPrint(F(" set_yaw_angle         |  ID/NAME:ANGLE       | // Set motor angle via IMU wrist angle"));
    commandPrint(F(" oled                  |  VALUE               | // Turn OLED on/off, get status"));
    commandPrint(F(" =========================================================================================="));
  } else {
    debugPrint("Unknown command: " + token);
  }
}

