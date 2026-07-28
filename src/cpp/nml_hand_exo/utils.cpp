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


// IMU variables (print delay logic currently disabled)
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

sensors_event_t orientationData, linearAccelData;



void telemetryPrintln(const String& msg) {
  // Route a device->host line to the correct USB CDC(s).
#if defined(DUAL_CDC) && DUAL_CDC
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
    COMMAND_SERIAL.println(cmdMsg);
  #else
    Serial2.println(cmdMsg);  // fallback
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

void parseMessage(NMLHandExo& exo, GestureController& gc, Adafruit_BNO055& imu, String token) {

  token.trim();        // Remove any trailing white space
  token.toLowerCase(); // Set all characters to lowercase

  String cmd = getArg(token, 0, ':'); // Get the command part before the first colon
  int id = -1; // Default to -1 if not found
  int val = 0; // Default value for commands that require a value

  // ========== Supported high-level commands ==========
  if (cmd == "enable") {
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
          uint32_t baud = exo.getBaudRate(i);  
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
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setBaudRate(id, val);

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
      commandPrint(ok ? "OK: set_velocity" : "ERROR: set_velocity requires velocity mode and a valid motor ID");
    }

  } else if (cmd == "get_velocity") {
    String arg = getArg(token, 1);
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor present velocity: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        float rpm = exo.getPresentVelocity(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", velocity: " + String(rpm, 3) + " rpm}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        float rpm = exo.getPresentVelocity(id);
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", velocity: " + String(rpm, 3) + " rpm}");
      }
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

  } else if (cmd == "get_angle") {
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor angles: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        float val = exo.getRelativeAngle(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", angle: " + String(val) + "}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        float val = exo.getRelativeAngle(id);
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", angle: " + String(val, 3) + "}");
      }
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

  } else if (cmd == "get_absolute_angle") {
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor absolute angles: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        float val = exo.getAbsoluteAngle(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", absolute_angle: " + String(val) + "}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        float val = exo.getAbsoluteAngle(id);
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", absolute_angle: " + String(val, 3) + "}");
      }
    }

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

  } else if (cmd == "get_current") {
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor absolute angles: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        float current_mA = exo.getCurrent(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", current: " + String(current_mA, 3) + " mA}\n";
      }
      commandPrint(info);
    } else {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) {
      float current_mA = exo.getCurrent(id);
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", current: " + String(current_mA, 3) + " mA}");
      }
    }

  } else if (cmd == "set_current_lim") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setCurrentLimit(id, val);

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

  } else if (cmd == "get_torque") {
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor Torque: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        float torque = exo.getTorque(id);
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", torque: " + String(torque, 4) + "}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        float torque = exo.getTorque(id);
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", torque: " + String(torque, 4) + " N·m}");
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
    String target = getArg(token, 1);
    if (target == "all") {
      exo.homeAllMotors();
    } else {
      id = exo.getMotorID(target);
      if (id != -1) exo.setHome(id);
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
      exo.setMotorControlMode(modeStr);
      commandPrint("Motor control mode: " + modeStr + " (torque remains off until explicitly enabled)");
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
    commandPrint(exo.getDeviceInfo());

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
    commandPrint(F(" debug                 |  ON/OFF              | // Set verbose output on/off"));
    commandPrint(F(" set_reply_route       |  BOTH/TELEM/CMD      | // Route replies: both(legacy)/telem(decoupled)/cmd (dual-CDC)"));
    commandPrint(F(" get_reply_route       |                      | // Get current reply/telemetry CDC route"));
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
    commandPrint(F(" set_current_lim       |  ID/NAME:VAL         | // Set current draw limit for motor"));
    commandPrint(F(" get_goal_current      |  ID/NAME/ALL         | // Get direct current goal in mA"));
    commandPrint(F(" set_current           |  ID:SIGNED_MA        | // Direct current command (current mode)"));
    commandPrint(F(" stop                  |  ID/ALL              | // Zero direct velocity/current goals"));
    commandPrint(F(" set_command_timeout   |  MILLISECONDS        | // Set direct-control watchdog (50-5000 ms)"));
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
    commandPrint(F(" get_gesture_state     |                      | // Get exo gesture state"));
    commandPrint(F(" cycle_gesture         |                      | // Executes the next gesture in the library"));
    commandPrint(F(" cycle_gesture_state   |                      | // Cycles the next gesture state"));
    commandPrint(F(" set_zero_offset       |  ID/NAME/ALL:VALUE   | // Set the zero offset for a motor to an arbitrary angle"));
    commandPrint(F(" set_flip              |  ID/NAME:0/1         | // Set motor direction flip (1=inverted, 0=normal)"));
    commandPrint(F(" get_flip              |  ID/NAME/ALL         | // Get motor direction flip status"));
    commandPrint(F(" calibrate_exo         |  VALUE:VALUE         | // start the calibration routine for the exo"));
    commandPrint(F(" get_imu               |                      | // Returns list of accel & gyro values"));
    commandPrint(F(" set_yaw_angle         |  ID/NAME:ANGLE       | // Set motor angle via IMU wrist angle"));
    commandPrint(F(" oled                  |  VALUE               | // Turn OLED on/off, get status"));
    commandPrint(F(" =========================================================================================="));
  } else {
    debugPrint("Unknown command: " + token);
  }
}

