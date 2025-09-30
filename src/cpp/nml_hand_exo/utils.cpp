/**
 * @file utils.cpp
 * @brief A cpp file for supporting functions
 *
 */
#include "utils.h"
#include <Arduino.h>
#include "nml_hand_exo.h"
#include "gesture_controller.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


// IMU variables (print delay logic currently disabled)
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

sensors_event_t orientationData, linearAccelData;



void debugPrint(const String& msg) {
  // Prints out message to debug serial port if VERBOSE is set to true
  if (VERBOSE) {
    #if defined(DEBUG_SERIAL)
      //DEBUG_SERIAL(msg);
      DEBUG_SERIAL.println(msg);
    #else
      Serial.println(msg);  // fallback
    #endif
  }
}

void commandPrint(const String& msg) {
  // Append delimiter if it's not already present
  String cmdMsg = msg;  // Make a copy to modify
  String cmdDelimiter = COMMAND_DELIMITER;
  if (!cmdMsg.endsWith(cmdDelimiter)) {
    cmdMsg += cmdDelimiter;
  }

  // Prints out message to DEBUG/CMD serial port regardless of VERBOSE mode 
  #if defined(COMMAND_SERIAL)
    // Need to print to all serial devices for data output
    COMMAND_SERIAL.println(cmdMsg);
    DEBUG_SERIAL.println(cmdMsg);
  #else
    Serial2.println(cmdMsg);  // fallback
    Serial.println(cmdMsg);  // fallback
  #endif
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
  unsigned long tStart = micros();

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  
  //while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
    // wait
  //}
}

void getIMUData(Adafruit_BNO055& bno) {
  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    printCount = 0;
  } else {
    printCount++;
  }

  String imu_data = "Heading: " + String(orientationData.orientation.x) + ", Pitch: " + String(orientationData.orientation.y) + ", Roll: " + String(orientationData.orientation.z); 

  commandPrint(imu_data);
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
    arg.toUpperCase();
    if (arg == "ALL") {
      for (int i = 0; i < exo.getMotorCount(); i++) {
        exo.enableTorque(i, true);
      }
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) exo.enableTorque(id, true);
    }

  } else if (cmd == "disable") {
    String arg = getArg(token, 1);  // local copy
    arg.trim();
    arg.toUpperCase();
    if (arg == "ALL") {
      for (int i = 0; i < exo.getMotorCount(); i++) {
        exo.enableTorque(i, false);
      }
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) exo.enableTorque(id, false);
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
        float current_mA = exo.getCurrent(id) * 2.69f;
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", current: " + String(current_mA, 3) + " mA}\n";
      }
      commandPrint(info);
    } else {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) {
      float current_mA = exo.getCurrent(id) * 2.69f;
        commandPrint("Motor: {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
          ", current: " + String(current_mA, 3) + " mA}");
      }
    }

  } else if (cmd == "set_current_lim") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setCurrentLimit(id, val);

  } else if (cmd == "get_current_lim") {
    String arg = getArg(token, 1);  // local copy
    arg.trim(); arg.toUpperCase();
    if (arg == "ALL") {
      String info = "Motor absolute angles: \n";
      for (int i = 0; i < exo.getMotorCount(); ++i) {
        uint8_t id = exo.getMotorIDByIndex(i);
        float current_mA = exo.getCurrentLimit(id) * 2.69f;
        info += "Motor " + String(i) + ": {name: " + exo.getMotorNameByID(id) + ", id: " + String(id) +
            ", current_limit: " + String(current_mA, 3) + "mA}\n";
      }
      commandPrint(info);
    } else {
      id = getArgMotorID(exo, token, 1);
      if (id != -1) {
        float current_mA = exo.getCurrentLimit(id) * 2.69f;
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
            ", Torque: " + String(val) + "}";
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
    String limitsStr = getArg(token, 2);
    int colonIndex = limitsStr.indexOf(':');
    if (colonIndex != -1) {
      float lowerLimit = limitsStr.substring(0, colonIndex).toFloat();
      float upperLimit = limitsStr.substring(colonIndex + 1).toFloat();
      if (id != -1) exo.setMotorLimits(id, lowerLimit, upperLimit);
    } else {
      commandPrint("[Error] Invalid limits format. Use 'lower:upper'");
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

  } else if (cmd == "get_motor_mode") {
    String mode = exo.getMotorControlMode();
    commandPrint("Motor control mode: " + mode);

  } else if (cmd == "set_motor_mode") {
    String modeStr = getArg(token, 1);
    if (modeStr == "position" || modeStr == "current_position" || modeStr == "velocity") {
      exo.setMotorControlMode(modeStr);
    } else {
      commandPrint("Invalid motor mode. Use 'position', 'current_position', or 'velocity'.");
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
    commandPrint("Command not supported yet");

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
  
  } else if (token == "help") {
    commandPrint(F(" ================================== List of commands ======================================"));
    commandPrint(F(" led                   |  ID/NAME/ALL:ON/OFF  | // Turn motor or system LED on/off"));
    commandPrint(F(" help                  |                      | // Display available commands"));
    commandPrint(F(" home                  |  ID/NAME/ALL         | // Set specific motor (or all) to home position"));
    commandPrint(F(" info                  |                      | // Gets information about exo device. Returns string of metadata with comma delimiters"));
    commandPrint(F(" debug                 |  ON/OFF              | // Set verbose output on/off"));
    commandPrint(F(" reboot                |  ID/NAME/ALL         | // Reboot motor"));
    commandPrint(F(" version               |                      | // Get current software version"));
    commandPrint(F(" enable                |  ID/NAME             | // Enable torque for motor"));
    commandPrint(F(" disable               |  ID/NAME             | // Disable torque for motor"));
    commandPrint(F(" get_enable            |  ID/NAME             | // Get the torque enable status of the motor"));
    commandPrint(F(" get_baud              |  ID/NAME             | // Get baud rate for motor"));
    commandPrint(F(" set_baud              |  ID/NAME:VALUE       | // Set baud rate for motor"));
    commandPrint(F(" get_goal_velocity     |  ID/NAME             | // Get current velocity profile for motor"));
    commandPrint(F(" set_goal_velocity     |  ID/NAME/ALL:VALUE   | // Set velocity profile for motor"));
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
    commandPrint(F(" set_current_lim       |  ID/NAME:VAL         | // Set current draw limit for motor"));
    commandPrint(F(" get_motor_limits      |  ID/NAME             | // Get motor limits (upper and lower bounds)"));
    commandPrint(F(" set_motor_limits      |  ID/NAME:VAL:VAL     | // Set motor limits (upper and lower bounds)"));
    commandPrint(F(" set_upper_limit       |  ID/NAME:ANGLE       | // Set the absolute upper bound position limit for the motor"));
    commandPrint(F(" set_lower_limit       |  ID/NAME:ANGLE       | // Set the absolute lower bound position limit for the motor"));
    commandPrint(F(" get_motor_mode        |                      | // Get motor control mode"));
    commandPrint(F(" set_motor_mode        |  VALUE               | // Set motor control mode ('POSITION', 'CURRENT_POSITION', 'VELOCITY')"));
    commandPrint(F(" get_exo_mode          |                      | // Get exo device operation mode"));
    commandPrint(F(" set_exo_mode          |  VALUE               | // Set exo device operation mode (FREE', 'GESTURE_FIXED', 'GESTURE_CONTINUOUS')"));
    commandPrint(F(" gesture_list          |                      | // Get gestures in library"));
    commandPrint(F(" set_gesture           |  NAME:VALUE          | // Set exo gesture"));
    commandPrint(F(" get_gesture           |                      | // Get exo gesture"));
    commandPrint(F(" set_gesture_state     |  NAME:VALUE          | // Set exo gesture state"));
    commandPrint(F(" get_gesture_state     |                      | // Get exo gesture state"));
    commandPrint(F(" cycle_gesture         |                      | // Executes the next gesture in the library"));
    commandPrint(F(" cycle_gesture_state   |                      | // Cycles the next gesture state"));
    commandPrint(F(" calibrate_exo         |  VALUE:VALUE         | // start the calibration routine for the exo"));
    commandPrint(F(" get_imu               |                      | // Returns list of accel & gyro values"));
    commandPrint(F(" =========================================================================================="));
  } else {
    debugPrint("Unknown command: " + token);
  }
}

