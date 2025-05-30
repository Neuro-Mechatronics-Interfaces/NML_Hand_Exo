#include "nml_hand_exo.h"
#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  //#define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  //#define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  //#define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  //#define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  //#define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  //#define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  //#define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

// Hardware constants for the dynamixel hardware and motors used
const float DXL_PROTOCOL_VERSION = 2.0;
const int PULSE_RESOLUTION = 4096;           // Ticks per revolution
const float XL330_TORQUE_CONSTANT = 0.00038; // N*m / mA

bool VERBOSE = false; // default to true

//HardwareSerial& DEBUG_SERIAL = Serial; // default debug output
Stream& DEBUG_SERIAL = Serial;       // Using Serial USB for debugging output
Stream* debugStream = &DEBUG_SERIAL; // Making pointer for serial object

// Debug printing function 
void debugPrint(const String& msg) {
  if (VERBOSE && debugStream) {
    debugStream->println("[DEBUG] " + msg);
  }
}


NMLHandExo::NMLHandExo(const uint8_t* ids, int numMotors, const int jointLimits[][2])
  : dxl_(DXL_SERIAL, DXL_DIR_PIN), ids_(ids), jointLimits_(jointLimits), numMotors_(numMotors) {}

// Utility functions
void NMLHandExo::initializeSerial(int baud) {
  // Initialize serial communication with DYNAMIXEL hardware using the specified baudrate. Has to match hardware
  dxl_.begin(baud);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl_.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}
void NMLHandExo::initializeMotors() {
  for (int i = 0; i < numMotors_; i++) {
    uint8_t id = ids_[i];
    dxl_.torqueOff(id);
    dxl_.setOperatingMode(id, OP_POSITION);  // Default mode is set to position mode
    dxl_.torqueOn(id);
  }
}
int NMLHandExo::getMotorID(const String& token) {
  String target = token;
  target.trim();
  target.toUpperCase();
  int id = target.toInt(); // Try converting to integer

  // Check if it was a valid number (e.g., not "WRIST")
  if (id != 0 || target == "0") {
    return id;
  }
  return getMotorIDByName(target);  // Otherwise, try name lookup
}
int NMLHandExo::getIndexById(uint8_t id) {
  for (int i = 0; i < numMotors_; i++) {
    if (ids_[i] == id) return i;
  }
  return -1;
}
int NMLHandExo::getMotorIDByName(const String& name) {
  String n = name;
  n.toUpperCase();
  if (n == "WRIST") return 1;
  if (n == "THUMB") return 2;
  if (n == "INDEX") return 3;
  if (n == "MIDDLE") return 4;
  if (n == "RING") return 5;
  if (n == "PINKY") return 6;
  return -1;
}
int NMLHandExo::angleToTicks(float angle_deg, int index) {
  // Map degrees to ticks: assume full range = 4096 ticks = 360 deg
  float deg_per_tick = 300.0 / PULSE_RESOLUTION;
  int ticks = static_cast<int>(angle_deg / deg_per_tick);
  return ticks;
}
void NMLHandExo::calibrateZero(uint8_t id) {
  int index = getIndexById(id);
  if (index != -1) {
    float current_angle = dxl_.getPresentPosition(id, UNIT_DEGREE);
    zeroOffsets_[index] = current_angle;
    debugPrint("[DEBUG] Calibrated zero for motor " + String(id) + ": " + String(current_angle, 2) + " deg");
  } else {
    debugPrint("[ERROR] Invalid motor ID for zero calibration: " + String(id));
  }
}
void NMLHandExo::resetAllZeros() {
  for (int i = 0; i < numMotors_; ++i) {
    uint8_t id = ids_[i];
    float current_angle = dxl_.getPresentPosition(id, UNIT_DEGREE);
    zeroOffsets_[i] = current_angle;
    debugPrint("[DEBUG] Zero offset set for motor " + String(id) + ": " + String(current_angle, 2) + " deg");
  }
}

// Position comands 
float NMLHandExo::getRelativeAngle(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return -1;

  float abs_angle = dxl_.getPresentPosition(id, UNIT_DEGREE);
  return abs_angle - zeroOffsets_[index];
}
float NMLHandExo::getAbsoluteAngle(uint8_t id) {
  return dxl_.getPresentPosition(id, UNIT_DEGREE);
}
float NMLHandExo::getZeroOffset(uint8_t id) {
  int index = getIndexById(id);
  return (index != -1) ? zeroOffsets_[index] : 0.0f;
}
void NMLHandExo::setAngleById(uint8_t id, float angle_deg) {
  int index = getIndexById(id);
  if (index == -1) return;

  // Apply offset to relative angle position
  float abs_goal = zeroOffsets_[index] + angle_deg;

  // Clamp angle to joint limits (in degrees)
  abs_goal = constrain(angle_deg, jointLimits_[index][0], jointLimits_[index][1]);

  // Set new goal tick position
  dxl_.setGoalPosition(id, abs_goal, UNIT_DEGREE);
  debugPrint("Setting motor " + String(id) + " to angle " + String(abs_goal, 2));
}
void NMLHandExo::setAngleByAlias(const String& alias, float angleDeg) {
  String name = alias;
  name.toUpperCase();
  if (name == "WRIST") setAngleById(1, angleDeg);
  else if (name == "THUMB") setAngleById(2, angleDeg);
  else if (name == "INDEX") setAngleById(3, angleDeg);
  else if (name == "MIDDLE") setAngleById(4, angleDeg);
  else if (name == "RING") setAngleById(5, angleDeg);
  else if (name == "PINKY") setAngleById(6, angleDeg);
}

// Torque commands
void NMLHandExo::enableTorque(uint8_t id, bool enable) {
  if (enable) {
    dxl_.torqueOn(id);
    debugPrint("Motor " + String(id) + " enabled");
  } else {
    dxl_.torqueOff(id);
    debugPrint("Motor " + String(id) + " disabled");
  }
}
int16_t NMLHandExo::getCurrent(uint8_t id) {
  return dxl_.readControlTableItem(PRESENT_CURRENT, id);
}
float NMLHandExo::getTorque(uint8_t id) {
  // Each unit = 2.69 mA; torque constant = 0.38 mN·m/mA = 0.00038 N·m/mA
  int16_t raw_current = NMLHandExo::getCurrent(id);
  float current_mA = raw_current * 2.69;
  float torque_Nm = current_mA * XL330_TORQUE_CONSTANT;
  return torque_Nm;  // in N·m
}

// Velocity commands
void NMLHandExo::setVelocityLimit(uint8_t id, uint32_t vel) {
  dxl_.writeControlTableItem(PROFILE_VELOCITY, id, vel);
  debugPrint("Velocity limit set for motor " + String(id) + ": " + String(vel));
}
uint32_t NMLHandExo::getVelocityLimit(uint8_t id) {
  return dxl_.readControlTableItem(PROFILE_VELOCITY, id);
}

// Acceleration commands
void NMLHandExo::setAccelerationLimit(uint8_t id, uint32_t acc) {
  dxl_.writeControlTableItem(PROFILE_ACCELERATION, id, acc);
  debugPrint("Acceleration limit set for motor " + String(id) + ": " + String(acc));
}
uint32_t NMLHandExo::getAccelerationLimit(uint8_t id) {
  return dxl_.readControlTableItem(PROFILE_ACCELERATION, id);
}

// Motor-specific commands
void NMLHandExo::rebootMotor(uint8_t id) {
  dxl_.reboot(id);
  debugPrint("Motor ID:" + String(id) + " rebooted");
}
void NMLHandExo::getMotorInfo(uint8_t id) {
  dxl_.ping(id);  // could be expanded to read Model Number, Version, etc.
  debugPrint("Pinged motor ID: " + String(id));
}
void NMLHandExo::setBaudRate(uint8_t id, uint32_t baudrate) {
  dxl_.writeControlTableItem(BAUD_RATE, id, baudrate);
  debugPrint("Motor ID:" + String(id) + " baudrate set to " + String(baudrate));
}
uint32_t NMLHandExo::getBaudRate(uint8_t id) {
  return dxl_.readControlTableItem(BAUD_RATE, id);
}
void NMLHandExo::setMotorLED(uint8_t id, bool state) {
  // Sets specified motor LED to the specified state
  if (state) {
    dxl_.ledOn(id);
  } else {
    dxl_.ledOff(id);
  }
}
void NMLHandExo::setAllMotorLED(bool state) {
  // Sets the state of all motor LEDs to the specified state
  for (int i = 0; i < numMotors_; i++) {
    uint8_t id = ids_[i];
    setMotorLED(id, state);
  }
}
