/**
 * @file config.h
 * @brief Header file for custom definitions and configs for the NML Hand Exoskeleton project.
 *
 */
#pragma once
#include <Arduino.h>

// ========= Board specific configuration ===================
/// @brief Serial port for Dynamixel communication.
/// @brief Pin assignment for the Dynamixel direction control pin.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  //#include <SoftwareSerial.h>
  //SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL Serial1
  #define COMMAND_SERIAL Serial2
  //#define BLE_SERIAL soft_serial
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define COMMAND_SERIAL Serial1
  //#define DEBUG_SERIAL Serial1
  //#define DEBUG_SERIAL SerialUSB
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL   Serial1
  #define COMMAND_SERIAL Serial2
  //#define DEBUG_SERIAL SerialUSB
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DEBUG_SERIAL Serial
  #define COMMAND_SERIAL Serial2
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DEBUG_SERIAL Serial
  #define COMMAND_SERIAL Serial2
  #define DXL_SERIAL   Serial3
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL Serial1
  #define COMMAND_SERIAL Serial3  // D13(TX)/D14(RX) — HC-05 Bluetooth UART
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL   Serial1
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

// ======================= Define IMU Usage ==========================
#define IMU_ENABLED_DEFAULT true     // change to false if you usually run without IMU

#define IMU_I2C_ADDR_PRIMARY  0x28   // BNO055 default

#define IMU_I2C_ADDR_ALTERNATE 0x29  // ADR pin pulled high

// ======================================= PIN CONFIGURATION =============================================
// =======================================================================================================

// Pin definitions for mode switching and gesture control
//constexpr int CYCLE_GESTURE_PIN = 0;
constexpr int MODESWITCH_PIN = 1;
constexpr int GESTURE_PINCH_BUTTON_PIN = 2;
constexpr int GESTURE_KEYGRIP_BUTTON_PIN = 3;
constexpr int GESTURE_GRASP_BUTTON_PIN = 4;
constexpr int CYCLE_GESTURE_STATE_PIN = 5;

// I2C Pins (default)
//constexpr int SDL = 11;
//constexpr int SCA = 12;

// Serial port pins (also separated on board)
//constexpr int DYNAMIXEL_TX_PIN = 14; // TX pin for Dynamixel communication
//constexpr int DYNAMIXEL_RX_PIN = 13; // RX pin for Dynamixel communication

/// @brief Pin definition for the status LED (built-in LED on most boards)
//constexpr int STATUS_LED_PIN = LED_BUILTIN;
constexpr int STATUS_LED_PIN = 0;



// ======================================= USER CONFIGURATION ============================================
// =======================================================================================================

/// @brief Command delimiter for parsing commands from the serial input.
//constexpr char* COMMAND_DELIMITER = ";";
constexpr const char* COMMAND_DELIMITER = ";";

/// @brief Default exo mode on startup
constexpr const char* DEFAULT_EXO_MODE = "gesture_fixed"; // Available modes are "free", "gesture_fixed", and "gesture_continuous"

/// @brief Verbose output toggle for debugging.
constexpr bool DEFAULT_VERBOSE = true;

// ---- Hand-side build selector -----------------------------------------------
// 0 = right exo only  (IDs 11-19, HAND_SIDE="right")
// 1 = left exo only   (IDs  1-9,  HAND_SIDE="left")
// 2 = dual            (IDs  1-9 left + 11-19 right on one bus, HAND_SIDE="dual")
//
// Both hands share ONE OpenRB-150 controller and one Dynamixel bus.
// Left IDs: 1-9.  Right IDs: 11-19.
// Calibration arrays are overwritten at runtime by apply_calibration.
#define BUILD_LEFT_HAND 2

// ---- Motor IDs (single-exo modes only — dual defines arrays directly) --------
#if BUILD_LEFT_HAND == 0   // right only
  constexpr uint8_t WRIST_ID     = 11;
  constexpr uint8_t WRIST2_ID    = 12;
  constexpr uint8_t THUMBADD_ID  = 13;
  constexpr uint8_t THUMBROT_ID  = 14;
  constexpr uint8_t THUMBFLEX_ID = 15;
  constexpr uint8_t INDEX_ID     = 16;
  constexpr uint8_t MIDDLE_ID    = 17;
  constexpr uint8_t RING_ID      = 18;
  constexpr uint8_t PINKY_ID     = 19;
  constexpr const char* HAND_SIDE = "right";
#elif BUILD_LEFT_HAND == 1 // left only
  constexpr uint8_t WRIST_ID     =  1;
  constexpr uint8_t WRIST2_ID    =  2;
  constexpr uint8_t THUMBADD_ID  =  3;
  constexpr uint8_t THUMBROT_ID  =  4;
  constexpr uint8_t THUMBFLEX_ID =  5;
  constexpr uint8_t INDEX_ID     =  6;
  constexpr uint8_t MIDDLE_ID    =  7;
  constexpr uint8_t RING_ID      =  8;
  constexpr uint8_t PINKY_ID     =  9;
  constexpr const char* HAND_SIDE = "left";
#else                      // dual (BUILD_LEFT_HAND == 2)
  constexpr const char* HAND_SIDE = "dual";
#endif

// ---- Motor enable flags (single-exo modes only) ----------------------------
// Set to 0 to exclude a motor that is not connected.
// In dual mode (BUILD_LEFT_HAND == 2) all 18 motors are always included.
#if BUILD_LEFT_HAND != 2
#define ENABLE_WRIST     1
#define ENABLE_WRIST2    1
#define ENABLE_THUMBADD  1
#define ENABLE_THUMBROT  1
#define ENABLE_THUMBFLEX 1
#define ENABLE_INDEX     1
#define ENABLE_MIDDLE    1
#define ENABLE_RING      1
#define ENABLE_PINKY     1
#endif

// ---- Motor arrays -----------------------------------------------------------
// Dual mode uses fixed 18-entry arrays (left IDs 1-9 first, right IDs 11-19 second).
// Single-exo modes use the ENABLE_* flags to build 9-entry arrays.

#if BUILD_LEFT_HAND == 2   // ===== DUAL MODE =====

/// @brief Motor ID Array: left (1-9) then right (11-19).
constexpr uint8_t MOTOR_IDS[] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9,          // left:  wrist wrist2 thumbadd thumbrot thumbflex index middle ring pinky
  11, 12, 13, 14, 15, 16, 17, 18, 19  // right: wrist wrist2 thumbadd thumbrot thumbflex index middle ring pinky
};

/// @brief Motor name Array (must match MOTOR_IDS order).
/// Individual motor commands use Dynamixel ID numbers, not names, to avoid
/// duplicate-name collisions between left and right in dual mode.
constexpr const char* MOTOR_NAMES[] = {
  "wrist", "wrist2", "thumbadd", "thumbrot", "thumbflex", "index", "middle", "ring", "pinky",
  "wrist", "wrist2", "thumbadd", "thumbrot", "thumbflex", "index", "middle", "ring", "pinky"
};

/// @brief Home states [left placeholders, right calibrated].
/// Left values are placeholders overwritten at runtime by apply_calibration.
constexpr float HOME_STATES[] = {
  // left (IDs 1-9) — placeholders, calibrate before use
  180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0,
  // right (IDs 11-19) — wrist, wrist2, thumbadd, thumbrot, thumbflex, index, middle, ring, pinky
  149.1, 180.0, 180.0, 251.86, 374.53, 162.8, 106.83, 68.99, 115.37
};

/// @brief Physical joint limits [min, max] for each motor.
constexpr float jointLimits[][2] = {
  // left (IDs 1-9) — placeholders
  {0.0, 360.0}, {0.0, 360.0}, {0.0, 360.0}, {0.0, 360.0}, {0.0, 360.0},
  {0.0, 360.0}, {0.0, 360.0}, {0.0, 360.0}, {0.0, 360.0},
  // right (IDs 11-19) — wrist, wrist2, thumbadd, thumbrot, thumbflex, index, middle, ring, pinky
  {320, 166}, {42.0, 190.0}, {140.0, 260.0}, {160.26, 260.86}, {88.53, 174.27},
  {166.8, 239.93}, {50.5, 104.83}, {66.99, 125.06}, {400.1, 460.37}
};

/// @brief Default flip direction per motor.
constexpr bool DEFAULT_FLIPS[] = {
  // left (IDs 1-9) — placeholders
  false, false, false, false, false, false, false, false, false,
  // right (IDs 11-19) — wrist, wrist2, thumbadd, thumbrot, thumbflex, index, middle, ring, pinky
  false, false, false, true, false, false, true, false, true
};

#else  // ===== SINGLE-EXO MODE (BUILD_LEFT_HAND == 0 or 1) =====

/// @brief Motor ID Array (auto-built from enable flags)
constexpr uint8_t MOTOR_IDS[] = {
#if ENABLE_WRIST
  WRIST_ID,
#endif
#if ENABLE_WRIST2
  WRIST2_ID,
#endif
#if ENABLE_THUMBADD
  THUMBADD_ID,
#endif
#if ENABLE_THUMBROT
  THUMBROT_ID,
#endif
#if ENABLE_THUMBFLEX
  THUMBFLEX_ID,
#endif
#if ENABLE_INDEX
  INDEX_ID,
#endif
#if ENABLE_MIDDLE
  MIDDLE_ID,
#endif
#if ENABLE_RING
  RING_ID,
#endif
#if ENABLE_PINKY
  PINKY_ID,
#endif
};

/// @brief Motor name Array (must match MOTOR_IDS order)
constexpr const char* MOTOR_NAMES[] = {
#if ENABLE_WRIST
  "wrist",
#endif
#if ENABLE_WRIST2
  "wrist2",
#endif
#if ENABLE_THUMBADD
  "thumbadd",
#endif
#if ENABLE_THUMBROT
  "thumbrot",
#endif
#if ENABLE_THUMBFLEX
  "thumbflex",
#endif
#if ENABLE_INDEX
  "index",
#endif
#if ENABLE_MIDDLE
  "middle",
#endif
#if ENABLE_RING
  "ring",
#endif
#if ENABLE_PINKY
  "pinky",
#endif
};

/// @brief Home states for each motor in degrees (found experimentally).
constexpr float HOME_STATES[] = {
#if ENABLE_WRIST
  149.1,
#endif
#if ENABLE_WRIST2
  180.0,           // placeholder -- needs real calibration
#endif
#if ENABLE_THUMBADD
  180.0,           // placeholder -- needs real calibration
#endif
#if ENABLE_THUMBROT
  251.86,
#endif
#if ENABLE_THUMBFLEX
  374.53,
#endif
#if ENABLE_INDEX
  162.8,
#endif
#if ENABLE_MIDDLE
  106.83,
#endif
#if ENABLE_RING
  68.99,
#endif
#if ENABLE_PINKY
  115.37,
#endif
};

/// @brief Physical joint limits [min, max] for each motor (found experimentally).
constexpr float jointLimits[][2] = {
#if ENABLE_WRIST
  {-189, 2840},
#endif
#if ENABLE_WRIST2
  {0.0, 360.0},   // placeholder -- needs real calibration
#endif
#if ENABLE_THUMBADD
  {0.0, 360.0},   // placeholder -- needs real calibration
#endif
#if ENABLE_THUMBROT
  {220.26, 251.86},
#endif
#if ENABLE_THUMBFLEX
  {374.53, 415.27},
#endif
#if ENABLE_INDEX
  {162.8, 224.93},
#endif
#if ENABLE_MIDDLE
  {64.5, 106.83},
#endif
#if ENABLE_RING
  {68.99, 119.06},
#endif
#if ENABLE_PINKY
  {74.1, 115.37},
#endif
};

/// @brief Default flip direction per motor (overwritten at runtime by calibration).
constexpr bool DEFAULT_FLIPS[] = {
#if ENABLE_WRIST
  false,
#endif
#if ENABLE_WRIST2
  false,           // placeholder -- needs real calibration
#endif
#if ENABLE_THUMBADD
  false,           // placeholder -- needs real calibration
#endif
#if ENABLE_THUMBROT
  true,
#endif
#if ENABLE_THUMBFLEX
  false,
#endif
#if ENABLE_INDEX
  false,
#endif
#if ENABLE_MIDDLE
  true,
#endif
#if ENABLE_RING
  false,
#endif
#if ENABLE_PINKY
  true,
#endif
};

#endif  // BUILD_LEFT_HAND == 2 vs single-exo

/// @brief Default baud rate for the debug serial connection.
constexpr long DEBUG_BAUD_RATE = 1000000;

/// @brief Default baud rates for BLE communication.
constexpr long COMMAND_BAUD_RATE = 115200;

/// @brief Default baud rate for Dynamixel communication.
constexpr long DYNAMIXEL_BAUD_RATE = 1000000;

/// @brief Total number of gesture contained in the library
constexpr int N_GESTURES = 6;

/// @brief Maximum number of gesture buttons that can be configured
constexpr int MAX_GESTURE_BUTTONS = 6; // Maximum number of gesture buttons that can be configured

/// @brief Maximum number of states configurable per gesture
constexpr long MAX_STATES_PER_GESTURE = 5;

/// @brief Current limit for XC330-T288 motors.
///
/// ROBOTIS documents Current Limit(38) for the XC330-T288 as 0-910 units,
/// about 1 mA per unit. This project uses the full documented range because
/// participant finger spasticity can require higher assistive torque.
constexpr int MOTOR_CURRENT_LIMIT = 910;

/// @brief Direct-control limits used by serial velocity/current commands.
constexpr float DIRECT_VELOCITY_LIMIT_RPM = 10.0f;
constexpr int DIRECT_CURRENT_LIMIT_MA = MOTOR_CURRENT_LIMIT;
constexpr unsigned long DIRECT_COMMAND_TIMEOUT_MS = 250;
constexpr float DIRECT_LIMIT_MARGIN_DEG = 2.0f;

/// @brief Debounce duration for mode switch button in milliseconds.
constexpr int BUTTON_DEBOUNCE_DURATION = 50; // ms debounce for physical button

/// @brief DYNAMIXEL protocol version used.
constexpr float DXL_PROTOCOL_VERSION = 2.0;

/// @brief Ticks per revolution for the Dynamixel servos.
constexpr int PULSE_RESOLUTION = 4096;

/// @brief Estimated torque constant for XC330-T288 servos, in N*m/mA.
///
/// Based on ROBOTIS stall torque/current at the recommended 11.1 V supply:
/// 0.92 N*m / 0.80 A = 0.00115 N*m/mA. XC330 current telemetry is measured at
/// the input power source, so displayed torque should be treated as an estimate.
constexpr float XC330_T288_TORQUE_CONSTANT = 0.00115f; // Nm / mA

// =======================================================================================================
// =======================================================================================================


/// @brief Number of motors in the system.
constexpr int N_MOTORS = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);

// Direction control pin - define per board type
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
constexpr int DXL_DIR_PIN = 2;
#define DEBUG_SERIAL Serial

#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
constexpr int DXL_DIR_PIN = 2;
#elif defined(ARDUINO_OpenCM904)
constexpr int DXL_DIR_PIN = 22;
#elif defined(ARDUINO_OpenCR)
constexpr int DXL_DIR_PIN = 84;
#elif defined(ARDUINO_OpenRB)
constexpr int DXL_DIR_PIN = -1;
#else
constexpr int DXL_DIR_PIN = 2;
#endif

// ===== OLED (SSD1306) =====
/// @brief Enable by default
#define OLED_ENABLED_DEFAULT   true     // set false if most runs are headless

/// @brief I2C primary adress
#define OLED_I2C_ADDR_PRIMARY  0x3C     // 128x32 default

/// @brief Alternative I2C address
#define OLED_I2C_ADDR_ALT      0x3D     // some 128x64 boards use 0x3D

/// @brief OLED screen width
#define OLED_SCREEN_WIDTH      128

/// @brief OLED screen height
#define OLED_SCREEN_HEIGHT     32       // change to 64 if you have a 128x64

/// @brief OLED center text helper
#define OLED_CENTER_TEXT       1        // center text helper

/// @brief OLED updated/refresh period
#define OLED_UPDATE_PERIOD_MS  50       // rate-limit screen refresh (reduce I2C)
