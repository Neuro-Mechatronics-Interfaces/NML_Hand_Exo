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
  #define COMMAND_SERIAL Serial2
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

// Define servo IDs
constexpr uint8_t PINKY_ID     = 10; // 2
constexpr uint8_t RING_ID      = 11; // 1
constexpr uint8_t INDEX_ID     = 13; // 3
constexpr uint8_t MIDDLE_ID    = 12; // 4
constexpr uint8_t THUMBFLEX_ID = 15; // 5
constexpr uint8_t THUMBROT_ID  = 14; // 5
constexpr uint8_t WRIST_ID     = 0;

/// @brief Hand orientation (right or left)
constexpr bool IS_RIGHT_HAND = true; // true for right hand, false for left hand

// ---- Motor enable flags ------------------------------------------------
// Set to 0 to exclude a motor that is not connected.
// Set to 1 to include it.  All arrays below are built automatically.
#define ENABLE_WRIST     0
#define ENABLE_THUMBROT  1
#define ENABLE_THUMBFLEX 1
#define ENABLE_INDEX     1
#define ENABLE_MIDDLE    1
#define ENABLE_RING      1
#define ENABLE_PINKY     1

/// @brief Motor ID Array (auto-built from enable flags)
constexpr uint8_t MOTOR_IDS[] = {
#if ENABLE_WRIST
  WRIST_ID,
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

/// @brief Default baud rate for the debug serial connection.
constexpr long DEBUG_BAUD_RATE = 57600;

/// @brief Default baud rates for BLE communication.
constexpr long COMMAND_BAUD_RATE = 57600;

/// @brief Default baud rate for Dynamixel communication.
constexpr long DYNAMIXEL_BAUD_RATE = 57600;

/// @brief Total number of gesture contained in the library
constexpr int N_GESTURES = 7;

/// @brief Maximum number of gesture buttons that can be configured
constexpr int MAX_GESTURE_BUTTONS = 6; // Maximum number of gesture buttons that can be configured

/// @brief Maximum number of states configurable per gesture
constexpr long MAX_STATES_PER_GESTURE = 5;

/// @brief Default current limit for Dynamixel servos.
constexpr int MOTOR_CURRENT_LIMIT = 200;

/// @brief Debounce duration for mode switch button in milliseconds.
constexpr int BUTTON_DEBOUNCE_DURATION = 50; // ms debounce for physical button

/// @brief DYNAMIXEL protocol version used.
constexpr float DXL_PROTOCOL_VERSION = 2.0;

/// @brief Ticks per revolution for the Dynamixel servos.
constexpr int PULSE_RESOLUTION = 4096;

/// @brief Torque constant for XL330 servos, in N*m/mA.
constexpr float XL330_TORQUE_CONSTANT = 0.00038; // Nm / mA

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

