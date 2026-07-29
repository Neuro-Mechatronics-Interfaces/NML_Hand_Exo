/**
 * @file config.h
 * @brief Header file for custom definitions and configs for the NML Hand Exoskeleton project.
 *
 */
#pragma once
#include <Arduino.h>
#define BT_SKIP

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

// ================= Dual USB-CDC transport (OpenRB-150) =================
// Splits the single native USB CDC into TWO ACM interfaces on one cable so
// host commands and telemetry no longer share one channel (head-of-line
// blocking fix at the device side):
//   CMD_SERIAL   -> host COMMAND input   (primary CDC, USB interface 0 / MI_00)
//   TELEM_SERIAL -> replies + telemetry  (second  CDC, USB interface 2 / MI_02)
//
// The second CDC is a plain second instance of the core's PluggableUSB
// `Serial_` class (defined in the .ino). The core already advertises the
// IAD composite device class (0xEF/0x02/0x01) and each CDC emits its own IAD,
// so Windows binds BOTH interfaces to usbser.sys as two COM ports with no
// custom .inf. Endpoint budget: EP0 (control) + 3 (CDC1) + 3 (CDC2) = 7 =
// the core's USB_ENDPOINTS cap, so exactly two CDCs fit (no room for a 3rd
// USB function).
//
// Enabled by default on OpenRB-150. Define SINGLE_CDC before building to
// force legacy one-port behavior (bench debugging over a single COM port).
// The Bluetooth COMMAND_SERIAL (Serial3 / HC-05) path is unchanged either way.
#if defined(ARDUINO_OpenRB) && !defined(SINGLE_CDC)
  #define DUAL_CDC 1
#endif

#if defined(DUAL_CDC) && DUAL_CDC
  // Second CDC instance lives in the .ino; declare it for every translation unit.
  extern Serial_ SerialTelem;
  #ifdef DUAL_CDC_SWAP
    // Fallback: if USB enumeration ever assigns SerialTelem to interface 0
    // (static-init order surprise), define DUAL_CDC_SWAP to swap the roles so
    // the lower COM port (MI_00) still carries commands. One-line mitigation,
    // no protocol change.
    #define CMD_SERIAL   SerialTelem
    #define TELEM_SERIAL Serial
  #else
    #define CMD_SERIAL   Serial
    #define TELEM_SERIAL SerialTelem
  #endif
#else
  // Single-CDC fallback: commands and replies share DEBUG_SERIAL (legacy behavior).
  #define CMD_SERIAL   DEBUG_SERIAL
  #define TELEM_SERIAL DEBUG_SERIAL
#endif

// ---- Reply routing (dual-CDC, runtime-switchable) ---------------------------
// Controls which USB CDC(s) command replies / telemetry are emitted on. The
// default is BOTH so a LEGACY single-port host still gets request->response on
// whichever CDC it opened (backward compatible). A dual-port host can send
// `set_reply_route:telem` to make the command CDC input-only and fully decouple
// command writes from telemetry reads. In single-CDC builds this is a no-op
// (both routes resolve to the one port). See gReplyRoute in nml_hand_exo.cpp.
#define REPLY_ROUTE_BOTH   0   // command CDC + telemetry CDC (legacy-safe default)
#define REPLY_ROUTE_TELEM  1   // telemetry CDC only (command CDC becomes input-only)
#define REPLY_ROUTE_CMD    2   // command CDC only

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

// ---- OLD -----
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
// --- END OLD ---

// // ---- HOME_STATES (paste into config.h, MOTOR_IDS order) ----
// constexpr float HOME_STATES[] = {
//   206.62, 152.24, 218.59, 229.50, 129.45, 189.82, 80.0, 80.00, 65.0, 206.62, 152.24, 218.59, 229.50, 129.45, 189.82, 80.0, 80.00, 65.0
// };

// constexpr float jointLimits[][2] = {
//   {198.79, 209.62},
//   {145.0, 164.56},
//   {160.25, 260.0},
//   {200.0, 350.0},
//   {97.24, 182.34},
//   {100.0, 220.0},
//   {70.0, 92.5},
//   {70.0, 92.5},
//   {12.5, 72.5},
//   {198.79, 209.62},
//   {145.0, 164.56},
//   {160.25, 260.0},
//   {200.0, 350.0},
//   {97.24, 182.34},
//   {100.0, 220.0},
//   {70.0, 92.5},
//   {70.0, 92.5},
//   {12.5, 72.5}
// };

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

/// @brief Default baud rate for the USB debug/command serial connection.
constexpr long DEBUG_BAUD_RATE = 1000000;

/// @brief Default baud rates for BLE communication.
constexpr long COMMAND_BAUD_RATE = 115200;

/// @brief Default baud rate for Dynamixel communication.
///
/// Keep this matched to DEBUG_BAUD_RATE so the GUI/OpenRB link and OpenRB/DXL
/// bus use the same default rate. Live diagnostics on the full exo chain showed
/// intermittent CRC/timeout/overflow errors at 2 Mbps even for single-motor
/// position reads. 1 Mbps was stable in repeated tests.
constexpr long DYNAMIXEL_BAUD_RATE = 1000000;

/// @brief Total number of gesture contained in the library
/// 6 postures (grasp, keygrip, pinch_index, pinch_middle, pinch_ring, peace)
/// plus one per-digit flex/extend gesture for each of the 5 digits.
constexpr int N_GESTURES = 11;

// ---- Per-digit gesture travel ----------------------------------------------
// Each value is a fraction of that MOTOR's calibrated range (max - min), added
// to its home position:
//
//     target = home +/- fraction * (limit_max - limit_min)
//
// The sign is then flipped for any motor whose DEFAULT_FLIPS entry is true, so
// "positive = curls inward" only holds if that motor's flip flag is correct.
//
//   0.0      -> sit exactly at home
//   positive -> travel one way, negative -> travel the other
//
// If a digit moves the WRONG WAY, negate its constant here (or fix that
// motor's DEFAULT_FLIPS entry, which affects every gesture). If a digit moves
// TOO FAR, shrink the magnitude.
//
// The thumb gets three independent knobs because its joints do NOT share a
// flip direction (DEFAULT_FLIPS has thumbrot true, thumbadd/thumbflex false),
// so a single shared value drives them physically opposite ways.
//
// NOTE: a joint whose home lies outside its calibrated limits has ZERO travel
// no matter what is set here -- setAbsoluteAngle() clamps it. Run `check_limits`
// to find those before tuning.

constexpr float FLEX_THUMBADD    =  0.15f;
constexpr float EXTEND_THUMBADD  =  0.15f;
constexpr float FLEX_THUMBROT    =  0.15f;
constexpr float EXTEND_THUMBROT  =  0.15f;
constexpr float FLEX_THUMBFLEX   =  0.15f;
constexpr float EXTEND_THUMBFLEX =  0.15f;

constexpr float FLEX_INDEX       =  0.35f;
constexpr float EXTEND_INDEX     =  -0.15f;
constexpr float FLEX_MIDDLE      =  0.40f;
constexpr float EXTEND_MIDDLE    =  -0.15f;
constexpr float FLEX_RING        =  0.50f;
constexpr float EXTEND_RING      =  -0.15f;
constexpr float FLEX_PINKY       =  0.25f;
constexpr float EXTEND_PINKY     =  -0.15f;

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

/// @brief Working effort commanded in current-based position mode, in mA.
///
/// This is GOAL_CURRENT -- the current a motor actually applies while holding
/// or chasing its goal -- as distinct from MOTOR_CURRENT_LIMIT above, which is
/// only the ceiling. They are NOT the same knob: initializing GOAL_CURRENT to
/// the ceiling makes every motor push at the part maximum, so several digits
/// held against resistance at once can pull amps and brown out the supply.
///
/// Keep this at the lowest value that still moves the digit. The ceiling stays
/// high so `set_current:<motor>:<mA>` can raise effort at runtime for
/// participants whose spasticity needs it.
constexpr int DEFAULT_GOAL_CURRENT_MA = 150;

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

