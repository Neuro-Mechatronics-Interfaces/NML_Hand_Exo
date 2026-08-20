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

constexpr float HOME_STATES[] = {
  // left (IDs 1-9) — placeholders, calibrate before use
  228.45, 132.70, 220.79, 232.23, 122.76, 193.34, 80.26, 98.21, 63.80,
  // right (IDs 11-19) — wrist, wrist2, thumbadd, thumbrot, thumbflex, index, middle, ring, pinky
  228.45, 132.70, 220.79, 232.23, 122.76, 193.34, 80.26, 98.21, 63.80
};

/// @brief Physical joint limits [min, max] for each motor.
constexpr float jointLimits[][2] = {
  // left (IDs 1-9) — placeholders
  {38.00, 194.00}, {167.00, 310.00}, {175.00, 285.00}, {169.00, 269.00}, {290.00, 367.00},
  {107.00, 184.00}, {164.00, 223.00}, {136.00, 204.00}, {65.00, 155.00},
  // right (IDs 11-19) — wrist, wrist2, thumbadd, thumbrot, thumbflex, index, middle, ring, pinky
  {201.17, 269.46}, {90.02, 163.42}, {198.09, 227.13}, {160.26, 260.86}, {106.74, 147.31},
  {162.27, 231.35}, {49.54, 102.52}, {74.71, 127.78}, {14.96, 109.65}
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
/// 6 postures (grasp, keygrip, pinch_index, pinch_middle, pinch_ring, peace),
/// one extend/rest/flex gesture for each digit and the wrist, plus three
/// research-facing gestures for the individual thumb motors.
constexpr int N_GESTURES = 15;

// ---- Per-joint gesture travel ----------------------------------------------
// Each value is a fraction of that joint's TRAVEL -- the signed distance from
// home to the flexion end of its calibrated window:
//
//     target = home + fraction * gesture_span(motor)
//
// gesture_span() is resolved per motor by NMLHandExo::getGestureSpan() from
// home and [limit_min, limit_max], so 0.0 sits exactly at home and 1.0 sits
// exactly ON the flexion endstop. Nothing in between ever clamps, and the
// mapping is invertible -- which is what makes `get_gesture_angle` able to
// report the same 0-100 number back.
//
// This is NOT a fraction of the window WIDTH (limit_max - limit_min), which is
// what these constants used to mean. The old form silently assumed home sat at
// one edge of the window; when it did not, every state clamped to the same
// boundary and the joint had zero visible travel while still acking OK. The
// wrist and wrist2 axes are exactly where that assumption broke.
//
// The thumb gets three independent knobs per state because its joints do NOT
// share a flip direction (DEFAULT_FLIPS has thumbrot true, thumbadd/thumbflex
// false), so a single shared value drives them physically opposite ways.
//
// Every joint gets THREE knobs, one per gesture state:
//
//   EXTEND_*  -> the extension posture, and the 0% end of the percent axis
//   REST_*    -> intermediate posture
//   FLEX_*    -> the flexion posture, and the 100% end of the percent axis
//
// EXTEND_* and FLEX_* do double duty: they are the two states, AND they are the
// endpoints `set_gesture_angle` interpolates between. So
//
//   set_gesture_angle:<g>:0    == set_gesture:<g>:extend
//   set_gesture_angle:<g>:100  == set_gesture:<g>:flex
//   set_gesture_angle:<g>:50   == halfway between the two postures
//
// and `get_gesture_angle` inverts it. Retuning EXTEND_*/FLEX_* therefore moves
// the percent axis with them -- which is the point: a host's percentages stay
// meaningful across a retune instead of pointing at raw travel that no longer
// corresponds to a posture. REST_* is NOT a percent-axis endpoint; it just
// lands wherever its value falls on that axis (`get_gesture_angle` reports
// where). Since 0.6.1 it IS the zero-degree anchor for `get_gesture_sang` and
// the signed-degree field in `get_gesture_angles`.
//
// EXTEND_* need not be 0.0. It was pinned there while the axis ran from home,
// because home IS the extension endstop as calibration defines it (the median
// of the EXTENSION samples, see CalibrationDialog._save_profile) and anything
// below clamped. Now that 0% follows EXTEND_* rather than home, a non-zero
// value is a first-class extension posture that sits slightly off the endstop.
// A hand at home then reads BELOW 0% -- `get_gesture_angle` reports 101 for it,
// which is correct, not a fault.
//
// If a joint moves the WRONG WAY, negate its constants here (or fix that
// motor's DEFAULT_FLIPS entry, which affects every gesture). If a joint moves
// TOO FAR, shrink the magnitude. Keep EXTEND_* != FLEX_*, or that joint has no
// axis to interpolate along and stops reporting a position.
//
// NOTE: a joint whose calibrated window has (almost) no room on either side of
// home has ZERO travel no matter what is set here. Run `check_limits`, which
// prints the resolved span per motor, to find those before tuning.

constexpr float EXTEND_THUMBADD  =  0.10f;
constexpr float REST_THUMBADD    =  0.50f;
constexpr float FLEX_THUMBADD    =  0.90f;
constexpr float EXTEND_THUMBROT  =  0.10f;
constexpr float REST_THUMBROT    =  0.50f;
constexpr float FLEX_THUMBROT    =  0.90f;
constexpr float EXTEND_THUMBFLEX =  0.10f;
constexpr float REST_THUMBFLEX   =  0.50f;
constexpr float FLEX_THUMBFLEX   =  0.90f;

constexpr float EXTEND_INDEX     =  0.10f;
constexpr float REST_INDEX       =  0.25f;
constexpr float FLEX_INDEX       =  0.50f;
constexpr float EXTEND_MIDDLE    =  0.10f;
constexpr float REST_MIDDLE      =  0.25f;
constexpr float FLEX_MIDDLE      =  0.50f;
constexpr float EXTEND_RING      =  0.10f;
constexpr float REST_RING        =  0.25f;
constexpr float FLEX_RING        =  0.50f;
constexpr float EXTEND_PINKY     =  0.10f;
constexpr float REST_PINKY       =  0.25f;
constexpr float FLEX_PINKY       =  0.50f;

// Wrist flexion/extension. Both `wrist` and `wrist2` are mounted on the back of
// the arm and pull on the dorsal aspect of the wrist, so they act in concert and
// share these constants -- commanding one alone leaves the other holding
// position against it. Wrist ROM windows are much larger in absolute degrees
// than a digit's, so the same fraction buys far more travel.
constexpr float EXTEND_WRIST     =  0.05f;
constexpr float REST_WRIST       =  0.25f;
constexpr float FLEX_WRIST       =  0.95f;

// ---- Gesture fractional axis ------------------------------------------------
// Knobs for the home -> flexion-endstop axis every gesture percentage rides on
// (NMLHandExo::getGestureSpan / gestureFractionToAngle / gestureAngleToFraction).

/// @brief Least travel, in degrees, that counts as a usable gesture range.
///
/// Below this a joint is reported as having no travel rather than being driven
/// a fraction of nothing: `get_gesture_angle` returns 255 for it, signed-angle
/// queries return nan, and `check_limits` flags it NO_TRAVEL. Sized just above
/// position-read noise.
constexpr float GESTURE_MIN_TRAVEL_DEG = 2.0f;

/// @brief How much more travel the non-flip side must offer to be chosen.
///
/// The flip flag is the calibrated flexion DIRECTION, and normally home sits on
/// the extension endstop so all the travel is on the flip-indicated side. When
/// home instead lands mid-window -- which is how the stock wrist (home 310 in a
/// [166, 320] window) and wrist2 configurations are shaped -- the flip side is a
/// stub a few degrees wide and every gesture state clamps onto the same
/// boundary. Past this ratio the opposite side is taken as the real flexion
/// direction, since a window that lopsided cannot mean anything else. Kept
/// deliberately high so a joint whose two sides are comparable (a home that is
/// genuinely mid-range) keeps the direction calibration recorded for it.
constexpr float GESTURE_SPAN_OVERRIDE_RATIO = 2.0f;

/// @brief Slack, as a fraction of the axis, before a joint reads out of range.
///
/// Gearing backlash and position quantization put a settled joint a hair past
/// its endpoint routinely; without this, a hand parked at 100% would report the
/// out-of-range code about half the time.
constexpr float GESTURE_FRACTION_TOLERANCE = 0.02f;

/// @brief Least EXTEND_* to FLEX_* separation for a joint to carry a position.
///
/// A joint whose two endpoints coincide holds still across the whole percent
/// axis, so where it sits says nothing about the percentage. Such joints are
/// still commanded -- they hold their posture -- but are left out of the mean
/// `get_gesture_angle` reports, and out of the division that would otherwise
/// blow up on their zero-width segment.
constexpr float GESTURE_AXIS_MIN_SEPARATION = 0.02f;

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

// ---- Combined-motor current budget -----------------------------------------
// MOTOR_CURRENT_LIMIT and DEFAULT_GOAL_CURRENT_MA above are PER-MOTOR knobs.
// Neither bounds what the fleet draws together, which is the quantity the power
// supply actually cares about: a posture that commands every joint at once --
// `grasp:open` drives all of them to home, which calibration places AT the
// extension endstop -- can leave many motors stalled and each drawing its full
// goal current, browning out the board.
//
// The budget below caps the COMBINED draw. Enforcement (see
// serviceCurrentGovernor in nml_hand_exo.cpp) is two-sided:
//
//   feed-forward  Issuing a goal immediately clamps the fleet to the static
//                 worst case, budget / sum(nominal of moving motors), so the
//                 very first transient is bounded before anything is measured.
//   feedback      Measured aggregate PRESENT_CURRENT then relaxes that clamp
//                 back toward full per-motor effort whenever the real draw
//                 leaves headroom -- so one working motor keeps full torque and
//                 the clamp only bites when the fleet genuinely nears the cap.
//
// Sized for the supply, NOT for the motors. Default assumes roughly 1 A usable,
// leaving headroom for the board, OLED and IMU. Raise it with
// `set_total_current_lim:<mA>` once you know what your supply sustains; watch
// `current_status` for the measured aggregate while you do.
constexpr int TOTAL_CURRENT_BUDGET_MA = 800;

/// @brief Current a motor is allowed once it is judged to have settled, in mA.
///
/// A motor that reached its target draws near zero regardless of this value;
/// the point is to cap what a STALLED motor can keep pulling. The worst case
/// must fit the budget on its own, so keep N_MOTORS * HOLD_CURRENT_MA below
/// TOTAL_CURRENT_BUDGET_MA (18 * 25 = 450 mA against a 800 mA default).
constexpr int HOLD_CURRENT_MA = 25;

/// @brief Minimum gap between the governor's individual motor reads, in ms.
///
/// The governor reads ONE motor per update() call, not the whole fleet at once.
/// Reading all 18 in a burst would block the control loop for milliseconds and
/// put that jitter straight onto command latency, which the dual-CDC work
/// exists to avoid. One read is roughly 350 us, so a full sweep of 18 motors
/// completes in about 18 * this value and no single loop pass pays for more
/// than one read. Sampling only runs while motors are active, so an idle exo
/// adds no Dynamixel bus traffic at all.
constexpr unsigned long CURRENT_GOVERNOR_SAMPLE_INTERVAL_MS = 2;

/// @brief How long after the last goal the governor keeps sampling, in ms.
constexpr unsigned long CURRENT_GOVERNOR_ACTIVE_MS = 2000;

/// @brief Consecutive read failures before the governor stops trusting
/// measurements and falls back to the static worst-case clamp.
constexpr uint8_t CURRENT_GOVERNOR_MAX_READ_FAILS = 3;

/// @brief Fraction of the budget below which the governor relaxes its clamp.
/// The gap between this and 1.0 is deadband; without it the clamp oscillates.
constexpr float CURRENT_GOVERNOR_RELEASE_FRACTION = 0.80f;

/// @brief How fast the clamp relaxes per completed sweep once there is headroom.
/// Recovery is deliberately slower than the clamp-down, which is immediate.
constexpr float CURRENT_GOVERNOR_RECOVERY_STEP = 0.08f;

/// @brief Minimum change in the allocation scale worth re-writing GOAL_CURRENT for.
///
/// Each refresh is up to N_MOTORS register writes, so pushing every marginal
/// change would cost more bus time than the sampling does. The deadband keeps
/// the writes proportionate to the change that prompted them.
constexpr float CURRENT_GOVERNOR_APPLY_DEADBAND = 0.03f;

/// @brief A motor drawing at least this fraction of its allowance is pushing
/// against something rather than travelling.
constexpr float STALL_CURRENT_FRACTION = 0.80f;

/// @brief Least current at which a joint can actually break friction and move.
///
/// The budget must never allocate a motor LESS than this while expecting it to
/// travel. Spreading the budget evenly over every commanded motor does exactly
/// that: 18 motors sharing 800 mA get 44 mA each, too little to move, so they
/// all strain at their full allowance and the fleet parks just under the cap --
/// no overshoot to clamp, no headroom to relax, nothing moving, every command
/// still acking. Admission (see refreshCurrentAllocation) exists to prevent it:
/// fewer motors move at a usable current instead of all of them uselessly.
constexpr uint16_t MIN_MOVE_CURRENT_MA = 75;

/// @brief Give up on a move this long after it was funded, in ms.
///
/// Bounds how long a motor may hold an admission slot, so one joint jammed
/// against an obstruction cannot stop the rest of a gesture from ever being
/// dispatched. Also what turns "no movement" into a reported verdict.
constexpr unsigned long MOVE_TIMEOUT_MS = 2500;

/// @brief Position error within which a move counts as having reached its goal.
constexpr float GESTURE_REACH_TOLERANCE_DEG = 4.0f;

/// @brief Draw below which a motor counts as having arrived, in mA.
///
/// This is how "arrived" is told apart from "stalled" without position reads: a
/// motor that reached its target coasts at nearly nothing, while one pushing on
/// an endstop or a spastic finger keeps pulling. The distinction matters --
/// arrived motors are released from the budget, but a motor doing real work is
/// left at full effort unless the fleet is genuinely over budget.
constexpr uint16_t CURRENT_SETTLED_MA = 25;

/// @brief How long a motor is presumed to still be travelling, in ms.
///
/// Only after this has elapsed is a low draw read as "arrived" rather than
/// "has not started pulling yet". Long enough for a full-range digit move.
constexpr unsigned long MOVE_WINDOW_MS = 500;

/// @brief How long a motor must draw stall-level current before it is demoted
/// to HOLD_CURRENT_MA, in ms. This is what stops a fleet of motors parked on
/// their endstops from holding at full effort indefinitely.
constexpr unsigned long STALL_HOLD_MS = 250;

/// @brief Direct-control limits used by serial velocity/current commands.
constexpr float DIRECT_VELOCITY_LIMIT_RPM = 50.0f;
/// @brief XC330 VELOCITY_LIMIT register value nearest 50 rpm (0.229 rpm/unit).
constexpr uint32_t DIRECT_VELOCITY_LIMIT_RAW = 218;
constexpr int DIRECT_CURRENT_LIMIT_MA = MOTOR_CURRENT_LIMIT;
constexpr unsigned long DIRECT_COMMAND_TIMEOUT_MS = 250;
/// @brief Distance from a joint limit where direct velocity begins tapering.
constexpr float DIRECT_VELOCITY_SOFT_ZONE_DEG = 10.0f;
/// @brief Hard stop distance retained inside each calibrated joint limit.
constexpr float DIRECT_LIMIT_MARGIN_DEG = 2.0f;

/// @brief Phase-1, read-only contact instrumentation.
///
/// The sampler is disabled at boot and can run only while the global motor
/// mode is VELOCITY. It performs one Dynamixel register read per service pass,
/// alternating PRESENT_CURRENT and PRESENT_POSITION over explicit IDs. It
/// never writes a motor register or changes a control decision.
constexpr uint8_t SHADOW_TELEMETRY_MAX_MOTORS = 9;
constexpr unsigned long SHADOW_TELEMETRY_DEFAULT_INTERVAL_MS = 2;
constexpr unsigned long SHADOW_TELEMETRY_MIN_INTERVAL_MS = 2;
constexpr unsigned long SHADOW_TELEMETRY_MAX_INTERVAL_MS = 100;

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
