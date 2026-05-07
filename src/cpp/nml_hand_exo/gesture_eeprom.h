/**
 * @file gesture_eeprom.h
 * @brief EEPROM persistence for gestureLibrary [0-1] fractions and calibration name.
 *
 * Only the named-pair float values in gestureLibrary and the active calibration
 * profile name are stored here.  Motor home/limits/flip values are NOT stored —
 * those are pushed from Python at runtime by apply_calibration().
 *
 * Requires FlashStorage_SAMD (Arduino Library Manager) for SAMD21 (OpenRB-150).
 */
#pragma once
#include <Arduino.h>
#include "gesture_library.h"

// ---- Layout constants -------------------------------------------------------
//
// GESTURE_EEPROM_MAGIC encodes the schema version in its lower 16 bits.
// The upper 16 bits (0xCA13) are a fixed namespace marker.
//
// BUMP THE MAGIC whenever any of the following change, or a previously saved
// EEPROM will silently restore values into the wrong gestures/joints/sides:
//   • N_GESTURES or the ORDER of gestures in gestureLibrary[]
//   • EEPROM_N_STATES or the ORDER of states (open=0, close=1)
//   • EEPROM_N_JOINTS or the ORDER of namedPairs within any gesture state
//   • EEPROM_N_SIDES (i.e. N_HAND_SIDES changed)
//
// Schema history:
//   0xCA130001  — original, pre-versioning (not explicitly tied to an order)
//   0xCA130002  — v2: 6 gestures × 2 states × 9 joints (single side)
//                  gestures : grasp(0), keygrip(1), pinch_index(2),
//                             pinch_middle(3), pinch_ring(4), peace(5)
//                  states   : open(0), close(1)
//                  joints   : wrist(0), wrist2(1), thumbadd(2), thumbflex(3),
//                             thumbrot(4), index(5), middle(6), ring(7), pinky(8)
//   0xCA130003  — v3: adds side dimension → values[gesture][state][side][joint]
//                  sides    : left(0), right(1)  — EEPROM_N_SIDES = N_HAND_SIDES
// See docs/eeprom_schema.md for full layout details.

constexpr uint32_t GESTURE_EEPROM_MAGIC = 0xCA130003u;
constexpr uint8_t  CAL_NAME_LEN         = 16;   // 15 usable chars + null
constexpr uint8_t  EEPROM_N_STATES      = 2;    // open=0, close=1
constexpr uint8_t  EEPROM_N_JOINTS      = 9;    // see joint order above
constexpr uint8_t  EEPROM_N_SIDES       = N_HAND_SIDES;  // 1 (single-exo) or 2 (dual)
constexpr int      GESTURE_CAL_EEPROM_ADDR = 0;

// ---- Stored struct ----------------------------------------------------------
// Total size (dual):   4 + 16 + 6*2*2*9*4 = 884 bytes
// Total size (single): 4 + 16 + 6*2*1*9*4 = 452 bytes

struct GestureCalEEPROM {
    uint32_t magic;
    char     calName[CAL_NAME_LEN];
    float    values[N_GESTURES][EEPROM_N_STATES][EEPROM_N_SIDES][EEPROM_N_JOINTS];
};

// ---- In-memory calibration name (set by set_cal_name, loaded by EEPROM) -----
extern char currentCalName[CAL_NAME_LEN];

// ---- API --------------------------------------------------------------------

/// Set the in-memory calibration name (does NOT write to EEPROM).
void gestureCalSetName(const char* name);

/// Return the current in-memory calibration name.
const char* gestureCalGetName();

/// Serialize current gestureLibrary values + currentCalName → EEPROM.
void gestureCalSaveToEEPROM();

/// Deserialize EEPROM → gestureLibrary values + currentCalName.
/// Returns false if EEPROM magic is invalid (first boot or corrupt).
bool gestureCalLoadFromEEPROM();

/// Update a single named-pair value in the live gestureLibrary array.
/// side = GESTURE_SIDE_ALL (0xFF) to update all sides; 0 = left, 1 = right.
/// Returns false if the gesture/state/joint combination was not found on any side.
bool gestureCalSetValue(const String& gesture, const String& state,
                        const String& joint, float value,
                        uint8_t side = GESTURE_SIDE_ALL);
