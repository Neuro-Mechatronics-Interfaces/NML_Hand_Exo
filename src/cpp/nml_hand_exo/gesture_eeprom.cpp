/**
 * @file gesture_eeprom.cpp
 * @brief EEPROM persistence for gestureLibrary [0-1] fractions and calibration name.
 */
#include "gesture_eeprom.h"
#include "utils.h"

// FlashStorage_SAMD provides EEPROM emulation on SAMD21 (OpenRB-150).
// Install via Arduino Library Manager: search "FlashStorage_SAMD".
#include <FlashStorage_SAMD.h>

char currentCalName[CAL_NAME_LEN] = "";

void gestureCalSetName(const char* name) {
    if (!name) return;
    strncpy(currentCalName, name, CAL_NAME_LEN - 1);
    currentCalName[CAL_NAME_LEN - 1] = '\0';
}

const char* gestureCalGetName() {
    return currentCalName;
}

void gestureCalSaveToEEPROM() {
    GestureCalEEPROM data;
    data.magic = GESTURE_EEPROM_MAGIC;
    strncpy(data.calName, currentCalName, CAL_NAME_LEN);
    data.calName[CAL_NAME_LEN - 1] = '\0';

    for (int g = 0; g < N_GESTURES; ++g) {
        const int nStates = min((int)gestureLibrary[g].numStates, (int)EEPROM_N_STATES);
        for (int s = 0; s < EEPROM_N_STATES; ++s) {
            for (int j = 0; j < EEPROM_N_JOINTS; ++j) {
                if (s < nStates && j < gestureLibrary[g].states[s].nPairs) {
                    data.values[g][s][j] = gestureLibrary[g].states[s].namedPairs[j].value;
                } else {
                    data.values[g][s][j] = 0.0f;
                }
            }
        }
    }

    EEPROM.put(GESTURE_CAL_EEPROM_ADDR, data);
    EEPROM.commit();
    debugPrint("[EEPROM] Gesture calibration saved. Name: " + String(currentCalName));
}

bool gestureCalLoadFromEEPROM() {
    GestureCalEEPROM data;
    EEPROM.get(GESTURE_CAL_EEPROM_ADDR, data);

    if (data.magic != GESTURE_EEPROM_MAGIC) {
        debugPrint("[EEPROM] No valid gesture calibration found (magic mismatch).");
        return false;
    }

    strncpy(currentCalName, data.calName, CAL_NAME_LEN);
    currentCalName[CAL_NAME_LEN - 1] = '\0';

    for (int g = 0; g < N_GESTURES; ++g) {
        const int nStates = min((int)gestureLibrary[g].numStates, (int)EEPROM_N_STATES);
        for (int s = 0; s < nStates; ++s) {
            const int nPairs = min((int)gestureLibrary[g].states[s].nPairs, (int)EEPROM_N_JOINTS);
            for (int j = 0; j < nPairs; ++j) {
                gestureLibrary[g].states[s].namedPairs[j].value = data.values[g][s][j];
            }
        }
    }

    debugPrint("[EEPROM] Gesture calibration loaded. Name: " + String(currentCalName));
    return true;
}

bool gestureCalSetValue(const String& gesture, const String& state,
                        const String& joint, float value) {
    const int gIdx = findGestureIndex(gesture);
    if (gIdx < 0) return false;

    const int sIdx = findStateIndex(gestureLibrary[gIdx], state);
    if (sIdx < 0) return false;

    String jointLower = joint;
    jointLower.toLowerCase();

    const int nPairs = gestureLibrary[gIdx].states[sIdx].nPairs;
    for (int j = 0; j < nPairs; ++j) {
        const char* pName = gestureLibrary[gIdx].states[sIdx].namedPairs[j].joint;
        if (pName && String(pName).equalsIgnoreCase(jointLower)) {
            gestureLibrary[gIdx].states[sIdx].namedPairs[j].value = value;
            return true;
        }
    }
    return false;
}
