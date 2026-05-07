/**
 * @file gesture_library.h
 * @brief Header file for the gesture library used in the NML Hand Exoskeleton project.
 *
 */
#ifndef GESTURE_LIBRARY_H
#define GESTURE_LIBRARY_H
#include "config.h"

/// @brief Named pair for sparse states ===
struct KeyAngle {
    const char* joint;   // e.g., "thumb"
    float value;         // degrees (relative or absolute depending on isRelative)
};
constexpr uint8_t MAX_SPARSE_PER_STATE = N_MOTORS;  // up to all joints

/// @brief Struct to represent a single gesture state (now supports dense or sparse, absolute or relative)
struct GestureState {
    const char* name;                 // e.g., "open", "close"
    bool isRelative;                  // true => values are deltas from "home" baseline
    bool isSparse;                    // true => use namedPairs[], else use jointAngles[]

    // Dense representation (keep for backward compatibility)
    float jointAngles[N_MOTORS];

    // Sparse representation (named pairs)
    KeyAngle namedPairs[MAX_SPARSE_PER_STATE];
    uint8_t nPairs;
};

/// @brief Struct to represent a gesture with its name and states.
struct GestureMap {
    const char* name;              // e.g., "pinch", "open"
    GestureState states[MAX_STATES_PER_GESTURE];
    int numStates;
};

/// @brief Gesture library indexed [side][gesture].
/// Side 0 = left (or the only side in single-exo mode), side 1 = right (dual only).
/// gestureLibraryInit() must be called from setup() to copy side-0 defaults to side-1.
extern GestureMap gestureLibrary[N_HAND_SIDES][N_GESTURES];

int jointIndexByName(const char* jointName);
void resolveStateAngles(const GestureState& state,
                        const float* homeAngles,   // length N_MOTORS
                        float* outAngles);         // length N_MOTORS

/// @brief Copy side-0 defaults into side-1 at runtime (dual mode only).
/// Call once from setup() before gestureCalLoadFromEEPROM().
void gestureLibraryInit();

/// @brief Find a gesture index by name (searches side 0; all sides share the same names).
inline int findGestureIndex(const String& gestureName) {
    for (int i = 0; i < N_GESTURES; i++) {
        if (gestureName.equalsIgnoreCase(gestureLibrary[0][i].name)) {
            return i;
        }
    }
    return -1;
}

/// @brief Helper function to find the index of a state in a gesture.
inline int findStateIndex(const GestureMap& gesture, const String& stateName) {
    for (int i = 0; i < gesture.numStates; i++) {
        if (stateName.equalsIgnoreCase(gesture.states[i].name)) {
            return i;
        }
    }
    return -1;
}

#endif
