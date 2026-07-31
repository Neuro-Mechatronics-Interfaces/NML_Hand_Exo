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

/// @brief Number of gestures defined in the library.
extern GestureMap gestureLibrary[N_GESTURES];

int jointIndexByName(const char* jointName);
/// @brief Resolve a gesture state into absolute per-motor angles.
/// @param outTouched Optional, length N_MOTORS. Set true for joints this state
///        actually commands. A sparse state only claims the joints it names, so
///        callers can leave the rest of the hand where it is rather than
///        driving it home. Dense states claim every joint.
void resolveStateAngles(const GestureState& state,
                        const float* homeAngles,   // length N_MOTORS
                        float* outAngles,          // length N_MOTORS
                        bool* outTouched = nullptr);
                        
/// @brief Helper function to find the index of a gesture by name.
inline int findGestureIndex(const String& gestureName) {
    for (int i = 0; i < N_GESTURES; i++) {
        if (gestureName.equalsIgnoreCase(gestureLibrary[i].name)) {
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
