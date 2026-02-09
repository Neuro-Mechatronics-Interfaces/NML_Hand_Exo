#include "gesture_library.h"

// Map joint name -> index using MOTOR_NAMES from config.h
int jointIndexByName(const char* jointName) {
    if (!jointName) return -1;
    String target = String(jointName); target.toLowerCase();
    for (int i = 0; i < N_MOTORS; ++i) {
        if (MOTOR_NAMES[i] && *MOTOR_NAMES[i]) {
            String s = String(MOTOR_NAMES[i]); s.toLowerCase();
            if (s.equals(target)) return i;
        }
    }
    return -1;
}

// Build a dense angle array from a state, applying relative math if requested
void resolveStateAngles(const GestureState& state,
                        const float* homeAngles,
                        float* outAngles) {
    // Start with either zeros (absolute) or the home baseline (relative)
    for (int i = 0; i < N_MOTORS; ++i) {
        outAngles[i] = state.isRelative ? (homeAngles ? homeAngles[i] : 0.0f) : 0.0f;
    }

    if (state.isSparse) {
        // Apply only specified joints
        for (uint8_t k = 0; k < state.nPairs; ++k) {
            int idx = jointIndexByName(state.namedPairs[k].joint);
            if (idx < 0 || idx >= N_MOTORS) continue;
            if (state.isRelative) {
                outAngles[idx] = (homeAngles ? homeAngles[idx] : 0.0f) + state.namedPairs[k].value;
            } else {
                outAngles[idx] = state.namedPairs[k].value;
            }
        }
    } else {
        // Dense: one value per joint
        for (int i = 0; i < N_MOTORS; ++i) {
            outAngles[i] = state.isRelative
                           ? (outAngles[i] + state.jointAngles[i]) // already init'd to home
                           : state.jointAngles[i];
        }
    }
}

// ====== Library contents ======
// Note: "pinch"/"keygrip" left as absolute dense to avoid breaking existing logic.
// You can convert them to relative later if you want.

GestureMap gestureLibrary[N_GESTURES] = {
    // --- HOME: capture baseline (all zeros relative) ---
    // {
    //   "home",
    //   {
    //     // name,  is Relative to home, isSparse,         dense...,          sparse...,    nPairs
    //     { "home",         true,           false,    {0,0,0,0,0,0},          {},           0 }
    //   },
    //   1
    // },

    // --- GRASP: (relative + sparse, normalized 0.0–1.0) ---
    // Values are fraction of calibrated range: 0.0 = home/open, 1.0 = fully closed
    {
      "grasp",
      {
        { "open",  true,  true,     {0},
          {
            {"thumbflex",  0.0},
            {"thumbrot",   1.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 6 },
        { "close", true,  true,     {0},
          { {"thumbflex",  1.0},
            {"thumbrot",   1.0},
            {"index",      1.0},
            {"middle",     1.0},
            {"ring",       1.0},
            {"pinky",      1.0}
          }, 6 }
      }, 2
    },

    // --- KEYGRIP: fingers closed, thumb presses side of index ---
    {
      "keygrip",
      {
        { "open",  true, true,   {0},
          {
            {"thumbflex",  0.0},
            {"thumbrot",   0.0},
            {"index",      1.0},
            {"middle",     1.0},
            {"ring",       1.0},
            {"pinky",      1.0}
          }, 6 },
        { "close", true, true,   {0},
          {
            {"thumbflex",  1.0},
            {"thumbrot",   0.0},
            {"index",      1.0},
            {"middle",     1.0},
            {"ring",       1.0},
            {"pinky",      1.0}
          }, 6 }
      }, 2
    },

    // --- PINCH INDEX: thumb + index pinch ---
    {
      "pinch_index",
      {
        { "open",  true,  true,  {0},
          {
            {"thumbflex",  0.0},
            {"thumbrot",   1.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 6 },
        { "close", true,  true,  {0},
          {
            {"thumbflex",  1.0},
            {"thumbrot",   1.0},
            {"index",      1.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 6 }
      }, 2
    },

    // --- PINCH MIDDLE: thumb + middle pinch ---
    {
      "pinch_middle",
      {
        { "open",  true,  true,  {0},
          {
            {"thumbflex",  0.0},
            {"thumbrot",   1.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 6 },
        { "close", true,  true,  {0},
          {
            {"thumbflex",  1.0},
            {"thumbrot",   1.0},
            {"index",      0.0},
            {"middle",     1.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 6 }
      }, 2
    },

    // --- PINCH RING: thumb + ring pinch ---
    {
      "pinch_ring",
      {
        { "open",  true,  true,  {0},
          {
            {"thumbflex",  0.0},
            {"thumbrot",   1.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 6 },
        { "close", true,  true,  {0},
          {
            {"thumbflex",  1.0},
            {"thumbrot",   1.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       1.0},
            {"pinky",      0.0}
          }, 6 }
      }, 2
    },
};
