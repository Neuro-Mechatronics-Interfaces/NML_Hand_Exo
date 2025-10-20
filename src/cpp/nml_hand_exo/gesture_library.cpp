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

    // --- GRASP: (relative + sparse) ---
    {
      "grasp",
      {
        { "open",  true,  true,     {0}, 
          {
            {"thumbflex",   0.0},
            {"thumbrot",   60.0},
            {"index",       0.0},
            {"middle",      0.0},
            {"ring",        0.0},
            {"pinky",       0.0}
          }, 6 },
        { "close", true,  true,     {0}, 
          { {"thumbflex",  60.0},
            {"thumbrot",   60.0},
            {"index",      60.0},
            {"middle",     60.0},
            {"ring",       60.0},
            {"pinky",      60.0}
          }, 6 }
      }, 2
    },

    // --- KEYGRIP: ---
    {
      "keygrip",
      {
        // Open: thumb=0, index/middle/ring/pinky=30, ignore wrist
        { "open",  true, true,   {0}, 
          {
            {"thumbflex",  0.0},
            {"thumbrot",   0.0},
            {"index",     60.0},
            {"middle",    60.0},
            {"ring",      60.0},
            {"pinky",     60.0}
          }, 5 },
        // Close: thumb=30, index/middle/ring/pinky=30, ignore wrist
        { "close", true, true,   {0}, 
          {
            {"thumbflex", 60.0},
            {"thumbrot",   0.0},
            {"index",     60.0},
            {"middle",    60.0},
            {"ring",      60.0},
            {"pinky",     60.0}
             }, 5 }
      }, 2
    },

    // --- PINCH: (keep as dense absolute; your remapper uses these) ---
    {
      "pinch_index",
      {
        // open: thumb=0, index=0, others closed=30; wrist omitted
        { "open",  true,  true,  {0},
          {
            {"thumbflex",   0.0},
            {"thumbrot",   60.0},
            {"index",       0.0}, // this one
            {"middle",      0.0},
            {"ring",        0.0},
            {"pinky",       0.0}
          }, 5 },
        // close: thumb=30, index=30, others closed=30
        { "close", true,  true,  {0},
          {
            {"thumbflex", 60.0},
            {"thumbrot",  60.0},
            {"index",     60.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 5 }
      }, 2
    },

    {
      "pinch_middle",
      {
        { "open",  true,  true,  {0},
          {
            {"thumbflex",  0.0},
            {"thumbrot",  60.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 5 },
        { "close", true,  true,  {0},
          {
            {"thumbflex", 60.0},
            {"thumbrot",  60.0},
            {"index",      0.0},
            {"middle",    60.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 5 }
      }, 2
    },

    {
      "pinch_ring",
      {
        { "open",  true,  true,  {0},
          {
            {"thumbflex",  0.0},
            {"thumbrot",  60.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 5 },
        { "close", true,  true,  {0},
          {
            {"thumbflex",  60.0},
            {"thumbrot",  60.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",      60.0},
            {"pinky",      0.0}
          }, 5 }
      }, 2
    },
};
