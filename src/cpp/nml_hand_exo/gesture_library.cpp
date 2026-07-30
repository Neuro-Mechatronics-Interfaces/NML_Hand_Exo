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
                        float* outAngles,
                        bool* outTouched) {
    // Start with either zeros (absolute) or the home baseline (relative)
    for (int i = 0; i < N_MOTORS; ++i) {
        outAngles[i] = state.isRelative ? (homeAngles ? homeAngles[i] : 0.0f) : 0.0f;
        if (outTouched) outTouched[i] = !state.isSparse;
    }

    if (state.isSparse) {
        // Apply every configured motor with the named joint.  In dual builds,
        // names occur once per side and both motors must receive the posture.
        for (uint8_t k = 0; k < state.nPairs; ++k) {
            if (!state.namedPairs[k].joint) continue;
            String target = String(state.namedPairs[k].joint);
            target.toLowerCase();
            for (int i = 0; i < N_MOTORS; ++i) {
                if (!MOTOR_NAMES[i] || !*MOTOR_NAMES[i]) continue;
                String name = String(MOTOR_NAMES[i]);
                name.toLowerCase();
                if (!name.equals(target)) continue;
                outAngles[i] = state.isRelative
                    ? (homeAngles ? homeAngles[i] : 0.0f) + state.namedPairs[k].value
                    : state.namedPairs[k].value;
                if (outTouched) outTouched[i] = true;
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
            {"thumbrot",   140.0},
            {"index",       0.0},
            {"middle",      0.0},
            {"ring",        0.0},
            {"pinky",       0.0}
          }, 6 },
        { "close", true,  true,     {0}, 
          { {"thumbflex",  90.0},
            {"thumbrot",   140.0},
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
          }, 6 },
        // Close: thumb=30, index/middle/ring/pinky=30, ignore wrist
        { "close", true, true,   {0}, 
          {
            {"thumbflex", 60.0},
            {"thumbrot",   0.0},
            {"index",     60.0},
            {"middle",    60.0},
            {"ring",      60.0},
            {"pinky",     60.0}
             }, 6 }
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
            {"thumbrot",   140.0},
            {"index",       0.0}, // this one
            {"middle",      0.0},
            {"ring",        0.0},
            {"pinky",       0.0}
          }, 5 },
        // close: thumb=30, index=30, others closed=30
        { "close", true,  true,  {0},
          {
            {"thumbflex", 60.0},
            {"thumbrot",  140.0},
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
            {"thumbrot",  150.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 5 },
        { "close", true,  true,  {0},
          {
            {"thumbflex", 60.0},
            {"thumbrot",  150.0},
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
            {"thumbrot",  160.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 5 },
        { "close", true,  true,  {0},
          {
            {"thumbflex",  60.0},
            {"thumbrot",  160.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",      60.0},
            {"pinky",      0.0}
          }, 5 }
      }, 2
    },

    { "peace", {
        { "open", true, true, {0}, {{"thumbadd", EXTEND_FRACTION}, {"thumbflex", EXTEND_FRACTION}, {"thumbrot", EXTEND_FRACTION}, {"index", EXTEND_FRACTION}, {"middle", EXTEND_FRACTION}, {"ring", EXTEND_FRACTION}, {"pinky", EXTEND_FRACTION}}, 7, true },
        { "close", true, true, {0}, {{"thumbadd", FLEX_FRACTION}, {"thumbflex", FLEX_FRACTION}, {"thumbrot", FLEX_FRACTION}, {"index", EXTEND_FRACTION}, {"middle", EXTEND_FRACTION}, {"ring", FLEX_FRACTION}, {"pinky", FLEX_FRACTION}}, 7, true }
      }, 2 },

    { "thumb", {
        { "extend", true, true, {0}, {{"thumbadd", EXTEND_FRACTION}, {"thumbflex", EXTEND_FRACTION}, {"thumbrot", EXTEND_FRACTION}}, 3, true },
        { "rest",   true, true, {0}, {{"thumbadd", REST_FRACTION},   {"thumbflex", REST_FRACTION},   {"thumbrot", REST_FRACTION}},   3, true },
        { "flex",   true, true, {0}, {{"thumbadd", FLEX_FRACTION},   {"thumbflex", FLEX_FRACTION},   {"thumbrot", FLEX_FRACTION}},   3, true }
      }, 3 },
    { "thumbadd", {
        { "extend", true, true, {0}, {{"thumbadd", EXTEND_FRACTION}}, 1, true },
        { "rest",   true, true, {0}, {{"thumbadd", REST_FRACTION}},   1, true },
        { "flex",   true, true, {0}, {{"thumbadd", FLEX_FRACTION}},   1, true }
      }, 3 },
    { "thumbrot", {
        { "extend", true, true, {0}, {{"thumbrot", EXTEND_FRACTION}}, 1, true },
        { "rest",   true, true, {0}, {{"thumbrot", REST_FRACTION}},   1, true },
        { "flex",   true, true, {0}, {{"thumbrot", FLEX_FRACTION}},   1, true }
      }, 3 },
    { "thumbflex", {
        { "extend", true, true, {0}, {{"thumbflex", EXTEND_FRACTION}}, 1, true },
        { "rest",   true, true, {0}, {{"thumbflex", REST_FRACTION}},   1, true },
        { "flex",   true, true, {0}, {{"thumbflex", FLEX_FRACTION}},   1, true }
      }, 3 },
    { "index", {
        { "extend", true, true, {0}, {{"index", EXTEND_FRACTION}}, 1, true },
        { "rest",   true, true, {0}, {{"index", REST_FRACTION}},   1, true },
        { "flex",   true, true, {0}, {{"index", FLEX_FRACTION}},   1, true }
      }, 3 },
    { "middle", {
        { "extend", true, true, {0}, {{"middle", EXTEND_FRACTION}}, 1, true },
        { "rest",   true, true, {0}, {{"middle", REST_FRACTION}},   1, true },
        { "flex",   true, true, {0}, {{"middle", FLEX_FRACTION}},   1, true }
      }, 3 },
    { "ring", {
        { "extend", true, true, {0}, {{"ring", EXTEND_FRACTION}}, 1, true },
        { "rest",   true, true, {0}, {{"ring", REST_FRACTION}},   1, true },
        { "flex",   true, true, {0}, {{"ring", FLEX_FRACTION}},   1, true }
      }, 3 },
    { "pinky", {
        { "extend", true, true, {0}, {{"pinky", EXTEND_FRACTION}}, 1, true },
        { "rest",   true, true, {0}, {{"pinky", REST_FRACTION}},   1, true },
        { "flex",   true, true, {0}, {{"pinky", FLEX_FRACTION}},   1, true }
      }, 3 },
    { "wrist", {
        { "extend", true, true, {0}, {{"wrist", WRIST_EXTEND_FRACTION}, {"wrist2", WRIST_EXTEND_FRACTION}}, 2, true },
        { "rest",   true, true, {0}, {{"wrist", WRIST_REST_FRACTION},   {"wrist2", WRIST_REST_FRACTION}},   2, true },
        { "flex",   true, true, {0}, {{"wrist", WRIST_FLEX_FRACTION},   {"wrist2", WRIST_FLEX_FRACTION}},   2, true }
      }, 3 },
};
