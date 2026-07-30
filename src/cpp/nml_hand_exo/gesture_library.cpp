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
        // A dense state specifies every joint; a sparse state only claims the
        // joints it actually names.  Callers use this to leave unnamed joints
        // alone instead of driving them back to home.
        if (outTouched) outTouched[i] = !state.isSparse;
    }

    if (state.isSparse) {
        // Apply specified joints, matching ALL motors with the same name.
        // In dual-exo mode MOTOR_NAMES[] contains duplicate names (e.g. "wrist"
        // appears twice: once for left IDs 1-9 and once for right IDs 11-19).
        // The old jointIndexByName() only returned the first match, so the right
        // hand never received gesture angles.  This loop finds every matching
        // index so both exos are commanded by the same named value.
        for (uint8_t k = 0; k < state.nPairs; ++k) {
            if (!state.namedPairs[k].joint) continue;
            String pairName = String(state.namedPairs[k].joint);
            pairName.toLowerCase();
            for (int i = 0; i < N_MOTORS; ++i) {
                if (!MOTOR_NAMES[i] || !*MOTOR_NAMES[i]) continue;
                String mName = String(MOTOR_NAMES[i]); mName.toLowerCase();
                if (!mName.equals(pairName)) continue;
                if (state.isRelative) {
                    outAngles[i] = (homeAngles ? homeAngles[i] : 0.0f) + state.namedPairs[k].value;
                } else {
                    outAngles[i] = state.namedPairs[k].value;
                }
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

    // --- GRASP: (relative + sparse, normalized 0.0–1.0) ---
    // Values are fraction of calibrated range: 0.0 = home/open, 1.0 = fully closed
    {
      "grasp",
      {
        { "open",  true,  true,     {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   0.0},
            {"thumbflex",  0.0},
            {"thumbrot",   0.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 9 },
        { "close", true,  true,     {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   1.0},
            {"thumbflex",  1.0},
            {"thumbrot",   1.0},
            {"index",      1.0},
            {"middle",     1.0},
            {"ring",       1.0},
            {"pinky",      1.0}
          }, 9 }
      }, 2
    },

    // --- KEYGRIP: fingers closed, thumb presses side of index ---
    {
      "keygrip",
      {
        { "open",  true, true,   {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   0.0},
            {"thumbflex",  0.0},
            {"thumbrot",   0.0},
            {"index",      1.0},
            {"middle",     1.0},
            {"ring",       1.0},
            {"pinky",      1.0}
          }, 9 },
        { "close", true, true,   {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   1.0},
            {"thumbflex",  1.0},
            {"thumbrot",   0.0},
            {"index",      1.0},
            {"middle",     1.0},
            {"ring",       1.0},
            {"pinky",      1.0}
          }, 9 }
      }, 2
    },

    // --- PINCH INDEX: thumb + index pinch ---
    {
      "pinch_index",
      {
        { "open",  true,  true,  {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   0.0},
            {"thumbflex",  0.0},
            {"thumbrot",   0.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 9 },
        { "close", true,  true,  {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   1.0},
            {"thumbflex",  1.0},
            {"thumbrot",   1.0},
            {"index",      1.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 9 }
      }, 2
    },

    // --- PINCH MIDDLE: thumb + middle pinch ---
    {
      "pinch_middle",
      {
        { "open",  true,  true,  {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   0.0},
            {"thumbflex",  0.0},
            {"thumbrot",   0.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 9 },
        { "close", true,  true,  {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   1.0},
            {"thumbflex",  1.0},
            {"thumbrot",   1.0},
            {"index",      0.0},
            {"middle",     1.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 9 }
      }, 2
    },

    // --- PINCH RING: thumb + ring pinch ---
    {
      "pinch_ring",
      {
        { "open",  true,  true,  {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   0.0},
            {"thumbflex",  0.0},
            {"thumbrot",   0.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 9 },
        { "close", true,  true,  {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   1.0},
            {"thumbflex",  1.0},
            {"thumbrot",   1.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       1.0},
            {"pinky",      0.0}
          }, 9 }
      }, 2
    },

    // --- PEACE SIGN: index + middle extended, ring + pinky closed, thumb tucked ---
    {
      "peace",
      {
        { "open",  true,  true,  {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   0.0},
            {"thumbflex",  0.0},
            {"thumbrot",   0.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       0.0},
            {"pinky",      0.0}
          }, 9 },
        { "close", true,  true,  {0},
          {
            {"wrist",      0.0},
            {"wrist2",     0.0},
            {"thumbadd",   1.0},
            {"thumbflex",  1.0},
            {"thumbrot",   1.0},
            {"index",      0.0},
            {"middle",     0.0},
            {"ring",       1.0},
            {"pinky",      1.0}
          }, 9 }
      }, 2
    },

    // --- PER-JOINT EXTENSION / REST / FLEXION -------------------------------
    // One gesture per digit (plus the wrist) so a host can drive a single joint
    // without composing a whole posture.  Travel comes from the EXTEND_*/REST_*/
    // FLEX_* constants in config.h -- tune magnitude and direction there.
    //
    // States are ordered extend -> rest -> flex so travel is monotonic: a
    // cycle_gesture_state sweep walks the joint through its range in order
    // instead of jumping across it, and states[0] (the state cycleGesture() and
    // the button callbacks land on) stays "extend", which is exactly home.
    //
    // Each state names ONLY its own joints.  executeGesture() commands just the
    // named joints, so selecting a joint leaves the rest of the hand exactly
    // where it was.  Returning to a whole-hand rest is still an explicit
    // gesture (grasp:open), not an implicit side effect of every other gesture.
    //
    // The thumb lists its three joints separately because they do not share a
    // flip direction; one shared value would drive them opposite ways.
    {
      "thumb",
      {
        { "extend", true, true, {0},
          {
            {"thumbadd", EXTEND_THUMBADD},
            {"thumbrot", EXTEND_THUMBROT},
            {"thumbflex", EXTEND_THUMBFLEX}
          }, 3 },
        { "rest", true, true, {0},
          {
            {"thumbadd", REST_THUMBADD},
            {"thumbrot", REST_THUMBROT},
            {"thumbflex", REST_THUMBFLEX}
          }, 3 },
        { "flex", true, true, {0},
          {
            {"thumbadd", FLEX_THUMBADD},
            {"thumbrot", FLEX_THUMBROT},
            {"thumbflex", FLEX_THUMBFLEX}
          }, 3 }
      }, 3
    },

    {
      "index",
      {
        { "extend", true, true, {0},
          {
            {"index", EXTEND_INDEX}
          }, 1 },
        { "rest", true, true, {0},
          {
            {"index", REST_INDEX}
          }, 1 },
        { "flex", true, true, {0},
          {
            {"index", FLEX_INDEX}
          }, 1 }
      }, 3
    },

    {
      "middle",
      {
        { "extend", true, true, {0},
          {
            {"middle", EXTEND_MIDDLE}
          }, 1 },
        { "rest", true, true, {0},
          {
            {"middle", REST_MIDDLE}
          }, 1 },
        { "flex", true, true, {0},
          {
            {"middle", FLEX_MIDDLE}
          }, 1 }
      }, 3
    },

    {
      "ring",
      {
        { "extend", true, true, {0},
          {
            {"ring", EXTEND_RING}
          }, 1 },
        { "rest", true, true, {0},
          {
            {"ring", REST_RING}
          }, 1 },
        { "flex", true, true, {0},
          {
            {"ring", FLEX_RING}
          }, 1 }
      }, 3
    },

    {
      "pinky",
      {
        { "extend", true, true, {0},
          {
            {"pinky", EXTEND_PINKY}
          }, 1 },
        { "rest", true, true, {0},
          {
            {"pinky", REST_PINKY}
          }, 1 },
        { "flex", true, true, {0},
          {
            {"pinky", FLEX_PINKY}
          }, 1 }
      }, 3
    },

    // Wrist flexion/extension. Names only the `wrist` joint: `wrist2` is a
    // separate DOF, addressed by the "rad" gesture below. Coupling both into one
    // gesture would move an axis the caller never asked for.
    {
      "wrist",
      {
        { "extend", true, true, {0},
          {
            {"wrist", EXTEND_WRIST}
          }, 1 },
        { "rest", true, true, {0},
          {
            {"wrist", REST_WRIST}
          }, 1 },
        { "flex", true, true, {0},
          {
            {"wrist", FLEX_WRIST}
          }, 1 }
      }, 3
    },

    // Second wrist axis (the `wrist2` motor). Named for radial/ulnar deviation
    // -- see the EXTEND_RAD/REST_RAD/FLEX_RAD note in config.h, the anatomy is
    // unverified. "flex"/"extend" mean whichever direction calibration recorded
    // for wrist2, and the names stay uniform so set_gesture_angle (which keys
    // off the "flex" state) works here too.
    {
      "rad",
      {
        { "extend", true, true, {0},
          {
            {"wrist2", EXTEND_RAD}
          }, 1 },
        { "rest", true, true, {0},
          {
            {"wrist2", REST_RAD}
          }, 1 },
        { "flex", true, true, {0},
          {
            {"wrist2", FLEX_RAD}
          }, 1 }
      }, 3
    },
};
