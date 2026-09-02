#include "gesture_library.h"
#include "gesture_controller.h"
#include "utils.h"
#include "oled.h"

static ExoState mapGestureStateToExoState(const String& gesture, const String& state) {
  String g = gesture; g.toLowerCase();
  String s = state;   s.toLowerCase();

  if (g.indexOf("pinch") >= 0) {
    if (g.indexOf("index")  >= 0) return EXO_INDEX_PINCH;
    if (g.indexOf("middle") >= 0) return EXO_MIDDLE_PINCH;
    if (g.indexOf("ring")   >= 0) return EXO_RING_PINCH;
  }
  if (g.indexOf("key") >= 0) {
    return (s.indexOf("open") >= 0) ? EXO_KEYGRIP_OPEN : EXO_KEYGRIP_CLOSE;
  }
  if (g.indexOf("grasp") >= 0 || g.indexOf("power") >= 0) {
    return (s.indexOf("open") >= 0) ? EXO_GRASP_OPEN : EXO_GRASP_CLOSE;
  }

  // Per-joint extend/rest/flex gestures. Matched on the EXACT gesture name, not
  // a substring: "index" as a substring also appears in "pinch_index", which is
  // handled above and must keep its own state.
  struct DigitEntry { const char* name; ExoState flex; };
  static const DigitEntry kDigits[] = {
    { "thumb",  EXO_THUMB_FLEX  },
    { "thumbadd",  EXO_THUMB_FLEX  },
    { "thumbrot",  EXO_THUMB_FLEX  },
    { "thumbflex", EXO_THUMB_FLEX  },
    { "index",  EXO_INDEX_FLEX  },
    { "middle", EXO_MIDDLE_FLEX },
    { "ring",   EXO_RING_FLEX   },
    { "pinky",  EXO_PINKY_FLEX  },
    // No "rad" row: wrist2 moves with `wrist` now, so nothing can reach the
    // EXO_RAD_* screens. The enum keeps them so oled.cpp stays exhaustive.
    { "wrist",  EXO_WRIST_FLEX  },
  };
  for (uint8_t i = 0; i < sizeof(kDigits) / sizeof(kDigits[0]); ++i) {
    if (g == kDigits[i].name) {
      // The enum lays each joint out as FLEX, EXTEND, REST in consecutive slots.
      if (s.indexOf("extend") >= 0) return (ExoState)(kDigits[i].flex + 1);
      if (s.indexOf("rest")   >= 0) return (ExoState)(kDigits[i].flex + 2);
      return kDigits[i].flex;
    }
  }

  return EXO_READY; // fallback for unknowns/idle
}

GestureController::GestureController(NMLHandExo& exo)
  : exo_(exo),
    cycleGesturePin(-1),
    gestureStateSwitchPin(-1),
    lastCycleGestureDebounceTime(0),
    lastGestureStateDebounceTime(0),
    lastCycleGestureButtonState(HIGH),
    lastGestureStateButtonState(HIGH),
    cycleGestureButtonState(HIGH),
    gestureStateButtonState(HIGH) 
{
    currentGesture_ = gestureLibrary[0].name;
    currentGestureState_ = gestureLibrary[0].states[0].name;
    numGestures_ = N_GESTURES;
}

void GestureController::executeGesture(const String& gesture, const String& state) {

  int gIdx = findGestureIndex(gesture);
  if (gIdx == -1) {
    debugPrint("[GestureController] Unknown gesture: " + gesture);
    return;
  }

  int sIdx = findStateIndex(gestureLibrary[gIdx], state);
  if (sIdx == -1) {
    debugPrint("[GestureController] Unknown state: " + state + " for gesture " + gesture);
    return;
  }

  const GestureState& st = gestureLibrary[gIdx].states[sIdx];

  // Build home baseline in *index order*
  float home[N_MOTORS];
  for (int i = 0; i < exo_.getMotorCount(); ++i) {
    uint8_t id = exo_.getMotorIDByIndex(i);
    home[i] = exo_.getZeroAngle(id);
  }

  // Resolve this state into absolute angles.
  // Gesture values are normalized 0.0–1.0 (fraction of motor range).
  float absAngles[N_MOTORS];
  bool touched[N_MOTORS];
  resolveStateAngles(gestureLibrary[gIdx].states[sIdx], home, absAngles, touched);

  if (st.isRelative) {
    // Place each normalized offset on that motor's gesture axis: 0 = home,
    // 1 = the flexion endstop. Sharing gestureFractionToAngle() with
    // setGestureAngle() is what keeps `set_gesture:index:flex` and
    // `set_gesture_angle:index:<FLEX_INDEX*100>` on the same target -- and what
    // lets get_gesture_angle report either of them back as the same number.
    for (int i = 0; i < exo_.getMotorCount(); ++i) {
      if (!touched[i]) continue;
      uint8_t id = exo_.getMotorIDByIndex(i);
      float fraction = absAngles[i] - home[i];  // 0.0–1.0 from resolveStateAngles
      absAngles[i] = exo_.gestureFractionToAngle(id, fraction);
    }
  }

  // Command absolute targets.
  //
  // The per-motor trace is accumulated and emitted as ONE line rather than one
  // println per motor.  Each telemetryPrintln is a blocking USB-CDC write, so
  // the old per-motor form cost N blocking writes per gesture (N = 18 in
  // dual-exo builds) and dominated command round-trip latency.  Same
  // information, one write.
  // Only joints this state actually names are commanded. A sparse state that
  // lists one digit leaves every other joint untouched, so switching gestures
  // no longer implies an invisible "open everything else" -- REST is an
  // explicit gesture, not a side effect of every other one.
  String trace;
  for (int i = 0; i < exo_.getMotorCount(); ++i) {
    if (!touched[i]) continue;
    uint8_t id = exo_.getMotorIDByIndex(i);
    if (!exo_.isMotorConnected(id)) continue;
    if (VERBOSE) {
      if (trace.length()) trace += ", ";
      trace += String(id) + "->" + String(absAngles[i], 2);
    }
    exo_.setAbsoluteAngle(id, absAngles[i]);
  }
  debugPrint("[GestureController] targets: " + trace);

  // float* angles = gestureLibrary[gIdx].states[sIdx].jointAngles;
  // const GestureState& st = gestureLibrary[gIdx].states[sIdx];
  // for (int i = 0; i < exo_.getMotorCount(); i++) {
  //   uint8_t id = exo_.getMotorIDByIndex(i);  // ID from index
  //   float rel = angles[i];
  //   float abs_preview = exo_.getZeroAngle(id) + rel;
  //   char buffer[96];
  //   snprintf(buffer, sizeof(buffer), "[GestureController] motor %d: %s %.2f deg (abs preview %.2f)",
  //         i, st.isRelative ? "relative" : "absolute", rel, abs_preview);
  //   debugPrint(buffer);

  //   // If your gesture values are relative, send as relative for clearer logs:
  //   if (st.isRelative) {
  //     exo_.setRelativeAngle(id, rel);
  //   } else {
  //     exo_.setAbsoluteAngle(id, rel);
  //   }
  // }

  currentGesture_ = gesture;
  currentGestureState_ = state;
  debugPrint("[GestureController] Executed gesture: " + gesture + ", state: " + state);

  // OLED: reflect the new state
  oledSetState(mapGestureStateToExoState(currentGesture_, currentGestureState_));
}
void GestureController::executeCurrentGestureNewState(const String& state) {
  // Get the current gesture
  String gesture = getCurrentGesture();

  // Find the gesture index
  int gIdx = findGestureIndex(gesture);
  if (gIdx == -1) {
    debugPrint("[GestureController] Unknown gesture: " + gesture);
    return;
  }

  // Check if the state exists for the gesture
  int sIdx = findStateIndex(gestureLibrary[gIdx], state);
  if (sIdx == -1) {
    debugPrint("[GestureController] Error: State '" + state + "' not found for gesture '" + gesture + "'.");
    return;
  }

  // Execute the gesture with the specified state
  executeGesture(gesture, state);
}
bool GestureController::setGestureAngle(const String& gesture, float percent,
                                        uint8_t* movedOut, uint8_t* stuckOut) {
  if (movedOut) *movedOut = 0;
  if (stuckOut) *stuckOut = 0;

  int gIdx = findGestureIndex(gesture);
  if (gIdx == -1) {
    debugPrint("[GestureController] Unknown gesture: " + gesture);
    return false;
  }

  // Multi-joint postures (grasp, keygrip, pinch_*) have no flex state and are
  // deliberately not addressable here: one percentage cannot describe a posture.
  GestureAxisPoint axis[N_MOTORS];
  uint8_t nAxis = resolveGestureAxis(gIdx, axis, N_MOTORS);
  if (nAxis == 0) {
    debugPrint("[GestureController] Gesture '" + gesture +
               "' has no flex state; not angle-addressable");
    return false;
  }

  percent = constrain(percent, 0.0f, 100.0f);
  const float t = percent / 100.0f;

  // Interpolate the gesture's OWN endpoints: 0% is its extend posture, 100% is
  // its flex posture, and each motor moves its own share. Where a gesture drives
  // several motors the ratio between them therefore holds at every percentage,
  // which a shared per-motor fraction could not do -- and 100 lands exactly on
  // `set_gesture:<g>:flex` instead of somewhere past it.
  String trace;
  uint8_t moved = 0;
  uint8_t stuck = 0;
  for (uint8_t k = 0; k < nAxis; ++k) {
    // A motor with no calibrated travel cannot be positioned. Commanding it
    // anyway would be a goal it is already sitting on, counted as a move and
    // acked as success -- the exact failure this whole path exists to make
    // visible. Report it instead.
    if (fabsf(exo_.getGestureSpan(axis[k].id)) < GESTURE_MIN_TRAVEL_DEG) {
      ++stuck;
      debugPrint("[GestureController] motor " + String(axis[k].id) + " (" +
                 exo_.getMotorNameByID(axis[k].id) +
                 ") has no calibrated travel; skipped");
      continue;
    }

    const float fraction = axis[k].extendFraction +
                           t * (axis[k].flexFraction - axis[k].extendFraction);
    const float target = exo_.gestureFractionToAngle(axis[k].id, fraction);
    if (VERBOSE) {
      if (trace.length()) trace += ", ";
      trace += String(axis[k].id) + "->" + String(target, 2);
    }
    exo_.setAbsoluteAngle(axis[k].id, target);
    ++moved;
  }
  if (movedOut) *movedOut = moved;
  if (stuckOut) *stuckOut = stuck;

  // Deliberately does NOT touch currentGesture_/currentGestureState_: this is a
  // direct positioning command, not a state-machine transition. Writing a
  // synthetic state name here would make the next cycle_gesture_state fail its
  // findStateIndex() lookup and silently do nothing.
  debugPrint("[GestureController] " + gesture + " @ " + String(percent, 1) +
             "% targets: " + trace);
  return true;
}
uint8_t GestureController::resolveGestureAxis(int gestureIndex,
                                             GestureAxisPoint* out,
                                             uint8_t maxPoints) {
  if (!out || maxPoints == 0) return 0;
  if (gestureIndex < 0 || gestureIndex >= N_GESTURES) return 0;

  // The "flex" state defines both membership and the 100% end. A gesture
  // without one is not angle-addressable, which is what keeps multi-joint
  // postures out without hard-coding a list of them.
  const int fIdx = findStateIndex(gestureLibrary[gestureIndex], "flex");
  if (fIdx == -1) return 0;
  const int eIdx = findStateIndex(gestureLibrary[gestureIndex], "extend");
  const int rIdx = findStateIndex(gestureLibrary[gestureIndex], "rest");
  const GestureState& flex = gestureLibrary[gestureIndex].states[fIdx];

  uint8_t n = 0;
  for (uint8_t k = 0; k < flex.nPairs && n < maxPoints; ++k) {
    if (!flex.namedPairs[k].joint) continue;
    String pairName = String(flex.namedPairs[k].joint);
    pairName.toLowerCase();

    // 0% is the extend state's value for this joint. A gesture with no extend
    // state anchors at home instead, which is where extend sat before it became
    // an independently tunable posture.
    float extendFraction = 0.0f;
    if (eIdx != -1) {
      const GestureState& ext = gestureLibrary[gestureIndex].states[eIdx];
      for (uint8_t e = 0; e < ext.nPairs; ++e) {
        if (!ext.namedPairs[e].joint) continue;
        String extName = String(ext.namedPairs[e].joint);
        extName.toLowerCase();
        if (extName.equals(pairName)) {
          extendFraction = ext.namedPairs[e].value;
          break;
        }
      }
    }

    // Signed-angle read-back is anchored at this joint's rest posture. Keep a
    // missing rest as NAN so legacy/future gestures can still use the percent
    // query without inventing a physical zero for the signed query.
    float restFraction = NAN;
    if (rIdx != -1) {
      const GestureState& rest = gestureLibrary[gestureIndex].states[rIdx];
      for (uint8_t r = 0; r < rest.nPairs; ++r) {
        if (!rest.namedPairs[r].joint) continue;
        String restName = String(rest.namedPairs[r].joint);
        restName.toLowerCase();
        if (restName.equals(pairName)) {
          restFraction = rest.namedPairs[r].value;
          break;
        }
      }
    }

    // Match EVERY motor carrying this name: dual builds list each name twice
    // (left IDs 1-9, right IDs 11-19), same as resolveStateAngles().
    for (int i = 0; i < exo_.getMotorCount() && n < maxPoints; ++i) {
      uint8_t id = exo_.getMotorIDByIndex(i);
      String mName = exo_.getMotorNameByID(id);
      mName.toLowerCase();
      if (!mName.equals(pairName)) continue;
      out[n].id = id;
      out[n].extendFraction = extendFraction;
      out[n].restFraction = restFraction;
      out[n].flexFraction = flex.namedPairs[k].value;
      ++n;
    }
  }
  return n;
}
uint8_t GestureController::readGestureAngles(GestureAngleRecord* out,
                                            uint8_t maxRecords,
                                            const String& only) {
  if (!out || maxRecords == 0) return 0;

  String wanted = only;
  wanted.trim();
  wanted.toLowerCase();

  // -- 1. Which gestures are being reported, and which motors do they name? --
  GestureAxisPoint axis[N_MOTORS];
  uint8_t ids[N_MOTORS];
  uint8_t idCount = 0;
  int8_t gestureIdx[N_GESTURES];
  uint8_t gestureCount = 0;

  for (int g = 0; g < N_GESTURES && gestureCount < maxRecords; ++g) {
    if (wanted.length() && !wanted.equalsIgnoreCase(gestureLibrary[g].name)) continue;
    uint8_t nAxis = resolveGestureAxis(g, axis, N_MOTORS);
    if (nAxis == 0) continue;
    gestureIdx[gestureCount++] = (int8_t)g;
    for (uint8_t k = 0; k < nAxis; ++k) {
      bool seen = false;
      for (uint8_t n = 0; n < idCount; ++n) {
        if (ids[n] == axis[k].id) { seen = true; break; }
      }
      if (!seen && idCount < N_MOTORS) ids[idCount++] = axis[k].id;
    }
  }
  if (gestureCount == 0) return 0;

  // -- 2. One batched position read for the whole set --------------------
  //
  // Per-motor reads would be up to N_MOTORS round trips on the Dynamixel bus
  // for a query a host may poll after every command; this path is the one the
  // fast-telemetry frame already uses.
  FastTelemetryRecord telem[N_MOTORS];
  uint8_t method = FAST_TELEM_METHOD_FAILED;
  uint8_t telemCount = exo_.getFastTelemetryRecords(ids, idCount, telem, method, 10);

  // -- 3. Project each motor back onto its own extend -> flex segment -----
  //
  // Exact inverse of setGestureAngle(): each motor is placed on the segment its
  // own two endpoints define, then the gesture's percentage is the mean of the
  // joints that carry information. Averaging PERCENTAGES rather than raw
  // fractions is what makes a multi-motor gesture read back the number that was
  // commanded, even though its joints travel different distances.
  uint8_t written = 0;
  for (uint8_t n = 0; n < gestureCount; ++n) {
    const int g = gestureIdx[n];
    const uint8_t nAxis = resolveGestureAxis(g, axis, N_MOTORS);

    float sum = 0.0f;
    uint8_t valid = 0;
    for (uint8_t k = 0; k < nAxis; ++k) {
      // A joint whose two endpoints coincide holds still across the whole
      // axis, so its position says nothing about the percentage -- and
      // dividing by that separation would blow up. Same for a motor with no
      // calibrated travel, which setGestureAngle() also refuses to command.
      const float separation = axis[k].flexFraction - axis[k].extendFraction;
      if (fabsf(separation) < GESTURE_AXIS_MIN_SEPARATION) continue;
      if (fabsf(exo_.getGestureSpan(axis[k].id)) < GESTURE_MIN_TRAVEL_DEG) continue;

      for (uint8_t r = 0; r < telemCount; ++r) {
        if (telem[r].id != axis[k].id || telem[r].error) continue;
        const float measured =
          exo_.gestureAngleToFraction(axis[k].id, telem[r].absolute_cdeg / 100.0f);
        sum += (measured - axis[k].extendFraction) / separation;
        ++valid;
        break;
      }
    }

    out[written].gesture = (uint8_t)g;
    out[written].signedAngleDeg = NAN;
    if (valid == 0) {
      out[written].code = GESTURE_ANGLE_UNAVAILABLE;
    } else {
      const float mean = sum / (float)valid;
      if (mean < -GESTURE_FRACTION_TOLERANCE) {
        out[written].code = GESTURE_ANGLE_BELOW_RANGE;
      } else if (mean > 1.0f + GESTURE_FRACTION_TOLERANCE) {
        out[written].code = GESTURE_ANGLE_ABOVE_RANGE;
      } else {
        // Inside tolerance but past an end: report the endpoint, not a code.
        // Backlash routinely leaves a settled joint a hair beyond 0 or 100.
        out[written].code =
          (uint8_t)lroundf(constrain(mean, 0.0f, 1.0f) * 100.0f);
      }

      // OpenSim-style signed angle: use the FIRST motor named by the gesture
      // as the physical reference, regardless of how many motors contribute
      // to the aggregate percentage. Convert the mean percentage back onto
      // that motor's gesture fraction, then measure physical travel from its
      // rest state. fabs(span) removes installation/flip direction; the sign
      // is the convention the protocol promises (toward flex positive, toward
      // extend negative), rather than an encoder-direction artifact.
      const GestureAxisPoint& reference = axis[0];
      const float referenceSpan = fabsf(exo_.getGestureSpan(reference.id));
      const float restToFlex = reference.flexFraction - reference.restFraction;
      if (!isnan(reference.restFraction) &&
          referenceSpan >= GESTURE_MIN_TRAVEL_DEG &&
          fabsf(restToFlex) >= GESTURE_AXIS_MIN_SEPARATION) {
        const float referenceFraction =
          reference.extendFraction +
          mean * (reference.flexFraction - reference.extendFraction);
        const float flexDirection = restToFlex > 0.0f ? 1.0f : -1.0f;
        out[written].signedAngleDeg =
          (referenceFraction - reference.restFraction) *
          flexDirection * referenceSpan;
      }
    }
    ++written;
  }
  return written;
}
void GestureController::setCycleGestureButton(const int pin) {
  cycleGesturePin = pin;
  pinMode(pin, INPUT_PULLUP);
  delay(100);  // Give pin state time to settle

  lastCycleGestureButtonState = HIGH;
  cycleGestureButtonState = HIGH;
  lastCycleGestureDebounceTime = 0;
  debugPrint("Gesture state switch button set on pin " + String(cycleGesturePin));
}
void GestureController::setGestureStateSwitchButton(const int pin) {
  gestureStateSwitchPin = pin;
  pinMode(pin, INPUT_PULLUP);
  delay(100);  // Give pin state time to settle

  lastGestureStateButtonState = HIGH;
  gestureStateButtonState = HIGH;
  lastGestureStateDebounceTime = 0;
  debugPrint("Gesture state switch button set on pin " + String(gestureStateSwitchPin));
}
void GestureController::setPinchCycleButton(int pin) {
  pinchCycleButtonPin_ = pin;
  pinMode(pinchCycleButtonPin_, INPUT_PULLUP);
  delay(100);

  lastPinchCycleButtonState = HIGH;
  pinchCycleButtonState = HIGH;
  lastPinchCycleDebounceTime = 0;
  debugPrint("[Pinch] Cycle button on pin " + String(pin));
}
void GestureController::setGestureButtonCallback(const String& gesture, const int pin) {
    if (gestureButtonCount_ >= MAX_GESTURE_BUTTONS) {
        debugPrint(F("[GestureController] Maximum gesture buttons reached, cannot add more."));
        return;
    }

    // Check if the gesture already exists
    for (int i = 0; i < gestureButtonCount_; ++i) {
        if (gestureButtons_[i].gestureName == gesture) {
        debugPrint("[GestureController] Gesture button for '" + gesture + "' already exists.");
        return;
        }
    }

    // Add new gesture button
    GestureButton& gb = gestureButtons_[gestureButtonCount_++];
    gb.pin = pin;
    gb.gestureName = gesture;
    gb.buttonState = HIGH; // Default state
    gb.lastButtonState = HIGH;
    gb.lastDebounceTime = 0;

    pinMode(pin, INPUT_PULLUP);
    debugPrint("Gesture button for '" + gesture + "' set on pin " + String(pin));
}
bool GestureController::checkGestureStateButtonPressed() {
  if (gestureStateSwitchPin == -1) return false;
  int reading = digitalRead(gestureStateSwitchPin);
  if (reading != lastGestureStateButtonState) {
    lastGestureStateDebounceTime = millis();
  }

  if ((millis() - lastGestureStateDebounceTime) > BUTTON_DEBOUNCE_DURATION) {
    if (reading != gestureStateButtonState) {
      gestureStateButtonState = reading;
      if (gestureStateButtonState == LOW) {
        // === Button was pressed ===
        return true;
      }
    }
  }
  lastGestureStateButtonState = reading;
  return false;
}
bool GestureController::checkCycleGestureButtonPressed() {
    if (cycleGesturePin == -1) return false;
    int reading = digitalRead(cycleGesturePin);
    if (reading != lastCycleGestureButtonState) {
        lastCycleGestureDebounceTime = millis();
    }

    if ((millis() - lastCycleGestureDebounceTime) > BUTTON_DEBOUNCE_DURATION) {
      if (reading != cycleGestureButtonState) {
          cycleGestureButtonState = reading;
          if (cycleGestureButtonState == LOW) {
            // === Button was pressed ===
            return true;
          }
        }
    }
    lastCycleGestureButtonState = reading;
    return false;
}
bool GestureController::checkPinchCycleButtonPressed() {
  if (pinchCycleButtonPin_ == -1) return false;
  int reading = digitalRead(pinchCycleButtonPin_);
  if (reading != lastPinchCycleButtonState) {
    lastPinchCycleDebounceTime = millis();
  }

  if ((millis() - lastPinchCycleDebounceTime) > BUTTON_DEBOUNCE_DURATION) {
    // if the reading has stabilized and changed from the stable state
    if (reading != pinchCycleButtonState) {
      pinchCycleButtonState = reading;         // update the stable state
      if (pinchCycleButtonState == LOW) {      // pressed on pull-up wiring
        lastPinchCycleButtonState = reading;   // keep these in sync
        return true;
      }
    }
  }
  lastPinchCycleButtonState = reading;
  return false;
}
void GestureController::cycleGesture() {
    // Cycle through the gestures
    debugPrint("Current gesture: " + currentGesture_);
    int gIdx = findGestureIndex(currentGesture_);
    if (gIdx == -1) {
        debugPrint(F("[GestureController] Error: current gesture not found."));
        return;
    }
    int newIdx = (gIdx + 1) % numGestures_;  // Cycle through gestures
    debugPrint("New gesture index: " + String(newIdx));
    if (newIdx == 0) {
        debugPrint(F("[GestureController] Wrapped back to first gesture."));
    }

    currentGesture_ = gestureLibrary[newIdx].name;

    if (gestureLibrary[newIdx].numStates > 0) {
      currentGestureState_ = gestureLibrary[newIdx].states[0].name;
      debugPrint("[GestureController] Cycling gesture to: '" + currentGesture_ +"' (index: " + String(newIdx) + "), state: '" + currentGestureState_ + "'");
      executeGesture(currentGesture_, currentGestureState_);
    } else {
      debugPrint("[GestureController] Gesture " + currentGesture_ + " has no states.");
    }
}
String GestureController::getCurrentGesture() {
    return currentGesture_;
}
String GestureController::getCurrentGestureState() {
    return currentGestureState_;
}
void GestureController::cycleGestureState() {
    // Cycle through the states of the current gesture
    int gIdx = findGestureIndex(currentGesture_);
    if (gIdx == -1) {
        debugPrint(F("[GestureController] Error: current gesture not found."));
        return;
    }    
    debugPrint("Gesture index: " + String(gIdx));
    int currentStateIdx = findStateIndex(gestureLibrary[gIdx], currentGestureState_);
    if (currentStateIdx == -1) {
        debugPrint("[GestureController] Error: state not found for gesture: " + currentGesture_);
        return;
    }

    int nextStateIdx = (currentStateIdx + 1) % gestureLibrary[gIdx].numStates;
    String newState = gestureLibrary[gIdx].states[nextStateIdx].name;

    debugPrint("[GestureController] Cycling state to: " + newState +
               " (index: " + String(nextStateIdx) + ")");
    executeGesture(currentGesture_, newState);
}
void GestureController::update() {
    // Check if the gesture state button was pressed
    if (checkCycleGestureButtonPressed()) {
        debugPrint(F("[GestureController] cycle gesture button pressed"));
        String exo_mode = exo_.getExoOperatingMode();
        flashPin(STATUS_LED_PIN, 100, 1);
        if (exo_mode == "GESTURE_FIXED" || exo_mode == "GESTURE_CONTINUOUS") {
            // toggle the gesture index, call executeGesture(...)
            cycleGesture();
        }
    }

    // Check if the gesture state button was pressed
    if (checkGestureStateButtonPressed()) {
        debugPrint(F("[GestureController] gesture state button pressed"));
        flashPin(STATUS_LED_PIN, 100, 1);
        String gesture = getCurrentGesture();
        if (exo_.getExoOperatingMode() == "GESTURE_FIXED" || exo_.getExoOperatingMode() == "GESTURE_CONTINUOUS") {
            // toggle the gesture state, call executeGesture(...)
            cycleGestureState();
        }
    }


    // === Check all gesture buttons ===
    for (int i = 0; i < gestureButtonCount_; ++i) {
    GestureButton& gb = gestureButtons_[i];
    int reading = digitalRead(gb.pin);

    if (reading != gb.lastButtonState) {
        gb.lastDebounceTime = millis();
    }

    // Check if the pinch gesture button was pressed
    if (checkPinchCycleButtonPressed()) {
        activePinchIdx_ = (activePinchIdx_ + 1) % 3;  // index->middle->ring->index
        const char* names[3] = { "pinch_index", "pinch_middle", "pinch_ring" };
        currentGesture_ = names[activePinchIdx_];
        flashPin(STATUS_LED_PIN, 100, activePinchIdx_ + 1);
        debugPrint("[GestureController] Gesture button pressed for: " + gb.gestureName + ", specific: " + currentGesture_);
        // re-apply current state so posture updates immediately
        executeGesture(currentGesture_, currentGestureState_);
    }


    if ((millis() - gb.lastDebounceTime) > BUTTON_DEBOUNCE_DURATION) {
        if (reading != gb.buttonState) {
            gb.buttonState = reading;
            if (gb.buttonState == LOW) {
                flashPin(STATUS_LED_PIN, 100, 1);
                debugPrint("[GestureController] Gesture button pressed for: " + gb.gestureName);

                int gIdx = findGestureIndex(gb.gestureName);
                if (gIdx != -1 && gestureLibrary[gIdx].numStates > 0) {
                    currentGesture_ = gestureLibrary[gIdx].name;
                    currentGestureState_ = gestureLibrary[gIdx].states[0].name;
                    executeGesture(currentGesture_, currentGestureState_);
                }
            }
        }
    }

    gb.lastButtonState = reading;
}

}


