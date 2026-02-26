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
  resolveStateAngles(gestureLibrary[gIdx].states[sIdx], home, absAngles);

  if (st.isRelative) {
    // Scale normalized offsets by each motor's calibrated range,
    // then apply flip direction.
    for (int i = 0; i < exo_.getMotorCount(); ++i) {
      uint8_t id = exo_.getMotorIDByIndex(i);
      float fraction = absAngles[i] - home[i];  // 0.0–1.0 from resolveStateAngles
      float range = exo_.getMotorLimitMax(id) - exo_.getMotorLimitMin(id);
      if (exo_.isMotorFlipped(id)) {
        absAngles[i] = home[i] - fraction * range;
      } else {
        absAngles[i] = home[i] + fraction * range;
      }
    }
  }

  // Command absolute targets
  for (int i = 0; i < exo_.getMotorCount(); ++i) {
    uint8_t id = exo_.getMotorIDByIndex(i);
    debugPrint("[GestureController] motor " + String(id) + " -> abs " + String(absAngles[i], 2));
    exo_.setAbsoluteAngle(id, absAngles[i]);
  }

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


