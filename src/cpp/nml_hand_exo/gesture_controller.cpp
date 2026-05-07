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
    for (uint8_t si = 0; si < N_HAND_SIDES; ++si) {
        currentGesture_[si]      = gestureLibrary[0][0].name;
        currentGestureState_[si] = gestureLibrary[0][0].states[0].name;
    }
    numGestures_ = N_GESTURES;
}

void GestureController::executeGesture(const String& gesture, const String& state,
                                       uint8_t side) {

  int gIdx = findGestureIndex(gesture);
  if (gIdx == -1) {
    debugPrint("[GestureController] Unknown gesture: " + gesture);
    return;
  }

  // Use side 0 for state-name lookup (structure is identical across all sides).
  int sIdx = findStateIndex(gestureLibrary[0][gIdx], state);
  if (sIdx == -1) {
    debugPrint("[GestureController] Unknown state: " + state + " for gesture " + gesture);
    return;
  }

  // For each motor, determine its side, skip if not targeted, then look up
  // the per-side fraction and compute the absolute target angle.
  for (int i = 0; i < exo_.getMotorCount(); ++i) {
    uint8_t id = exo_.getMotorIDByIndex(i);

    // Side 0 = left (IDs 1-9), side 1 = right (IDs 11-19).
    // In single-exo builds N_HAND_SIDES == 1 so motorSide is always 0.
    uint8_t motorSide = (N_HAND_SIDES > 1 && id > 9) ? 1 : 0;
    if (side != GESTURE_SIDE_ALL && motorSide != side) continue;

    const GestureState& st = gestureLibrary[motorSide][gIdx].states[sIdx];

    float fraction = 0.0f;
    if (st.isSparse) {
      // Match by motor name (MOTOR_NAMES[] index == loop index in dual mode).
      String motorName = String(MOTOR_NAMES[i]);
      motorName.toLowerCase();
      for (uint8_t k = 0; k < st.nPairs; ++k) {
        if (st.namedPairs[k].joint &&
            String(st.namedPairs[k].joint).equalsIgnoreCase(motorName)) {
          fraction = st.namedPairs[k].value;
          break;
        }
      }
    } else {
      fraction = st.jointAngles[i];
    }

    float home = exo_.getZeroAngle(id);
    float absAngle;
    if (st.isRelative) {
      float range = exo_.getMotorLimitMax(id) - exo_.getMotorLimitMin(id);
      absAngle = exo_.isMotorFlipped(id)
                 ? home - fraction * range
                 : home + fraction * range;
    } else {
      absAngle = fraction;
    }

    debugPrint("[GestureController] motor " + String(id) +
               " (side " + String(motorSide) + ") -> abs " + String(absAngle, 2));
    exo_.setAbsoluteAngle(id, absAngle);
  }

  // Update per-side gesture tracking for the affected sides.
  uint8_t startSide = (side == GESTURE_SIDE_ALL) ? 0              : side;
  uint8_t endSide   = (side == GESTURE_SIDE_ALL) ? N_HAND_SIDES   : side + 1;
  for (uint8_t si = startSide; si < endSide; ++si) {
    currentGesture_[si]      = gesture;
    currentGestureState_[si] = state;
  }

  debugPrint("[GestureController] Executed gesture: " + gesture + ", state: " + state);

  // OLED: reflect the left/only-side state.
  oledSetState(mapGestureStateToExoState(currentGesture_[0], currentGestureState_[0]));
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
  int sIdx = findStateIndex(gestureLibrary[0][gIdx], state);
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
    debugPrint("Current gesture: " + currentGesture_[0]);
    int gIdx = findGestureIndex(currentGesture_[0]);
    if (gIdx == -1) {
        debugPrint(F("[GestureController] Error: current gesture not found."));
        return;
    }
    int newIdx = (gIdx + 1) % numGestures_;  // Cycle through gestures
    debugPrint("New gesture index: " + String(newIdx));
    if (newIdx == 0) {
        debugPrint(F("[GestureController] Wrapped back to first gesture."));
    }

    for (uint8_t si = 0; si < N_HAND_SIDES; ++si)
        currentGesture_[si] = gestureLibrary[0][newIdx].name;

    if (gestureLibrary[0][newIdx].numStates > 0) {
      for (uint8_t si = 0; si < N_HAND_SIDES; ++si)
          currentGestureState_[si] = gestureLibrary[0][newIdx].states[0].name;
      debugPrint("[GestureController] Cycling gesture to: '" + currentGesture_[0] +"' (index: " + String(newIdx) + "), state: '" + currentGestureState_[0] + "'");
      executeGesture(currentGesture_[0], currentGestureState_[0]);
    } else {
      debugPrint("[GestureController] Gesture " + currentGesture_[0] + " has no states.");
    }
}
String GestureController::getCurrentGesture() {
    return currentGesture_[0];
}
String GestureController::getCurrentGestureState() {
    return currentGestureState_[0];
}
void GestureController::cycleGestureState() {
    // Cycle through the states of the current gesture
    int gIdx = findGestureIndex(currentGesture_[0]);
    if (gIdx == -1) {
        debugPrint(F("[GestureController] Error: current gesture not found."));
        return;
    }
    debugPrint("Gesture index: " + String(gIdx));
    int currentStateIdx = findStateIndex(gestureLibrary[0][gIdx], currentGestureState_[0]);
    if (currentStateIdx == -1) {
        debugPrint("[GestureController] Error: state not found for gesture: " + currentGesture_[0]);

        return;
    }

    int nextStateIdx = (currentStateIdx + 1) % gestureLibrary[0][gIdx].numStates;
    String newState = gestureLibrary[0][gIdx].states[nextStateIdx].name;

    debugPrint("[GestureController] Cycling state to: " + newState +
               " (index: " + String(nextStateIdx) + ")");
    executeGesture(currentGesture_[0], newState);
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
        for (uint8_t si = 0; si < N_HAND_SIDES; ++si)
            currentGesture_[si] = names[activePinchIdx_];
        flashPin(STATUS_LED_PIN, 100, activePinchIdx_ + 1);
        debugPrint("[GestureController] Gesture button pressed for: " + gb.gestureName + ", specific: " + currentGesture_[0]);
        // re-apply current state so posture updates immediately
        executeGesture(currentGesture_[0], currentGestureState_[0]);
    }


    if ((millis() - gb.lastDebounceTime) > BUTTON_DEBOUNCE_DURATION) {
        if (reading != gb.buttonState) {
            gb.buttonState = reading;
            if (gb.buttonState == LOW) {
                flashPin(STATUS_LED_PIN, 100, 1);
                debugPrint("[GestureController] Gesture button pressed for: " + gb.gestureName);

                int gIdx = findGestureIndex(gb.gestureName);
                if (gIdx != -1 && gestureLibrary[0][gIdx].numStates > 0) {
                    for (uint8_t si = 0; si < N_HAND_SIDES; ++si) {
                        currentGesture_[si]      = gestureLibrary[0][gIdx].name;
                        currentGestureState_[si] = gestureLibrary[0][gIdx].states[0].name;
                    }
                    executeGesture(currentGesture_[0], currentGestureState_[0]);
                }
            }
        }
    }

    gb.lastButtonState = reading;
}

}


