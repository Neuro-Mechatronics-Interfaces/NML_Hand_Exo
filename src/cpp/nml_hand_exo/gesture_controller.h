#ifndef GESTURE_CONTROLLER_H
#define GESTURE_CONTROLLER_H

#include "Arduino.h"
#include "nml_hand_exo.h"
#include "gesture_library.h"

/// @brief Structure to hold gesture button information
struct GestureButton {
    String gestureName;
    int pin;
    int lastButtonState;
    int buttonState;
    unsigned long lastDebounceTime;
};

/// @brief Class to manage predefined gestures and apply them to the NMLHandExo device.
class GestureController {
public:
    /// @brief Constructor
    /// @param exo Pointer to the NMLHandExo instance
    GestureController(NMLHandExo& exo);

    /// @brief Destructor
    ~GestureController() {
        // No dynamic memory to free, but can be extended if needed
    }

    /// @brief Execute a predefined gesture with a specific state
    /// @param gesture Name of the gesture to execute (e.g. "pinch")
    /// @param state State of the gesture (e.g., "a")
    void executeGesture(const String& gesture, const String& state);

    /// @brief Execute a predefined state with the current gesture
    /// @param state State of the gesture (e.g., "a")
    void executeCurrentGestureNewState(const String& state);

    /// @brief Drive a per-joint gesture to an arbitrary point in its range.
    ///
    /// Continuous generalization of the extend/rest/flex states: the percentage
    /// is a fraction of each named motor's calibrated range measured from home,
    /// in the flexion direction (flip-aware). 0 = home / extension endstop,
    /// 100 = the flexion endstop. Only gestures that define a "flex" state are
    /// addressable this way; multi-joint postures are not.
    /// @param gesture Name of a per-joint gesture (e.g. "index", "wrist")
    /// @param percent Position in [0, 100]; out-of-range values are clamped
    /// @return True if the gesture was found and commanded
    bool setGestureAngle(const String& gesture, float percent);

    /// @brief Assign pin for gesture state switch interrupt.
    /// @param pin Interrupt pin.
    void setGestureStateSwitchButton(const int pin);

    /// @brief Assign pin for gesture cycling switch interrupt.
    /// @param pin Interrupt pin.
    void setCycleGestureButton(const int pin);

    /// @brief Assign pin for pinch gesture cycling switch interrupt
    /// @param pin Interrupt pin.
    void setPinchCycleButton(int pin);

    /// @brief Assign a button pin to directly activate a named gesture.
    /// @param gesture Name of the gesture (must exist in gestureLibrary)
    /// @param pin Pin number for the button
    void setGestureButtonCallback(const String& gesture, const int pin);

    /// @brief Check if the gesture state button was pressed.
    /// @return button state
    bool checkGestureStateButtonPressed();

    /// @brief Update the button state and handle mode switching.
    /// @return button state
    bool checkCycleGestureButtonPressed();

    /// @brief Check if pinch gesture state button was pressed.
    /// @return button state
    bool checkPinchCycleButtonPressed();

    /// @brief Cycle through the exo operating modes.
    void cycleGesture();

    /// @brief Cycle through the current gesture state.
    void cycleGestureState();

    /// @brief Get the current gesture being executed.
    /// @return Current gesture name
    String getCurrentGesture();

    /// @brief Get the current gesture state being executed.
    /// @return Current gesture state name
    String getCurrentGestureState();

    /// @brief Update the gesture controller state, including checking for button presses.
    void update();


private:
    /// @brief Apply the gesture to the exoskeleton
    NMLHandExo& exo_;

    /// @brief Pointer to the gesture library
    int numGestures_;  // Number of gestures in the library

    /// @brief Current gesture being executed
    String currentGesture_ = "";  // Default gesture

    /// @brief Current gesture state
    String currentGestureState_ = "";

    /// @brief Mode switch pin
    int gestureStateSwitchPin = -1;

    /// @brief Pinch cycle pin
    int pinchCycleButtonPin_ = -1;

    /// @brief Mode switch flag triggered by the mode switch interrupt callback
    static volatile bool gestureStateSwitchFlag;

    /// @brief Last interrupt time for mode switch button
    bool lastGestureStateButtonState = false; // Last state of the mode switch button

    /// @brief Current state of the mode switch button
    bool gestureStateButtonState = HIGH;

    /// @brief Last state of the pinch switch button
    bool lastPinchCycleButtonState = HIGH;

    /// @brief Current state of the pinch switch button
    bool pinchCycleButtonState = HIGH;

    /// @brief Last debounce time for mode switch button
    unsigned long lastGestureStateDebounceTime = 0;

    /// @brief Last debounce time for pinch switch button
    unsigned long lastPinchCycleDebounceTime = 0;

    uint8_t activePinchIdx_ = 0;              // 0=index, 1=middle, 2=ring

    // @brief Gesture cycle pin
    int cycleGesturePin = -1;

    /// @brief Cycle gesture flag for interrupt callback
    static volatile bool cycleGestureFlag;

    /// @brief Last interrupt time for gesture switch button
    bool lastCycleGestureButtonState = false; // Last state of the mode switch button

    /// @brief Current state of the gesture switch button
    int cycleGestureButtonState = HIGH;

    /// @brief Last debounce time for mode switch button
    unsigned long lastCycleGestureDebounceTime = 0;

    /// @brief Array of gesture buttons
    GestureButton gestureButtons_[MAX_GESTURE_BUTTONS];

    /// @brief Number of gesture buttons configured
    int gestureButtonCount_ = 0;
};

#endif  // GESTURE_CONTROLLER_H
