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

/// @brief Percentage code meaning the joint sits below its calibrated travel.
constexpr uint8_t GESTURE_ANGLE_BELOW_RANGE = 101;

/// @brief Percentage code meaning the joint sits above its calibrated travel.
constexpr uint8_t GESTURE_ANGLE_ABOVE_RANGE = 102;

/// @brief Percentage code meaning no position is available for this gesture.
///
/// Either every motor it names has less than GESTURE_MIN_TRAVEL_DEG of
/// calibrated travel, or their positions could not be read. `check_limits`
/// distinguishes the two.
constexpr uint8_t GESTURE_ANGLE_UNAVAILABLE = 255;

/// @brief Where one gesture currently sits on its percent and signed-degree axes.
///
/// The signed angle uses the first motor named by the gesture as its physical
/// reference. Its `rest` posture is 0 degrees, motion from rest toward `flex`
/// is positive, and motion from rest toward `extend` is negative. The value is
/// a physical delta derived from that motor's calibrated joint-limit span;
/// NAN means the signed angle is unavailable.
struct GestureAngleRecord {
    uint8_t gesture;       ///< Index into gestureLibrary.
    uint8_t code;          ///< 0-100 percent, or one of the codes above.
    float signedAngleDeg;  ///< Signed physical delta from rest, in degrees.
};

/// @brief One motor's endpoints and rest anchor on a gesture's percent axis.
///
/// The percentage interpolates a gesture between its own `extend` and `flex`
/// postures, so every motor it names needs BOTH of its endpoints, not a single
/// travel fraction. Keeping them together is what lets a multi-motor gesture
/// hold the ratio between its joints at every percentage, and what makes the
/// read-back an exact inverse.
struct GestureAxisPoint {
    uint8_t id;             ///< Dynamixel ID.
    float extendFraction;   ///< Position at 0%, from the gesture's extend state.
    float restFraction;     ///< Position defined by the gesture's rest state.
    float flexFraction;     ///< Position at 100%, from its flex state.
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
    /// interpolates the gesture between its OWN two endpoint postures.
    ///
    ///   0   -> exactly `set_gesture:<gesture>:extend`
    ///   100 -> exactly `set_gesture:<gesture>:flex`
    ///
    /// Each named motor moves along its own extend -> flex segment, so a
    /// gesture driving several motors keeps the ratio between them at every
    /// percentage. Each segment is then placed on that motor's calibrated
    /// travel by NMLHandExo::gestureFractionToAngle(). Only gestures that
    /// define a "flex" state are addressable this way; multi-joint postures
    /// are not.
    /// @param gesture Name of a per-joint gesture (e.g. "index", "wrist")
    /// @param percent Position in [0, 100]; out-of-range values are clamped
    /// @param movedOut Optional: motors actually commanded.
    /// @param stuckOut Optional: motors skipped for having no calibrated
    ///        travel. Non-zero here is the difference between "the firmware
    ///        accepted this" and "the joint can move", which an OK ack alone
    ///        cannot express.
    /// @return True if the gesture was found and commanded
    bool setGestureAngle(const String& gesture, float percent,
                         uint8_t* movedOut = nullptr,
                         uint8_t* stuckOut = nullptr);

    /// @brief Position a gesture from a SIGNED value anchored at its rest pose.
    ///
    /// The rest-anchored sibling of setGestureAngle(): where that command's 0
    /// is the extend posture, this one's 0 is the gesture's calibrated REST
    /// posture, so the sign carries direction the way the continuous decoder
    /// stream does -- positive toward flex, negative toward extend.
    ///
    ///   signed = -100  ->  the gesture's extend posture
    ///   signed =    0  ->  its rest posture (per motor, from the rest state)
    ///   signed = +100  ->  its flex posture
    ///
    /// Each motor interpolates through its OWN rest fraction, so a gesture that
    /// drives several motors (the thumb, the coupled wrist pair) keeps every
    /// motor on its calibrated rest at signed 0 even when their rests do not
    /// line up on a single percentage. A motor whose gesture defines no rest
    /// state (restFraction is NaN) falls back to the linear extend<->flex axis,
    /// i.e. it behaves as setGestureAngle((signed + 100) / 2) for that motor.
    ///
    /// @param gesture Angle-addressable gesture name.
    /// @param signedValue Position in [-100, 100]; clamped to that range.
    /// @param movedOut Optional out: motors actually commanded.
    /// @param stuckOut Optional out: motors skipped for having no travel.
    /// @return true if the gesture is angle-addressable, false otherwise.
    bool setGestureSignedAngle(const String& gesture, float signedValue,
                               uint8_t* movedOut = nullptr,
                               uint8_t* stuckOut = nullptr);

    /// @brief Sample where the angle-addressable gestures currently sit.
    ///
    /// The read-back half of setGestureAngle(): both use the same extend ->
    /// flex axis, so a gesture commanded to 40 reports 40 back once it
    /// arrives. A gesture spanning several motors (the thumb, the wrist pair,
    /// or any joint in a dual build) reports the mean of the per-motor
    /// percentages -- averaging percentages, not raw travel fractions, is what
    /// makes that hold when its joints travel different distances.
    ///
    /// 0% is the `extend` posture, NOT home. A hand parked at home therefore
    /// reads below 0 (code 101) whenever the extend constants are non-zero.
    ///
    /// Each record also carries a signed physical angle derived from the first
    /// motor in the gesture: rest is 0 degrees, flexion is positive, extension
    /// is negative. Positions come from ONE batched Dynamixel read rather than
    /// a read per motor, so polling any reply form costs one bus transaction.
    ///
    /// @param out Destination array.
    /// @param maxRecords Capacity of @p out.
    /// @param only Optional single gesture name; empty means every addressable
    ///        gesture, in gestureLibrary order.
    /// @return Number of records written. 0 with a non-empty @p only means the
    ///         gesture is unknown or not angle-addressable.
    uint8_t readGestureAngles(GestureAngleRecord* out, uint8_t maxRecords,
                              const String& only = String());

    /// @brief Resolve one gesture's percent axis and rest anchor per motor.
    ///
    /// Shared by setGestureAngle() and readGestureAngles() so the command and
    /// its read-back cannot disagree about where 0 and 100 are.
    /// @param gestureIndex Index into gestureLibrary.
    /// @param out Destination array.
    /// @param maxPoints Capacity of @p out; N_MOTORS is always enough.
    /// @return Number of points written; 0 if the gesture is not addressable.
    uint8_t resolveGestureAxis(int gestureIndex, GestureAxisPoint* out,
                               uint8_t maxPoints);

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
