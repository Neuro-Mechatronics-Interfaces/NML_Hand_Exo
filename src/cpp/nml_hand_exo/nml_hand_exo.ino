/*
MIT License

Copyright (c) 2025 Jonathan Shulgach

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "config.h"
#include "utils.h"
#include "oled.h"
#include "nml_hand_exo.h"
#include "gesture_controller.h"
//#include <Adafruit_ISM330DHCX.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Create IMU device (The Adafruit_BNO055 library can be downloaded from Arduino's Library Manager)
Adafruit_BNO055 bno055;  //= Adafruit_BNO055(55, 0x28)

#if defined(DUAL_CDC) && DUAL_CDC
// Second native USB-CDC (ACM) interface: telemetry / command replies.
// Plugs into the core's PluggableUSB list right after the primary Serial,
// taking USB interface 2 and endpoints 4-6 automatically. See config.h.
Serial_ SerialTelem(USBDevice);
#endif

// TO-DO: Move these to config.h or nml_hand_exo.h
//#define DEBUG_SERIAL Serial
//#define COMMAND_SERIAL Serial2

// Create the exo device with the motor parameters and id values
NMLHandExo exo(MOTOR_IDS, N_MOTORS, jointLimits, HOME_STATES);
GestureController gc(exo);  // pass exo reference

/// @brief Longest accepted command line; longer input is discarded.
static constexpr uint16_t MAX_COMMAND_LEN = 192;

/// @brief Non-blocking line assembler for one input stream.
///
/// Replaces Stream::readStringUntil('\n'), which BLOCKS until the terminator
/// arrives or Stream::_timeout expires -- 1000 ms by default, and setTimeout()
/// was never called anywhere in this firmware.  available()>0 only promises a
/// single byte, so any command split across USB packets (routine on CDC) parked
/// the entire control loop in that blocking read.  Worse, an unconnected UART
/// with a floating RX line reports bytes available but never delivers a '\n',
/// stalling a full second every iteration.
///
/// poll() consumes only bytes that are already buffered and returns
/// immediately, so the loop can service every interface every pass.
struct LineReader {
  String buf;
  bool overflowed = false;

  /// @brief Drain buffered bytes; returns true and fills `out` on a full line.
  bool poll(Stream& stream, String& out) {
    while (stream.available() > 0) {
      char c = (char)stream.read();
      if (c == '\n') {
        if (overflowed) {
          // Discard the whole over-length line rather than emitting its tail
          // as if it were a command.
          buf = "";
          overflowed = false;
          continue;
        }
        out = buf;
        buf = "";
        out.trim();
        if (out.length() > 0) return true;
        continue;  // blank line: skip it and keep draining this pass
      }
      if (c != '\r') buf += c;
      if (buf.length() > MAX_COMMAND_LEN) {  // runaway-input guard
        buf = "";
        overflowed = true;
      }
    }
    return false;
  }
};

static LineReader gCmdLines;    // primary USB CDC
static LineReader gTelemLines;  // second USB CDC (also accepts commands)
static LineReader gBtLines;     // HC-05 / UART command path


// OLED display instance
//volatile bool gOledEnabled = OLED_ENABLED_DEFAULT;
//Adafruit_SSD1306 gDisplay(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, -1);

void setup() {

  // LEDs for command/connection feedback
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);  // initially off

  // Serial connections
  DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
  COMMAND_SERIAL.begin(COMMAND_BAUD_RATE);     // (Optional) Establish port with TX/RX pins for incomming serial data/commands
#if defined(DUAL_CDC) && DUAL_CDC
  SerialTelem.begin(DEBUG_BAUD_RATE);     // Second USB-CDC for telemetry / replies (baud is nominal for CDC)
#endif

  // Command input is assembled non-blocking by LineReader, so nothing here
  // should ever sit in a Stream read. This caps the damage if some future path
  // does call a blocking Stream helper: Arduino's default is 1000 ms.
  DEBUG_SERIAL.setTimeout(5);
  COMMAND_SERIAL.setTimeout(5);
#if defined(DUAL_CDC) && DUAL_CDC
  SerialTelem.setTimeout(5);
#endif

  // Setup IMU
  //initializeIMU(ism330dhcx);
  //initializeIMU(bno055);

  // Setup OLED with startup animation
  if (oledInit()) {
    oledSetState(EXO_READY);
    oledStartupAnimation();  // Show brain animation for ~3 seconds, then "READY"
  }

  // Setup exo
  exo.initializeSerial(DYNAMIXEL_BAUD_RATE);
  exo.initializeMotors();       // Initialize motors and set them to "current position" mode
  // exo.resetAllZeros();       // (Optional) Defines the current position of the motors as the home position
  exo.setMotorNames(MOTOR_NAMES);
  //exo.setModeSwitchButton(MODESWITCH_PIN);

  // Default state is button control
  exo.setExoOperatingMode(DEFAULT_EXO_MODE);

  // Setup gesture controller
  //gc.setCycleGestureButton(CYCLE_GESTURE_PIN);
  gc.setGestureStateSwitchButton(CYCLE_GESTURE_STATE_PIN);
  gc.setGestureButtonCallback("grasp", GESTURE_GRASP_BUTTON_PIN);
  gc.setGestureButtonCallback("keygrip", GESTURE_KEYGRIP_BUTTON_PIN);
  //gc.setGestureButtonCallback("pinch", GESTURE_PINCH_BUTTON_PIN); // Old button
  gc.setPinchCycleButton(GESTURE_PINCH_BUTTON_PIN);

  // Flash LEDs to let user know system ready to go
  flashPin(STATUS_LED_PIN, 100, 4);
  debugPrint(F("Exo device ready to receive commands"));
}

void loop() {
  // Record iteration timing first: the loop period bounds how fast any command
  // can possibly be picked up. Query with `loop_stats`.
  loopStatsTick();

  // Handle data from the host COMMAND connection (primary USB CDC).
  // In dual-CDC mode CMD_SERIAL is the command port; in single-CDC fallback it
  // resolves to DEBUG_SERIAL (legacy behavior).
  // Each while-loop drains every complete line already buffered on that
  // interface, so a burst is handled in one pass instead of one line per
  // iteration. None of these calls block.
  String input;

  while (gCmdLines.poll(CMD_SERIAL, input)) {
    debugPrint("Received: " + input);
    parseMessage(exo, gc, bno055, input);
  }

#if defined(DUAL_CDC) && DUAL_CDC
  // Also accept commands on the telemetry CDC. This keeps a LEGACY single-port
  // host working no matter which of the two COM ports it opened: either CDC
  // accepts commands, and replies mirror back per the default BOTH route.
  while (gTelemLines.poll(TELEM_SERIAL, input)) {
    debugPrint("Received: " + input);
    parseMessage(exo, gc, bno055, input);
  }
#endif

  // Handle data from the BLE/command connection
#if defined(BT_SKIP)
  // then we skip polling BT lines
#else
  while (gBtLines.poll(COMMAND_SERIAL, input)) {
    debugPrint("Received: " + input);
    parseMessage(exo, gc, bno055, input);
  }
#endif

  //updateIMU(bno055); //constantly updating the IMU values. Keeps position consistent

  // Update the exo state, including checking for button pressed, mode switching, and internal routines
  exo.update();

  // Check for any updates needed with the gesture controller
  gc.update();

  oledTick();

}
