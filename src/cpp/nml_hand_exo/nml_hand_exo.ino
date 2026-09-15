/*
MIT License

Copyright (c) 2026 Jonathan Shulgach

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

#include "io_profile.h"
#include "io_profile.h"
#include "config.h"
#include "utils.h"
#include "oled.h"
#include "nml_hand_exo.h"
#include "gesture_controller.h"
//#include <Adafruit_ISM330DHCX.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#if EXO_AXON_USB
#include "AxonUsbPeripheral.h"
// Must plug before the core's SerialUSB: Axon = interface 0, CDC = 1/2.
axon_exo::AxonUsbPeripheral gAxon __attribute__((init_priority(101)));
#endif

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
  // Pulse cutoffs run before host traffic or peripheral work.
  exo.serviceRomCalibration();
#if EXO_AXON_USB
  gAxon.poll([](void* context, uint8_t id,
                axon_exo::AxonUsbPeripheral::Field field,
                axon_exo::MotorSample& sample) {
    // One bounded read-only Dynamixel transaction per call; the peripheral
    // staggers the fields across passes so no single pass blocks. Torque is
    // derived from the current already sampled this cycle (no extra bus read),
    // so it never issues its own transaction.
    auto* exo = static_cast<NMLHandExo*>(context);
    using Field = axon_exo::AxonUsbPeripheral::Field;
    if (exo->telemetryEstimated()) return false; // Axon has no estimated-source flag.
    if (exo->getIndexById(id) < 0) return false;  // not a motor of this build
    if (field == Field::kAngle) {
      // The angle register still reads in every mode, but it is only a
      // controlled/commanded quantity in a position-holding mode. In VELOCITY or
      // pure CURRENT the shaft is free, so read it but report it unavailable so a
      // consumer never treats an uncontrolled position as commanded state.
      return exo->readAxonAngle(id, sample.angle) && exo->modeControlsPosition();
    }
    if (field == Field::kCurrent) {
      // getCurrent returns PRESENT_CURRENT in mA (~1 mA/raw unit). A bus error
      // yields the Dynamixel library's sentinel; the id was validated above so
      // a known motor that reads back is treated as measured. Current is only a
      // controlled quantity in a current-driving mode; in POSITION/VELOCITY the
      // loop draws whatever it needs, so mark it unavailable there.
      sample.current_mA = exo->getCurrent(id);
      return exo->modeControlsCurrent();
    }
    // kTorque: pure arithmetic from the current sampled this cycle; valid only
    // if that current read was itself a valid (mode-controlled) measurement.
    sample.torque_Nm = sample.current_mA * XC330_T288_TORQUE_CONSTANT;
    return sample.current_ok;
  }, &exo);
#endif
  // Record iteration timing first: the loop period bounds how fast any command
  // can possibly be picked up. Query with `loop_stats`.
  loopStatsTick();

  // Handle data from the host COMMAND connection (primary USB CDC).
  // In dual-CDC mode CMD_SERIAL is the command port; in single-CDC fallback it
  // resolves to DEBUG_SERIAL (legacy behavior).
  // Handle at most one complete line per interface per pass. Parsing a command
  // can perform I/O even though LineReader is non-blocking. Service ROM between
  // replies so a continuous stream of requests cannot starve its deadlines.
  String input;

  if (gCmdLines.poll(CMD_SERIAL, input)) {
    debugPrint("Received: " + input);
    parseMessage(exo, gc, bno055, input);
    exo.serviceRomCalibration();
  }

#if defined(DUAL_CDC) && DUAL_CDC
  // Also accept commands on the telemetry CDC. This keeps a LEGACY single-port
  // host working no matter which of the two COM ports it opened: either CDC
  // accepts commands, and replies mirror back per the default BOTH route.
#if EXO_USB_PROTOBUF
  pollProtobufUsb(exo, gc);
  exo.serviceRomCalibration();
#else
  if (gTelemLines.poll(TELEM_SERIAL, input)) {
    debugPrint("Received: " + input);
    parseMessage(exo, gc, bno055, input);
    exo.serviceRomCalibration();
  }
#endif
#endif

  // Handle data from the BLE/command connection
#if defined(BT_SKIP)
  // then we skip polling BT lines
#else
  if (gBtLines.poll(COMMAND_SERIAL, input)) {
    debugPrint("Received: " + input);
    parseMessage(exo, gc, bno055, input, true);
    exo.serviceRomCalibration();
  }
#endif

  //updateIMU(bno055); //constantly updating the IMU values. Keeps position consistent

  // Update the exo state, including checking for button pressed, mode switching, and internal routines
  exo.update();

  // Check for any updates needed with the gesture controller
  if (!exo.isRomCalibrating()) {
    EXO_PROFILE(PERIPHERAL);
    EXO_PROFILE(PERIPHERAL);
    gc.update();
    oledTick();
  }

}
