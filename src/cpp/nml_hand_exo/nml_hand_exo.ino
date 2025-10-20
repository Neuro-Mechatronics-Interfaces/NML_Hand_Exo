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

// Create IMU device (The "ISM330DLC" library can be downloaded from Arduino's Library Manager)
//Adafruit_ISM330DHCX ism330dhcx; //deprecated
Adafruit_BNO055 bno055;  //= Adafruit_BNO055(55, 0x28)

// TO-DO: Move these to config.h or nml_hand_exo.h
//#define DEBUG_SERIAL Serial
//#define COMMAND_SERIAL Serial2

// Create the exo device with the motor parameters and id values
NMLHandExo exo(MOTOR_IDS, N_MOTORS, jointLimits, HOME_STATES);
GestureController gc(exo);  // pass exo reference


// OLED display instance
//volatile bool gOledEnabled = OLED_ENABLED_DEFAULT;
//Adafruit_SSD1306 gDisplay(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, -1);

void setup() {

  // LEDs for command/connection feedback
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);  // initially off

  // Serial connections
  DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);    // Setting a default baud rate of 57600
  COMMAND_SERIAL.begin(COMMAND_BAUD_RATE);     // (Optional) Establish port with TX/RX pins for incomming serial data/commands

  // Setup IMU
  //initializeIMU(ism330dhcx);
  initializeIMU(bno055);

  // Setup OLED
  oledInit();
  oledSetState(EXO_READY);

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
  // Handle data from the debug connection
  if (DEBUG_SERIAL.available() > 0) {
    String input = DEBUG_SERIAL.readStringUntil('\n');
    input.trim();
    debugPrint("Received: " + input);
    parseMessage(exo, gc, bno055, input);
  }

  // Handle data from the BLE/command connection
  if (COMMAND_SERIAL.available() > 0) {
    String input = COMMAND_SERIAL.readStringUntil('\n');
    input.trim();
    debugPrint("Received: " + input);
    parseMessage(exo, gc, bno055, input);
  }

  //updateIMU(bno055); //constantly updating the IMU values. Keeps position consistent

  // Update the exo state, including checking for button pressed, mode switching, and internal routines
  exo.update();

  // Check for any updates needed with the gesture controller
  gc.update();

  oledTick();

}
