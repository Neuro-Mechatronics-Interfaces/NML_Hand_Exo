/**
 * @file utils.h
 * @brief Header file for supporting function definitions
 *
 */
#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "nml_hand_exo.h"
#include "gesture_controller.h"
//#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_BNO055.h>

/// @brief Prints a debug message to the debug serial port (VERBOSE specific)
/// @param msg The message to print
void debugPrint(const String& msg);

/// @brief Prints a debug message to the debug serial port
/// @param msg The message to print
void commandPrint(const String& msg);

/// @brief Flashes a pin on and off for a specified duration and number of repetitions
/// @param pin The pin number to flash
/// @param durationMs The duration in milliseconds for which the pin should be on and off
/// @param repetitions The number of times to flash the pin
void flashPin(int pin, int durationMs, int repetitions);

/// @brief Extracts an argument from a string based on the index and delimiter
/// @param line The input string containing arguments
/// @param index The index of the argument to extract (0-based)
/// @param delimiter The character used to separate arguments in the string (default is ':')
/// @return The extracted argument as a String
String getArg(const String line, const int index, char delimiter);

/// @brief Extracts a motor ID from a string based on the index
/// @param exo The NMLHandExo instance containing motor IDs
/// @param line The input string containing motor IDs
/// @param index The index of the motor ID to extract (0-based)
/// @return The extracted motor ID as an integer
int getArgMotorID(NMLHandExo& exo, const String& line, const int index);

/// @brief Parses a message and executes the corresponding command on the exo device or gesture controller
/// @param exo The NMLHandExo instance to control
/// @param gc The GestureController instance to control gestures
/// @param token The input string containing the command to parse
void parseMessage(NMLHandExo& exo, GestureController& gc, Adafruit_BNO055& imu, String token);

/// @brief Initializes the IMU device 
/// @param imu The imu instance
//void initializeIMU(Adafruit_ISM330DHCX& imu);
//void initializeIMU(Adafruit_BNO055& imu);
bool initializeIMU(Adafruit_BNO055& imu);

/// @brief Sends an event request and prints out the data from the IMU
/// @param imu The imu instance
//void getIMUData(Adafruit_ISM330DHCX& imu); 
// void getIMUData(Adafruit_BNO055& imu);
void updateIMU(Adafruit_BNO055& imu);
String getIMUData(Adafruit_BNO055& imu);

/// @brief Returns the IMU yaw (heading) and also prints it via commandPrint
float getIMUYaw(Adafruit_BNO055& imu);

#endif
