#include "../nml_hand_exo/config.h"
#include "../nml_hand_exo/config.h"
#include "../nml_hand_exo/utils.h"
#include "../nml_hand_exo/nml_hand_exo.h"
#include "../nml_hand_exo/gesture_controller.h"
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <iostream>
#include <string>

TwoWire Wire;

int main() {
    NMLHandExo exo(MOTOR_IDS, N_MOTORS, jointLimits, HOME_STATES);
    exo.setMotorNames(MOTOR_NAMES);

    GestureController gc(exo);
    Adafruit_BNO055 imu;

    std::string line;
    while (std::getline(std::cin, line)) {
        if (!line.empty())
            parseMessage(exo, gc, imu, String(line.c_str()));
    }
}
