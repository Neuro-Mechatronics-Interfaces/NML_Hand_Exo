#ifndef DYNAMIXEL2ARDUINO_H
#define DYNAMIXEL2ARDUINO_H
#include "Arduino.h"
#include <cstdint>
#include <map>

namespace ControlTableItem {
    constexpr uint8_t GOAL_CURRENT = 0;
    constexpr uint8_t PRESENT_CURRENT = 1;
    constexpr uint8_t CURRENT_LIMIT = 2;
    constexpr uint8_t PROFILE_VELOCITY = 3;
    constexpr uint8_t GOAL_VELOCITY = 4;
    constexpr uint8_t PRESENT_VELOCITY = 5;
    constexpr uint8_t PROFILE_ACCELERATION = 6;
    constexpr uint8_t BAUD_RATE = 7;
};

using namespace ControlTableItem;

enum OperatingMode {
    OP_CURRENT = 0,
    OP_VELOCITY = 1,
    OP_POSITION = 3,
    OP_EXTENDED_POSITION = 4,
    OP_CURRENT_BASED_POSITION = 5,
    OP_PWM = 16,
};

enum ParamUnit {
    UNIT_RAW = 0,
    UNIT_PERCENT,
    UNIT_RPM,
    UNIT_DEGREE,
    UNIT_MILLI_AMPERE
};

struct MotorState {
    bool torque = false;
    float position = 180.0f;
    OperatingMode op = OP_CURRENT_BASED_POSITION;
    int32_t controltable[16];
};
class Dynamixel2Arduino {
    std::map<uint8_t, MotorState> motors;
    MotorState& get(uint8_t id) {
        return motors[id];
    }
public:
    Dynamixel2Arduino(SerialClass, uint8_t) {}
    void begin(long) {}
    void reboot(uint8_t) {}
    void ping(uint8_t) {}
    void ledOn(uint8_t) {}
    void ledOff(uint8_t) {}
    void setPortProtocolVersion(float) {}
    void setOperatingMode(uint8_t id, OperatingMode op) {
        get(id).op = op;
    }
    void torqueOff(uint8_t id) {
        get(id).torque = false;
    }
    void torqueOn(uint8_t id) {
        get(id).torque = true;
    }
    void writeControlTableItem(uint8_t cti, uint8_t id, uint32_t val) {
        get(id).controltable[cti] = val;
    }
    uint32_t readControlTableItem(uint8_t cti, uint8_t id) {
        return get(id).controltable[cti];
    }
    bool getTorqueEnableStat(uint8_t id) {
        return get(id).torque;
    }
    void setGoalPosition(uint8_t id, float angle, uint8_t unit) {
        // assumes unit = Unit_degree
        // no real way of simulating force so it will just snap in place
        get(id).position = angle;
    }
    void setPresentPosition(uint8_t id, uint8_t unit, float angle) {
        // assumes unit = UNIT_DEGREE
        get(id).position = angle;
    }
    float getPresentPosition(uint8_t id, uint8_t unit) {
        // assumes unit = UNIT_DEGREE
        return get(id).position;
    }
};

#endif
