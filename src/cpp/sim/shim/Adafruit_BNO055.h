#ifndef ADAFRUIT_BNO055_H
#define ADAFRUIT_BNO055_H

#include "Adafruit_Sensor.h"

class Adafruit_BNO055 {
public: 
    enum vector_type_t { 
        VECTOR_EULER = 0, VECTOR_LINEARACCEL 
    };
    Adafruit_BNO055(int = 0, int = 0) {}
    bool begin(int = 0) { return true; }
    void setExtCrystalUse(bool) {}
    void getEvent(sensors_event_t* e, vector_type_t = VECTOR_EULER) {
        e->orientation = {0, 0, 0};
    }
    int getTemp() { return 22; }
};

#endif
