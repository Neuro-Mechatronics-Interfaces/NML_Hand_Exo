#ifndef ADAFRUIT_SENSOR_H
#define ADAFRUIT_SENSOR_H

struct sensors_event_t {
    struct { float x, y, z; } orientation;
};

#endif
