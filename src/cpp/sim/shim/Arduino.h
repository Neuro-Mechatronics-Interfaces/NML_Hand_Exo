#ifndef ARDUINO_H
#define ARDUINO_H

#include <cstdint>
#include <chrono>
#include <thread>

#define ARDUINO_OpenRB

#include "Stream.h"
#include "Serial.h"

#define LOW 0x0
#define HIGH 0x1

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define LED_BUILTIN 13

// ignoring pins because no real good way of simulating them
inline void pinMode(uint8_t pin, uint8_t mode) {}
inline void digitalWrite(uint8_t pin, uint8_t val) {}
inline int digitalRead(uint8_t pin) { return HIGH; }

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define sq(x) ((x)*(x))

inline unsigned long millis() {
    static auto start = std::chrono::steady_clock::now();
    return (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
}
inline unsigned long micros() {
    static auto start = std::chrono::steady_clock::now();
    return (unsigned long)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
}

inline void delay(unsigned long t) {
    std::this_thread::sleep_for(std::chrono::milliseconds(t));
}

#endif
