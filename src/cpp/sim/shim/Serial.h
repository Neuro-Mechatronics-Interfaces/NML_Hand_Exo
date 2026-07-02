#ifndef SERIAL_H
#define SERIAL_H

#include <iostream>

#include "Stream.h"
#include "String.h"

class SerialClass : public Stream {

};
class StdoutSerial : public SerialClass {
public: 
    void println(const String &s) const { std::cout << s.c_str() << '\n' << std::flush; }
    void println(const char* s) const { std::cout << s << '\n' << std::flush; }
    void println(int n) const { std::cout << n << '\n' << std::flush; }
    void print(const String &s) const { std::cout << s.c_str() << std::flush; }
    void print(const char* s) const { std::cout << s << std::flush; }
    void print(int n) const { std::cout << n << std::flush; }
};
class StderrSerial : public SerialClass {
public: 
    void println(const String &s) const { std::cerr << s.c_str() << '\n' << std::flush; }
    void println(const char* s) const { std::cerr << s << '\n' << std::flush; }
    void println(int n) const { std::cerr << n << '\n' << std::flush; }
    void print(const String &s) const { std::cerr << s.c_str() << std::flush; }
    void print(const char* s) const { std::cerr << s << std::flush; }
    void print(int n) const { std::cerr << n << std::flush; }
};

inline StderrSerial Serial;
inline StderrSerial Serial1;
inline StderrSerial Serial2;
inline StdoutSerial Serial3;

#endif
