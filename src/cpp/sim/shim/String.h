#ifndef STRING_H
#define STRING_H
#include <string>
#include <cstdint>
#include <sstream>
#include <algorithm>
#include <iomanip>

class __FlashStringHelper;
#define F(x) (reinterpret_cast<const __FlashStringHelper*>(x))

class String {
    std::string s;
public: 
    String() {}
    String(std::string s) : s(s) {}
    String(const char* c) : s(c ? c : "") {}
    String(int n) : s(std::to_string(n)) {}
    String(unsigned int n) : s(std::to_string(n)) {}
    String(long n) : s(std::to_string(n)) {}
    String(unsigned long n) : s(std::to_string(n)) {}
    String(float n) : s(std::to_string(n)) {}
    String(long long n) : s(std::to_string(n)) {}
    String(unsigned long long n) : s(std::to_string(n)) {}
    String(float n, uint8_t d) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(d) << n;
        s = stream.str();
    }
    String(const __FlashStringHelper* f) : s(reinterpret_cast<const char*>(f)) {}
    String operator+(const String &other) const { return String(s + other.s); }
    String& operator+=(const String &other) { s += other.s; return *this; }
    bool operator==(const String &other) const { return s == other.s; }
    bool operator!=(const String &other) const { return s != other.s; }
    bool operator==(const char* c) const { return s == c; }
    bool operator!=(const char* c) const { return s != c; }
    char& operator[](const uint8_t i) { return s[i]; }

    const char* c_str() const { return s.c_str(); }
    int toInt() const { return s.size() ? std::stoi(s) : 0; }
    float toFloat() const { return s.size() ? std::stof(s) : 0; }
    void toUpperCase() { std::transform(s.begin(), s.end(), s.begin(), ::toupper); }
    void toLowerCase() { std::transform(s.begin(), s.end(), s.begin(), ::tolower); }
    size_t length() const { return s.length(); }
    bool equals(const String &other) const { return s == other.s; }
    int indexOf(const char c, const uint8_t from = 0) const {
        auto pos = s.find(c, from);
        return pos == std::string::npos ? -1 : pos;
    }
    int indexOf(const char* c, const uint8_t from = 0) const {
        auto pos = s.find(c, from);
        return pos == std::string::npos ? -1 : pos;
    }
    String substring(uint8_t l, uint8_t r) const { return s.substr(l, r - l); }
    void trim() {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        }));

        s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), s.end());
    }
    bool equalsIgnoreCase(const String &other) const {
        return other.s.size() == s.size() && std::equal(s.begin(), s.end(), other.s.begin(), [](unsigned char c1, unsigned char c2) {
            return std::tolower(c1) == std::tolower(c2);
        });
    }
    bool endsWith(const String& other) const {
        if (s.size() < other.s.size()) return false;
        return s.rfind(other.s) == (s.size() - other.s.size());
    }
};

inline String operator+(String s, const char* c) { return s + String(c); }
inline String operator+(const char* c, String s) { return String(c) + s; }
#endif
