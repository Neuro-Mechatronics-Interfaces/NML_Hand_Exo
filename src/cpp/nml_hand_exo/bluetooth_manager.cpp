#include "bluetooth_manager.h"
#include "config.h"
#include "utils.h"
#include "oled.h"

BluetoothManager::BluetoothManager(HardwareSerial& btSerial, int statePin, int enPin)
    : serial_(btSerial), statePin_(statePin), enPin_(enPin),
      connected_(false), lastPollMs_(0) {}

void BluetoothManager::begin() {
    if (statePin_ >= 0) {
        pinMode(statePin_, INPUT);
        connected_ = (digitalRead(statePin_) == HIGH);
    }
    if (enPin_ >= 0) {
        pinMode(enPin_, OUTPUT);
        digitalWrite(enPin_, LOW);  // Data mode (not AT mode)
    }
    debugPrint("[BT] BluetoothManager ready | STATE pin=" + String(statePin_)
               + " | EN pin=" + String(enPin_)
               + " | initial=" + (connected_ ? "connected" : "disconnected"));
}

void BluetoothManager::update() {
    if (statePin_ < 0) return;

    unsigned long now = millis();
    if (now - lastPollMs_ < POLL_INTERVAL_MS) return;
    lastPollMs_ = now;

    bool prev = connected_;
    connected_ = (digitalRead(statePin_) == HIGH);

    if (connected_ != prev) {
        if (connected_) {
            debugPrint(F("[BT] Host connected"));
            oledSetState(EXO_BT_CONNECTED);
        } else {
            debugPrint(F("[BT] Host disconnected"));
            oledSetState(EXO_BT_DISCONNECTED);
        }
    }
}

bool BluetoothManager::isConnected() const {
    if (statePin_ < 0) return false;
    return connected_;
}

String BluetoothManager::getStatus() const {
    return isConnected() ? "connected" : "disconnected";
}

void BluetoothManager::setStatePin(int pin) {
    statePin_ = pin;
    if (pin >= 0) {
        pinMode(pin, INPUT);
        connected_ = (digitalRead(pin) == HIGH);
    }
}

void BluetoothManager::setEnPin(int pin) {
    enPin_ = pin;
    if (pin >= 0) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}

bool BluetoothManager::runATConfig(const String& deviceName,
                                    const String& pinCode,
                                    uint32_t      dataBaud) {
    debugPrint(F("[BT] Starting AT configuration (mini-AT mode)..."));

    // Switch to HC-05 AT baud rate (38400 for full AT mode, same as data baud
    // for mini-AT while the module is not actively connected to a host).
    const uint32_t atBaud = (enPin_ >= 0) ? 38400 : dataBaud;
    serial_.end();
    serial_.begin(atBaud);
    delay(500);

    bool ok = true;
    ok &= sendAT("AT");                              // Sanity check
    ok &= sendAT("AT+NAME=" + deviceName);           // Device name
    ok &= sendAT("AT+PSWD=" + pinCode);              // PIN code
    ok &= sendAT("AT+UART=" + String(dataBaud) + ",0,0");  // Baud, stop bits, parity

    // Restore data-mode baud rate
    serial_.end();
    serial_.begin(dataBaud);
    delay(100);

    debugPrint(ok ? F("[BT] AT configuration complete") : F("[BT] AT configuration had errors"));
    return ok;
}

bool BluetoothManager::sendAT(const String& cmd,
                               const String& expectedResponse,
                               unsigned long timeoutMs) {
    // HC-05 AT commands need \r\n terminator
    serial_.print(cmd);
    serial_.print("\r\n");

    unsigned long start = millis();
    String resp = "";
    while (millis() - start < timeoutMs) {
        if (serial_.available()) {
            resp += (char)serial_.read();
            if (resp.indexOf(expectedResponse) >= 0) {
                debugPrint("[BT] AT \"" + cmd + "\" -> OK");
                return true;
            }
        }
    }
    debugPrint("[BT] AT \"" + cmd + "\" timed out, got: " + resp);
    return false;
}
