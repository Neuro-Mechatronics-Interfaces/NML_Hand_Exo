#pragma once
#include <Arduino.h>
#ifndef BT_PIN_CODE
    #define BT_PIN_CODE     "1234"
#endif

// Manages an HC-05 (or compatible) UART Bluetooth Classic module.
//
// STATE pin: HIGH when a Bluetooth host is actively connected.
//            Pass -1 if not wired — isConnected() always returns false.
// EN pin:    Pull HIGH *before* module power-on to enter full AT config mode.
//            Pass -1 if not wired — runATConfig() will warn and bail.
//
// Typical wiring for OpenRB-150:
//   HC-05 TX  → D14 (Serial3 RX)
//   HC-05 RX  → D13 (Serial3 TX)
//   HC-05 VCC → 3.3 V or 5 V (check your module)
//   HC-05 GND → GND
//   HC-05 STATE → any free digital pin  (define as BT_STATE_PIN in config.h)
//   HC-05 EN    → any free digital pin  (define as BT_EN_PIN   in config.h)
//
// Usage in sketch:
//   BluetoothManager bt(COMMAND_SERIAL, BT_STATE_PIN, BT_EN_PIN);
//   void setup() { ... bt.begin(); }
//   void loop()  { ... bt.update(); }

class BluetoothManager {
public:
    // btSerial  — the HardwareSerial port wired to the HC-05 (typically Serial3).
    // statePin  — GPIO that reads the HC-05 STATE output.  -1 = not connected.
    // enPin     — GPIO driving the HC-05 EN input.          -1 = not connected.
    BluetoothManager(HardwareSerial& btSerial, int statePin = -1, int enPin = -1);

    // Initialise GPIO pins.  Call from setup() after the serial port is open.
    void begin();

    // Poll the STATE pin and update the OLED on connect/disconnect transitions.
    // Call once per loop() tick — lightweight (reads a GPIO every 250 ms).
    void update();

    // True if the STATE pin is wired and currently HIGH (host connected).
    bool isConnected() const;

    // "connected" or "disconnected"
    String getStatus() const;

    // Send AT configuration commands to the HC-05.
    // Works in mini-AT mode (module idle, not paired to a host) at the module's
    // current data baud rate.  For full AT mode, pull EN HIGH before powering
    // the module; this function temporarily switches the serial port to 38400,
    // sends the commands, then restores the data baud rate.
    // Returns true if all AT exchanges received "OK".
    bool runATConfig(const String& deviceName,
                     const String& pinCode  = BT_PIN_CODE,
                     uint32_t      dataBaud = 57600);

    void setStatePin(int pin);
    void setEnPin(int pin);
    int  getStatePin() const { return statePin_; }
    int  getEnPin()    const { return enPin_;    }

private:
    HardwareSerial& serial_;
    int             statePin_;
    int             enPin_;
    bool            connected_;
    unsigned long   lastPollMs_;

    static constexpr unsigned long POLL_INTERVAL_MS = 250;

    // Send one AT command and wait for expectedResponse within timeoutMs.
    bool sendAT(const String& cmd, const String& expectedResponse = "OK",
                unsigned long timeoutMs = 1000);
};
