# Hardware Debug Guide - HC-05 Not Communicating with Arduino

## Problem
- `info` command is sent successfully to COM12/COM13
- **No response** is received from either port
- This means the HC-05 is paired with Windows, but the Arduino isn't responding through it

## Diagnostic Checklist

### 1. **Is the Arduino Connected and Running?**

Connect the Arduino via **USB cable directly** (not through HC-05) and test:

```powershell
# Test direct serial connection to Arduino
python examples/01_basic/example_serial_exo.py
# When prompted for port, select the Arduino's USB port (usually COMx with lower number)
# When prompted for baudrate, try 57600 or 115200
```

**Expected output**: Should print motor info without freezing

**If it fails**: 
- Arduino firmware doesn't respond to "info" command
- Check firmware is uploaded to the Arduino board
- Try uploading the firmware again from Arduino IDE

---

### 2. **Is the HC-05 Actually Powered?**

Check the HC-05 module physically:
- **Red LED slow blink (every 2s)**: Good, waiting to pair ✓
- **Red LED fast blink**: Paired but idle ✓
- **Red LED solid on**: Powered but not blinking — problem
- **No red LED**: Not powered — check power connections

---

### 3. **HC-05 Physical Connections**

The HC-05 UART connections are critical. **Incorrect wiring = no communication.**

```
HC-05 Pin          Arduino Pin      Notes
─────────────────────────────────────────────
5V                 5V               (or 3.3V with voltage regulator)
GND                GND              
TX                 RX0 or RX1       (depends on Arduino model)
RX                 TX0 or TX1       (with voltage divider if needed)
```

**For Arduino Uno/Mega with RX0/TX0 (Serial)**:
- HC-05 TX → Arduino RX0 (pin 0)
- HC-05 RX → Arduino TX0 (pin 1) **with voltage divider**
- HC-05 GND → Arduino GND
- HC-05 5V → Arduino 5V

**Important**: Arduino's RX expects 0-5V, but HC-05 TX outputs 3.3V. Use a **voltage divider** on HC-05 RX (since HC-05 RX is 3.3V tolerant):

```
Arduino TX → [10kΩ resistor] → HC-05 RX
           ├─→ [20kΩ resistor] → GND
                                  ^
                                  This divides 5V to ~3.3V
```

---

### 4. **Test HC-05 ↔ Arduino Connection Directly** (if physically accessible)

If you can access the Arduino with HC-05 wired to it:

**Option A: Use Serial Monitor on Arduino IDE**
1. Open Arduino IDE
2. Arduino → Serial Monitor
3. Set baud rate to 57600
4. Type `info` and press Enter
5. Should see motor information

**If it works**: HC-05 hardware is fine, issue is with COM port detection
**If it fails**: HC-05 isn't connected to Arduino properly

---

### 5. **Check Windows Bluetooth Device Properties**

1. Go to: **Settings → Devices → Bluetooth & devices**
2. Find the HC-05 device (usually "HC-05" or similar)
3. Click it, then **Device options**
4. Look for **COM Ports**:
   - Note the port numbers (should be COM12 and COM13)
   - "Outgoing" is the one to use for PC→HC-05 communication ✓
5. If COM ports don't appear:
   - Unpair and re-pair the HC-05
   - Check Windows Device Manager for errors

---

### 6. **Alternative: Test with Different Baud Rate**

HC-05 might be configured for a different baud rate than 57600:

```powershell
.\.handexo311\Scripts\activate

# Edit test_bluetooth_ports.py and change baudrate parameter:
# Change: test_port(port)
# To:     test_port(port, baudrate=115200)

python test_bluetooth_ports.py
```

Try these baud rates in order:
- 9600 (default factory reset)
- 38400
- 57600 (project default)
- 115200

---

## Resolution Steps

### If Direct USB Works, but Bluetooth Doesn't:

1. **Power-cycle HC-05** (unplug power for 10 seconds)
2. Check physical wiring again (especially voltage divider on RX)
3. Verify HC-05 is actually in slave mode (it should be paired passively, not trying to pair itself)
4. Try different COM port (the GUI now shows both options)

### If Direct USB Also Fails:

1. Arduino firmware might not be the exo firmware
2. Upload the correct firmware from this repo:
   - Check `src/cpp/` for the Arduino code
   - Or find firmware hex file and upload via Arduino IDE

### If Both Work:

1. Wiring between HC-05 and Arduino is the issue
2. Double-check voltage divider resistor values
3. Ensure solid solder/connector joints (not loose)
4. Try shorter wires if using very long cables (RF noise)

---

## Quick Tests to Run

1. **Test Arduino directly via USB first**:
   ```powershell
   python examples/01_basic/example_serial_exo.py
   ```

2. **If that works, check HC-05 LED**:
   - Should blink slowly or fast (not solid)

3. **Verify Windows sees HC-05 correctly**:
   - Settings → Bluetooth → Should list "HC-05"
   - Device Manager → Ports → Should show COM12/COM13

4. **If still failing, test baud rates**:
   - Modify test_bluetooth_ports.py to try 9600, 38400, 115200
   - If any works, use that baud rate in GUI

---

## Support Info

When debugging:
- Provide: Which COM ports show up in Windows?
- Provide: Does the Arduino respond via direct USB?
- Provide: What does the HC-05 LED do?
- Provide: How is HC-05 wired to Arduino?

This will help identify if it's:
- Firmware issue (Arduino not responding)
- Wiring issue (HC-05 not connected to Arduino)
- Configuration issue (baud rate mismatch)
- Port detection issue (Windows not recognizing ports)
