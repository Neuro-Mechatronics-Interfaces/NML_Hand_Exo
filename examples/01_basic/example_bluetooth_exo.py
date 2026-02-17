"""
Bluetooth (HC-05) connection example for NML Hand Exoskeleton.

Hardware setup
--------------
HC-05 module wired to OpenRB-150 Serial2 (COMMAND_SERIAL):
  HC-05 TX  →  OpenRB-150 D13 (RX2)
  HC-05 RX  →  OpenRB-150 D14 (TX2)  [use a 1kΩ/2kΩ voltage divider if needed]
  HC-05 VCC →  3.3 V or 5 V (check your module's onboard regulator)
  HC-05 GND →  GND

Pairing on Windows
------------------
1. Power the OpenRB-150 so the HC-05 is active (LED blinks rapidly).
2. Open Windows Bluetooth settings and pair with "HC-05" (default PIN: 1234).
3. After pairing, Windows assigns two virtual COM ports (Outgoing and Incoming).
   Go to:  Device Manager → Ports (COM & LPT)
   Look for "Standard Serial over Bluetooth link" — use the OUTGOING port number.
4. Set BT_PORT below to that COM port (e.g. "COM5").

Arduino baud rate
-----------------
The firmware is set to COMMAND_BAUD_RATE = 9600 (HC-05 factory default).
If you reconfigured your HC-05 via AT commands to a different rate, update
COMMAND_BAUD_RATE in src/cpp/nml_hand_exo/config.h and set BT_BAUDRATE here
to the same value.
"""

from nml_hand_exo.interface import HandExo, SerialComm

# ── Update this to the Bluetooth outgoing COM port assigned by Windows ──
BT_PORT = "COM5"
BT_BAUDRATE = 9600   # Must match COMMAND_BAUD_RATE in config.h

comm = SerialComm(port=BT_PORT, baudrate=BT_BAUDRATE)
exo = HandExo(comm, verbose=False)

try:
    exo.connect()
    print("Connected via HC-05 Bluetooth")
    print(exo.version())
    print(exo.get_exo_mode())
    print(exo.get_home())
    print(exo.info())
    print(exo.get_absolute_motor_angle())
    print(exo.get_motor_angle())
    print(exo.get_gesture())
    print(exo.get_gesture_state())
except Exception as e:
    print(f"Error: {e}")
finally:
    exo.close()
