Quickstart Guide
================

Welcome to the **NML Hand Exoskeleton Quickstart Guide**! This guide will help
you get the system up and running in just a few steps.

What You’ll Need
----------------

- A compatible microcontroller (e.g. OpenRB-150) flashed with the firmware.
- The NML Hand Exoskeleton hardware (properly assembled).
- A USB cable to connect the device to your computer.
- Python 3.x (optional, for higher-level scripting).
- A serial terminal program (e.g. PuTTY, Arduino Serial Monitor) or the provided Python API.

Step 1. Connect the Device
--------------------------

1. Plug the exoskeleton’s control board into your computer’s USB port.
2. Ensure that the device is recognized by your operating system (typically shows up as a COM port on Windows or `/dev/ttyUSBx` on Linux).

Step 2. Launch Serial Terminal
------------------------------

1. Open your preferred serial terminal program.
2. Set the baud rate to **57600** (default).
3. Select the correct serial port (COMx on Windows, `/dev/ttyUSBx` on Linux, or 'dev.usbmodemxxx' on MacOS).
4. Connect and open the terminal.

Step 3. Send a Test Command
---------------------------

Once connected, try sending a simple command to make sure you can communicate with the dynamixel motors. See if the led can turn on:

   .. code-block:: text

       len:1:on

If it turns on, you are good to go! If not, check your connections and ensure the device is powered on.


Next Steps
----------

- Check out the :doc:`usage` guide for more detailed instructions on wearing
  and operating the exoskeleton safely.
- Dive into the :doc:`cpp_api` or :doc:`python_api` documentation for deeper integration.
