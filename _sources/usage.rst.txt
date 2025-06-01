Usage Guide
===========

This guide explains how to safely wear, operate, and troubleshoot the **NML Hand Exoskeleton**.

Wearing the Exoskeleton
-----------------------

1. Inspect the exoskeleton for damage or loose components before use.
2. Place the device on your hand and wrist, ensuring a snug but comfortable fit.
3. Fasten all straps securely.
4. Make sure the device is powered off during fitting.

Connecting to the Device
------------------------

1. Connect the device to your computer via USB.
2. Confirm that it appears in your operating system as a serial device (COMx on Windows, `/dev/ttyUSBx` on Linux).
3. Use a serial terminal or the provided Python API to communicate with the device.

Recommended Serial Settings:
- Baud rate: **57600**
- Data bits: **8**
- Parity: **None**
- Stop bits: **1**

Supported Commands
------------------

Below is a complete list of supported commands that can be sent over the serial interface:

- **enable:** *ID*
  Enable torque for the specified motor.

  Example::

      enable:1

- **disable:** *ID*
  Disable torque for the specified motor.

  Example::

      disable:1

- **set_baud:** *ID:VALUE*
  Set the baud rate for a specific motor.

  Example::

      set_baud:1:57600

- **get_baud:** *ID*
  Get the current baud rate of the specified motor.

  Example::

      get_baud:1

- **set_vel:** *ID:VALUE*
  Set the velocity limit for a specific motor.

  Example::

      set_vel:1:300

- **get_vel:** *ID*
  Get the current velocity limit of the specified motor.

  Example::

      get_vel:1

- **set_acc:** *ID:VALUE*
  Set the acceleration limit for a specific motor.

  Example::

      set_acc:1:500

- **get_acc:** *ID*
  Get the current acceleration limit of the specified motor.

  Example::

      get_acc:1

- **set_angle:** *ID:ANGLE*
  Set the relative angle (in degrees) of a specific motor.

  Example::

      set_angle:1:45

- **get_angle:** *ID*
  Get the relative angle of a specific motor.

  Example::

      get_angle:1

- **get_absangle:** *ID*
  Get the absolute angle of a specific motor.

  Example::

      get_absangle:1

- **get_current:** *ID*
  Get the current draw from a specific motor (in mA).

  Example::

      get_current:1

- **get_torque:** *ID*
  Get the torque output reading from a specific motor (in N·m).

  Example::

      get_torque:1

- **led:** *ID/NAME/ALL:ON/OFF*
  Turn the LED on or off for a specific motor or all motors.

  Examples::

      led:1:on
      led:all:off

- **debug:** *ON/OFF*
  Toggle verbose debug messages on or off.

  Examples::

      debug:on
      debug:off

- **reboot:** *ID*
  Reboot a specific motor.

  Example::

      reboot:1

- **home:** *ID/ALL*
  Move a specific motor or all motors to their stored home position.

  Examples::

      home:1
      home:all

- **set_zero:** *ID/ALL*
  Set the current position of a motor (or all motors) as the new zero offset.

  Examples::

      set_zero:1
      set_zero:all

- **get_zero:** *ID*
  Get the stored zero offset for a specific motor.

  Example::

      get_zero:1

- **info**
  Retrieve device information including version, motor IDs, angles, and torque.

  Example::

      info

- **help**
  Prints a list of all supported commands.

  Example::

      help

Python API Integration
----------------------

Use the provided Python API for more advanced scripting.

1. Import the module:

    .. code-block:: python

        from hand_exo import HandExo

2. Connect to the device:

    .. code-block:: python

        exo = HandExo('COM3', baudrate=57600)

3. Move a motor:

    .. code-block:: python

        exo.set_motor_angle(1, 30)

For details, see the :doc:`python_api` page.

Safety Notes
------------

- Always power off the exoskeleton before putting it on or taking it off.
- Never exceed the documented joint limits.
- Stop using the device immediately if you experience pain or unusual resistance.

For more details on specific commands, refer to the :doc:`cpp_api` and :doc:`python_api` references.
