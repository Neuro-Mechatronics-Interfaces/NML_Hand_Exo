Python API
==========

The Python API provides a convenient way to interact with the **NML Hand Exoskeleton** using Python scripts. This API wraps serial communication and exposes user-friendly methods to control the device.

Installation
------------

Install the Python API locally:

.. code-block:: bash

    pip install ./python

Basic Usage
-----------

Start by importing the module and connecting to the device:

.. code-block:: python

    from hand_exo import HandExo

    exo = HandExo('COM3', baudrate=57600)  # Replace 'COM3' with your actual port

    # Enable the wrist motor (ID:1)
    exo.enable_motor(1)

    # Move the wrist to 45 degrees
    exo.set_motor_angle(1, 45)

    # Disable the wrist motor
    exo.disable_motor(1)

API Reference
-------------

.. automodule:: hand_exo
    :members:
    :undoc-members:
    :show-inheritance:

Key Classes
-----------

**HandExo**

This is the main class for controlling the exoskeleton.

**Methods:**

- **enable_motor(id: int)**
  Enable torque for the specified motor.

- **disable_motor(id: int)**
  Disable torque for the specified motor.

- **set_motor_angle(id: int, angle: float)**
  Move a motor to a specified relative angle (in degrees).

- **get_motor_angle(id: int) -> float**
  Get the relative angle of a motor.

- **get_motor_torque(id: int) -> float**
  Get the torque output from a motor (in N·m).

- **get_motor_current(id: int) -> float**
  Get the current draw from a motor (in mA).

Advanced Topics
---------------

- Refer to the :doc:`usage` guide for a complete list of motor IDs and recommended commands.
- For advanced users, consult the :doc:`cpp_api` for direct integration with the underlying C++ firmware.

Safety Notes
------------

- Always call `disable_motor` on all motors before unplugging the device.
- Confirm correct baud rate and port settings to avoid communication errors.

Contributions
-------------

Contributions and bug reports are welcome! See the GitHub repository for details.
