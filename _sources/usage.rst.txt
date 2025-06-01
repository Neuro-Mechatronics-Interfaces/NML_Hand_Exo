Python API Usage Guide
=======================

This section shows how to use the Python API to communicate with the NML Hand Exoskeleton. Each function sends a specific serial command to the device, which is also documented here for transparency and debugging.

Installation
------------

Install the package locally using:

.. code-block:: bash

   git clone https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo.git
   cd NML_Hand_Exo
   pip install -e .


Creating an Instance
--------------------

To connect to the device:

.. code-block:: python

   from hand_exo import HandExo

   exo = HandExo(port='COM3', baudrate=57600, verbose=True)

Common Commands
---------------

Below is a list of common Python methods and the serial commands they send:

First lets see if we can see the motor LED turn on:

.. code-block:: python

   exo.enable_led(1) # Sends "led:1:on"

If this works the LED on that motor will now be on. Let's turn it off and enable the motor torque for the motor with the ID 1:

.. code-block:: python

   exo.disable_led(1)   # Sends "led:1:off"
   exo.enable_torque(1) # Sends "enable:1"

You should feel the motor engage. If you want to disable the torque, you can do so with:

.. code-block:: python

   exo.disable_torque(1) # Sends "disable:1"

Let's keep the torque enabled for now. We can get the motor's current position relatve to the zero position offset (different from the absolute position):

.. code-block:: python

   angle = exo.get_motor_angle(1) # Sends "get_angle:1"

To reset the motor position to its zero position, you can use the home command:

.. code-block:: python

   exo.home(1) # Sends "home:1",

   # You can also reset all motor positions
   # exo.home('all')

To set the motor to a specific angle, you can use:

.. code-block:: python

   exo.set_motor_angle(1, 45) # Sends "set_angle:1:45" # Counter-Clockwise
   exo.set_motor_angle(1, -45) # Sends "set_angle:1:-45" # Clockwise

.. note::

   - The angle is relative to the zero position offset, not the absolute position.
   - There are joint limits configured in the Arduino code that will prevent the angle commands from moving past these limits.

The zero position for every motor configured on the microcontroller is stored in the firmware. You can see what the current value is with:

.. code-block:: python

   zero_position = exo.get_zero(1) # Sends "get_zero:1"

If you want to set the current position as the new zero position, you can use:

.. code-block:: python

   exo.set_zero(1) # Sends "set_zero:1"

Now the home command will set the motor to this new zero position.

If you want to see the absolute position of the motor, you can use:

.. code-block:: python

   abs_angle = exo.get_absolute_motor_angle(1) # Sends "get_absangle:1"

Setting the absolute position of the motor is possible too:

.. code-block:: python

   exo.set_absolute_motor_angle(1, 90) # Sends "set_absangle:1:90"

.. warning::

   - Setting the absolute angle will not change the zero position offset. Please be careful when using this command after installing motors to prevent damage.

All motors have a default velocity and acceleration component to them

.. code-block:: python

   vel = exo.get_motor_velocity(1)  # Sends "get_vel:1"
   accel = exo.get_motor_acceleration(1)  # Sends "get_accel:1"

We can adjust the speed and acceleration of the motors. Let's increase both by 20%

.. code-block:: python

    vel = vel + 0.2*vel
    accel = accel + 0.2*accel
    exo.set_motor_velocity(1, vel)  # Sends "set_vel:1:{vel}"
    exo.set_motor_acceleration(1, accel)  # Sends "set_accel:1:{accel}"

The motors can also provide torque and current readings. You can retrieve these values with:

  .. code-block:: python

     torque = exo.get_motor_torque(1)
     current = exo.get_motor_current(1)

If the motor reaches its stall torque and disables itself, the LED will begin flashing every second. The only way to continue using the motor is to reboot it. You can do this with:

  .. code-block:: python

     exo.reboot_motor(1)  # Sends "reboot:1"


All the information regarding the status info of the exo can be retrieved with:

.. code-block:: python

   info = exo.info()  # Sends "info"

This returns a dictionary with the following keys:

- `version`: Firmware version
- `n_motors`: Number of motors connected
- `motor_xx`: Dictionary with motor information, created for each motor ID

  - `id`: Motor ID
  - `angle`: Current angle of the motor
  - `zero`: Zero position offset
  - `velocity`: Current velocity setting
  - `acceleration`: Current acceleration setting
  - `torque`: Current torque reading
  - `current`: Current current reading


Anytime you need to know which commands are available you can use the help command:

  .. code-block:: python

     help_text = exo.help()

This returns a string with all available commands and their descriptions.

When you're all done with the exoskeleton, you can close the connection:

  .. code-block:: python

     exo.close()  # No command is sent to the device.

---

Additional Notes
----------------

- The `verbose=True` option prints sent and received commands to the terminal with debugging output. Enable this upon initialization or by sending the `debug:on` command.
- The `port` parameter should be set to the correct serial port for your device (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux).
- The `baudrate` parameter should match the baud rate set in the firmware (default is 57600).
