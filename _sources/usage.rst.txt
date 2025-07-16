Python HandExo Class
=======================

This section shows how to use the Python API to communicate with the NML Hand Exoskeleton. Each function sends a specific serial command to the device, which is also documented here for transparency and debugging.

Installation
------------

Install the package locally using:

.. code-block:: bash

   git clone https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo.git
   cd NML_Hand_Exo
   pip install -e .


USB Serial Connection
--------------------------

We can connect to the device using a USB serial connection

.. code-block:: python

   from hand_exo import HandExo, SerialComm

   # Define serial communication parameters
   com = SerialComm(port='COM3', baudrate=57600) # Change 'COM3' to your device's serial port

   # Create an instance of the HandExo class
   exo = HandExo(com)


TCP/Wi-Fi Connection
--------------------------

If you want to connect to the device over TCP or Wi-Fi, you can use the `TCPComm` class instead of `SerialComm`. You can use the PicoW module as the server to handle client connections and exchange serial data. For setting up the pico server, check out the :doc:`wifi_server` documentation for more.

.. code-block:: python

   from hand_exo import HandExo, TCPComm

   # Define TCP communication parameters
   comm = TCPComm(ip="192.168.1.200", port=5001") # Default IP and port for the PicoW server, change as needed
   exo = HandExo(comm)

LED Control
---------------

First lets see if we can see the motor LED turn on:

.. code-block:: python

   exo.enable_led(1) # Sends "led:1:on"

If this works the LED on that motor will now be on. Let's turn it off:

.. code-block:: python

   exo.disable_led(1)   # Sends "led:1:off"

Motor Control
----------------

Enable the motor torque for the motor with the ID 1:

.. code-block:: python

   exo.enable_torque(1) # Sends "enable:1"

You should feel the motor engage. If you want to disable the torque, you can do so with:

.. code-block:: python

   exo.disable_torque(1) # Sends "disable:1"

Let's keep the torque enabled for now.

Position Control & Angles
----------------------------

We can get the motor's current position relatve to the zero position offset (different from the absolute position):

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

The home/zero position for every motor configured on the microcontroller is stored in the firmware. You can see what the current value is with:

.. code-block:: python

   m1_home_pos = exo.get_home(1) # Sends "get_home:1"

If you want to set the current position as the new home/zero position, you can use:

.. code-block:: python

   exo.set_home(1) # Sends "set_home:1"

Now the home command will set the motor to this new home/zero position.

If you want to see the absolute position of the motor, you can use:

.. code-block:: python

   abs_angle = exo.get_absolute_motor_angle(1) # Sends "get_absangle:1"

Setting the absolute position of the motor is possible too:

.. code-block:: python

   exo.set_absolute_motor_angle(1, 90) # Sends "set_absangle:1:90"

.. warning::

   - Setting the absolute angle will not change the zero position offset. Please be careful when using this command after installing motors to prevent damage.

The exo firmware has a built-in safety feature that prevents the motors from moving past their limits. These limits can be accessed with:

.. code-block:: python

   limits = exo.get_motor_limits(1)  # Sends "get_limits:1"

This returns a tuple with the minimum and maximum angle limits for the motor, for example: (-90, 90))
You can also set new limits for the motor:

.. code-block:: python

   exo.set_motor_upper_limit(1, 90)  # Sends "set_upper_limit:1:90"
   exo.set_motor_lower_limit(1, -90)  # Sends "set_lower_limit:1:-90"
   exo.set_motor_limits(1, -100, 100)  # Sends "set_limits:1:-100:100"


.. warning::

   - The programatic joint limits are set to prevent the motors from colliding with the exo device. Please be careful when using this command after installing motors to prevent damage.


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


Exo Information
----------------

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

Gesture Control
---------------

The NML Hand Exoskeleton supports pre-programmed gestures within the firmware. However, setting the exo to gestures requires the exo operating mode to be in either `GESTURE_FIXED` or `GESTURE_CONTINUOUS`. Let's check the current exo operating mode:

  .. code-block:: python

     mode = exo.get_exo_mode()  # Sends "get_exo_mode"

If it returns `FREE`, then we need to switch it. To set the operating mode to `GESTURE_FIXED`, you can use:

    .. code-block:: python

         exo.set_exo_mode('GESTURE_FIXED')  # Sends "set_exo_mode:GESTURE_FIXED"

Now that the mode is configured, we can command the exo to execute a gesture. Gestures usually have at least one state we need to declare too. Let's command a grasping gesture with the initial state as "open":

  .. code-block:: python

     exo.set_gesture("grasp", 'open')  # Sends "set_gesture:grasp:open"

To change the gesture state, you can simply call the same method with a different state:

    .. code-block:: python

     exo.set_gesture("grasp", 'close')  # Sends "set_gesture:grasp:close"

What if we don't know the exact current gesture name? We can call the `get_gesture` command to get the currently assigned gesture. We can also use the `gesture_list` to find all programmed gestures:

  .. code-block:: python

     cur_gesture = exo.get_gesture()  # Sends "get_gesture" to return the currently set gesture, or ...
     all_gestures = exo.gesture_list()  # Sends "gesture_list" to return the full programmed list of gestures and states

This returns a list of available gestures along with the motor positions. Gestures and states can be cycled through just as easily:

  .. code-block:: python

     exo.cycle_gesture()  # Sends "cycle_gesture" to cycle through the next gesture in the list
     exo.cycle_gesture_state()  # Sends "cycle_gesture_state" to cycle through the next state of the current gesture


IMU
----------------
The NML Hand Exoskeleton also has an IMU (Inertial Measurement Unit) that can be used to get the current orientation of the exoskeleton. To get the orientation of the exoskeleton, you can use:

  .. code-block:: python

     orientation = exo.get_imu_angles() # Returns a tuple with the roll, pitch, and yaw angles in degrees
     orientation_rad = exo.get_imu_angles(degrees=False) # Returns a tuple with the roll, pitch, and yaw angles in degrees
     imu_data = exo.get_imu_angles(raw=True) # Returns a tuple with the temperature accelerometer, gyroscope, and magnetometer data (if available) in raw format


Additional Notes
----------------

- The `verbose=True` option prints sent and received commands to the terminal with debugging output. Enable this upon initialization or by sending the `debug:on` command.
- The `port` parameter should be set to the correct serial port for your device (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux).
- The `baudrate` parameter should match the baud rate set in the firmware (default is 57600).
