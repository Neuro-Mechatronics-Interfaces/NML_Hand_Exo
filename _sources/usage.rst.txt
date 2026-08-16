Python usage
============

Install the package
-------------------

For development from a repository checkout:

.. code-block:: powershell

   py -3.11 -m venv .venv
   .\.venv\Scripts\Activate.ps1
   python -m pip install -e .

USB serial
----------

.. code-block:: python

   from nml_hand_exo import HandExo, SerialComm

   exo = HandExo(SerialComm(port="COM12", baudrate=1_000_000))
   try:
       exo.connect()
       info = exo.info()
       print(info["motors"])
       print(exo.get_motor_angle("all"))
   finally:
       exo.close()

TCP bridge
----------

.. code-block:: python

   from nml_hand_exo import HandExo, TCPComm

   exo = HandExo(TCPComm(ip="192.168.1.200", port=5001))

Motor commands
--------------

Use integer IDs returned by ``exo.info()``. This is mandatory for calibration
and strongly recommended for every motor command because left and right motors
share names in dual firmware.

.. code-block:: python

   motor_id = 15
   lower, upper = exo.get_motor_limits(motor_id)
   angle = exo.get_motor_angle(motor_id)

   exo.enable_motor(motor_id)
   exo.set_motor_angle(motor_id, min(upper, angle + 5.0))
   exo.disable_motor(motor_id)

Do not change joint or current limits without validating the participant,
hardware assembly, firmware configuration, and calibration profile.

Applications
------------

.. code-block:: powershell

   handexo --help
   handexo gui
   handexo emg-intent
   nml-task-cue

See the repository ``docs/serial_protocol.md`` for the firmware contract and
``examples/README.md`` for hardware-specific scripts.
