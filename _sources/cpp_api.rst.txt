C++ firmware API
================

The OpenRB-150 firmware is maintained under ``src/cpp/nml_hand_exo``.

``config.h``
   Hardware constants, motor IDs and names, baud rates, joint limits, and the
   single/dual-side build selection.

``nml_hand_exo.h`` and ``nml_hand_exo.cpp``
   The ``NMLHandExo`` device-control class and Dynamixel operations.

``utils.cpp``
   The source of truth for host-visible serial command names and parsing.

``gesture_controller.cpp`` and ``gesture_library.cpp``
   Sparse normalized gesture definitions and execution.

See ``docs/serial_protocol.md`` for the host contract. Protocol changes must be
updated in firmware, the Python response parser, documentation, and tests in the
same change.
