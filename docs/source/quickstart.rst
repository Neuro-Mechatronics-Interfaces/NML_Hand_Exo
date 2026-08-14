Quickstart
==========

Prerequisites
-------------

* Python 3.10–3.12 (Python 3.11 recommended).
* An OpenRB-150 flashed with matching NML Hand Exo firmware.
* A validated calibration profile for the connected side.
* A supervised bench setup with immediate access to power and emergency stop.

Install
-------

.. code-block:: powershell

   py -3.11 -m venv .venv
   .\.venv\Scripts\Activate.ps1
   python -m pip install -e .

Check the applications without opening hardware
-----------------------------------------------

.. code-block:: powershell

   handexo --version
   handexo --help
   nml-task-cue --help

Connect
-------

Launch ``handexo gui``, select the active side, scan for the OpenRB serial
port, and connect. Dual-CDC firmware exposes two COM ports on one USB cable;
select either member and allow the GUI to pair its sibling interface.

Before motion
-------------

#. Confirm every expected integer Dynamixel ID is present.
#. Apply and inspect the correct side-specific calibration profile.
#. Confirm current and velocity limits.
#. Arm only the motors required by the experiment.
#. Test stop behavior before placing the exoskeleton on a participant.

See :doc:`usage` for the Python API and the repository architecture documents
for dual-side and telemetry details.
