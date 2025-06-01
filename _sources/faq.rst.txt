Frequently Asked Questions (FAQ)
=================================

Below are some common questions and answers about the **NML Hand Exoskeleton**.

General Questions
-----------------

**Q: What is the recommended power supply voltage?**
A: The device typically operates at 12V DC. Check your hardware documentation for details.

**Q: What is the maximum supported torque?**
A: The XL330 servo supports up to 0.43 Nm. For details, refer to the manufacturer's datasheet.

**Q: Can I use a different baud rate?**
A: Yes, but make sure to configure it both on the device and in your terminal or Python API.

Safety
------

**Q: Is it safe to wear the exoskeleton continuously?**
A: It’s recommended to limit continuous wear to 30 minutes at a time and to take breaks. Stop immediately if you feel discomfort or pain.

**Q: Can I disable the motors quickly in an emergency?**
A: Yes, send `disable:all` to immediately disengage torque on all motors.

Troubleshooting
---------------

**Q: The device isn't responding—what should I check?**
A:
1. Verify the USB connection.
2. Check the baud rate and serial settings.
3. Make sure the correct COM port is selected.

**Q: I get an error about unknown commands—why?**
A: Ensure the command syntax matches exactly (case insensitive). Refer to the :doc:`usage` guide for supported commands.

More Information
----------------

For additional help, see the :doc:`usage`, :doc:`quickstart`, and :doc:`python_api` pages.
