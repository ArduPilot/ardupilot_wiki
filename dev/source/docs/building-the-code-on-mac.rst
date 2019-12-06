.. _building-the-code-on-mac:

=============================================================
Archived: Building ArduPilot for APM2.x on MacOS with Arduino
=============================================================

.. warning::

   **ARCHIVED ARTICLE**

   ArduPilot no longer supports Arduino or AVR.

.. warning::

   Copter 3.3 firmware (and later) no longer fits on APM boards.
   The last firmware builds that can be installed (v3.2.1) can be
   downloaded from here:
   `APM2.x <https://download.ardupilot.org/downloads/wiki/firmware/ArduCopter_APM_2.0_Firmware_3.2.1.zip>`__
   and `APM 1.0 <https://download.ardupilot.org/downloads/wiki/firmware/ArduCopter_APM_1.0_Firmware_3.2.1.zip>`__.

   Plane, Rover and AntennaTracker builds can still be installed.

To build the ardupilot source code on MacOS for AVR targets (such as the
APM1 or APM2) you have two choices. The first option is to build using a
modified version of the Arduino build environment. You can get it from
https://firmware.ardupilot.org/ under the Tools directory.

The second choice is to build using the 'make' tool on the command line.
If using the Arduino tool, then after installing it, you need to do the
following:

-  Choose your board type under the ArduPilot menu
-  Set your Sketchbook location under **File \| Preferences** to point
   at the root of your git checkout of the ardupilot sources
-  Stop and restart Arduino
