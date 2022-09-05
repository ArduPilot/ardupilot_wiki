.. _common-limited-firmware:

==========================
Board Firmware Limitations
==========================

ArduPilot's code base includes so many drivers and features that not all can still fit on many autpilots with cpu's only having 1MB. Therefore many boards have some features and drivers ommitted in their standard builds available on the `ArduPilot Firmware Server <https://firmware.ardupilot.org>`__.

The inclusion of these features in any given board's firmware build is controlled by certain code phrases. If a board's firmware does not include a feature or driver that the user desires, it can be included in a custom firmware created using the `Custom Firmware Server <https://custom.ardupilot.org>`__. This page lists all the build options, and the name of the feature/driver used on the Custom Firmware Server's checkboxes.

Also provided is a list of autopilot boards that link to a page listing features and drivers NOT included in its firmware. Simply click on the board you are interested in and you will be directed to its list of features/drivers NOT included in the standard firmware. You can then reference the features table below as to an explanation of what it is and how to enable it on a custom build, if desired.

ArduPilot Feature/Driver Options
================================


============================   =======================================
Option                            Custom Build Server Checkbox
============================   =======================================
AC_AVOID_ENABLED                Safety:Enable Avoidance
AP_AIRSPEED_ENABLED             Sensors:Enable Airspeed Sensors
AP_AIRSPEED_MS4525_ENABLED      Airspeed Drivers:Enable MS4525 Airspeed
MODE_FLIP_ENABLED               Copter:Enable Mode Flip
============================   =======================================

Features/Drivers Not Included by Autopilot
==========================================

.. toctree::


   CubeOrange <common-CubeOrange-limitations>
