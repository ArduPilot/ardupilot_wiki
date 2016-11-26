.. _learning-the-ardupilot-codebase:

===============================
Learning the ArduPilot Codebase
===============================

Introduction
============

The ArduPilot code base is quite large (about 700k lines for the core
ardupilot git tree) and can be quite intimidating to a new user. This
page is meant to give some suggestions on how to come up to speed on the
code quickly. It assumes you already are familiar with the key concepts
of C++ and the many of the examples currently assume you will be
exploring the code on a Linux system.

This page and the pages linked below are designed to be used as a
tutorial. You should work through each page step by step, trying things
for yourself as you go. We will also be adding more pages over time. If
you think some important information is missing or could be improved
then please `open an issue <https://github.com/ArduPilot/ardupilot/issues>`__ on the
ArduPilot github and we'll try to get to it when we can.

Tutorial steps
--------------

-  :ref:`Introduction and code structure <learning-ardupilot-introduction>`
-  :ref:`The Example Sketches <learning-ardupilot-the-example-sketches>`
-  :ref:`ArduPilot threading <learning-ardupilot-threading>`
-  :ref:`UARTs and the console <learning-ardupilot-uarts-and-the-console>`
-  :ref:`RCInput and RCOutput <learning-ardupilot-rc-input-output>`
-  :ref:`Storage and EEPROM management <learning-ardupilot-storage-and-eeprom-management>`
-  :ref:`The vehicle code <learning-ardupilot-vehicle-code>`

Upcoming Tutorials
------------------

We will be adding more tutorial sections in the future. Check back
occasionally to see the new sections.

-  MAVLink telemetry handling
-  The Dataflash library for onboard logging
-  Analog input
-  GPIOs
-  Timing and profiling
-  PX4 device drivers
-  I2C Drivers
-  SPI Drivers
-  CANBUS drivers and uavcan
-  memory management
-  Maths functions
-  Inside the AP_AHRS attitude and position estimator
-  Porting ArduPilot to a new board
-  AP_HAL Utility functions
-  PIDs and other control libraries
-  Inside the SITL simulator
-  Inside the AP_Param parameter system
-  AP_Notify for buzzers and LEDs
-  The ArduPilot autotest system
-  How autobuilds work, and developer autobuilds
-  How ardupilot boot process works on PX4

Welcome to ArduPilot!
=====================

If you have gotten this far you should be in a good position to start
getting involved with ArduPilot development. Please `join the drones-discuss mailing list <https://groups.google.com/forum/#!forum/drones-discuss>`__ and
join in on the discussions. We also have a weekly mumble developer
conference call, some Skype chat channels and some IRC channels. Ask any
ardupilot developer for details.

Welcome to ArduPilot development, we hope you enjoy being part of our
community!

.. toctree::
    :maxdepth: 1

    Learning ArduPilot — Introduction <learning-ardupilot-introduction>
    Learning ArduPilot — The Example Sketches <learning-ardupilot-the-example-sketches>
    Learning ArduPilot – Threading <learning-ardupilot-threading>
    Learning ArduPilot – UARTs and the Console <learning-ardupilot-uarts-and-the-console>
    Learning ArduPilot — RC Input and Output <learning-ardupilot-rc-input-output>
    Learning ArduPilot – Storage and EEPROM management <learning-ardupilot-storage-and-eeprom-management>
    Learning ArduPilot – Vehicle Code <learning-ardupilot-vehicle-code>