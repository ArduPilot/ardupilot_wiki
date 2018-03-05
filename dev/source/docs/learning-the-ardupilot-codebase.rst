.. _learning-the-ardupilot-codebase:

===============================
Learning the ArduPilot Codebase
===============================

The ArduPilot code base is quite large (about 700k lines for the core
ardupilot git tree) and can be quite intimidating to a new user. This
page is meant to give some suggestions on how to come up to speed on the
code quickly. It assumes you already are familiar with the key concepts
of C++ and the many of the examples currently assume you will be
exploring the code on a Linux system.

This page and the pages linked below are designed to be used as a
tutorial. You should work through each page step by step, trying things
for yourself as you go. If you think some important information is missing or could be improved
then please `open an issue for the wiki <https://github.com/ArduPilot/ardupilot_wiki/issues>`__ and we will try to get to it when we can.

Tutorial steps
--------------

.. toctree::
    :maxdepth: 1

    Introduction <learning-ardupilot-introduction>
    Library Description <apmcopter-programming-libraries>
    Library Example Sketches <learning-ardupilot-the-example-sketches>
    Sensor Drivers <code-overview-sensor-drivers>
    Threading <learning-ardupilot-threading>
    UARTs and the Console <learning-ardupilot-uarts-and-the-console>
    RC Input and Output <learning-ardupilot-rc-input-output>
    Storage and EEPROM management <learning-ardupilot-storage-and-eeprom-management>
    EKF <ekf>
    Copter - Vehicle Code introduction <apmcopter-code-overview>
    Copter - Attitude Control <apmcopter-programming-attitude-control-2>
    Copter - Adding Parameters <code-overview-adding-a-new-parameter>
    Copter - Adding a new flight mode (ver 3.6 and higher) <apmcopter-adding-a-new-flight-mode>
    Copter - Adding a new flight mode (ver 3.5 and lower) <apmcopter-adding-a-new-flight-mode-35>
    Copter - Scheduling your new code to run intermittently <code-overview-scheduling-your-new-code-to-run-intermittently>
    Copter - Object Avoidance <code-overview-object-avoidance>
    Rover - Adding a new drive mode <rover-adding-a-new-drive-mode>
    Plane - Architecture overview <plane-architecture>
    Adding a new Log message <code-overview-adding-a-new-log-message>
    Adding a new MAVLink message <code-overview-adding-a-new-mavlink-message>
    Adding a new MAVLink Gimbal <code-overview-adding-support-for-a-new-mavlink-gimbal>

.. note::

   There are currently 4 vehicles in ArduPilot (Copter, Plane, Rover, Antenna Tracker) and while there are a lot of common elements between different vehicle types, they are each different. For now we only have a detailed description of the code structure for the Copter code.

Other Tutorials
---------------

Although not strictly part of ArduPilot this tutorial may also be useful

- `DroneKit <http://python.dronekit.io/>`__
