.. _mission-planner-initial-setup:

=============================
Mission Planner Initial Setup
=============================


This section of Mission Planner, invoked by the Menu item  ``Setup``
at the top of Mission Planner, has several subsections. The subsection
are where you set up and configure you auto pilot to prepare it for your
particular vehicle. Typically these sections are "must do" actions that
are required.

What you see when you enter this section depends on whether or not you
are connected. Each menu item will bring up a new screen, each is
discussed below with links to more detail.

Install Firmware
~~~~~~~~~~~~~~~~

You will see this menu item If the auto pilot is Not connected. If you
have a new auto pilot or if you want to update the control software that
resides in you autopilot, you must install (upload) the
:ref:`firmware <common-glossary>` into it.

The firmware is located at `firmware.ardupilot.org <https://firmware.ardupilot.org>`__ .
If the autopilot has ArduPilot firmware already installed, you can use this page to upload firmware for different vehicles or version. See this :ref:`Loading firmware <common-loading-firmware-onto-pixhawk>` page. Otherwise, you must use other methods than Mission Planner for getting ArduPilot installed for the first time, see this :ref:`section<common-loading-firmware-onto-chibios-only-boards>` . 

From this screen you can also select "All Options" allowing you to select and load any variation of the firmware, or "Load custom firmware", most often used when a developer has trial code to load.

Install Firmware Legacy
~~~~~~~~~~~~~~~~~~~~~~~

Yet another way load older versions of the firmware. Again, shown only when not connected.


Mandatory Hardware
~~~~~~~~~~~~~~~~~~

You will see this menu item If the auto pilot IS connected.  Click this
menu item to see the items you must setup before you attempt to
operate your vehicle.  Specifics are located in the Ardupilot.org documents which
cover you specific vehicle (Copter, Plane, Rover).


Optional Hardware
~~~~~~~~~~~~~~~~~

This submenu allows the configuration of optional devices, many of which can be configured while Mission Planner is unconnected. Programming of the Sik  Telemetry Radio, UAVCAN setup, PX4 Optical Flow sensor,Antenna Tracker can be done here, as well as setup of a joystick to be used in conjunction with Mission Planner.


When connected, peripherals such as Battery Monitors, Integrated OSD, Airspeed Sensors, and Rangefinders can be configured. Also, this submenu has a Motor Test function allowing you to test direction and order of Copter and Quadplane Motors.

Advanced
~~~~~~~~

This section is for advanced users only.