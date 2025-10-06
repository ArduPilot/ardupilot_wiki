.. _learning-ardupilot-introduction:

=================================
Learning ArduPilot — Introduction
=================================

This page introduces the basic structure of ArduPilot. Before you get
started you should work out what code exploring system you will use. You
could just use a web browser and look at https://github.com/ArduPilot/ardupilot/ but you will probably get a lot
more out of it if you have :ref:`cloned all of the git repositories <where-to-get-the-code>` and use a good programmer's IDE like the ones recommended :ref:`here <code-editing-tools-and-ides>`.

Basic structure
===============

.. image:: ../images/ArduPilot_HighLevelArchecture.png
    :target: ../_images/ArduPilot_HighLevelArchecture.png

The basic structure of ArduPilot is broken up into 5 main parts:

-  vehicle code
-  shared libraries
-  hardware abstraction layer (AP_HAL)
-  tools directories
-  external support code (e.g. mavlink)

Vehicle Code
------------

The vehicle directories are the top level directories that define the
firmware for each vehicle type.  Currently, there are 6 vehicle types: Plane, Copter, Rover, Sub, Blimp and AntennaTracker.
Although there are a lot of common elements between different vehicle types, they are each different. For now, we only have a :ref:`detailed description of the code structure for the Copter code <apmcopter-code-overview>`.

Along with the \*.cpp files, each vehicle directory contains a wscript
file which lists library dependencies.

Libraries
---------

The `libraries <https://github.com/ArduPilot/ardupilot/tree/master/libraries>`__ are
shared amongst all vehicle types.  These libraries include sensor drivers, attitude and position estimation (aka :ref:`EKF <ekf>`) and control code (i.e. PID controllers).
See the :ref:`Library Description <apmcopter-programming-libraries>`, :ref:`Library Example Sketches <learning-ardupilot-the-example-sketches>` and :ref:`Sensor Drivers <code-overview-sensor-drivers>` pages for more details.

AP_HAL
-------

The AP_HAL layer (Hardware Abstraction Layer) is how we make ArduPilot
portable to lots of different platforms. There is a top-level AP_HAL in
libraries/AP_HAL that defines the interface that the rest of the code
has to specific board features, and then there is a AP_HAL_XXX
subdirectory for each board type, for example, AP_HAL_ChibiOS for stm32-based
boards, AP_HAL_ESP32 for ESP32 boards and AP_HAL_Linux for Linux based
boards.

Tools directories
~~~~~~~~~~~~~~~~~

The tools directories are miscellaneous support directories. For
example, tools/autotest provides the autotest infrastructure behind the
`autotest.ardupilot.org <https://autotest.ardupilot.org/>`__ site and
tools/Replay provides our log replay utility.

External support code
~~~~~~~~~~~~~~~~~~~~~

On some platforms we need external support code to provide additional
features or board support. Currently the external trees are:

-  `ChibiOS <https://github.com/ArduPilot/ChibiOS>`__ - the ChibiOS
   RTOS used on stm32-based boards
-  `DroneCAN <https://github.com/DroneCAN>`__ - the CANBUS
   implementation used in ArduPilot
-  `mavlink <https://github.com/ArduPilot/mavlink>`__ - the mavlink
   protocol and code generator

.. note::

   Most of these are imported as :ref:`Git Submodules <git-submodules>` when you :ref:`build ArduPilot <building-the-code>`.

