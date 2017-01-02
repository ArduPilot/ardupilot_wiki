.. _learning-ardupilot-introduction:

=================================
Learning ArduPilot — Introduction
=================================

This page introduces the basic structure of ArduPilot. Before you get
started you should work out what code exploring system you will use. You
could just use a web browser and look at https://github.com/ArduPilot/ardupilot/ but you will probably get a lot
more out of it if you use a good programmers IDE like the ones recommended :ref:`here <code-editing-tools-and-ides>`.

Basic structure
===============

.. image:: ../images/ArduPilot_HighLevelArchecture.png
    :target: ../_images/ArduPilot_HighLevelArchecture.png

The basic structure of ArduPilot is broken up into 5 main parts:

-  vehicle directories
-  AP_HAL
-  libraries
-  tools directories
-  external support code

These will be described in detail below, but before moving on make sure
you have :ref:`cloned all of the git repositories <where-to-get-the-code>` you will need.

Vehicle Directories
-------------------

The vehicle directories are the top level directories that define the
firmware for each vehicle type. Currently there are 4 vehicle types -
Plane, Copter, APMrover2 and AntennaTracker.

Along with the \*.cpp files, each vehicle directory contains a make.inc
file which lists library dependencies. The Makefiles read this to create
the -I and -L flags for the build.

AP_HAL
-------

The AP_HAL layer (Hardware Abstraction Layer) is how we make ArduPilot
portable to lots of different platforms. There is a top level AP_HAL in
libraries/AP_HAL that defines the interface that the rest of the code
has to specific board features, then there is a AP_HAL_XXX
subdirectory for each board type, for example AP_HAL_AVR for AVR based
boards, AP_HAL_PX4 for PX4 boards and AP_HAL_Linux for Linux based
boards.

Tools directories
~~~~~~~~~~~~~~~~~

The tools directories are miscellaneous support directories. For
examples, tools/autotest provides the autotest infrastructure behind the
`autotest.ardupilot.org <http://autotest.ardupilot.org/>`__ site and
tools/Replay provides our log replay utility.

External support code
~~~~~~~~~~~~~~~~~~~~~

On some platforms we need external support code to provide additional
features or board support. Currently the external trees are:

-  `PX4NuttX <https://github.com/ArduPilot/PX4NuttX>`__ - the core NuttX
   RTOS used on PX4 boards
-  `PX4Firmware <https://github.com/ArduPilot/PX4Firmware>`__ - the base
   PX4 middleware and drivers used on PX4 boards
-  `uavcan <https://github.com/ArduPilot/uavcan>`__ - the uavcan CANBUS
   implementation used in ArduPilot
-  `mavlink <https://github.com/mavlink/mavlink>`__ - the mavlink
   protocol and code generator

.. note::

   Most of these are imported as :ref:`Git Submodules <git-submodules>` when you :ref:`build ArduPilot <building-the-code>`.

