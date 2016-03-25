.. _learning-ardupilot-introduction:

=================================
Learning ArduPilot — Introduction
=================================

This page introduces the basic structure of ArduPilot. Before you get
started you should work out what code exploring system you will use. You
could just use a web browser and look at
https://github.com/diydrones/ardupilot/ but you will probably get a lot
more out of it if you use a good programmers IDE that allows you to find
function, structure and class definitions and shows the code in a
structured manner.

Some suggestions are:

-  Eclipse on Windows, Linux or MacOS
-  Emacs on Linux, Windows or MacOS, with etags for finding code
   elements
-  Vim on emacs with ctags

There are a lot of other IDEs available, and many of them are
sufficiently customisable that they can handle something like ArduPilot
nicely. If you have a favourite and have worked out how to make
ArduPilot development a great experience then please consider
contributing a wiki page on how to use it.

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
`autotest.diydrones.com <http://autotest.diydrones.com/>`__ site and
tools/Replay provides our log replay utility.

External support code
~~~~~~~~~~~~~~~~~~~~~

On some platforms we need external support code to provide additional
features or board support. Currently the external trees are:

-  `PX4NuttX <https://github.com/diydrones/PX4NuttX>`__ - the core NuttX
   RTOS used on PX4 boards
-  `PX4Firmware <https://github.com/diydrones/PX4Firmware>`__ - the base
   PX4 middleware and drivers used on PX4 boards
-  `uavcan <https://github.com/diydrones/uavcan>`__ - the uavcan CANBUS
   implementation used in ArduPilot
-  `mavlink <https://github.com/mavlink/mavlink>`__ - the mavlink
   protocol and code generator

.. note::

   Most of these are imported as :ref:`Git Submodules <git-submodules>` when you
   build ArduPilot for PX4/Pixhawk.

Build system
============

The build system is based around make, but also supports the old arduino
IDE for AVR builds. The makefiles are in the `mk/ directory <https://github.com/diydrones/ardupilot/tree/master/mk>`__,
and define build rules for each type of supported board

To build a vehicle or other 'sketch' for a particular board target you
would type "make TARGET", where TARGET is the board type. The following
board types are currently available:

-  make apm1 - the APM1 board
-  make apm2 - the APM2 board
-  make px4-v1 - the PX4v1
-  make px4-v2 - the Pixhawk (and `Arsov AUAV-X2 <http://www.auav.co/product-p/auavx2.htm>`__)
-  make pxf - the BBB+PXF cape combination
-  make navio - the RaspberryPi+NavIO cape combination
-  make linux - a generic Linux build
-  make flymaple - the FlyMaple board
-  make vrbain - the VRBrain boards
-  make sitl - the SITL software in the loop simulation

More ports are being added all the time, so check "make help" file for
new targets.

For each of these builds you can add additional qualifiers, and on some
you can do a parallel build to speed things up. For example, in the
Copter directory you could do:

::

    make apm2-octa -j8

meaning do a build for OctaCopter on apm2 with an 8 way parallel build.
You should also look into enabling `ccache <http://ccache.samba.org>`__
for faster builds.

Some boards also support upload of firmware directly from make. For
example:

::

    make px4-v2-upload

will build and upload a sketch on a Pixhawk.

There are also helper make targets for specific boards, such as:

-  make clean - clean the build for non-px4 targets
-  make px4-clean - completely clean the build for PX4 targets
-  make px4-cleandep - cleanup just dependencies for PX4 targets
