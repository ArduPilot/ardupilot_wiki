.. _building_with_make:

=================================================
Archived: Building ArduPilot for APM2.x with Make
=================================================

.. warning::

   **ARCHIVED ARTICLE**

   ArduPilot no longer supports Arduino or AVR.

This article explains how to build the code for APM2.x with Make on
Windows, Mac and Linux.

.. warning::

   Copter 3.3 firmware (and later) and builds after Plane 3.4.0 no
   longer fit on APM boards. Plane, Rover and AntennaTracker builds can
   still be installed at time of writing but you can no longer build APM2.x
   off the master branch (you will need to build off a supported release
   branch).

   The last Copter firmware that can be built on APM 2.x `can be downloaded from here <https://download.ardupilot.org/downloads/wiki/firmware/ArduCopter_APM_2.0_Firmware_3.2.1.zip>`__.

Overview
========

The preferred way for developers to build APM applications is using the
Makefile system distributed with the repository. This Makefile system is
the canonical way to build APM. APM applications can no longer be built
with the standard Arduino IDE, as the APM build requires the Arduino IDE
base libraries to be excluded from the build.

For developers who would prefer not to use make, a modified Arduino IDE
which emulates the custom APM build process is available.

Requirements
============

The APM makefiles will work out of the box for Mac OS and Linux users.
Windows users can use the makefiles via a Unix compatibility system such
as Mingw or Cygwin.

Mac OS
======

Mac OS system requirements are:

-  An Arduino install of version 1.0 or greater. The Arduino
   installation provides an AVR compiler.
-  Apple Developer Tools (a free download from Apple). These provide
   command line utilities including GNU Make.

Arduino should be installed in a location that is indexed by Spotlight.

Linux
=====

Follow the instructions on :ref:`this page <building-the-code-onlinux>` to
build the code with Linux. Linux system requirements vary depending on
the distribution. The following are essential:

-  The GNU sed utility.
-  The GNU make utility, sometimes referred to as 'gmake'.
-  The GNU awk utility, often referred to as 'gawk'. **Warning:** Many
   Linux distributions do not come with 'gawk' by default. You can test
   for the presence of this tool with the 'which gawk' command.
-  The AVR-GCC toolchain.

There are various ways to get all of these utilities for your system.
You may consult `Arduino's Linux guide <http://playground.arduino.cc/learning/linux>`__ for instructions
on installing the AVR toolchain under Linux.

Windows
=======

Follow the instructions on :ref:`Building ArduPilot for APM2.x on Windows with Make <building-ardupilot-for-apm2-x-on-windows-with-make>`. These
instructions make use of the *PX4 Toolchain*'s development environment.

Preparing
=========

Locally clone the APM repository from
https://github.com/ArduPilot/ardupilot

::

    git clone https://github.com/ArduPilot/ardupilot.git
    cd ardupilot
    git submodule update --init --recursive

Building
========

To build a sketch for the default hardware platform you specified in
**config.mk**, you can invoke \ *make* with no arguments in a sketch
directory:

::

    $ cd ArduCopter
    $ make

The output of the build is, by default, located in
**$TMPDIR/_sketchname_.build**

::

    $ make
    %% param_table.o
    %% ArduCopter.cpp
    %% ArduCopter.o
    ...
    %% ArduCopter.elf
    %% ArduCopter.eep
    %% ArduCopter.hex

Uploading
=========

If you have configured the \`PORT\` variable properly, the \`upload\`
target will use \`avrdude\` to upload a built APM application to AVR
based platforms.

::

    $ cd ArduPlane
    $ make upload

For Pixhawk platforms, the \`px4-upload\` target will use the PX4 bootloader
to perform an upload.

Troubleshooting
===============

The build process itself attempts to diagnose problems that would
prevent a successful build outside of code errors. It may emit the
following diagnostics:

**WARNING: More than one copy of Arduino was found, using ...(Mac OS
only)**

Spotlight found more than one copy of Arduino installed on your system.
Check the path that is printed to ensure that the correct version is
being used. To avoid this problem, either remove old versions of Arduino
or specify the ARDUINO option when invoking the build system. Typically
the installation that will be chosen is the one that was most recently
launched.

**ERROR: must set BOARD before including this file.**

The sketch Makefile has not defined the BOARD variable. This is normally
set to \`atmega2560\` for APM2.x builds.

**ERROR: Spotlight cannot find Arduino on your system.(Mac OS only)**

Arduino is not installed, or it is installed in a location that is not
being indexed by Spotlight. You can either enable Spotlight for the
location where Arduino is installed, or specify the location explicitly
by setting the ARDUINO option as described above. Note that Spotlight
indexing may take some time, so enabling it for the location containing
Arduino may not immediately correct this issue.

**ERROR: Cannot find Arduino on this system**

(Linux and Windows only)Arduino was not found in one of the standard
locations. Either move Arduino to a standard location, or specify its
current location with the ARDUINO option in the **config.mk** file

**ERROR: cannot find the compiler tools anywhere on the path ...**

The compiler and related tools cannot be found. For Mac OS and Windows
the tools are normally part of Arduino and this message indicates that
the Arduino installation is damaged.

For Linux systems, this means that the AVR tools are not installed in a
standard location. Either set the TOOLPATH option to point to the
directory containing the AVR tools, or install them in a standard
location. Normally installing Arduino on a Linux system will result in a
correct installation of the AVR tools.

**ERROR: cannot find gawk - you may need to install GNU awk**

(Linux and Windows only)The GNU awk utility is required, but it has not
been installed or cannot be found. Check that \`gawk --version\` works
at the command promp. You may need to specify its location explicitly
with the AWK option.

.. toctree::
    :maxdepth: 1

    Building for APM2.x on Linux with Make <building-the-code-onlinux>
    Building ArduPilot for APM2.x on Windows with Make <building-ardupilot-for-apm2-x-on-windows-with-make>
