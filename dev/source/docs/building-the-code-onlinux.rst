.. _building-the-code-onlinux:

================================================
Building ArduPilot for APM2.x on Linux with Make
================================================

Quick start
===========

For Ubuntu, follow these steps to build the code. For other
distributions, see the advanced instructions below.

Setup
-----

Install git:

::

    sudo apt-get -qq -y install git

Clone the source:

::

    git clone https://github.com/ArduPilot/ardupilot.git
    cd ardupilot
    git submodule update --init --recursive

Run the install-prereqs-ubuntu.sh script:

::

    ardupilot/Tools/scripts/install-prereqs-ubuntu.sh -y

Reload the path (log-out and log-in to make permanent):

::

    . ~/.profile

Build
-----

Build for Copter:

::

    cd ardupilot/ArduCopter
    make

Build for Plane:

::

    cd ardupilot/ArduPlane
    make

Build for Rover:

::

    cd ardupilot/APMrover2
    make

Build for Antenna Tracker:

::

    cd ardupilot/AntennaTracker
    make

Advanced
========

To build the ardupilot firmware on Linux, you will need a cross-compiler
for the type of board you are building for. If you are using a debian
based distribution such as Ubuntu then the following should get you the
core packages you will need:

::

    sudo apt-get install gcc-avr avrdude avr-libc binutils-avr

You will also need some extra tools to use the SITL system and
additional development tools

::

    sudo apt-get install python-serial python-wxgtk2.8 python-matplotlib python-opencv python-pexpect python-scipy

Once installed, you can build by changing directory to the vehicle type
you want to build for, and running 'make' with the target you want. Look
in mk/targets.mk for a list of build targets.

Ubuntu Linux
------------

The following packages are required to build ardupilot for the APM1/APM2
(Arduino) platform in Ubuntu:

::

    gawk make git arduino-core g++

To build ardupilot for the PX4 platform, you'll first need to install
the PX4 toolchain and download the PX4 source code. See the `PX4 toolchain installation page <https://pixhawk.ethz.ch/px4/dev/toolchain_installation_lin>`__.

The easiest way to install all these prerequisites is to run the
**ardupilot/Tools/scripts/install-prereqs-ubuntu.sh** script, which will

install all the required packages and download all the required

software.

Building using make
-------------------

1. For Copter and AntennaTracker you'll need to run ``make configure``
from a sketch directory (ArduCopter or AntennaTracker) before you build
the project for the first time. This will create a **config.mk** file at
the top level of the repository. You can set some defaults in
**config.mk**

2. In the sketch directory, type ``make`` to build for APM2.
Alternatively, ``make apm1`` will build for the APM1 and \`make px4\`
will build for the Pixhawk. The binaries will generated in
\`/tmp/\ *sketchname*.build\`.

3. Type \`make upload\` to upload. You may need to set the correct
default serial port in your \`config.mk\`.
