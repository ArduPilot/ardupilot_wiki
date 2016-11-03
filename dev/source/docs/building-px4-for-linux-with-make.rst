.. _building-px4-for-linux-with-make:

=====================================================
Building ArduPilot for Pixhawk/PX4 on Linux with Make
=====================================================

This article shows how to build ArduPilot for Pixhawk 2, Pixhawk and PX4
on Linux with *Make*.

.. note::

   The commands for building Pixhawk 2 and Pixhawk are identical
   (``make px4-v2``). Building for PX4 is the same except that
   ``make px4-v1`` is used. 

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

    Tools/scripts/install-prereqs-ubuntu.sh -y

Reload the path (log-out and log-in to make permanent):

::

    . ~/.profile

Build
-----

The commands below show how to build for Pixhawk2/Pixhawk on the
different platforms. To build for PX4 replace ``make px4-v2`` with
``make px4-v1``.

Build for Copter:

::

    cd ArduCopter
    make px4-v2

Build for Plane:

::

    cd ArduPlane
    make px4-v2

Build for Rover:

::

    cd APMrover2
    make px4-v2

Build for Antenna Tracker:

::

    cd AntennaTracker
    make px4-v2

Advanced
========

To build for a Pixhawk2/Pixhawk/PX4 target on Linux you need the
following tools and git repositories:

-  The gcc-arm cross-compiler from
   `here <http://firmware.ardupilot.org/Tools/PX4-tools/>`__
-  The ardupilot git repository from
   `github.com/ArduPilot/ardupilot <https://github.com/ArduPilot/ardupilot>`__
-  gnu make, gawk and associated standard Linux build tools
-  On Ubuntu you will need to install the genromfs package.
-  On a 64 bit system you will also need to have installed libc6-i386.

Permissions
-----------

You need to make your user a member of the dialout group:

::

    sudo usermod -a -G dialout $USER

You will need to log out and then log back in for the group change to
take effect.

Also, it's worth mentioning here that you want to ensure that the
modemmanager package is not installed and the modem-manager process is
not running.

Directory Layout
----------------

The ardupilot, PX4NuttX and PX4Firmware git checkouts all need to be in
the same directory. The makefile looks in the directory above the
ardupilot directory to find the PX4NuttX and PX4Firmware trees.

Compiler
--------

You need the specific gcc-arm cross-compiler linked above. You need to
unpack it:

::

    tar -xjvf gcc-arm-none-eabi-4_6-2012q2-20120614.tar.bz2

and then add the bin directory from the tarball to your $PATH by editing
the $HOME/.bashrc file and adding a line like this to the end:

``export PATH=$PATH:/home/your_username/bin/gcc-arm-none-eabi-4_6-2012q2/bin``

ccache for faster builds
------------------------

Installing *ccache* will speed up your builds a lot. Once you install it
(for example with "sudo apt-get install ccache") you should link the
compiler into /usr/lib/ccache like this:

::

    cd /usr/lib/ccache
    sudo ln -s /usr/bin/ccache arm-none-eabi-g++
    sudo ln -s /usr/bin/ccache arm-none-eabi-gcc

Then add /usr/lib/ccache to the front of your $PATH

Building
--------

One you have the 3 git trees and compiler setup you do the build in your
vehicle directory. For example, if building Plane then do this:

::

    cd ArduPlane
    make px4

That will build two files **ArduPlane-v1.px4** and **ArduPlane-v2.px4**.
The v1 file is for PX4v1, the v2 file is for PX4v2 (the Pixhawk).

You can also build for just one board by using "make px4-v1" or "make
px4-v2".

The first time you build it will take quite a long time as it builds the
px4 archives. Subsequent builds will be faster (especially if you setup
ccache correctly).

Loading firmware
----------------

To load the firmware onto the board use

::

    make px4-v1-upload

or

::

    make px4-v2-upload

After it says "waiting for bootloader" plugin your PX4 on USB.

If upload consistently fails in the erase step then check if you are
running 'modemmanager' which can take control of the PX4 USB port.
Removing modemmanager can help.

Cleaning
--------

If there have been updates to the PX4NuttX or PX4Firmware git submodules
you may need to do a full clean build. To do that use:

::

    make px4-clean

that will remove the *PX4NuttX* archives so you can do a build from
scratch
