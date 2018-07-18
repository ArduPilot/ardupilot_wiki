.. _building-setup-linux:

===============================================
Setting up the Build Environment (Linux/Ubuntu)
===============================================

This article shows how to setup your build environment on Linux/Ubuntu machines.

Setup on Ubuntu
===============

Run the install-prereqs-ubuntu.sh script:

::

    Tools/scripts/install-prereqs-ubuntu.sh -y

Also follow the `MAVProxy's install instructions <https://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html#linux>`__ if you plan to use the simulator.

Reload the path (log-out and log-in to make permanent):

::

    . ~/.profile

Now you should be able to build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

Setup for other Distributions
=============================

To build for a Cube/Pixhawk target on Linux you need the
following tools and git repositories:

-  The gcc-arm cross-compiler from `here <http://firmware.ardupilot.org/Tools/STM32-tools/>`__
-  gnu make, gawk and associated standard Linux build tools
-  On a 64 bit system you will also need to have installed libc6-i386.

Also, it's worth mentioning here that you want to ensure that the
modemmanager package is not installed and the modem-manager process is
not running.

Permissions
-----------

You need to make your user a member of the dialout group:

::

    sudo usermod -a -G dialout $USER

You will need to log out and then log back in for the group change to take effect.

Compiler
--------

You need the specific gcc-arm cross-compiler linked above. You need to unpack it:

::

    tar -xjvf gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2

and then add the bin directory from the tarball to your $PATH by editing
the $HOME/.bashrc file and adding a line like this to the end:

``export PATH=$PATH:/home/your_username/bin/gcc-arm-none-eabi-4_9-2015q3/bin``

Now you should be able to build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

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

Cleaning
--------

If there have been updates to some git submodules you may need to do a full clean build. To do that use:

::

    make px4-clean

that will remove the *PX4NuttX* archives so you can do a `build <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ from scratch
