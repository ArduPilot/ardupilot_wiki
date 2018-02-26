.. _building-setup-mac:

=========================================
Setting up the Build Environment (MacOSX)
=========================================

This article shows how to setup your build environment on Mac OS X (ver 10.6 onwards).

#. Install `Homebrew <http://brew.sh>`__ for Mac OS X

#. Install xcode and say YES to install Command Line Tools

   ::
   
       xcode-select --install
       
#. Install the following packages using brew

   ::

       brew tap PX4/homebrew-px4
       brew update
       brew install genromfs
       brew install gcc-arm-none-eabi

#. Install the latest version of awk using brew (make sure
   **/usr/local/bin** takes precedence in your path):

   ::

       brew install gawk

#. Install *pip* and *pyserial* using the following commands:

   ::

       sudo easy_install pip
       sudo pip install pyserial future catkin_pkg empy

#. Follow the `MAVProxy's install instructions <https://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html#mac>`__ if you plan to use the simulator.

Now you should be able to build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

Cleaning
--------

If there have been updates to some git submodules you may need to do a full clean build. To do that use:

::

    make px4-clean

that will remove the *PX4NuttX* archives so you can do a `build <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ from scratch.
