.. _building-setup-mac:

=========================================
Setting up the Build Environment (MacOSX)
=========================================

This article shows how to manually setup a minimal build environment on MacOS (ver 10.6 onwards).

..  youtube:: wLK2wLwEXm4
    :width: 100%


There is a pre-built script at /ardupilot/Tools/scripts/install-prereqs-mac.sh that will install these pre-requisites.

Setup steps
-----------

#. MacOS will alert you when you enter a command in the terminal that requires Xcode Command Line Tools. You can also install Xcode Command Line Tools manually

   ::
   
       xcode-select --install

#. Install `Homebrew <http://brew.sh>`__ for MacOS (Homebrew is a respected package manager for MacOS)

   ::
   
      /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
 
#. Install the following packages using brew

   ::

       brew tap ardupilot/homebrew-px4
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
       sudo pip install pyserial future empy

#. Follow the `MAVProxy's install instructions <https://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html#mac>`__ if you plan to use the simulator.

Now you should be able to build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

Cleaning
--------

If there have been updates to some git submodules you may need to do a full clean build. To do that use:

::

    ./waf distclean

Commands `clean` and `distclean` can be used to clean the objects produced by the build.
`clean` keeps the configure information, cleaning only the objects for the current board. `distclean` cleans everything for every board, including the saved configure information.

Follow the instructions for `build <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ .
