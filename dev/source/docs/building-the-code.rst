.. _building-the-code:

=================
Building the code
=================

The linked articles below explain how to setup your build environment on Linux/Ubuntu, MacOS or Windows and then build ArduPilot with `waf <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

The instructions below assume that you have already :ref:`installed git <git-install>`, :ref:`forked <git-fork>` and :ref:`cloned <git-clone>` the ArduPilot repo.

Setting up the Build Environment
--------------------------------

- :ref:`Setup the Build Environment on Linux/Ubuntu <building-setup-linux>`
- :ref:`Setup the Build Environment on Windows <building-setup-windows>`
- :ref:`Setup the Build Environment on MacOSX <building-setup-mac>`

Building / Compiling
--------------------

**Linux / MacOSX users:**

- Linux and MacOSX users should build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

.. note::

   Do not use `sudo` unless specified in the instructions.

.. youtube:: lNSvAPZOM_o

**Windows users:**

- Windows users should follow the WSL setup directions for Windows 10 described in  :ref:`building-setup-windows10_new` or :ref:`building-setup-windows11` for Windows 11 systems . This creates a Linux-like environment called WSL/WSL2 from which the user can setup the build environment as explained in :ref:`Setup the Build Environment on Linux/Ubuntu <building-setup-linux>` and then use the instructions in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

**Board specific instructions:**

- :ref:`Building for NAVIO2 on RPi3 <building-for-navio2-on-rpi3>`
- :ref:`Building for Bebop2 on Linux <building-for-bebop-2>`
- :ref:`Building for Bebop on Linux <building-for-bebop-on-linux>`
- :ref:`Building for Beaglebone Black <building-for-beaglebone-black-on-linux>`

Mission Planner
---------------

- :ref:`Building Mission Planner with Visual Studio <building-mission-planner>`



Links to other pages of interest or advanced topics
---------------------------------------------------

.. toctree::
    :maxdepth: 1

    Setup the Build Environment on Linux/Ubuntu <building-setup-linux>
    Setup the Build Environment on Windows <building-setup-windows>
    Setup the waf Build Environment on Windows10 using WSL <building-setup-windows10_new>
    Setup the waf Build Environment on Windows11 using WSL2 <building-setup-windows11>
    Setup the waf Build Environment on Windows using Cygwin <building-setup-windows-cygwin>
    Setup Eclipse on Windows for building with waf <building-setup-windows-eclipse>
    Setup the Build Environment on MacOSX <building-setup-mac>
    Building for NAVIO2 on RPi3 <building-for-navio2-on-rpi3>
    Building for Erle-Brain 2 <building-for-erle-brain-2>
    Building for Erle-Brain <building-for-erle-brain>
    Building for Bebop 2 <building-for-bebop-2>
    Building for Bebop on Linux <building-for-bebop-on-linux>
    Building for BeagleBone Black <building-for-beaglebone-black-on-linux>
    Building Mission Planner with Visual Studio <building-mission-planner>
    ArduPilot Pre-Built Binaries <pre-built-binaries>

