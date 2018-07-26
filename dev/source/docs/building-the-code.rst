.. _building-the-code:

=================
Building the code
=================

The linked articles below explain how to setup your build environment on Linux/Ubuntu, MacOS or Windows and then build ArduPilot with either `waf <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ or make.

The instructions below assume that you have already :ref:`installed git <git-install>`, :ref:`forked <git-fork>` and :ref:`cloned <git-clone>` the ArduPilot repo.

Setting up the Build Environment
--------------------------------

- :ref:`Setup the Build Environment on Linux/Ubuntu <building-setup-linux>`
- :ref:`Setup the make Build Environment on Windows <building-setup-windows>`
- :ref:`Setup the waf Build Environment on Windows10 using WSL <building-setup-windows10>`
- :ref:`Setup the waf Build Environment on Windows using Cygwin <building-setup-windows-cygwin>`
- :ref:`Setup the Build Environment on MacOSX <building-setup-mac>`

Building / Compiling
--------------------

ArduPilot currently supports two build systems, `waf <https://waf.io/>`__ and **make** with waf being the recommended option especially for Linux/Ubuntu and MacOS users.
In most cases the build dependencies described for **waf** and **make** are the same, the only part of the instructions that changes is the build commmand. 

**Linux / MacOSX users:**

- Linux and MacOSX users should build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

**Windows users:**

Windows users have 3 options for setting up the build environment. All of the below options will allow building of native (SITL) and Pixhawk-based boards.

- :ref:`Building for Pixhawk on Windows with Make and Eclipse <building-px4-with-make>` 
- :ref:`Setup the Build Environment on Windows10 using WSL <building-setup-windows10>` 
- :ref:`Setup the Build Environment on Windows using Cygwin <building-setup-windows-cygwin>` 

**Board specific instructions:**

- :ref:`Building for NAVIO2 on RPi3 <building-for-navio2-on-rpi3>`
- :ref:`Building for Erle-Brain 2 <building-for-erle-brain-2>`
- :ref:`Building for Erle-Brain <building-for-erle-brain>`
- :ref:`Building for Bebop2 on Linux <building-for-bebop-2>`
- :ref:`Building for Bebop on Linux <building-for-bebop-on-linux>`
- :ref:`Building for Qualcomm Snapdragon Flight Kit <building-for-qualcomm-snapdragon-flight-kit>`
- :ref:`Building for Beaglebone Black <building-for-beaglebone-black-on-linux>`
- :ref:`WIP: OpenSolo QuickStart <solo_opensolo_quickstart>`

Mission Planner
---------------

- :ref:`Building Mission Planner with Visual Studio <buildin-mission-planner>`



Links to current build pages
----------------------------

.. toctree::
    :maxdepth: 1

    Setup the Build Environment on Linux/Ubuntu <building-setup-linux>
    Setup the make Build Environment on Windows <building-setup-windows>
    Setup the waf Build Environment on Windows10 using WSL <building-setup-windows10>
    Setup the waf Build Environment on Windows using Cygwin <building-setup-windows-cygwin>
    Setup the Build Environment on MacOSX <building-setup-mac>
    Building for Pixhawk on Windows with Make <building-px4-with-make>
    Building for NAVIO2 on RPi3 <building-for-navio2-on-rpi3>
    Building for Erle-Brain 2 <building-for-erle-brain-2>
    Building for Erle-Brain <building-for-erle-brain>
    Building for Bebop 2 <building-for-bebop-2>
    Building for Bebop on Linux <building-for-bebop-on-linux>
    Building for Qualcomm Snapdragon Flight Kit <building-for-qualcomm-snapdragon-flight-kit>
    Building for BeagleBone Black <building-for-beaglebone-black-on-linux>
    Building Mission Planner with Visual Studio <buildin-mission-planner>
    ArduPilot Pre-Built Binaries <pre-built-binaries>
    WIP: OpenSolo QuickStart <solo-opensolo-quickstart>


Deprecated Instructions
-----------------------

.. toctree::
    :maxdepth: 1
    
    Deprecated: Building ArduPilot with Arduino for Windows <building-ardupilot-with-arduino-windows>
    Deprecated: Editing & Building with Atmel Studio or Visual Studio <building-ardupilot-apm-with-visual-studio-visual-micro>
    Deprecated: Building for APM2.x with Make (Win, Mac, Linux) <building_with_make>
    Deprecated: APM2.x on MacOS with Arduino <building-the-code-on-mac>
    Deprecated: APM2.x on Linux with Make <building-the-code-onlinux>
    Deprecated: Building for Flymaple on Linux <building-apm-for-flymaple>
    Deprecated: Building for Pixhawk on Windows or Linux with QtCreator <building-px4-with-qtcreator>
    Deprecated: Building for NAVIO+ on RPi2 <building-for-navio-on-rpi2>


