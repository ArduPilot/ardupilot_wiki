.. _building-the-code:

=================
Building the code
=================

.. note:: 

   Code building has changed for newer releases to use **waf** build tools, replacing make.
   
   In most cases the build dependecies described for **make** are the same, the only part of the instructions changes is the issue of the **waf** build commmand. 
 
   see https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md

The linked articles below explain how to build ArduPilot for different
target hardware on the supported development environments (Linux,
Windows, Mac OSX). The included links also cover building the code for
ground stations.

Plane, Copter, Rover
====================

**Windows users:**

-  :ref:`Building ArduPilot on Windows10 with Bash on Ubuntu on Windows <building-ardupilot-onwindows10>`
-  :ref:`Building ArduPilot with Arduino for Windows <building-ardupilot-with-arduino-windows>`
-  :ref:`Pixhawk/PX4 on Windows with Make <building-px4-with-make>` 
-  :ref:`Editing & Building with Atmel Studio or Visual Studio <building-ardupilot-apm-with-visual-studio-visual-micro>`

**MacOS users:**

-  :ref:`APM2.x on MacOS with Arduino <building-the-code-on-mac>`
-  :ref:`Pixhawk/PX4 on MacOs with Make <building-px4-with-make-on-mac>`

**Linux users:**

-  :ref:`APM2.x on Linux with Make <building-the-code-onlinux>`
-  :ref:`Pixhawk/PX4 on Linux with Make <building-px4-for-linux-with-make>`
-  :ref:`Beaglebone Black with Make <building-for-beaglebone-black-on-linux>`
-  :ref:`Building for Flymaple on Linux <building-apm-for-flymaple>`
-  :ref:`Building for NAVIO+ on RPi2 <building-for-navio-on-rpi2>`
-  :ref:`Building for NAVIO2 on RPi3 <building-for-navio2-on-rpi3>`
-  :ref:`Building for Erle-Brain <building-for-erle-brain>`
-  :ref:`Building for Erle-Brain 2 <building-for-erle-brain-2>`
-  :ref:`Building for Bebop on Linux <building-for-bebop-on-linux>`
-  :ref:`Building for Bebop 2 on Linux <building-for-bebop-2>`

**IDE/Cross platform**

-  :ref:`Building With Make (Win, Mac, Linux) <building_with_make>`

**Related information**

-  :ref:`Git Submodules <git-submodules>`

Mission Planner
===============

-  :ref:`Building Mission Planner with Visual Studio <buildin-mission-planner>`

.. toctree::
    :maxdepth: 1

    Building Pixhawk/PX4 on Linux with make <building-px4-for-linux-with-make>
    Building for Pixhawk/PX4 on Windows with Make <building-px4-with-make>
    Building for Pixhawk/PX4 on Mac with Make <building-px4-with-make-on-mac>
    Building for Pixhawk/PX4 using Eclipse on Windows <editing-the-code-with-eclipse>
    Building for Pixhawk/PX4 on Windows or Linux with QtCreator <building-px4-with-qtcreator>
    Building for APM2.x with Make (Win, Mac, Linux) <building_with_make>
    Building ArduPilot for APM2.x on Windows with Arduino <building-ardupilot-with-arduino-windows>
    Building for APM2.x on Mac with Arduino <building-the-code-on-mac>
    Building APM with Atmel Studio or Visual Studio <building-ardupilot-apm-with-visual-studio-visual-micro>
    Building for NAVIO+ on RPi2 <building-for-navio-on-rpi2>
    Building for NAVIO2 on RPi3 <building-for-navio2-on-rpi3>
    Building Mission Planner with Visual Studio <buildin-mission-planner>
    Building for Erle-Brain 2 <building-for-erle-brain-2>
    Building for Erle-Brain <building-for-erle-brain>
    Building for Bebop 2 <building-for-bebop-2>
    Building for Qualcomm Snapdragon Flight Kit <building-for-qualcomm-snapdragon-flight-kit>
    Building for Bebop on Linux <building-for-bebop-on-linux>
    Building for BeagleBone Black <building-for-beaglebone-black-on-linux>
    Building for Flymaple on Linux <building-apm-for-flymaple>
    Git Submodules <git-submodules>