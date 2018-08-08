.. _debugging-with-gdb-on-stm32:

===========================
Debugging with GDB on STM32
===========================

This page describes how to setup GDB on Linux to debug issues on STM32. The specific commands were tested on Ubuntu 13.10. GDB can
also be set-up on Windows but `there is an issue passing the Ctrl-C command to GDB <http://stackoverflow.com/questions/711086/in-gdb-on-mingw-how-to-make-ctrl-c-stop-the-program>`__
which makes it difficult to use effectively.

.. warning::

    This page has not be maintained from some time, if something is wrong please contact us on `Gitter/ArduPilot <https://gitter.im/ArduPilot/ardupilot>`__.

Introduction
============

GDB (the GNU Debugger) "allows you to see what is going on \`inside'
another program while it executes or what another program was doing at
the moment it crashed." which can be useful when investigating very
low-level failures with the STM32 (it cannot be used with the
APM1/APM2)

This guide assumes that you have already successfully built the firmware
on your machine following the instructions for
:ref:`Windows <building-setup-windows>`,
:ref:`Mac <building-setup-mac>` or
:ref:`Linux <building-setup-linux>`.

A `BlackMagic probe <http://www.blacksphere.co.nz/main/index.php/blackmagic>`__ or `ST-Link V2 JST debugger <https://www.ebay.com/itm/ST-Link-V2-Stlink-Emulator-Downloader-Programming-Mini-Unit-STM8-STM32-KK/223056820813>`__ is
also required.  A BlackMagic probe can be purchased in the US from `Transition Robotics <http://transition-robotics.com/products/black-magic-probe-mini>`__,
`Tag-Connect <http://www.tag-connect.com/BLACK-SPHERE-DBG>`__ or `1 Bit Squared <http://1bitsquared.com/collections/frontpage/products/black-magic-probe>`__
or in NewZealand from
`Greenstage <http://shop.greenstage.co.nz/product/black-magic-debug-probe>`__.

Connecting the probe to the Pixhawk
===================================

.. image:: ../images/DebuggingWithGDB_PixhawkBlackMagicProbe.jpg
    :target: ../_images/DebuggingWithGDB_PixhawkBlackMagicProbe.jpg

The BlackMagic probe should be connected to the Pixhawk's JTAG connector
using the grey 10wire cable that came with the probe. Note that most
Pixhawk come with no headers soldered onto the JTAG connector because it
interferes with the case.

Connecting the ST-Link V2 JTAG debugger to the STM32
====================================================

The ST-Link debugger should be connected to the STM32's JTAG connector
using the JTAG breakout cable. Connect the
SWDIO and SWCLK (see `Debug port pinout <http://ardupilot.org/copter/docs/common-pixracer-overview.html#debug-port-jst-sm06b-connector>`__
for reference) to the corresponding SWDIO and SWCLK pins on the ST-Link debugger.

Installing GDB
==============

If using Ubuntu, GDB is likely already installed on your machine and it
will likely work although we recommend using the version available for
download here `http://firmware.ardupilot.org/Tools/STM32-tools <http://firmware.ardupilot.org/Tools/STM32-tools>`__

The gcc-arm-none-eabi*-linux.tar.bz2 file contains both the
recommended compiler and the recommended version of gdb.

If you will be using the ST-Link debugger, please install the gdb version found
in the gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2 file.

After installation you should find you have a tool called
arm-none-eabi-gdb.

Installing OpenOCD
==================

You will need to install OpenOCD if you are using the ST-Link debugger.

OpenOCD requires libusb which you can install using
``sudo apt-get install libusb-1.0-0-dev libusb-1.0-0``.

Next, download OpenOCD from the `project's SourceForge page <https://sourceforge.net/projects/openocd/>`__.
Make sure you just install version 0.10 as other versions may not be compatible.
Unpack the file, and navigate to the folder from the command line.
Next, execute the following commands

::

    ./configure
    make
    sudo make install

Download this `openocd.cfg <https://gist.github.com/d-v/cdec6b5295c9914d76f4d6bf8b3556cb>`__
 to the directory where you have built the arducopter.elf file.

Now, to start gdb. First, type ``openocd`` in your terminal in the directory where you downloaded the openocd.cfg file.
Note: there are 2 versions of the ST-link debugger on eBay, so if
the command does not work, change the first line to ``source [find interface/stlink-v2-1.cfg]``.

In another window, type ``arm-none-eabi-gdb arducopter.elf``. Next,
type ``target extended-remote :3333``. Now you have connected to the gdb
debugging session and can use the commands from the next section.

Starting GDB and running some commands
======================================

GDB requires both the firmware file that's been uploaded to the board
(i.e. arducopter.apj) which can normally be found in Copter, Plane or
APMRover2 directory and the firmware.elf file that can be found in
the build directory.

Change to your firmware directory and type the following:

``arm-none-eabi-gdb build/fmuv3/bin/arducopter``

.. image:: ../images/DebuggingWithGDB-startGBD.png
    :target: ../_images/DebuggingWithGDB-startGBD.png

Some useful commands:

``r`` -- restarts the process

``b function-name`` -- i.e. b setup -- sets a breakpoint at the start of
the "setup" function. Note a class name can be prepended such as
``b AC_AttitudeControl::init``

``Ctrl-C`` -- stops the code from executing so you can set breakpoints,
etc

``continue`` -- continues the code from wherever it was stopped

``show interrupted-thread`` -- shows address where execution has stopped
(see below)

``info line * <address>`` -- shows c++ line for a given address (i.e.
from show interrupted-thread)

``disassemble <address>`` -- converts given address into assembler code

``exit`` -- exits from the GDB

.. image:: ../images/GDB_commands2.jpg
    :target: ../_images/GDB_commands2.jpg
