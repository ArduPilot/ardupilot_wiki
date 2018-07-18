.. _using-DFU-to-load-bootloader:

=============================
Loading a bootloader with DFU
=============================

This page describes how to load a new bootloader onto a STM32 based
board (such as a Pixhawk1) via DFU. This is useful if you are either
trying to bring up a new board or if you have a corrupted bootloader
on an existing board.

What is DFU
===========

DFU is the "Direct Firmware Update" mode for some microcontrollers,
most notably the STM32F4 series. It allows you to load a firmware
(including a bootloader) over USB using widely available DFU
utilities.

Accessing DFU Mode
==================

You access it by pulling the "boot0" pin high on the processor when it
is powered on. On a Pixhawk1 this is done by pulling the "FMU-BOOT"
pad on the top surface of the Pixhawk1 high. The FMU-BOOT pad is
located between the buzzer and DSM/Spkt connectors on the Pixhawk1.
You should pull it up to 3.3V, but in a pinch it does work to pull it
up to 5V if you don't have 3.3V handy.

When the pin is pulled up apply power to the board (eg. plug in the
USB connector) and the board should boot into DFU mode.

dfu-util tool
=============

It is recommended that you install the dfu-util tool. On Linux
machines with apt you can do that with:

 sudo apt-get install dfu-util

On other systems please see `http://dfu-util.sourceforge.net/ <http://dfu-util.sourceforge.net/>`__.

Listing DFU Devices
===================

Run the following:

.. code-block:: bash
                
  dfu-util --list

You should get a result like this:

.. code-block:: bash
                
  dfu-util --list
  dfu-util 0.8

  Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
  Copyright 2010-2014 Tormod Volden and Stefan Schmidt
  This program is Free Software and has ABSOLUTELY NO WARRANTY
  Please report bugs to dfu-util@lists.gnumonks.org

  Found DFU: [0483:df11] ver=2200, devnum=49, cfg=1, intf=0, alt=3, name="@Device Feature/0xFFFF0000/01*004 e", serial="315A35663432"
  Found DFU: [0483:df11] ver=2200, devnum=49, cfg=1, intf=0, alt=2, name="@OTP Memory /0x1FFF7800/01*512 e,01*016 e/0x1FFE7800/01*512 e,01*016 e", serial="315A35663432"
  Found DFU: [0483:df11] ver=2200, devnum=49, cfg=1, intf=0, alt=1, name="@Option Bytes  /0x1FFFC000/01*016 e/0x1FFEC000/01*016 e", serial="315A35663432"
  Found DFU: [0483:df11] ver=2200, devnum=49, cfg=1, intf=0, alt=0, name="@Internal Flash /0x08000000/04*016Kg,01*064Kg,07*128Kg,04*016Kg,01*064Kg,07*128Kg", serial="315A35663432"
  
If you don't get that then do some googling on how to debug USB connection issues with DFU.

Loading a bootloader
====================

The current bootloaders suitable for ArduPilot on STM32 are here:

  `http://firmware.ardupilot.org/Tools/Bootloaders <http://firmware.ardupilot.org/Tools/Bootloaders>`__

download the px4fmuv2_bl.bin and run this:

.. code-block:: bash
                
  dfu-util -a 0 --dfuse-address 0x08000000 -D px4fmuv2_bl.bin
  
with some versions of dfu-util you may need this instead:

.. code-block:: bash
                
  dfu-util -a 0 -s 0x08000000 --dfuse-address 0x08000000 -D px4fmuv2_bl.bin

it should say "Downloading" and show a progress bar. On completion the board is ready to test the bootloader.

After you have the bootloader loaded power cycle with the boot0 pin
pulled down (note that it is already pulled down by a resistor on a
Pixhawk1, so just power cycle).

Then check your USB bus and you should see a device "PX4 BL FMU v2.x"
with vendorID 0x26ac and productID 0x0011. You can now use the normal
firmware load tools from ArduPilot to load a flight firmware.
