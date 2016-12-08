.. _companion-computer-nvidia-tx1:

===========================================
NVidia TX1 as a Companion Computer
===========================================

This page explains how to connect and configure an `NVidia TX1 <http://www.nvidia.com/object/jetson-tx1-dev-kit.html>`__ using `AuVidea.eu's J120 carrier board <http://auvidea.eu/index.php/2016-08-06-11-12-04/2016-02-03-12-30-02/j120-super-mini-computer-with-tx1>`__ so that it is able to communicate with a Pixhawk flight controller using the MAVLink protocol over a serial connection.

Connecting the Pixhawk and TX1
==============================

.. image:: ../images/NVidiaTX1_AuvideaJ120_Pixhawk.png

Connect the Pixhawk's TELEM2 port to the J120's UART0 port's Ground, TX and RX pins as shown in the image above.

The Pixhawk and TX1 should be powered separately (the J120/TX1 through it's 12V power input port, the Pixhawk through it's POWER port).  They should be powered on at about the same time or the TX1 powered on first to avoid the Pixhawk interrupting the TX1's bootloader.

Setup the Pixhawk
=================

Connect to the Pixhawk with a ground station (i.e. Mission Planner) and set the following parameters:

-  :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 1 (the default) to enable MAVLink on the serial port.
-  :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 921 so the Pixhawk can communicate with the TX1 at 921600 baud.
 
Setup the TX1
=============

The easiest way to setup the TX1 is to flash one of the existing binaries from `firmware.ardupilot.org <http://firmware.ardupilot.org/Companion>`__ (look for images starting with "tx1") or you can create your own image by following the instructions found `here in the ardupilot companion repo <https://github.com/ArduPilot/companion/tree/master/Nvidia_JTX1/Ubuntu>`__.

-  mount the TX1 back on the NVidia development board
-  download and unzip the latest image starting with "tx1" from `firmware.ardupilot.org <http://firmware.ardupilot.org/Companion>`__
-  official instructions on flashing images can be found `here <https://devtalk.nvidia.com/default/topic/898999/jetson-tx1/tx1-r23-1-new-flash-structure-how-to-clone-/post/4784149/#4784149>`__ but in short:

    - install the TX1 on an NVidia TX1 development board
    - install JetPack on an Ubuntu machine
    - connect a USB cable from the Ubuntu machine to the TX1 development board
    - power on the TX1 development board
    - put the TX1 into bootloader mode (Hold and keep pressed the "Force-Recovery" button, press and release the "Reset" button, release the "Force-Recovery" button).  You can check the TX1 is in bootloader mode by typing "lsusb" on the Ubuntu machine and look for "NVidia".
    - on the Ubuntu machine, from the ../JetPack/TX1/Linux_for_Tegra_tx1/bootloader directory run a command like below where "IMAGE.img" is replaced with the name of the image file downloaded above: ``sudo ./tegraflash.py --bl cboot.bin --applet nvtboot_recovery.bin --chip 0x21 --cmd "write APP IMAGE.img"``

Note: instructions on how the firmware.ardupilot.org image was created can be found `here <https://github.com/yankailab/OpenKAI/blob/master/setup/setup_TX1.txt>`__.
