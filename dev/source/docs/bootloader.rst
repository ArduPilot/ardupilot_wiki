.. _bootloader:

==========
Bootloader
==========

The source code for the bootloaders can be found in `AP_Bootloader <https://github.com/ArduPilot/ardupilot/tree/master/Tools/AP_Bootloader>`__
but pre-compiled binaries are available for many boards in the `Tools/Bootloaders <https://firmware.ardupilot.org/Tools/Bootloaders>`__ directory on our
firmware server.  Please refer to the `README text <https://github.com/ArduPilot/ardupilot/blob/master/Tools/bootloaders/README.md>`__ to see if one of the existing bootloaders is compatible for the new board.


Flash the bootloader
====================

We have currently two ways to flash the ArduPilot bootloader:

- flashing the one embedded into the firmware with MAVLink (recommended) via a GCS
- using DFU mode to flash. For this method, refer to :ref:`using-DFU-to-load-bootloader`

With MavLink
------------

ArduPilot firmware already include the bootloader. You can flash it using the MAVLink long command : MAV_CMD_FLASH_BOOTLOADER with parameter 5 at 290876

Mavproxy
--------

In Mavproxy terminal, type : flashbootloader

Mission Planner
...............

Use the SETUP->Install Fimware page's "Bootloader Update" button

Flash Firmware via UART (Telem Port)
====================================

Certain boards are capable of updating the firmware through a serial/UART connection in addition to the USB port on the autopilot. The boards capable of this will reference a UART in their respective ``ardupilot/libraries/AP_HAL_ChibiOS/hwdef/*your autopilot*/hwdef-bl.dat`` file. After verifying the board you want to flash is capable of this, navigate to the ``ardupilot/Tools/scripts`` directory. 

Within this directory is a script called ``uploader.py`` which you will use to flash the firmware. Typing ``./uploader.py --help`` will display the different arguments available to you. These will be modified depending on your specific hardware and baudrate configuration in your setup and autopilot. In this example a CubeBlack via a USB-TTL cable is used, with the baudrate set to 921600 on the Telem1 port. 

Verify your device with a simple ``dmesg``. The output should look something like this: 

.. code:: bash
         
         ftdi_sio 1-11.4:1.0: FTDI USB Serial Device converter detected
         usb 1-11.4: Detected FT232RL 
         usb 1-11.4: FTDI USB Serial Device converter now attached to ttyUSB0 

Now that we know which device to utilize for flashing, execute the script with the appropriate flags. The ``--baud-bootloader-flash "921600"`` is important to speed up the upload process. If left at the default ``"57600"`` you could be waiting some time for the firmware to upload. The ``--buad-flightstack "921600"`` needs to match your ``SERIALX_BAUDRATE`` in order to send the mavlink commands for the update. Successful execution should output to the terminal like this: 

.. code:: bash 

   ./uploader.py --port /dev/ttyUSB0 --baud-bootloader-flash "921600" --baud-flightstack "921600" ~/ardupilot/build/CubeBlack/bin/arducopter.apj 
   Loaded firmware for 9,0, size: 1710572 bytes, waiting for the bootloader...
   If the board does not respond within 1-2 seconds, unplug and re-plug the USB connector.
   Could not get external flash size, assuming 0
   Found board 9,0 bootloader rev 5 on /dev/ttyUSB0
   Bootloader Protocol: 5
   OTP:
      type: Hex 
      idtype: T
      vid: 6e686365
      pid: 676f6c6f
      coa: REM1MzYwNDQyNjFE//////////////////////////9DVTExTDk2MDA0NjIN/////////////////////////zA2LzE4LzE5IDA2OjI0OjA2////////////////////MkRBRToxMDEx//////////////////////////////8=
      sn: 003400483238511130313538
   ChipDes:
      family: STM32F42x
      revision: 3
   Chip:
      20016419 STM32F42x_43x rev3 (no 1M flaw)
   Info:
      flash size: 2080768
      ext flash size: 0
      board_type: 9 (fmuv3)
      board_rev: 0
   Identification complete
   Setting baudrate to 921600

   Erase  : [====================] 100.0%
   Program: [====================] 100.0%
   Verify : [====================] 100.0%
   Rebooting.


Verify the firmware update by executing ``mavproxy.py --master /dev/ttyUSB0,921600``. 
