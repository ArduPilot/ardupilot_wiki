.. _porting:

========================================
Porting to a new flight controller board
========================================

ArduPilot :ref:`supports a wide variety of flight controllers <common-autopilots>` with new controllers being added all the time.  This page spells out the steps to port ArduPilot to a new board with an emphasis on porting to STM32 based boards (the most common type) using `ChibiOS <http://www.chibios.org/dokuwiki/doku.php>`__.

Consider joining the `ArduPilot/ChibiOS gitter channel <https://gitter.im/ArduPilot/ChibiOS>`__ to speak with other developers about this topic.

..  youtube:: y2KCB0a3xMg
    :width: 100%

Step 1 - getting started
------------------------

- determine which microcontroller the new flight controllers uses.  if it is a CPU we already support (STM32F42x, STM32F40x STM32F41x, STM32F745, STM32F765 or STM32F777 where “x” can be any number), then the port should be relatively straight forward.  If it is another CPU, ping us on the `ArduPilot/ChibiOS gitter channel <https://gitter.im/ArduPilot/ChibiOS>`__ for advice on how to proceed.
- determine the crystal frequency (normally 8Mhz or 24Mhz).  refer to the schematic or read the writing on the crystal which is normally a small silver square.

Step 2 - create a hwdef.dat file for the board
----------------------------------------------

- make a subdir in `libraries/AP_HAL_ChibiOS/hwdef <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef>`__ for your board (i.e. “new-board”).  This directory name will eventually be used during the build process (i.e. “waf configure --board new-board”) so keep the name relatively short.
- copy/rename an existing template hwdef.dat that is similar to the CPU for your board into the directory created above.  For example, if the board has a STMF40x chip copy the `f405-min/hwdef.dat <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/f405-min>`__ file into the new directory.

Step 3 - configure and build a minimal firmware for the board
-------------------------------------------------------------

Follow the :ref:`Building the code <building-the-code>` instructions or take a shortcut and read the `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ file which includes doing the following:

- ``cd ardupilot`` (or wherever you have :ref:`cloned <git-clone>` ArduPilot to)
- ``./waf configure --board new-board``
- ``./waf copter``

If successful the build should produce an .apj file in build/new-board/bin/arducopter.apj

Step 4 - upload an ArduPilot compatible bootloader to the board
---------------------------------------------------------------

Some boards come with a bootloader pre-installed while others rely on the board manufacturer to use `dfu <http://dfu-util.sourceforge.net/>`__ to install the firmware to the board.  In either case, in order to conveniently load ArduPilot to the board over USB, an ArduPilot compatible bootloader must be uploaded to the board using `dfu <http://dfu-util.sourceforge.net/>`__. "dfu" can be downloaded from `here <http://dfu-util.sourceforge.net/>`__.

The source code for the bootloaders can be found in `AP_Bootloader
<https://github.com/ArduPilot/ardupilot/tree/master/Tools/AP_Bootloader>`__
but pre-compiled binaries are available for many boards in the
`Tools/Tootloaders
<http://firmware.ardupilot.org/Tools/Bootloaders>`__ directory on our
firmware server.  Please refer to the `README.txt <https://github.com/ArduPilot/ardupilot/blob/master/Tools/bootloaders/README.txt>`__ to see if one of the existing bootloaders is compatible for the new board.

.. note::

   please see the section at the end of this document on how to create a bootloader for your board

.. note::

   Your board must be plugged into USB *and* in DFU mode.  DFU mode is usually entered by shorting two pins together on the board.  Please see your board's documentation for details on how to accomplish this.

Upload the bootloader to the board ``dfu-util -a 0 --dfuse-address 0x08000000 -D new-board-bootloader.bin -R``

Step 5 - upload the minimal firmware onto the board
---------------------------------------------------

If using Mission Planner to load the firmware to the board:

- connect the board to the windows PC with a USB cable
- go to MP’s Initial Setup >> Install Firmware screen and click on the **Load custom firmware** and select the .apj file and press OK.  If the "Load custom firmware" link it not available go to the Config/Tuning >> Planner page and set the "Layout" to "Advanced"
- if the MP fails to load the firmware to the board it is possible the “APJ_BOARD_ID” from your hwdef.dat file does not match the .apj firmware file.  The board-id in the bootloader is listed in the bootloader's `README.txt <https://github.com/ArduPilot/ardupilot/blob/master/Tools/bootloaders/README.txt>`__ file.  A temporary work around is to change the APJ_BOARD_ID in the hwdef.dat file to match the bootloader's.  Longer term a bootloader specific to the new board needs to be created so that ground stations can differentiate this board from others and automatically load the correct firmware.

  .. note::

     Any time you make a change to the board definition file, you must clean up the build, and reconfigure WAF before re-compiling:
- ``./waf distclean``
- ``./waf config --board new-board``

.. note::

    Windows7/8 users may need to create a .ini file to allow the USB device to be recognised.  On Windows10 the board should be recognised automatically.

If using waf to upload (Linux, MacOSX only):

- connect the board to the PC with a USB cable
- commands are in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ but in short, ``./waf copter --upload``

After uploading, most likely no LEDs on the board will light up but it should be possible to connect to the board from your favourite ground station.  An error message should appear on the ground station HUD complaining, “failed to init barometer”.

Step 6 - fill in the hwdef.dat to specify pins used for each peripheral function
--------------------------------------------------------------------------------

- read the `fmuv3 hwdef.dat file <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/fmuv3/hwdef.dat>`__ (used for The Cube) to understand the full list of hardware configurations that must be specified.
- start filling in the new board’s hwdef.dat file for each bus (SPI, I2C, UART, CAN, etc).  Ideally you can refer to the board’s schematic to determine how pins should be configured but if the schematic is not available a trial-and-error approach may work because on each CPU, there are a limited number of pins that can be used for each peripheral function.  See the STM*.py scripts in the `AP_HAL_ChibiOS/hwdef/scripts directory <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef/scripts>`__ as a guide as to what pins can be used for each peripheral function
- as you enter new values into the hwdef.dat file you can re-compile and upload the firmware to test whether each peripheral function has begun working.

.. tip::

    to quickly check if the hwdef.dat file has any errors, run the `libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py>`__ script on the new hwdef.dat file and look for errors and warnings in the output

Step 7 - bring up the sensors
-----------------------------

similar to step 6, add the sensor related configuration to the hwdef.dat file
start with the baro first, then IMU, then compass and finally any other sensors
the default sensor orientation should also be filled in along with other things

upload and the firmware and test the sensors are working.

Step 8 - enable parameter storage
---------------------------------

For boards with storage, the storage method used (either FRAM or Flash) should be specified in the hwdef.dat file.

For an example of how FRAM is enabled, search for “ramtron” in the `fmuv3 hwdef.dat <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/fmuv3/hwdef.dat>`__ file.  In short you add a couple of lines like this:

- ``# enable RAMTROM parameter storage``
- ``define HAL_WITH_RAMTRON 1``

For boards using Flash, the bootloader load address needs to be selected so that loading the code does not erase the parameters.  See the FLASH_RESERVE_START_KB value in `skyviper-f412 <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/skyviper-f412/hwdef.dat>`__ and `skyviper-v2450 <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/skyviper-v2450/hwdef.dat>`__ as a reference.

It is also possible to use ardupilot on a board with no storage.  In this case configuration parameters will have their default values at startup.

The paramter defaults can be defined by creating a new file in the `/Tools/Frame_params <https://github.com/ArduPilot/ardupilot/tree/master/Tools/Frame_params>`__ directory and then add a reference to this file at the bottom of the hwdef.dat file like this:

- ``env DEFAULT_PARAMETERS '<path to defaults file>’``

Here is `how it was done for the skyviper <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/skyviper-v2450/hwdef.dat#L56>`__

Creating a Bootloader
---------------------

When doing an initial port you may be happy to use a bootloader that
was built for another board. That gets you going quickly, but also
means the bootloader will not have the right board ID for your board,
and may not have the right LED displays.

To create a bootloader that is just right for your board you need to
build the a hwdef-bl.dat for your board. That goes in the same
directory as your hwdef.dat, and has the same format, but should not
include things like I2C, SPI or CAN peripherals. There are lots of
examples of hwdef-bl.dat files already in the hwdef directory you can
use as examples.

The key things you must have in your hwdef-bl.dat are:

- You must set FLASH_BOOTLOADER_LOAD_KB to the location in kilobytes where the main code will start. This should be the same as FLASH_RESERVE_START_KB from your main hwdef.dat.
- you must set FLASH_RESERVE_START_KB to zero (so the bootloader is placed at the start of flash)
- Your UART_ORDER will control what ports the bootloader will be active on. Just having OTG1 for USB is fine, or you can list some serial UARTs.

To build the bootloader you do the following:

- ``./waf configure --board YourBoard --bootloader``
- ``./waf clean``
- ``./waf bootloader``


Next Steps
----------

If you have gotten this far, congratulations you have ported ArduPilot to a new board!  Please reach out to the other developers on the `ArduPilot/ChibiOS gitter channel <https://gitter.im/ArduPilot/ChibiOS>`__ to announce your success.

For widely available boards it is very likely we will help you get the board on the official list of supported boards including automatic firmware builds, easy uploading through the ground stations and onto our wiki!  In any case, we welcome new ports so please contact us.
