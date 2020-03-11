.. _bootloader:

==========
Bootloader
==========

The source code for the bootloaders can be found in `AP_Bootloader <https://github.com/ArduPilot/ardupilot/tree/master/Tools/AP_Bootloader>`__
but pre-compiled binaries are available for many boards in the `Tools/Bootloaders <https://firmware.ardupilot.org/Tools/Bootloaders>`__ directory on our
firmware server.  Please refer to the `README.txt <https://github.com/ArduPilot/ardupilot/blob/master/Tools/bootloaders/README.txt>`__ to see if one of the existing bootloaders is compatible for the new board.


Flash the bootloader
====================

We have currently two ways to flash the ArduPilot bootloader :
- flashing the one embedded into the firmware with MAVLink (recommended)
- using DFU mode to flash

With MavLink
------------

ArduPilot firmware already include the bootloader. You can flash it using the mavlink command : MAV_CMD_FLASH_BOOTLOADER with parameter 5 at 290876

Mavproxy
........

In Mavproxy terminal, type : flashbootloader

.. TODO: add picture

Mission Planner
...............

Use the CTRL+F menu to show the button ``Bootloader Upgrade``

.. TODO: add picture

With DFU
--------


Some boards come with a bootloader pre-installed while others rely on the board manufacturer to use `dfu <http://dfu-util.sourceforge.net/>`__ to install the firmware to the board.  In either case, in order to conveniently load ArduPilot to the board over USB, an ArduPilot compatible bootloader must be uploaded to the board using `dfu <http://dfu-util.sourceforge.net/>`__. "dfu" can be downloaded from `here <http://dfu-util.sourceforge.net/>`__.

.. note::

   Your board must be plugged into USB *and* in DFU mode.  DFU mode is usually entered by shorting two pins together on the board.  Please see your board's documentation for details on how to accomplish this.

Upload the bootloader to the board ``dfu-util -a 0 --dfuse-address 0x08000000 -D new-board-bootloader.bin -R``