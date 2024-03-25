.. _common-install-sdcard:

[copywiki destination="copter,plane,rover,planner,blimp"]

===========================
Loading Firmware via sdcard
===========================

It is possible to update the ArduPilot firmware on certain autopilots by placing a specifically-named file onto an SD card and running the autopilot's bootloader (e.g. by power-cycling the board.

.. note:: at this time only a few autopilots have this capability. See instructions below to determine if an autopilot has this feature. Only H7 based autopilots using MMC connected SD cards are capable of this and not all that do have had the feature added as of yet.

Why would I want to do this?
============================

There are several reasons this technique may be of use:

  - There is no convenient, reliable access to the vehicle's USB port
  - It is desired to update the vehicle firmware over a telemetry radio
  - The vehicle is remote and the local operators may struggle with the normal interfaces to update the firmware. This allows OEMs to remotely add an updated file onto users' SD cards which would upload on the next boot of the vehicle.


The Bootloader
==============

Currently, no autopilot ships with a bootloader capable of flashing from the SD card.  Updating the bootloader is required to support flashing from SD card.  Note that updating the autopilot's bootloader is an operation which can make your board non-operational, and difficult to recover.  More-so with boards that do not expose a "boot0" pin, such as the CubeOrange.  Be aware of this risk, and be prepared to spend considerable time recovering a board if something bad happens when updating the bootloader.

:ref:`The instructions on updating the bootloader <common-bootloader-update>` can be followed to update your bootloader; be aware that you must use a "latest" firmware to obtain a suitable bootloader.


The Firmware File
=================

The file to be loaded onto the SD card is of a special'y named ".abin" file created for this purpose.  This is a binary file with a small amount of text providing some information about the binary. Most notably, a checksum which the bootloader will verify before attempting to flash the board.

If you are building your own firmware, and the board is configured to support flashing-from-SD-card then your build products will automatically include the .abin file.  For example:

  ::

         BUILD SUMMARY
    Build directory: /home/pbarker/rc/ardupilot/build/CubeOrange
    Target         Text (B)  Data (B)  BSS (B)  Total Flash Used (B)  Free Flash (B)  External Flash Used (B)
    ---------------------------------------------------------------------------------------------------------
    bin/arduplane   1868612      3536   258740               1872148           93928  Not Applicable

    Build commands will be stored in build/CubeOrange/compile_commands.json
    'plane' finished successfully (24.283s)
    pbarker@fx:~/rc/ardupilot(master)$ ls -l build/CubeOrange/bin
    total 18792
    -rwxrwxr-x 1 pbarker pbarker 3135448 Sep 29 19:15 arduplane
    -rw-rw-r-- 1 pbarker pbarker 1872247 Sep 29 19:15 arduplane.abin
    -rw-rw-r-- 1 pbarker pbarker 1684192 Sep 29 19:15 arduplane.apj
    -rwxrwxr-x 1 pbarker pbarker 1872152 Sep 29 19:15 arduplane.bin
    -rw-rw-r-- 1 pbarker pbarker 5148900 Sep 29 19:15 arduplane.hex
    -rw-rw-r-- 1 pbarker pbarker 5509380 Sep 29 19:15 arduplane_with_bl.hex


Boards which support flash-from-sdcard will also have ``.abin`` files available for download from `firmware.ardupilot.org <https://firmware.ardupilot.org/>`__



Firmware File Name
==================

The filename which is used when generating firmware is *not* the correct name to use when placing the firmware on the SD card.  When the ``.abin`` files are generated they contain the vehicle name, for example, "arduplane.abin".

There is only one correct filename that may be used to flash-from-sdcard; this is ``ardupilot.abin``.  When placing the file on the SD card, ensure the file has been renamed to ``ardupilot.abin``.


Transferring the File to the SD card
====================================

This can be done in your operating system as you would ordinarily interact with the SD card (e.g. file browser.).

You can also transfer the file to the SD card via ``MAVFTP``.

Ensure the file is the correct size before continuing.

After the transfer is complete, the directory listing should look something like this:

  ::

        RTL> ftp put /home/pbarker/arducopter.abin ardupilot.abin
        RTL> Putting /home/pbarker/arducopter.abin as ardupilot.abin
        Sent file of length  1847687
        RTL> ftp list
        RTL> Listing /
         D APM
           ardupilot.abin	1847687
        Total size 1804.38 kByte

Triggering the Flash Update
===========================

Power cycle the board to enter the bootloader which will automaticallt check for the firmware update file and begin flashing it.

It should take roughly 1 minute to verify the firmware and flash it to the vehicle's internal flash.

If the process completes successfully the file will renamed to ``ardupilot-flashed.abin``.  The vehicle should proceed to boot the firmware once flashing is complete.


Troubleshooting
===============

Several things can go wrong with the firmware flash, but some diagnostics are available to help work out what the problem might be.

  - At each stage of the flashing process, the ``ardupilot.abin`` file is renamed to reflect the stage
  - if the file is called ``ardupilot-verify.abin`` then the process failed when trying to checksum the file, or the board was interrupted when doing so.
  - if the file is called ``ardupilot-verify-failed.abin`` then the checksum the bootloader calculated did not match the bootloader in the ``.abin`` metadata.
  - if the file is called ``ardupilot-flash.abin`` the process failed when writing the firmware, or the board was interrupted while doing so.  The board is unlikely to boot into an ArduPilot firmware if this has happened, so a re-flash will be required.
  - if the file is called ``ardupilot-flashed.abin`` you should not need this "troubleshooting" section, as the flash process has succeeded!
