.. _pixhawk-force-px4io-flash:

==================================
Forcing a flash of the px4io board
==================================

Overview
========

When flashing a new firmware to PixHawk, an updated px4io firmware is
often flashed to the px4io board.

On occasion this flashing fails, and the board will either endlessly play the first part of the `updating IO firmware tone <https://download.ardupilot.org/downloads/wiki/pixhawk_sound_files/PX4_CompletedIOBoardFirmwareUpload.wav>`__, or will play the `failed-to-flash tone <https://download.ardupilot.org/downloads/wiki/pixhawk_sound_files/PX4_ReadyToUploadIOBoardFirmware.wav>`__.

The flash can sometimes be forced to completion; there are two ways to attempt this.


Hold safety switch during boot
==============================

Press and hold the safety switch while powering on the PixHawk.  You should hear the `updating IO firmware sound <https://download.ardupilot.org/downloads/wiki/pixhawk_sound_files/PX4_CompletedIOBoardFirmwareUpload.wav>`__ then boot should proceed `


Use the nsh console
===================

You will need to have :ref:`nsh console access <interfacing-with-pixhawk-using-the-nsh>` to attempt this method, and ONLY via USB connection with the SD card ejected.

.. note::

   On px4v1 you should disconnect the telemetry radio before attempting an update the the IO firmware

At the nsh console:

::

   px4io update /etc/px4io/px4io.bin
