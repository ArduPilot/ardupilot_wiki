.. _pixhawk-force-px4io-flash:

======================================
Forcing a flash of the IOMCU processor
======================================

Overview
========

When flashing a new firmware to autopilots with an IOMCU processor, an
updated firmware is often flashed to the IOMCU.

On occasion this flashing fails, and the board will either endlessly play the first part of the `updating IO firmware tone <https://download.ardupilot.org/downloads/wiki/pixhawk_sound_files/PX4_CompletedIOBoardFirmwareUpload.wav>`__, or will play the `failed-to-flash tone <https://download.ardupilot.org/downloads/wiki/pixhawk_sound_files/PX4_ReadyToUploadIOBoardFirmware.wav>`__.  It may also boot and inform you via a text message to your GCS that the update has failed.

The flash can sometimes be forced to completion.


Hold safety switch during boot
==============================

Press and hold the safety switch while powering on the autopilot.  You should hear the `updating IO firmware sound <https://download.ardupilot.org/downloads/wiki/pixhawk_sound_files/PX4_CompletedIOBoardFirmwareUpload.wav>`__ then boot should proceed.

.. note::

   On px4v1 you should disconnect the telemetry radio before attempting an update the IO firmware
