.. _common-mass-storage:

[copywiki destination="copter,plane,rover,sub,planner,dev"]

=================================
USB Mass Storage (SD Card Access)
=================================

Autopilots that support this feature can be rebooted into a **USB mass storage
mode**, which exports the microSD card to a connected computer as an ordinary
USB disk. The card then appears in the host's file manager and can be read and
written at normal USB speeds.

This is much faster than downloading logs over MAVLink, and is particularly
useful for copying terrain data onto the card
or for pulling large :ref:`dataflash logs <common-downloading-and-analyzing-data-logs-in-mission-planner>`
off it without removing the card from the vehicle.

.. note::

   While in mass storage mode the autopilot is **not** running flight code. Its
   normal USB connection to the ground station disappears, and it stays in this
   mode until power is removed.

Using it
========

The vehicle must be **disarmed**. The request is rejected while armed, including
when the usual forced-reboot override is supplied.

Send ``MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`` with ``param1 = 5``
(``REBOOT_SHUTDOWN_ACTION_REBOOT_TO_MASS_STORAGE``). In MAVProxy:

.. code-block:: text

    reboot massstorage

The autopilot's normal USB interfaces will disappear and an ArduPilot mass
storage device will enumerate in their place, usually auto-mounting like any
other USB drive.

To return to normal flight operation:

#. safely unmount or eject the disk on the host computer, so pending writes are
   flushed
#. **power-cycle the autopilot** - there is no command to leave mass storage mode

.. warning::

   Do not unplug the autopilot without ejecting the disk first. As with any USB
   drive, pulling it while writes are outstanding can corrupt the filesystem on
   the card.

Transfer speed depends on how the card is attached. A board with an SPI-connected
microSD measured roughly 700 kB/s writing and 870 kB/s reading; boards using SDIO
or SDMMC are faster.

Which autopilots support it
===========================

The feature is built into the firmware, so support depends on both the board and
the firmware build. All of the following are required:

- an STM32F4, F7 or H7 autopilot
- a microSD card slot (boards that log only to an onboard dataflash chip cannot
  use this)
- USB (``OTG1``)
- normal vehicle firmware - it is never included in bootloader, IO firmware or
  AP_Periph builds

Where those conditions are met, it is **enabled by default** on builds with at
least 2MB of program space, which in practice means most H7 autopilots.

Adding it to other builds
=========================

Boards that meet the hardware requirements but do not have the feature enabled by
default can include it in a :ref:`custom build <copter:common-custom-firmware>` by
selecting the **MASS_STORAGE** feature, or when building from source with:

.. code-block:: bash

    ./waf configure --board <board> --enable-MASS_STORAGE

It can be left out of a build that would otherwise include it with
``--disable-MASS_STORAGE``. If the option is requested on a board that cannot
support it, the build fails with an explanation rather than silently producing
firmware without the feature.

Ground station support
======================

MAVProxy supports the ``reboot massstorage`` command. Support in other ground
stations is being added; until it arrives, the reboot can be triggered from any
GCS that allows a raw ``MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`` command to be sent
with ``param1 = 5``.
