.. _common-loading-firmware-onto-chibios-only-boards:

=========================================
Loading Firmware onto ChibiOS-only boards
=========================================

Recent versions of ArduPilot (Copter-3.6, Plane-3.9, Rover-3.5) run on relatively small, non-Pixhawk, flight controllers using the ChibiOS operating system.
Examples of these boards include the :ref:`OpenPilot RevoMini <common-openpilot-revo-mini>`, :ref:`Mateksys F405-Wing <common-matekf405-wing>` and :ref:`Omnibus F4 Pro <common-omnibusf4pro>`.

Installing ArduPilot to these flight controller involves:

- Installing the required driver and flashing tool
- Downloading the appropriate ArduPilot firmware
- Loading ArduPilot to the board

.. note::

   Instructions for ArduPilot using ChibiOS to Pixhawk flight controllers can be found :ref:`here <common-loading-chibios-firmware-onto-pixhawk>`.

[copywiki destination="copter,plane,rover,planner"]

Download and Install Zadig (Windows only)
-----------------------------------------

- Download and run `Zadig <https://zadig.akeo.ie/>`__ (search for "Zadig 2.3" just below "Download") to allow accessing the board using USB.
- Choose "List all devices" option from options menu
- Select "STM32 BOOTLOADER" from the drop-down and press the "Replace Driver" button

  .. image:: ../../../images/loading-firmware-zadig.png
      :target: ../_images/loading-firmware-zadig.png
      :width: 450px

- Optionally you may wish to check the board is visible as a USB port:

  - Hold down the board's DFU button and plug in a USB cable (attached to your PC)
  - Open the windows device manager and look under "Universal Serial Bus devices" for "STM32 BOOTLOADER"

  .. image:: ../../../images/loading-firmware-device-manager.png
      :target: ../_images/loading-firmware-device-manager.png
      :width: 450px

Download the ArduPilot firmware
-------------------------------

- Download the ArduPilot firmware for your board from `firmware.ardupilot.org <http://firmware.ardupilot.org/>`__.  You can normally find the appropriate firmware by doing the following:

  - open `firmware.ardupilot.org <http://firmware.ardupilot.org/>`__
  - select click on the link for your vehicle type (i.e. `Plane <http://firmware.ardupilot.org/Plane/>`__, `Copter <http://firmware.ardupilot.org/Copter/>`__, `Rover <http://firmware.ardupilot.org/Rover/>`__, `Sub <http://firmware.ardupilot.org/Sub/>`__ or `Antenna Tracker <http://firmware.ardupilot.org/AntennaTracker/>`__)
  - select "beta" or "stable"
  - look for the directory with the name that most closely matches the flight controller
  - download the "arduXXX_with_bl.hex" file

Upload ArduPilot to the board
-----------------------------

- Download, install and run the `Betaflight Configurator <https://github.com/betaflight/betaflight-configurator/releases>`__.

  - Select "Firmware Flasher" on the left side of the screen
  - Select DFU from the top right
  - Push "Load Firmware [Local]" from the bottom right and select the arduXXX_with_bl.hex file downloaded above
  - Push "Flash Firmware" and after a few minutes the firmware should be loaded

  .. image:: ../../../images/loading-firmware-betaflight-configurator.png
      :target: ../_images/loading-firmware-betaflight-configurator.png
      :width: 450px

.. note::

    We expect future versions of Mission Planner and QGroundControl will allow uploading firmware via DFU which will remove the requirement to use the Betaflight Configurator

You may now reboot the board and :ref:`connect with your favourite ground station <common-connect-mission-planner-autopilot>` (Mission Planner, QGC, etc) and future firmware uploads should also be possible using the normal method for Pixhawk boards.
