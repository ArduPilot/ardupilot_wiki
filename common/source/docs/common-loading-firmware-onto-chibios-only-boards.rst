.. _common-loading-firmware-onto-chibios-only-boards:

================================================================
Loading Firmware onto boards without existing ArduPilot firmware
================================================================

ArduPilot now uses the ChibiOS operating system for all autopilots, including the many smaller boards targeted for multicopter racing applications.

Often, these boards have another autopilot software pre-installed. (If the board has ArduPilot already installed, see :ref:`common-loading-firmware-onto-pixhawk` for firmware loading instructions.)

Installing ArduPilot to these autopilot involves:

- Installing the required driver and flashing tool
- Downloading the appropriate ArduPilot firmware
- Loading ArduPilot to the board

.. note::

   Instructions for ArduPilot using ChibiOS to Pixhawk autopilots can be found :ref:`here <common-loading-chibios-firmware-onto-pixhawk>`.

[copywiki destination="copter,plane,rover,planner,blimp"]

Download driver and flashing tool
=================================

The `STM32CubeProgrammer <https://www.st.com/en/development-tools/stm32cubeprog.html>`__ will install the required DFU (direct firmware upload) drivers and can be used to flash the firmware to autopilots in DFU mode. This is available for Windows, Linux, and MacOS systems. Download and install this program. You may be required to also install `JAVA <https://java.com/en/download/>`__ in order to setup this program.

Download the ArduPilot firmware
===============================

- Download the ArduPilot firmware for your board from `firmware.ardupilot.org <https://firmware.ardupilot.org/>`__.  You can normally find the appropriate firmware by doing the following:

  - open `firmware.ardupilot.org <https://firmware.ardupilot.org/>`__
  - select click on the link for your vehicle type (i.e. `Plane <https://firmware.ardupilot.org/Plane/>`__, `Copter <https://firmware.ardupilot.org/Copter/>`__, `Rover <https://firmware.ardupilot.org/Rover/>`__, `Sub <https://firmware.ardupilot.org/Sub/>`__ or `Antenna Tracker <https://firmware.ardupilot.org/AntennaTracker/>`__)
  - select "beta" or "stable"
  - look for the directory with the name that most closely matches the autopilot
  - download the "arduXXX_with_bl.hex" file clicking on it. It will usually be saved in your Downloads folder.

Using Beta and Developer Versions
=================================

Beta
----

Prior to ``Stable`` releases, a ``Beta`` version or versions get released. These may be used if you wish to try newer features or help the developers further flight test the code. Since they are "beta" versions, there possibly still may be bugs (although this is possible even in Stable firmware). However, it has been tested by the development team, and already flight tested. This release allows a wider user base to final test the firmware before its release as ``Stable``. Experienced ArduPilot users are encouraged to test fly this firmware and provide feedback.

Latest Developer Version
------------------------

This reflects the current state of the development branch of the ArduPilot code. It has been reviewed by the development team, passed all automated test suites,  and in most cases, if significant changes have been made, test flown. This code gets built daily and is available for testing by experienced users. This corresponds to an "alpha" release, and may have bugs, although very rarely "crash inducing". Very shortly after an addition that changes or introduces a feature is added, the :ref:`Upcoming Features <common-master-features>` section of the Wiki is updated with information about the addition or change.

Upload the firmware to autopilot
================================

- Hold down the board's DFU button or temporarily bridge its "BOOT" pins, and plug in a USB cable (attached to your PC). Release button or unbridge once powered.
- Open the windows device manager and look under "Universal Serial Bus devices" for "STM32 BOOTLOADER" to confirm that the board is in DFU mode.

  .. image:: ../../../images/loading-firmware-device-manager.png
      :target: ../_images/loading-firmware-device-manager.png
      :width: 450px


- Start the STM32CubeProgrammer

.. image:: ../../../images/STM32CubeProgrammer1.jpg
      :target: ../_images/STM32CubeProgrammer1.jpg


#. Select the connection method: USB
#. Make sure a USB port shows...that means the board is detected in DFU mode.
#. Press "Connect"
#. Then the boards cpu specifics will appear here.
#. Press "Open file" to select the "arduXXX_with_bl.hex" file you downloaded.
#. The file name will appear in the tab.

.. image:: ../../../images/STM32CubeProgrammer2.jpg
      :target: ../_images/STM32CubeProgrammer2.jpg


7. Press "Download" to flash the file to the board.


You may now reboot the board and :ref:`connect with your favourite ground station <common-connect-mission-planner-autopilot>` (Mission Planner, QGC, etc) and future firmware uploads should also be possible using the normal method for Pixhawk boards.

Alternate method
================


Download and Install Zadig (Windows only)
-----------------------------------------

- Download and run `Zadig <https://zadig.akeo.ie/>`__ (search for "Zadig 2.3" just below "Download") to allow accessing the board using USB.
- Choose "List all devices" option from options menu
- Select "STM32 BOOTLOADER" from the drop-down and press the "Replace Driver" button

  .. image:: ../../../images/loading-firmware-zadig.png
      :target: ../_images/loading-firmware-zadig.png
      :width: 450px

- Optionally, you may wish to check the board is visible as a USB port:

  - Hold down the board's DFU button and plug in a USB cable (attached to your PC)
  - Open the windows device manager and look under "Universal Serial Bus devices" for "STM32 BOOTLOADER" to confirm that the board is in DFU mode.

  .. image:: ../../../images/loading-firmware-device-manager.png
      :target: ../_images/loading-firmware-device-manager.png
      :width: 450px


Download the ArduPilot firmware
-------------------------------

- Download the ArduPilot firmware for your board from `firmware.ardupilot.org <https://firmware.ardupilot.org/>`__.  You can normally find the appropriate firmware by doing the following:

  - open `firmware.ardupilot.org <https://firmware.ardupilot.org/>`__
  - select click on the link for your vehicle type (i.e. `Plane <https://firmware.ardupilot.org/Plane/>`__, `Copter <https://firmware.ardupilot.org/Copter/>`__, `Rover <https://firmware.ardupilot.org/Rover/>`__, `Sub <https://firmware.ardupilot.org/Sub/>`__ or `Antenna Tracker <https://firmware.ardupilot.org/AntennaTracker/>`__)
  - select "beta" or "stable"
  - look for the directory with the name that most closely matches the autopilot
  - download the "arduXXX_with_bl.hex" file clicking on it. It will usually be saved in your Downloads folder.

Upload ArduPilot to the board
-----------------------------

- Download, install and run the `Betaflight Configurator <https://github.com/betaflight/betaflight-configurator/releases>`__.

  - Select "Firmware Flasher" on the left side of the screen
  - Select DFU from the top right
  - Push "Load Firmware [Local]" from the bottom right and select the arduXXX_with_bl.hex file you downloaded above.
  - Push "Flash Firmware" and after a few minutes the firmware should be loaded

  .. image:: ../../../images/loading-firmware-betaflight-configurator.png
      :target: ../_images/loading-firmware-betaflight-configurator.png
      :width: 450px



You may now reboot the board and :ref:`connect with your favourite ground station <common-connect-mission-planner-autopilot>` (Mission Planner, QGC, etc) and future firmware uploads should also be possible using the normal method for Pixhawk boards.

Loading firmware onto Boards with external flash
================================================

Some recent boards, most notably those from Seriously Pro Racing (http://www.seriouslypro.com/), use MCUs with small amounts of internal flash but with much larger externally connected flash chips. These boards require extra steps to load ArduPilot firmware. Typically some kind of bootloader resides on the internal flash and then the main firmware resides on the external flash.

Loading firmware using SSBL
---------------------------

The SPRacing series of boards come pre-installed with a proprietary bootloader on the internal flash and require the use of a second stage bootloader to load further firmware. There are a couple of options to load firmware with these boards, but whichever option you choose you will need to initially load ArduPilot using SSBL. Please follow the "INSTALLATION" instructions at https://github.com/spracing/ssbl in order to load SSBL onto your board. Once SSBL is loaded please follow the PX4 instructions to load ArduPilot onto the board https://github.com/spracing/ssbl#px4-installation-to-external-flash but instead of using PX4 firmware please use the arducopter.bin firmware image. A summary of the steps follows:

- Download https://github.com/spracing/ssbl/releases and install SSBL to external flash following https://github.com/spracing/ssbl#installation-to-external-flash
- Download the latest ArduPilot external flash binary, for instance https://firmware.ardupilot.org/Copter/latest/SPRacingH7/arducopter.bin
- Use dd to pad the binary to 2MB:

.. code-block:: none

   dd if=/dev/zero ibs=1k count=2048 of=AP_2MB.bin
   dd conv=notrunc if=arducopter.bin of=AP_2MB.bin
   
- Put the board into SSBL dfu mode - power off, hold BIND (not BOOT), power on - LED flashes fast, release BIND, LED flashed slow - DFU mode enabled
- Flash the binary using

.. code-block:: none

   dfu-util -D AP_2MB.bin -s 0x90100000:0x200000

- Verify the flash. The dfu-util command below copies the contents of the flash back to the computer, the diff command will tell you if the contents are identical or different. Do not attempt to fly if diff doesn't say the files are identical - retry.

.. code-block:: none

   dfu-util -U AP_2MB-VERIFY.bin -s 0x90100000:0x200000
   diff -sb AP_2MB.bin AP_2MB-VERIFY.bin

- Power off, install an SD card (.note: you *must* install an SD card, the firmware will not boot without it), power on
- Configure the board as normal using Mission Planner

At this point you should have working firmware on the board. If you want to load new firmware you will need to follow steps 2-7 again (you cannot use Mission Planner to load firmware). If you are certain that you will never want to load betaflight on the board then you can install the ArduPilot bootloader.

Installing the ArduPilot bootloader
-----------------------------------

.. warning:: installing the ArduPilot bootloader is a one-way operation. You cannot restore the board to factory configuration or load betaflight after this step - you would have to return the board to Seriously Pro to be re-flashed with factory firmware, assuming that is possible

**If you are certain that you only want to use ArduPilot on the board**, then flashing the ardupilot bootloader enables much simpler subsequent upgrades.

- You must initially have a working version of ArduPilot installed on the board - follow the steps above.
- You now must remove the copy protection on the internal flash. This is a destructive operation requiring complete erasure of the flash. ArduPilot provides support to make this easy. Set :ref:`BRD_OPTIONS<BRD_OPTIONS>` = 16.
- Power off and power on the board. The board will not appear to boot but the flash sector is being erased. Wait a few seconds and then power off the board.
- Hold down the ``boot`` button (boot *not* bind this time) and power on the autopilot. This will put the board in dfu mode.
- Download the ArduPilot bootloader, e.g. https://github.com/ArduPilot/ardupilot/blob/master/Tools/bootloaders/SPRacingH7_bl.bin
- Install the bootloader via dfu:

.. code-block:: none

   dfu-util -a 0 --dfuse-address 0x08000000 -D SPRacingH7_bl.bin

- Reboot the board.
- You can now use your favorite tool to upload the ArduPilot firmware
