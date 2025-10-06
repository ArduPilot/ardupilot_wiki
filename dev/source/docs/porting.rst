.. _porting:

========================================
Porting to a new flight controller board
========================================

ArduPilot :ref:`supports a wide variety of flight controllers <common-autopilots>` with new controllers being added all the time.  This page spells out the steps to port ArduPilot to a new board with an emphasis on porting to STM32 based boards (the most common type) using `ChibiOS <http://www.chibios.org/dokuwiki/doku.php>`__.

.. note:: Any firmware customization of ArduPilot code must abide by the terms of the GPL3.0+ open source software license. ArduPilot also reminds developers and manufacturers to adhere to the appropriate trademark and copyright laws when developing new autopilots

.. image:: ../../../images/gpl3.png
    :target: https://www.gnu.org/licenses/gpl-3.0.en.html


Consider joining the `ArduPilot Discord Chat <https://ardupilot.org/discord>`__ to speak with other developers about this topic.

..  youtube:: y2KCB0a3xMg
    :width: 100%


.. note:: Creating a fully functional, acceptable board port can be a non-trivial task. Meeting the code, as well as required documentation, requirements is sometimes a lengthy, iterative process. You may want to consider using outside consulting services such as `Foss UAV <https://fossuav.com/>`__  or  `ArduPilot Documentation Consulting <https://www.hwurzburg.com/>`__ . In addition, if you wish to have a dedicated wiki page for you board, wiki documentation consultants such as our `Wiki Maintainer <https://www.hwurzburg.com/>`__ can assist in this.

Steps
=====
#. :ref:`Getting Started <getting_started>`
#. :ref:`Create the hardware definition files <create-hwdefs>` for the bootloader and autopilot/peripheral
#. :ref:`Build the firmware <build_firmware>`
#. :ref:`Test the firmware <test-firmware>`
#. :ref:`Create an acceptable PR (pull request) <submit-pr>` for dev team review with documentation that meets ArduPilot's requirements.

.. _getting_started:

Step 1 - Getting Started
------------------------
- Set up the local build environment on your computer. See :ref:`building-the-code` and create a local working branch for your board code development.
- Create a PR against the Tools/AP_Bootloaders/board_types.txt file with a unique board ID for your device. This is usually quickly merged to reserve that board id and prevent other developers from claiming it for their board.
- determine which microcontroller the new flight controllers uses. If it is a MCU we already support, for example STM32F405, STM32F427, STM32F745, STM32F765, STM32F777, STM32H743, or STM32H757, then the port should be relatively straight forward. If it is another MCU, ping us on the `ArduPilot Discord Chat <https://ardupilot.org/discord>`__ for advice on how to proceed.
- determine the crystal frequency (normally 8Mhz or 24Mhz). Refer to the schematic or read the writing on the crystal which is normally a small silver square.

.. note::

    The MCU must have at least 1 MB of flash to run the flight controller code. However, processors with lower flash memory can be used to develop DroneCAN peripherals which integrate many of ArduPilot's peripheral drivers for airspeed sensors, gps, compass, baro, etc. See the :ref:`ap-peripheral-landing-page` section for more information.

.. tip:: Choose your board name carefully! Use 13 characters or less for your board name, otherwise it may be truncated when the board name is sent from the flight controller to a ground station such as Mission Planner.

.. _create-hwdefs:

Step 2 - create a hwdef.dat and hwdef-bl.dat files for the board
----------------------------------------------------------------
- make a local subdir in `libraries/AP_HAL_ChibiOS/hwdef <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef>`__ for your board (i.e. “/YourBoard”).  This directory name will eventually be used during the build process (i.e. “waf configure --board new-board”) so keep the name relatively short and descriptive of your board, similar, if not the same as, the board id you created above. This directory name will also be used as the firmware name and in MAVLink messages.
- copy/rename an existing template hwdef.dat that is similar to the CPU for your board into the directory created above. For example, if the board has a STMF40x chip, the `MatekF405 <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef.dat>`__, `SpeedyBeeF405Mini <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/SpeedyBeeF405Mini/hwdef.dat>`__, or `MambaF405v2 <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/MambaF405v2/hwdef.dat>`__ hwdefs (among others) provide examples from which to start. For H7 boards with an IOMCU co-processor the `Cube Orange Plus <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef/CubeOrangePlus>`__  or `Pixhawk6X <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X>`__ are good starting points. If the board does not include an IOMCU, then omitting ``IOMCU_UART USART6`` from the hwdef.dat is all that is required.

.. tip:: The `FMUV3 board <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/fmuv3/hwdef.dat>`__ is commented heavily and contains most of the HAL directives used in hardware definition files.

.. tip:: The `scripts directory <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef/scripts>`__ contains pin function assignments for the ArduPilot supported microprocessors for reference.

.. _build_firmware:

Step 3 - Build the firmware
---------------------------
- First, create the bootloader. To create a bootloader that is just right for your board you need to build the a hwdef-bl.dat for your board. That goes in the same directory as your hwdef.dat, and has the same format, but should not include things like I2C, SPI or CAN peripherals. There are lots of examples of hwdef-bl.dat files already in the `hwdef <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef>` directory you can use as examples.

The key things you must have in your hwdef-bl.dat are:

- You must set FLASH_BOOTLOADER_LOAD_KB to the location in kilobytes where the main code will start. This should be the same as FLASH_RESERVE_START_KB from your main hwdef.dat.
- you must set FLASH_RESERVE_START_KB to zero (so the bootloader is placed at the start of flash)
- Your SERIAL_ORDER will control what ports the bootloader will be active on. Just having OTG1 for USB is fine, or you can list some serial UARTs.

.. note:: if the board has SD card capability, you may want to include the capability of loading firmware directly from the SD card in the bootloader. See :ref:`Building autoflash firmware <building-autoflash-firmware>` for more information on this feature and instructions on how to modify your files to allow this capability to be included in your firmware.

To build the bootloader you do the following:

.. code::

    Tools/scripts/build_bootloaders.py YourBoard 

- the bootloader will be created in the local ``Tools/bootloaders`` directory in your local working branch.

Then create the board firmware. Follow the :ref:`Building the code <building-the-code>` instructions or take a shortcut and read the `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ file which includes doing the following:

.. code ::

    cd ardupilot (or wherever you have :ref:`cloned <git-clone>` ArduPilot to)
    ./waf configure --board YourBoard
    ./waf copter  (or plane or whichever vehicle you are going to test the firmware on.)

If successful the build should produce an .apj file in ``ardupilot/build/YourBoard/bin/``

.. _test-firmware:

Step 4 - Load and test the firmware
-----------------------------------
Use `dfu <http://dfu-util.sourceforge.net/>`__ to install the firmware to the board. See :ref:`common-loading-firmware-onto-chibios-only-boards`.

.. note::

   Your board must be plugged into USB *and* in DFU mode. DFU mode is usually entered by shorting two pins together on the board or using its "boot" button as it powers up. Please see your board's documentation for details on how to accomplish this.


After the first firmware install with bootloader is done, you can upload new firmware to your board at the same time a new firmware build is done:

- connect the board to the PC with a USB cable
- commands are in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ but in short, ``./waf copter --upload``

Or simply connect to the board from your favourite ground station. Error messages should appear on the ground station HUD complaining, “failed to init xyz" if that sensor or peripheral has not been configured properly in the hwdef.dat

A board port submitted to ArduPilot should be tested before submitting to ArduPilot for inclusion in the project.

The test setup should have at least a receiver and GPS/Compass attached to the board. Load the firmware and test the following:

- the board boots and connects to a GCS via USB (Mission Planner, QGC, MAVProxy,etc.)
- the GPS and Compass is recognized.
- the RC is recognized and RC input follows the TX
- the pitch and roll reports (ie HUD horizon) follows autopilot movement correctly (ie IMU is oriented properly and working). test each IMU individually using the INSx_USE parameters.
- the board arms with default arming checks(may need to force arm if GPS does not have a good enough fix / is indoors)
- attach a battery either directly if onboard power sensors or via external power module and make sure voltage and current indications are correctly displayed
- attach a test servo to each output after setting the output to a normal function like elevator, and exercise each one with TX while in MANUAL mode to check output functionality. Bdshot capable outputs should be tested with a BLHeli32/AM32 esc for passthrough mode communication (non-IOMCU outputs).
- move the GPS to each UART output after setting all other UART protocols to NONE and the tested UART to GPS to assure the UART is functioning.
- UARTs with CTS/RTS lines should use a telemetry radio with those connected to be sure they function with BRD_SERx__CTSRTS=1

.. _submit-pr:

Submit the board to ArduPilot
-----------------------------
If you have gotten this far, congratulations you have ported ArduPilot to a new board!  Please reach out to the other developers on the `ArduPilot Discord Chat in the hardware channel <https://ardupilot.org/discord>`__ to announce your success.

If you wish for the board firmware to be downloadable from ArduPilot and possibly be listed in its documentation, you will need to submit a PR (pull-request) to the ArduPilot repo. You can push your local branch to your web based ``fork`` and then submit a PR from it to ArduPilot which will be reviewed by Dev team members.The PR should contain the following files:

- hwdef.dat with correct board id
- hwdef-bl.dat with correct board id
- :ref:`README.md <readme_file>` with board pinout, images, and configuration data needed for a user.This has some fairly rigid requirements for content and format. A link to this file will be included in the ArduPilot wiki once the board is merged.If you wish a dedicated page in the Wiki for your board, you can develop a Wiki PR for it or have it created as a service by a contractor, like our `Wiki Maintainer <https://www.hwurzburg.com>`__
- defaults.parm if board specific defaults are needed. Note do not define things already defaulted. Put Serial port protocol default changes and Battery monitor params in the hwdef file, not in the defautls.param file.
- the above files should be in a single commit titled: "hwdef: YourBoard"
- in Tools: add a commit for your bootloader titled: "bootloaders:YourBoard" (so a total of two commits)
- in the PR's description box, list the tests you have done on the board.
- do **NOT** push "merge commits" after this. You do not have to keep it up to date with master. If something is added to master since you last rebased your local branch on master that causes a CI failure, use the "Update branch" button on the PR to REBASE (not merge)


Notes on parameter storage
--------------------------
For boards with storage, the storage method used (either FRAM or Flash) should be specified in the hwdef.dat file.

For an example of how FRAM is enabled, search for “ramtron” in the `fmuv3 hwdef.dat <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/fmuv3/hwdef.dat>`__ file.  In short you add a couple of lines like this:

- ``# enable RAMTROM parameter storage``
- ``define HAL_WITH_RAMTRON 1``

For boards using Flash, the bootloader load address needs to be selected so that loading the code does not erase the parameters. See the FLASH_RESERVE_START_KB value in `skyviper-f412 <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/skyviper-f412/hwdef.dat>`__ and `skyviper-v2450 <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/skyviper-v2450/hwdef.dat>`__ as a reference.

It is also possible to use ardupilot on a board with no storage.  In this case configuration parameters will have their default values at startup.

The parameter defaults can be defined by creating a defaults.parm file in the same directory as the hwdef.dat file.  In the case that you are including another hwdef.dat, you may also consider using ``@include PATH_TO_OTHER_DEFAULTS_PARM`` in your new defaults.parm file.

Here is `how it was done for the skyviper <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/skyviper-v2450/defaults.parm>`__



.. toctree::
    :hidden:
    
    readme_file
    common-install-sdcard
