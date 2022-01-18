.. _common-oem-customization:

=================
OEM Customization
=================

ArduPilot provides several ways for an OEM to provide firmware on their products that has been customized to their specific system configuration:

- Ability to have specific defaults for parameters to match included system components, such as gimbals, or entire systems' peripherals, such as in ready-to-fly vehicles. This allows the user to "reset to defaults" using Mission Planner or MAVProxy in case of a problem with parameters having been accidentally changed,  and only have to re-calibrate compass/IMU/and RC, at most, in order to be ready to fly.

- Ability to provide LUA scripts in ROM for special functions without the end-user having to load them on the SD card

Customization Steps
===================

This assumes that the OEM has setup the build environment (:ref:`building-the-code`) and cloned the ArduPilot GitHub repo locally (:ref:`where-to-get-the-code`), in order to build its own customized version of the firmware.

1. Create a branch with the version of firmware you wish to base the customization upon. This will usually be the current stable version. In order to do this for ArduPlane Stable, for example, assuming you are already in the ArduPilot directory on your PC:

::

    git fetch https://github.com/ArduPilot/ardupilot.git <version>
                           where <version> is the tag for the stable version for the
                           desired vehicle: ArduPlane-stable, ArduCopter-stable,
                           APMrover2-stable,etc.

    git checkout -b <your branch name> FETCH_HEAD
    git submodule update --init --recursive

2. In the libraries/AP_HAL_ChibiOS/hwdef directory, create a new subdirectory for your customized  board definitions....in this example the directory will be named ``OEM_MatekF405-Wing`` to create a derivative for that board.

3. Create a new hwdef.dat file in this format. In this case our ready-to-fly plane will be using a MatekF405-Wing autopilot and contains only a single line:


                   include ../MatekF405-Wing/hwdef.dat

4. Now in that same directory, copy the base board's hwdef_bl.dat bootloader file, and then include a file named ``defaults.parm``. This file will be the parameter overrides of the standard defaults to match your systems configuration. Things like output function assignments, auxiliary RC switches, flight and tuning parameters, etc.

.. warning:: The ``defaults.parm`` file should be as small as possible. Some boards only allow 1024 bytes total for this file. Every ASCII byte in the file counts against this limit (except for comment lines). Use integer values where possible. Below is a simple example.

::

       # setup for NTF LEDs on output5
       SERVO5_FUNCTION 120
       NTF_LED_TYPES 256

5. You can also embed LUA scripts in the ROM of the chip that will automatically be run. Since these currently only run on autopilots with a lot of flash space, they are only restricted in total aggregate size to available free flash memory. Put the scripts in a sub-directory called ``scripts``, ie 
.....AP_HAL_ChibiOS/hwdef/OEM_MatekF405-Wing/scripts. Files must end in ``.lua``. 

.. warning:: the user may also run LUA scripts off the SD card, so care should be taken in naming the embedded script files to not conflict with potential user files common names. Notification  should be provided in the product documentation of the embedded script's names for the user.

6. Now build as normal with OEM-MatekF405-Wing as the board name in the configuration. The defaults and scripts will be embedded appropriately.

Alternative To Customizing hwdef.dat
------------------------------------

Instead of creating a separate branch and modifying the hwdef file, you can also insert LUA scripts or even informational files into the ROMFS of the build. Simply go to the local ardupilot repository's "build" folder and creating a sub-folder named "ROMFS_custom". Place your LUA scripts in a sub-folder in this directory, named "scripts" (i.e. the path ardupilot/build/ROMFS_custom/scripts). You can have other sub-folders for informational files and these will be included and view-able when examining the @ROMFS folder with MAVftp.

[copywiki destination="plane,copter,rover,dev"]
