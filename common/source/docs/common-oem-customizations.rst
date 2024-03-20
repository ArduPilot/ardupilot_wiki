.. _common-oem-customization:

=================
OEM Customization
=================

ArduPilot provides several ways for an OEM to provide firmware on their products that has been customized to their specific system configuration:

- Ability to have specific defaults for parameters to match included system components, such as gimbals, or entire systems' peripherals, such as in ready-to-fly vehicles. This allows the user to "reset to defaults" using Mission Planner or MAVProxy in case of a problem with parameters having been accidentally changed, and only have to re-calibrate compass/IMU/and RC, at most, to be ready to fly.
- Ability to provide Lua scripts in ROM for special functions without the end-user having to load them on the SD card. Read more about Lua scripting in ArduPilot :ref:`here<common-lua-scripts>`.
- Ability to change the firmware string displayed to the user.
- Ability to include pictures and informational files in available free flash space.
- Ability to change parameters and mark them as read-only so that users cannot change them using :ref:`APJ Tools<dev:apjtools-intro>`.

Customization Steps
===================

This section assumes that the OEM has set up the build environment (:ref:`building-the-code`) and cloned the ArduPilot GitHub repo locally (:ref:`where-to-get-the-code`), to build its customized version of the firmware.

#. Create a branch with the version of firmware you wish to base the customization upon. This will usually be the current stable version. To do this for ArduPlane Stable, for example, assuming you are already in the ArduPilot directory on your PC:

    .. code-block:: bash

        git fetch https://github.com/ArduPilot/ardupilot.git <version>
                            where <version> is the tag for the stable version for the
                            desired vehicle: ArduPlane-stable, ArduCopter-stable,
                            APMrover2-stable,etc.

        git checkout -b <your branch name> FETCH_HEAD
        git submodule update --init --recursive

#. In the ``libraries/AP_HAL_ChibiOS/hwdef`` directory, create a new subdirectory for your customized board definitions. In this example, the directory will be named ``OEM_CubeOrange`` to create a derivative for that board.

#. Create a new ``hwdef.dat`` file in this format. In this case, our ready-to-fly plane will be using a CubeOrange autopilot and only requires a single line:

    ::

        include ../CubeOrange/hwdef.dat

#. The firmware name can be customized by adding one line to the ``hwdef.dat`` file.

    ::

        define AP_CUSTOM_FIRMWARE_STRING "MyMagicFrame"

    .. note:: Custom frame type strings can be created through Lua the scripting method ``motors:set_frame_string("Custom frame name")``.

#. Now in that same directory, copy the base board's ``hwdef_bl.dat`` bootloader file, and then include a file named ``defaults.parm``. This file will be the parameter overrides of the standard defaults to match your system's configuration. Things like output function assignments, auxiliary RC switches, flight and tuning parameters, etc.

    .. warning:: The ``defaults.parm`` file should be as small as possible. Some boards only allow 1024 bytes total for this file. Every ASCII byte in the file counts against this limit (except for comment lines). Use integer values where possible. Below is a simple example. Serial port protcols,baud rate,and options defaults can be set directly in the hwdef, as well as NTF_LED_TYPES, and battery monitor defaults, and should be done there instead of a defaults file.

    ::

        # setup for NTF LEDs on output5
        SERVO5_FUNCTION 120
        NTF_LED_TYPES 256

#. You can also embed :ref:`Lua scripts<common-lua-scripts>` in the ROM of the chip that will automatically run. Since Lua is currently only run on autopilots with a lot of flash space, they are only restricted in total aggregate size to available free flash memory. Put the scripts in a sub-directory called ``scripts``, i.e. ``libraries/AP_HAL_ChibiOS/hwdef/OEM_CubeOrange/scripts``. Files must end in ``.lua``.

    .. warning:: The user may also run Lua scripts off the SD card, so care should be taken in naming the embedded script file names to not conflict with potential user file names. It is recommended that the file names of embedded Lua scripts be provided in the product documentation for the user.

#. You can also imbed small pieces of documentation in the ROM of the chip that are readable when examining the @ROMFS folder via MAVFtp. These can be pictures or small informational documents. These must fit within the free flash space of the autopilot. These files can be located in sub-directories in ``libraries/AP_HAL_ChibiOS/hwdef/OEM_CubeOrange`` (e.g. ``libraries/AP_HAL_ChibiOS/hwdef/OEM_CubeOrange/AircraftManual``).

#. Now build as normal with OEM-CubeOrange as the board name in the configuration. The default parameters, Lua scripts, and the custom firmware name will be embedded appropriately.

Alternative To Customizing hwdef.dat
------------------------------------

Instead of creating a separate branch and modifying the hwdef file, you can also insert Lua scripts or even informational files into the ROMFS of the build. Simply go to the local ardupilot repository's "build" folder and create a sub-folder named "ROMFS_custom". Place your LUA scripts in a sub-folder in this directory, named "scripts" (i.e. the path ``ardupilot/ROMFS_custom/scripts``). You can have other sub-folders for informational files and these will be included and viewable when examining the @ROMFS folder with MAVFtp.

[copywiki destination="plane,copter,rover,dev"]
