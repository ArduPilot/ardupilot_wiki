.. _building-px4-with-make:

=======================================================
Building ArduPilot for Pixhawk/PX4 on Windows with Make
=======================================================

This article shows how to build ArduPilot for Pixhawk 2, Pixhawk and PX4
on Windows with *Make*.

.. note::

   The commands for building Pixhawk 2 and Pixhawk are identical
   (``make px4-v2``). Building for PX4 is the same except that
   ``make px4-v1`` is used. 

Build instructions
==================


#. Install `GitHub for Windows <http://windows.github.com/>`__
#. Ensure your github settings are set to leave line endings untouched.

   -  The "Git Shell (or Bash)" terminal was also installed when you
      installed Git.  Click on your new "Git Shell (or Bash)" Icon and
      type in the following in the Git "MINGW32" Terminal window:

      ::

          git config --global core.autocrlf false

#. Clone the ardupilot repository onto your machine:

   -  Go to the
      `GitHub/diydrones/ardupilot <https://github.com/diydrones/ardupilot>`__
      web page and click the **Clone in Desktop** button
   -  Warning: be careful that the directory path is less than about 50
      characters.  For example
      "C:\\Users\\rmackay9\\Documents\\GitHub\\ardupilot" is short
      enough but
      "C:\\Users\\rmackay9\\Documents\\GitHub\\rmackay9-ardupilot" is
      too long.  This limit is because during compiling temporary files
      are created with much much longer paths which can exceed Windows'
      260 character path limit.

Initialise and update submodules

::

    git submodule update --init --recursive


Download and install the *PX4 toolchain* by running the
`px4_toolchain_installer_v14_win.exe <http://firmware.diydrones.com/Tools/PX4-tools/px4_toolchain_installer_v14_win.exe>`__

Open the *PX4Console* and navigate to the target vehicle directory:

-  Start the *PX4Console*. This can be found under **Start \| All
   Programs \| PX4 Toolchain** (Windows 7 machine) or you can directly
   run **C:\\px4\\toolchain\\msys\\1.0\\px4_console.bat**
-  Navigate to the vehicle-specific ArduPilot directory in the
   *PX4Console*. For example, to build Copter, navigate to:

   ::

       cd /c/Users/<username>/Documents/GitHub/ardupilot/ArduCopter


Build the firmware by entering one of the following commands:

+--------------------------------------+--------------------------------------+
| ``make px4-v2``                      | Build the Pixhawk2/Pixhawk firmware  |
|                                      | (identical) for a quad               |
+--------------------------------------+--------------------------------------+
| ``make px4-v2-hexa``                 | Build the Pixhawk firmware for a     |
|                                      | hexacopter.                          |
|                                      |                                      |
|                                      | # Other supported suffixes include   |
|                                      | "octa", "tri" and "heli".            |
|                                      |                                      |
|                                      | # More can be found in               |
|                                      | "mk/tagets.mk" under FRAMES          |
+--------------------------------------+--------------------------------------+
| ``make px4``                         | Build both PX4 and PixHawk firmware  |
|                                      | for a quadcopter                     |
+--------------------------------------+--------------------------------------+
| ``make clean``                       | "clean" the ardupilot directory      |
+--------------------------------------+--------------------------------------+
| ``make px4-clean``                   | "clean" the PX4Firmware and PX4NuttX |
|                                      | directories so the next build will   |
|                                      | completely rebuild them              |
+--------------------------------------+--------------------------------------+
| ``make px4-v2-upload``               | Build and upload the Pixhawk         |
|                                      | firmware for a quad (i.e. no need to |
|                                      | do step #7 below)                    |
+--------------------------------------+--------------------------------------+


The firmware will be created in the **ArduCopter** directory with the
**.px4** file extension.

.. image:: ../../../images/PX4_ArduCopter_Build.png
    :target: ../_images/PX4_ArduCopter_Build.png


-  Upload the firmware using the *Mission Planner* **Initial Setup \|
   Install Firmware** screen's **Load custom firmware** link

.. note::

   ArduPilot imports addition projects
   (`PX4Firmware <https://github.com/diydrones/PX4Firmware>`__,
   `PX4NuttX <https://github.com/diydrones/PX4NuttX>`__,
   `uavcan <https://github.com/diydrones/uavcan>`__) as *git submodules*
   when you build the project. If you built the project before the change
   to submodules you may get errors. See :ref:`Git Submodules <git-submodules>` for troubleshooting information.

Hints for speeding up compile time
==================================

Anti virus protection is likely to slow the compile times especially for
PX4 so it is recommended that the folders containing the ArduPilot
source code is excluded from your virus protections real-time scan.

The first scan after a ``make px4-clean`` will be very slow as it
rebuilds everything
