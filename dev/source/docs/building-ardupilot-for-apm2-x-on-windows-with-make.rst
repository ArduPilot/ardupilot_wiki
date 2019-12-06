.. _building-ardupilot-for-apm2-x-on-windows-with-make:

==================================================
Building ArduPilot for APM2.x on Windows with Make
==================================================

This article shows how to build ArduPilot for APM2.x on Windows with *Make*.

.. tip::

   The approach described here is useful if you want to :ref:`develop using Eclipse <building-apm2-with-eclipse-on-windows>`. :ref:`Building ArduPilot for APM2.x on Windows with Arduino <building-ardupilot-with-arduino-windows>` shows an
   alternative method for building ArduPilot for APM2.x.

.. warning::

   Copter 3.3 firmware (and later) and builds after Plane
   3.4.0 no longer fit on APM boards. Plane, Rover and AntennaTracker
   builds can still be installed at time of writing but you can no longer
   build APM2.x off the master branch (you will need to build off a
   supported release branch, or for the keen developer, from the AVR-master branch master-AVR and the tags from there.  see: https://github.com/ArduPilot/ardupilot/tree/master-AVR ).

   The last Copter firmware that can be built on APM 2.x 
   `can be downloaded from here <https://github.com/ArduPilot/ardupilot/archive/master-AVR.zip>`__.

Overview
========

These instructions use the :ref:`PX4 Toolchain <building-px4-with-make>`
along with the :ref:`Arduino Tools <building-ardupilot-with-arduino-windows>` to set up an
environment in which you can build for APM2.x targets with *make*. They
have been tested on Windows 10 to build the ArduCopter-3.2.1 branch.

.. note::

   This article replaces previous instructions to use a basic
   `Cygwin <http://www.cygwin.com/>`__ installation with the GNU sed, make
   and awk packages installed. The pre-built environment from the PX4
   Toolchain is a lot easier to set up.

Build instructions
==================

#. Install `GitHub for Windows <https://desktop.github.com/>`__
#. Ensure your github settings are set to leave line endings untouched.

   -  The "Git Shell (or Bash)" terminal was also installed when you
      installed Git.  Click on your new "Git Shell (or Bash)" Icon and
      type in the following in the Git "MINGW32" Terminal window:

      ::

          git config --global core.autocrlf false

#. Get the source code onto your machine

   -  In the Git "MINGW32" Terminal window navigate to where you want to
      put the source code and clone the repo

      ::

          git clone https://github.com/ArduPilot/ardupilot.git
          cd ardupilot
          git submodule update --init --recursive

   -  Checkout the branch you want to build (the last branch you can use
      for Copter is shown below):

      ::

          git checkout ArduCopter-3.2.1

#. Install the special ArduPilot Arduino package. This contains gcc
   4.8.2 and Eclipse "Luna".

   -  Download the installation zip:
      `ArduPilot-Arduino-1.0.3-gcc-4.8.2-windows.zip <https://firmware.ardupilot.org/Tools/Arduino/ArduPilot-Arduino-1.0.3-gcc-4.8.2-windows.zip>`__
   -  Unzip the file to the root of the C drive

      .. note::

         You can install
               anywhere. Later on we update **config.mk** to tell the build
               system where the tools are located.

#. Download and install the *PX4 toolchain* by running the
   `px4_toolchain_installer_v14_win.exe <https://firmware.ardupilot.org/Tools/STM32-tools/px4_toolchain_installer_v14_win.exe>`__
#. Open the *PX4Console* and navigate to the target vehicle directory:

   -  Start the *PX4Console*. This can be found under **Start \| All
      Programs \| PX4 Toolchain** (Windows 7 machine) or you can
      directly run **C:\\px4\\toolchain\\msys\\1.0\\px4_console.bat**
   -  Navigate to the vehicle-specific ArduPilot directory in the
      *PX4Console*. For example, to build Copter, navigate to:

      ::

          cd /c/Users/<username>/Documents/GitHub/ardupilot/ArduCopter

#. Configure the build system to find the *Arduino tools*:

   -  Enter the following command on the *PX4Console* to create
      **/ardupilot/config.mk**.

      ::

          make configure

   -  Open **config.mk** (created in the directory above ArduCopter) and
      define the ``ARDUINO`` variable as shown:

      ::

          ARDUINO = C:/arduino-1.0.3-windows

      .. note::

         You **must** specify the drive letter and use forward
               slashes for the path.

#. Build the firmware by entering the following command on the
   *PX4Console*:

   ::

       make apm2

   .. tip::

      This command can take several minutes before it is obvious that something is happening!


   The firmware will be created in a subfolder of the user's temp
   directory. For example you will find **ArduCopter.hex** in
   **C:\\Users\\\ *YourUserNameHere*\\AppData\\Local\\Temp\\ArduCopter.build**.

#. Upload the firmware using the *Mission Planner* **Initial Setup \|
   Install Firmware** screen's **Load custom firmware** link

Hints for speeding up compile time
==================================

Anti virus protection is likely to slow the compile times especially for
Pixhawk so it is recommended that the folders containing the ArduPilot
source code is excluded from your virus protections real-time scan.

The first scan after a ``make px4-clean`` will be very slow as it
rebuilds everything
