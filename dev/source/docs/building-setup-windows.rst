.. _building-setup-windows:

======================================
Setup the Build Environment on Windows
======================================

There are three options for building on windows.  We recommended the first option which uses Cygwin.

Setup for building with waf using Cygwin
----------------------------------------

#. :ref:`Install cygwin as described here <building-setup-windows-cygwin>`

#. :ref:`Setup Eclipse as described here <building-setup-windows-eclipse>` (optional)

Setup for building with waf using WSL (Windows10 only)
------------------------------------------------------

#. :ref:`Instructions are here <building-setup-windows10>`

Setup for building with Make (not recommended)
----------------------------------------------

#. Install the Pixhawk Toolchain by downloading and running the `pixhawk_toolchain_installer_latest.exe <http://firmware.ardupilot.org/Tools/STM32-tools/pixhawk_toolchain_installer_latest.exe>`__

#. If you wish to use Eclipse for editing and building, follow :ref:`these instructions <editing-the-code-with-eclipse>`

#. To run MAVProxy locally on Windows follow the `MAVProxy's install instructions for Windows <https://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html#windows>`__

Next you will probably want to :ref:`build for Pixhawk on Windows with Make <building-px4-with-make>`.
