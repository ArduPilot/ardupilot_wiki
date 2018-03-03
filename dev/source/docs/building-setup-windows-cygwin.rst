
.. _building-setup-windows-cygwin:

============================================================
Setting up the waf Build Environment on Windows using Cygwin
============================================================

These setup instructions describe how to setup `Cygwin <http://www.cygwin.com/>`__ on Windows so that waf can run natively and be used for building.

   .. warning::

      These instructions are new and have not been fully tested

Install Pixhawk Toolchain
-------------------------

- install the Pixhawk Toolchain by downloading and running the `pixhawk_toolchain_installer_latest.exe <http://firmware.ardupilot.org/Tools/PX4-tools/pixhawk_toolchain_installer_latest.exe>`__

Install Cygwin
--------------

#. open a web browser to `www.cygwin.com/install.html <https://www.cygwin.com/install.html>`__ and select "Run setup-x86_64.exe"

#. accept the all the prompts (including default file locations) until
   you reach the *Select Packages* dialog.
   
#. Select the required packages from the thousands of available packages.
   Refer to the list below and enter each package "Name" into the Search field as shown below.
   When you have found the package click on the **Skip** button to change it to a version number and then move onto the next package:

   .. figure:: ../images/Cygwin-select-install-gpp.png
      :target: ../_images/Cygwin-select-install-gpp.png

      Cygwin Installer: Select Package Dialog

   +----------------+----------------------------------------------------------------------------------+
   | Package Name   | Category / Name / Description                                                    |
   +================+==================================================================================+
   | autoconf       | Devel \| autoconf: Wrapper scripts for autoconf commands                         |
   +----------------+----------------------------------------------------------------------------------+
   | automake       | Devel \| automake: Wrapper scripts for automake and aclocal                      |
   +----------------+----------------------------------------------------------------------------------+
   | ccache         | Devel \| ccache: A C compiler cache for improving recompilation                  |
   +----------------+----------------------------------------------------------------------------------+
   | g++            | Devel \| gcc-g++ GNU Compiler Collection (C++)                                   |
   +----------------+----------------------------------------------------------------------------------+
   | git            | Devel \| git: Distributed version control system                                 |
   +----------------+----------------------------------------------------------------------------------+
   | libtool        | Devel \| libtool: Generic library support script                                 |
   +----------------+----------------------------------------------------------------------------------+
   | make           | Devel \| make: The GNU version of the 'make' utility                             |
   +----------------+----------------------------------------------------------------------------------+
   | gawk           | Interpreters \| gawk: GNU awk, a pattern scanning and processing language        |
   +----------------+----------------------------------------------------------------------------------+
   | libexpat       | Libs \| libexpat-devel: Expat XML parser library (development files)             |
   +----------------+----------------------------------------------------------------------------------+
   | libxml2-devel  | Libs \| libxml2-devel: Gnome XML library (development)                           |
   +----------------+----------------------------------------------------------------------------------+
   | libxslt-devel  | Libs \| libxslt-devel: XML template library (development files)                  |
   +----------------+----------------------------------------------------------------------------------+
   | python2-devel  | Python \| python2-devel: Python2 language interpreter (python3 does not work yet)|
   +----------------+----------------------------------------------------------------------------------+
   | procps         | System \| procps-ng: System and process monitoring utilities (required for pkill)|
   +----------------+----------------------------------------------------------------------------------+

#. When all the packages are selected, click through the rest of the
   prompts and accept all other default options (including
   the additional dependencies).
#. Select **Finish** to start downloading and installing the packages.

   .. warning::

      Sometimes the installation can stall because of anti-virus protection software is running.
      If this occurs, shutdown all other programs on your PC including the anti-virus protection and try again.

Set up directories/paths in Cygwin
----------------------------------

#. open and then close the "Cygwin64 Terminal" application from the desktop or start menu icon.  This creates initialisation files for the user in the Cygwin home directory.

#. with your favourite text editor, open C:\\cygwin64\\home\\<username>\\.bashrc and add this line to include the Pixhawk Toolchain's compiler.

   ::

       export PATH=$PATH:/cygdrive/c/pixhawk_toolchain/toolchain/bin

Build with Waf
==============

You should now be able to start the "Cygwin64 Terminal" application from your Windows Start menu and build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

   .. warning::

      The build may fail if the file path to some files is too long.  If the build fails, please try :ref:`cloning <git-clone>` ArduPilot into a directory very high in the directory structure (i.e. ~/ardupilot).
