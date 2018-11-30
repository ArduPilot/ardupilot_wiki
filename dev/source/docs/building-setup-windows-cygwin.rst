
.. _building-setup-windows-cygwin:

============================================================
Setting up the waf Build Environment on Windows using Cygwin
============================================================

These setup instructions describe how to setup `Cygwin <http://www.cygwin.com/>`__ on so that waf (the build system) can run natively on windows and build for all supported boards.

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
   | gcc-g++        | Devel \| gcc-g++ GNU Compiler Collection (C++)                                   |
   +----------------+----------------------------------------------------------------------------------+
   | git            | Devel \| git: Distributed version control system                                 |
   +----------------+----------------------------------------------------------------------------------+
   | libtool        | Devel \| libtool: Generic library support script                                 |
   +----------------+----------------------------------------------------------------------------------+
   | make           | Devel \| make: The GNU version of the 'make' utility                             |
   +----------------+----------------------------------------------------------------------------------+
   | gawk           | Interpreters \| gawk: GNU awk, a pattern scanning and processing language        |
   +----------------+----------------------------------------------------------------------------------+
   | libexpat-devel | Libs \| libexpat-devel: Expat XML parser library (development files)             |
   +----------------+----------------------------------------------------------------------------------+
   | libxml2-devel  | Libs \| libxml2-devel: Gnome XML library (development)                           |
   +----------------+----------------------------------------------------------------------------------+
   | libxslt-devel  | Libs \| libxslt-devel: XML template library (development files)                  |
   +----------------+----------------------------------------------------------------------------------+
   | python2-devel  | Python \| python2-devel: Python2 language interpreter (python3 does not work yet)|
   +----------------+----------------------------------------------------------------------------------+
   | python2-future | Python \| python2-future: Clean single-source support for Python 3 and 2         |
   +----------------+----------------------------------------------------------------------------------+
   | python2-libxml2| Python \| python2-libxml2: Gnome XML Library (Python bindings                    |
   +----------------+----------------------------------------------------------------------------------+
   | python2-pip    | Python \| python2-pip: Python package installation tool                          |
   +----------------+----------------------------------------------------------------------------------+
   | procps-ng      | System \| procps-ng: System and process monitoring utilities (required for pkill)|
   +----------------+----------------------------------------------------------------------------------+
   | gdb            | Devel \| gdb: The GNU Debugger                                                   |
   +----------------+----------------------------------------------------------------------------------+
   | ddd            | Devel \| ddd: DDD, the data display debugger                                     |
   +----------------+----------------------------------------------------------------------------------+
   | zip            | Archive \| zip: Info-ZIP compression utility                                     |
   +----------------+----------------------------------------------------------------------------------+   

   Or you can run

::

    setup-x86_64.exe -P autoconf,automake,ccache,gcc-g++,git,libtool,make,gawk,libexpat-devel,libxml2-devel,libxslt-devel,python2-devel,python2-future,python2-libxml2,python2-pip,procps-ng,gdb,ddd,zip

#. When all the packages are selected, click through the rest of the
   prompts and accept all other default options (including
   the additional dependencies).
#. Select **Finish** to start downloading and installing the packages.

   .. warning::

      Sometimes the installation can stall because of anti-virus protection software is running.
      If this occurs, shutdown all other programs on your PC including the anti-virus protection and try again.

Install the GCC compiler
-------------------------

- download and install the gcc-arm-non-eabi compiler from `firmware.ardupilot.org/Tools/STM32-tools <http://firmware.ardupilot.org/Tools/STM32-tools>`__ (`quick link is here <http://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-6-2017-q2-update-win32-sha2.exe>`__)
    - accept the license
    - install to the default location
    - accept the ssl certificate
    - check option to "Add path to environment variable"

.. image:: ../images/building-setup-windows-cygwin-gcc.png

Install MAVProxy
-----------------------------------------------------

The MAVProxy GCS is required when running SITL. If you're not planning to run SITL, you can skip this step.

#. To install MAVProxy on Windows follow the `MAVProxy's install instructions for Windows <https://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html#windows>`__

Set up directories/paths and extra packages in Cygwin
-----------------------------------------------------

#. open and then close the "Cygwin64 Terminal" application from the desktop or start menu icon.  This creates initialisation files for the user in the Cygwin home directory.

#. open the "Cygwin64 Terminal" application from the desktop or start menu icon.  Enter the following command to install additional Python packages:

::

    pip2 install empy
    pip2 install pyserial
    pip2 install pymavlink

Build with Waf
==============

You should now be able to start the "Cygwin64 Terminal" application from your Windows Start menu and build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

   .. warning::

      The build may fail if the file path to some files is too long.  If the build fails, please try :ref:`cloning <git-clone>` ArduPilot into a directory very high in the directory structure (i.e. ~/ardupilot).
      
      If during build process you get error of missing packet xyz, re-check that you have installed all cygwin packets listed above
