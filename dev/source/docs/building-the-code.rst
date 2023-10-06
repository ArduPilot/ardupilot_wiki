.. _building-the-code:

=================
Building the code
=================

The linked articles below explain how to setup your build environment on Linux/Ubuntu, MacOS or Windows and then build ArduPilot with `waf <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

The instructions below assume that you have already :ref:`installed git <git-install>`, :ref:`forked <git-fork>` and :ref:`cloned <git-clone>` the ArduPilot repo.

Setting up the Build Environment
--------------------------------

- :ref:`Setup the Build Environment on Linux/Ubuntu <building-setup-linux>`
- :ref:`Setup the Build Environment on Windows <building-setup-windows>`
- :ref:`Setup the Build Environment on MacOSX <building-setup-mac>`


Building / Compiling
--------------------

Once the build environment is set up as above, run the following commmands prior to the initial build. (For Windows users, this is using WSL, as set up above):

- `git clone https://github.com/ArduPilot/ardupilot`
- `cd ardupilot`
- `git submodule update --recursive --init`
- `./Tools/gittools/submodule-sync.sh`
- `./waf configure --board *BoardName*`
- `./waf clean` (Optional; may be useful in some cases)

`*BoardName*` above is the name of the board, as labeled by its associated folder in
`/ardupiot/libraries/AP_HAL_ChibiOS/hwdef`.

Run the following command, each time you wish to build:
`./waf copter`

(Substitute `plane`, `rover` etc for `copter` as required)


**Note for Windows users**
If possible, run the build steps above from a WSL directory, *not* from a directory on your
Windows filesystem. Building from the Windows filesystem is very slow.


**Details**

- Additional detailed information is available in this document: `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.


- This video demonstrates building on a Linux PC:

.. note::

   Do not use `sudo` unless specified in the instructions.

.. youtube:: lNSvAPZOM_o


**Board specific instructions:**

- :ref:`Building for NAVIO2 on RPi3 <building-for-navio2-on-rpi3>`
- :ref:`Building for Bebop2 on Linux <building-for-bebop-2>`
- :ref:`Building for Bebop on Linux <building-for-bebop-on-linux>`
- :ref:`Building for Beaglebone Black <building-for-beaglebone-black-on-linux>`

Mission Planner
---------------

- :ref:`Building Mission Planner with Visual Studio <building-mission-planner>`



Links to current build pages
----------------------------

.. toctree::
    :maxdepth: 1

    Setup the Build Environment on Linux/Ubuntu <building-setup-linux>
    Setup the Build Environment on Windows <building-setup-windows>
    Setup the waf Build Environment on Windows10 using WSL <building-setup-windows10>
    Setup the waf Build Environment on Windows using Cygwin <building-setup-windows-cygwin>
    Setup Eclipse on Windows for building with waf <building-setup-windows-eclipse>
    Setup the Build Environment on MacOSX <building-setup-mac>
    Building for NAVIO2 on RPi3 <building-for-navio2-on-rpi3>
    Building for Erle-Brain 2 <building-for-erle-brain-2>
    Building for Erle-Brain <building-for-erle-brain>
    Building for Bebop 2 <building-for-bebop-2>
    Building for Bebop on Linux <building-for-bebop-on-linux>
    Building for BeagleBone Black <building-for-beaglebone-black-on-linux>
    Building Mission Planner with Visual Studio <building-mission-planner>
    ArduPilot Pre-Built Binaries <pre-built-binaries>

