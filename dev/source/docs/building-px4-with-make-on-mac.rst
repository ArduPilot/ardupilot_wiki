.. _building-px4-with-make-on-mac:

===================================================
Building ArduPilot for Pixhawk/PX4 on Mac with Make
===================================================

This article shows how to build ArduPilot for Pixhawk 2, Pixhawk and PX4
on Mac OS X (ver 10.6 onwards) with *Make*.

.. note::

   The commands for building Pixhawk 2 and Pixhawk are identical
   (``make px4-v2``). To build for PX4 replace ``make px4-v2`` with ``make px4-v1`` in the instructions below. #. Install `Homebrew <http://brew.sh>`__\ for Mac OS X

#. Install xcode and say YES to install Command Line Tools

   ::
   
       xcode-select --install
       
#. Install the following packages using brew

   ::

       brew tap PX4/homebrew-px4
       brew update
       brew install genromfs
       brew install gcc-arm-none-eabi

#. Install the latest version of awk using brew (make sure
   **/usr/local/bin** takes precedence in your path):

   ::

       brew install gawk

#. Install *pip* and *pyserial* using the following commands:

   ::

       sudo easy_install pip
       sudo pip install pyserial

#. Now create your directory and install all the software:

   ::

       mkdir -p px4
       cd px4
       git clone https://github.com/ArduPilot/ardupilot.git
       cd ardupilot
       git submodule update --init --recursive

   .. note::

      `PX4Firmware <https://github.com/ArduPilot/PX4Firmware>`__,
         `PX4NuttX <https://github.com/ArduPilot/PX4NuttX>`__ and
         `uavcan <https://github.com/ArduPilot/uavcan>`__ are automatically
         imported as :ref:`Git Submodules <git-submodules>` when
         you build a vehicle.

#. Build the firmware by entering one of the following commands:

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
   **.px4** file extension, ready to load onto the Pixhawk. For example
   if you build for px4-v2, **ArduCopter-v2.px4** will be created

#. Occasionally you should pull *PX4Firmware* and *PX4NuttX* updates. To
   make sure it compiles correctly, run the clean option in make:

   ::

       make px4-clean
       make px4-[frame type]

   The available frame types are: quad, tri, hexa, y6, octa, octa-quad,
   hell.

#. To make and upload to your vehicle do:

   ::

       make px4-quad-upload
