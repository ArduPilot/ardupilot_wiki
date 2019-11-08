.. _building-for-navio-on-rpi2:

=====================================
Archived: Building for NAVIO+ on RPi2
=====================================

.. warning::

   **ARCHIVED ARTICLE**

   ArduPilot no longer supports NAVIO+ on Rpi2.

Overview
========

These instructions clarify how to build ArduPilot for the NAVIO+ board
on the NAVIO+'s RPi2 board itself using Waf build system.  These instructions assume the RPi2
has already been setup according to the manufacturer's (i.e. Emlid's)
instructions
`here <http://docs.emlid.com/Navio-APM/configuring-raspberry-pi/>`__.

Alternatively you can follow Emlid's instructions on how to build from
source found
`here <http://docs.emlid.com/Navio-APM/building-from-sources/>`__.

Setup
-----

Use an ssh terminal program such as `Putty <http://www.putty.org/>`__ to
log into the NAVIO+ board's RPI2.

Clone the source:

::

    git clone https://github.com/diydrones/ardupilot.git
    cd ardupilot
    git submodule update --init

.. note::

    Waf should always be called from the ardupilot's root directory.

To keep access to Waf convenient, use the following alias from the root ardupilot directory:

::

    alias waf="$PWD/modules/waf/waf-light"

Choose the board to be used:

::

    waf configure --board=navio

Build
-----

Now you can build arducopter. For quadcopter use the following command:

::

    waf --targets bin/arducopter-quad

To build for other frame types replace quad with one of the following options:

::

    coax heli hexa octa octa-quad single tri y6

In the end of compilation binary file with the name arducopter-quad will be placed in ``ardupilot/build/navio/bin/ directory``.

