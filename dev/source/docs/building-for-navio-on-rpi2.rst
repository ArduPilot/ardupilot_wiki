.. _building-for-navio-on-rpi2:

===========================
Building for NAVIO+ on RPi2
===========================

Overview
========

These instructions clarify how to build ArduPilot for the NAVIO+ board
on the NAVIO+'s RPi2 board itself.  These instructions assume the RPi2
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

Install the gcc-4.8 compiler using `these instructions <https://somewideopenspace.wordpress.com/2014/02/28/gcc-4-8-on-raspberry-pi-wheezy/>`__.

.. tip::

   When creating

   ::

       /etc/apt/preferences

   Type this instead:

   ::

       Package: *
       Pin: release n=wheezy
       Pin-Priority: 900
       Package: *
       Pin: release n=jessie
       Pin-Priority: 300
       Package: *
       Pin: release o=Raspbian
       Pin-Priority: 200

Clone the source:

::

    cd /home/pi
    git clone https://github.com/ArduPilot/ardupilot.git

Build
-----

Build for Copter:

::

    cd /home/pi/ardupilot/ArduCopter
    make -j4 navio

This will build the firmware for a quadcopter.  If you wish to build for
another frame type (such as hexacopter) append "-hexa" onto the end of
the make command (i.e. make -j4 navio-hexa).  The full list of available
frames can be found in the
`targets.mk <https://github.com/ArduPilot/ardupilot/blob/master/mk/targets.mk#L75>`__
file.

.. note::

   If building for Plane, Rover or Antenna Tracker replace the above
   "ArduCopter" with "ArduPlane", "APMrover2" or "AntennaTracker".

Move firmware to the executable directory
-----------------------------------------

Move the executable to the directory from where it is normally started:

::

    sudo cp ArduCopter.elf /opt/apm/bin/ArduCopter-quad

.. tip::

   If you are unable to copy the executable it may be because the
   destination file is locked because it is already running.  Use the
   following command to kill the running executable

   ::

       sudo killall ArduCopter-quad
