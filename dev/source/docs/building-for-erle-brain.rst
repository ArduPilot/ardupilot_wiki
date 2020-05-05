.. _building-for-erle-brain:

=======================================
Archived Topic: Building for Erle-Brain
=======================================

.. warning::

   This page is under construction. Links here are to the top
   level of Erle-Robotics documentation because deeper URLs keep on being
   broken.

These instructions explain how to build ArduPilot on the Erle-Brain
board.

.. tip::

   Alternatively you can follow Erle-Robotics's
   `documentation <http://erlerobotics.com/docs/>`__ on how to build from
   source.

Connection and setup
--------------------

Connect to Erle-Brain using microUSB:

::

    sudo ifconfig eth6 192.168.7.1

::

    ssh root@192.168.7.2

.. tip::

   Check the interface Erle-Brain creates using **ifconfig**
   command.

Give Erle-Brain Internet access connecting Ethernet wire into RJ45
connector and configure the interface:

::

    $ sudo ifconfig eth0 up
    $ sudo dhclient eth0
    #Check if Erle BRain has Internet access
    $ ping www.google.es
    # press |ctrl| |c| to exit

Clone the source:

::

    cd 
    git clone https://github.com/erlerobot/ardupilot.git
    cd ardupilot
    git submodule update --init --recursive

Build
-----

Build for Copter:

::

    cd /home/pi/ardupilot/ArduCopter
    make pxf -j4

This will build the firmware for a quadcopter.  If you wish to build for
another frame type (such as hexacopter) append "-hexa" onto the end of
the make command (i.e. make -j4 pxf-hexa).  The full list of available
frames can be found in the
`targets.mk <https://github.com/ArduPilot/ardupilot/blob/master/mk/targets.mk#L75>`__
file.

.. note::

   If building for Plane, Rover or Antenna Tracker replace the above
   "ArduCopter" with "ArduPlane", "Rover" or "AntennaTracker".

Move firmware to the executable directory
-----------------------------------------

Move the executable to the directory from where it is normally started:

::

    sudo cp ArduCopter.elf /~

.. tip::

   If you are unable to copy the executable it may be because the
   destination file is locked because it is already running.  Use the
   following command to stop the running service

   ::

       systemctl stop apm-copter.service

.. tip::

   Autopilot launch configuration is covered in the
   documentation.
