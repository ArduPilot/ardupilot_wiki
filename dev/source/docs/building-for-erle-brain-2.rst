.. _building-for-erle-brain-2:

=========================================
Archived Topic: Building for Erle-Brain 2
=========================================

These instructions explain how to build ArduPilot on board the
Erle-Brain 2.

.. note::

   Alternatively you can follow `Erle-Robotics's instructions <http://erlerobotics.com/docs/>`__ on how to build from
   source on your PC.

Connection and setup
--------------------

Give Erle-Brain Internet access by plugging the Ethernet wire into the
RJ-45 connector.

Connect to Erle-Brain 2:

::

    $ ssh erle@erle-brain-2.local

.. tip::

   Other connection methods are discussed in the manufacturers
   `documentation <http://erlerobotics.com/docs/>`__\ 

Clone the source:

::

    #Move to home
    cd ~/
    #Clone the ArduPilot repository in home
    git clone https://github.com/ArduPilot/ardupilot.git
    cd ardupilot
    git submodule update --init --recursive

Build
-----

Build for Copter:

::

    cd /home/erle/ardupilot/ArduCopter
    make erlebrain2 -j4

This will build the firmware for a quadcopter.  If you wish to build for
another frame type (such as hexacopter) append "-hexa" onto the end of
the make command (i.e. make erlebrain2-hexa -j4).  The full list of
available frames can be found in the
`targets.mk <https://github.com/ArduPilot/ardupilot/blob/master/mk/targets.mk#L3>`__
file.

.. note::

   If building for Plane, Rover or Antenna Tracker replace the above
   "ArduCopter" with "ArduPlane", "Rover" or "AntennaTracker".

Move firmware to the executable directory
-----------------------------------------

When you compile Copter (in **~/ardupilot/ArduCopter**) the executable
**ArduCopter.elf** is created in the same folder. You need to move the
executable to home (\`~/\`or \`/home/erle/\`) because there is a service
in Erle-Brain 2 (called **apm.service**) which automatically launches
the autopilot from those locations.

::

    #Assuming present working directory is: ~/ardupilot/ArduCopter
    sudo cp ArduCopter.elf ~/

.. tip::

   The service mentioned above, launches **ArduCopter** by default,
   using a bridge for sending telemetry data and GPS placed in **ttyAMA0**
   by default

   You can find additional information about the default launch process and
   launch configuration options in the
   `documentation <http://erlerobotics.com/docs/>`__.

.. tip::

   If you are unable to copy the executable it may be because the
   destination file is locked. This is because the autopilot is already
   running.  Use the following command to stop the running service:

   ::

       systemctl stop apm.service

   Now copy the executable in home (\`~\`or \`/home/erle/\`) and restart
   the service in order to execute the new executable:

   ::

       systemctl start apm.service

   Or simply, restart the board once you have copied the new executable
