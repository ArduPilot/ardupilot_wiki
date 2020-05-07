.. _building-for-bebop-2:

====================
Building for Bebop 2
====================

These instructions explain how to use ArduPilot for the
`Bebop2 <https://www.parrot.com/us/drones/parrot-bebop-2?ref=#parrot-bebop-2-details/>`__ on a Linux
machine. The Bebop 2 is based on the same architecture as the Bebop with
a few noticeable changes, not the least being a much better quality GPS
(UBlox GPS with a bigger antenna).

.. warning::

   Making the changes described in this article will void your
   warranty! Parrot's technical support will not help you with this hack or
   to recover your original software.

.. warning::

   Hacking a commercial product is risky! This software is still evolving.

   That said, it is almost always possible to recover a drone and members
   of the ardupilot dev team can likely help people hacking or recovering
   their Bebop on `this google group <https://groups.google.com/forum/#!forum/bebop-ardupilot>`__.

Building ArduCopter for Bebop 2
===============================

.. tip::

   You can skip this step if you just want to try out the
   (experimental) binary version.

The following steps show how to build a custom version of the Copter
software for Bebop 2:

Install armhf toolchain
-----------------------

#. Install Parrot's version of linaro *arm-linux-gnueabihf* toolchain that can be downloaded from
   `here <https://firmware.parrot.com/Toolchains/parrot-tools-linuxgnutools-2016.02-linaro_1.0.0-5_amd64.deb>`__

#. Install it (the toolchain will be extracted in /opt)

   ::

       sudo dpkg -i parrot-tools-linuxgnutools-2016.02-linaro_1.0.0-5_amd64.deb

#. Add the path to the toolchain to the PATH variable

   ::

       export PATH=/opt/arm-2016.02-linaro/bin:$PATH

Download and compile ArduCopter
-------------------------------

#. Clone ardupilot repository

   ::

       git clone https://github.com/ArduPilot/ardupilot.git
       cd ardupilot
       git submodule update --init --recursive

#. Building the flight control firmware is nearly identical for
   :ref:`building for the Pixhawk <building-px4-with-make>`
   except the build command is:
#. ::

       ./waf configure --board=bebop
       ./waf build


Uploading the Firmware
======================

Mission Planner can now upload stable and custom versions of ardupilot to the Bebop2.

..  youtube:: Ir0DyvlbTM0
    :width: 100%

Instructions below are for the manual method of uploading

#. Install adb (android debug tool):

   ::

       sudo apt-get install android-tools-adb

#. Connect to the Bebop2's WiFi network (BebopDrone-XXXX).
#. Enable adb server by pressing the power button 4 times.
#. Connect to the Bebop's adb server on port 9050:

   ::

       adb connect 192.168.42.1:9050

#. If the previous command returns an error, try again (press the power
   button 4 times and retry).
#. Remount the system partition as writeable:

   ::

       adb shell mount -o remount,rw /

#. Push the stripped arducopter binary to the Bebop2:

   ::

       adb mkdir /data/ftp/internal_000/APM
       adb push arducopter /data/ftp/internal_000/APM/

Starting ArduPilot
==================

#. Kill the regular autopilot:

   ::

       adb shell
       kk

#. Launch Copter:

   ::

       cd /data/ftp/internal_000/APM
       arducopter -A udp:192.168.42.255:14550:bcast -B /dev/ttyPA1 -C udp:192.168.42.255:14551:bcast -l /data/ftp/internal_000/APM/logs -t /data/ftp/internal_000/APM/terrain

Launch Copter at startup
========================

As for Bebop, modify the init script **/etc/init.d/rcS_mode_default**.
Comment the following line:

::

    DragonStarter.sh -out2null &

Replace it with:

::

    /data/ftp/internal_000/APM/arducopter -A udp:192.168.42.255:14550:bcast -B /dev/ttyPA1 -C udp:192.168.42.255:14551:bcast -l /data/ftp/internal_000/APM/logs -t /data/ftp/internal_000/APM/terrain &

#. Enable adb server by pressing the power button 4 times.
#. Connect to adb server as described before:

   ::

       adb connect 192.168.42.1:9050

#. Re-mount the system partition as writeable:

   ::

       adb shell mount -o remount,rw /

#. In order to avoid editing the file manually, you can download `this one <https://github.com/Parrot-Developers/ardupilot/releases/download/bebop-v0.1/rcS_mode_default>`__.
#. Save the original one and push this one to the bebop
#. ::

       adb shell cp /etc/init.d/rcS_mode_default /etc/init.d/rcS_mode_default_backup
       adb push rcS_mode_default /etc/init.d/

#. Sync and reboot:

   ::

       adb shell sync
       adb shell reboot

Recovery
========

For recovery, you can use the same cable as the one used on Bebop, see
:ref:`here <building-for-bebop-on-linux_recovery>`.

#. Remove the two screws using a torx T6 screwdriver 

   .. image:: ../images/bebop_remove_screws.jpg
      :target: ../_images/bebop_remove_screws.jpg
   
#. Remove the neck by pulling it towards the front of the Bebop

   .. image:: ../images/bebop_recovery_remove_neck.jpg
       :target: ../_images/bebop_recovery_remove_neck.jpg
   
#. The UART connector is located on the right side
   
   .. image:: ../images/bebop_uart_connection.jpg
       :target: ../_images/bebop_uart_connection.jpg
   
#. Plug the cable with the black wire at the front
   
   .. image:: ../images/bebop_connections3.jpg
       :target: ../_images/bebop_connections3.jpg
   
#. Connect to the bebop with the UART port using any terminal emulator
#. Copy the backup rcS file back to its place

   ::

       mount -o remount,rw /
       cp /etc/init.d/rcS_mode_default_backup /etc/init.d/rcS_mode_default

#. Sync and reboot

   ::

       sync
       reboot

Flying and RC over UDP
======================

Flying and RC over UDP instructions are the same as :ref:`the ones for Bebop <building-for-bebop-on-linux_flying>`

Basic configuration and frame parameters
========================================

#. The set of tuning parameters can be found
   `here <https://github.com/ArduPilot/ardupilot/blob/master/Tools/Frame_params/Parrot_Bebop2.param>`__.
   These are not yet fully tuned for Bebop 2
#. In order to do the basic configuration and calibration, you can use
   any of the GCSs and perform:

   #. Magnetometer Calibration
   #. RC Calibration
   #. Accelerometer Calibration

Additional information
======================

The loiter mode quality is very good compared to the first Bebop because
of the (much better) UBlox GPS. It is now safe to takeoff and land in
the mode you want.

There is still no support for video yet and the optical flow and sonar
are currently under development.

This is a good time to participate and help improve them!
