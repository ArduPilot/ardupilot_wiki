.. _building-for-bebop-2:

====================
Building for Bebop 2
====================

These instructions explain how to use ArduPilot for the
`Bebop2 <http://www.parrot.com/usa/products/bebop2/>`__ on a Linux
machine. The Bebop 2 is based on the same architecture as the Bebop with
a few noticeable changes, not the least being a much better quality GPS
(UBlox GPS with a bigger antenna).

.. warning::

   Making the changes described in this article will void your
   warranty! Parrot's technical support will not help you with this hack or
   to recover your original software.

.. warning::

   Hacking a commercial product is risky! This software is still evolving,
   and you may well find issues with the vehicle ranging from poor flight
   to complete software freeze.

   That said, it is almost always possible to recover a drone and members
   of the ardupilot dev team can likely help people hacking or recovering
   their Bebop on `this google group <https://groups.google.com/forum/#!forum/bebop-ardupilot>`__.
   Prepare to spend some time, patience and develop some hardware/software
   skills. 

Building ArduCopter for Bebop 2
===============================

The instructions are exactly the same as :ref:`the one used for Bebop <building-for-bebop-on-linux_build_arducopter_for_bebop>`

Uploading the Firmware
======================

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

       adb push arducopter /usr/bin/

Starting ArduPilot
==================

#. Kill the regular autopilot:

   ::

       kk

#. Launch Copter:

   ::

       arducopter -A udp:192.168.42.255:14550:bcast -B /dev/ttyPA1 -C udp:192.168.42.255:14551:bcast -l /data/ftp/internal_000/APM/logs -t /data/ftp/internal_000/APM/terrain

Launch Copter at startup
========================

As for Bebop, modify the init script **/etc/init.d/rcS_mode_default**.
Comment the following line:

::

    DragonStarter.sh -out2null &

Replace it with:

::

    arducopter -A udp:192.168.42.255:14550:bcast -B /dev/ttyPA1 -C udp:192.168.42.255:14551:bcast -l /data/ftp/internal_000/APM/logs -t /data/ftp/internal_000/APM/terrain &

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
