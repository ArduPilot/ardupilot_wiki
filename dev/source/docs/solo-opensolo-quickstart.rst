.. _solo_opensolo_quickstart:

==========================================================================================
WIP DOCUMENT!  You will almost certainly brick your Solo if you follow these instructions!
==========================================================================================

=========================
OpenSolo QuickStart Guide
=========================

.. note:

   This document is known to be incomplete.  In particular, if you are running a "Green Cube" Solo modifications must be made to the "Golden Image" on the Solo.  Details to come.

.. note:

   ArduPilot's master branch is missing throttle-slew-rate-limitting which is present on 3DR's ArduPilot branch.  In the absence of a Green Cube this makes flying ArduPilot-master on your Solo *very dangerous*.

Compilation
===========

Bring the vagrant virtual machine up on your host machine:

::

   git clone https://github.com/OpenSolo/solo-builder
   cd solo-builder
   vagrant up

ssh into the vagrant virtual machine and run the build:

::

   vagrant ssh -- -X
   time /vagrant/builder.sh # about 15 hours


View Artoo (controller) build products:

::

   ls /solo-build/build/tmp-eglibc/deploy/images/imx6solo-3dr-artoo/

View Solo build products:

::

   ls /solo-build/build/tmp-eglibc/deploy/images/imx6solo-3dr-1080p/

Update Solo:

::

   cd /solo-build/build/tmp-eglibc/deploy/images/imx6solo-3dr-1080p
   scp 3dr-solo.tar.gz root@10.1.1.10:/log/updates/
   ssh root@10.1.1.1 -C "cd /log/updates; md5sum 3dr-solo.tar.gz >3dr-solo.tar.gz.md5"
   ssh root@10.1.1.10 -C "touch /log/updates/UPDATE && /sbin/shutdown -r now"

.. note:

   If you have not SSH'd into your Solo before and changed things, you may be propted for a ssh password when doing 'ssh' or 'scp' commands. 
   The 'root' password to use is 'TjSDBkAu'.  more details here: https://dev.3dr.com/starting-network.html

.. note:

   Two reboots of Solo may be required

.. note:

   Green LEDs are good.  ssh takes some time to be available

After update, ensure the flash was successful:

   ::

      ls -l /log/updates/UPDATEFAILED
      cat /VERSION

The UPDATEFAILED file should NOT exist
The content of /VERSION should correspond to the build you made; in particular, ensure the date looks reasonable.

Update Artoo:

::

   cd /solo-build/build/tmp-eglibc/deploy/images/imx6solo-3dr-artoo
   scp 3dr-controller.tar.gz root@10.1.1.1:/log/updates/
   ssh root@10.1.1.1 -C "cd /log/updates; md5sum 3dr-controller.tar.gz >3dr-controller.tar.gz.md5"
   ssh root@10.1.1.1 -C "touch /log/updates/UPDATE && /sbin/shutdown -r now"

After update, ensure the flash was successful:

   ::

      ls -l /log/updates/UPDATEFAILED
      cat /VERSION

The content of /VERSION should correspond to the build you made; in particular, ensure the date looks reasonable.


Wipe Parameters / Set Parameters from magic Parameters
======================================================

Set a parameter to a nonsense value which will reset all parameters:

::

   param set SYSID_SW_MREV 0

That's for MAVProxy; use you GCS of choice to get the same effect.

Load the default parameters file (which can be found here <link>)

::

   param load /tmp/Solo_AC350_Params.param
   param load /tmp/Solo_AC350_Params.param

Again, use your GCS of choice to effect the same change.  Yes, do it twice.


Redo Calibrations
=================

In MAVProxy:

::

   accelcal

In your GCS of choice: click-click-click etc.


::

   magcal

In your GCS of choice: click-click-click etc.


Troubleshooting:
================
- the update will fail if there are multiple images present on /log/updates


Undo All This Madness? 
======================

A Standard 3DR "Factory Reset" will return you to your normal Solo, as 3DR made it: 
https://3drobotics.zendesk.com/hc/en-us/articles/208396933-Factory-Reset

You may also find you have to re-pair your Solo and Controller after the Factory Reset:
https://3dr.com/support/articles/pairing_the_controller/

