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
   scp 3dr-solo.tar.gz 3dr-solo.tar.gz.md5 root@10.1.1.10:/log/updates/
   ssh root@10.1.1.10 -C "touch /log/updates/UPDATE && /sbin/shutdown -r now"

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
   scp 3dr-controller.tar.gz 3dr-controller.tar.gz.md5 root@10.1.1.1:/log/updates/
   ssh root@10.1.1.1 -c "touch /log/updates/UPDATE && /sbin/shutdown -r now"

After update, ensure the flash was successful:

   ::

      ls -l /log/updates/UPDATEFAILED
      cat /VERSION

The content of /VERSION should correspond to the build you made; in particular, ensure the date looks reasonable.

