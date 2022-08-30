.. _solo_opensolo_quickstart:

=========================
OpenSolo QuickStart Guide
=========================

.. note::

   More complete information for developers can be found `here <https://github.com/OpenSolo/documentation>`__

.. note::

   ArduPilot's master branch is missing throttle-slew-rate-limitting which is present on 3DR's ArduPilot branch.  In the absence of a Green Cube this makes flying ArduPilot-master on your Solo *very dangerous*.

Prerequisites (get these first, and install them to your PC):
=============================================================
   Vagrant
   VirtualBox
   git
   
.. note::  

   If you are running windows, then you also need to nsure that git is set to leave line endings untouched. This command should fix that: 

::

   git config --global core.autocrlf false


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
   time /vagrant/builder.sh # about 2-15 hours ( depending on computer speed, and internet download speed ) 

note/s:

  The /vagrant folder inside of the solo-builder ‘vagrant up’ VM is actually the folder that you started the VM in ... 'solo-builder/'
  To get file/s out of the 'VM" you can ‘vagrant ssh’ into the VM and copy it to /vagrant and exit the vm… alternatively… 
  If you are on your solo's wifi, you can 'scp' the file/s directly to your solo/artoo. ( see 'update solo' below ) 

View Artoo (controller) build products:

::

   ls /solo-build/build/tmp-eglibc/deploy/images/imx6solo-3dr-artoo/

View Solo build products:

::

   ls /solo-build/build/tmp-eglibc/deploy/images/imx6solo-3dr-1080p/

Connect to your Solo/Controller Wifi from this point:

- SSID usually starts with ``SoloLink_``
- Default solo wifi password is ``sololink`` but you probably changed this when you first used it, right.
- If you want to continue to have internet access while Doing these next step:

   - get ``solo-cli`` repository and
   - run ``solo wifi --name=YOURHOMEWIFINAME --password=yourhomewifipassword`` to enable solo to get to the internet!


Update Solo :

::

   cd /solo-build/build/tmp-eglibc/deploy/images/imx6solo-3dr-1080p
   ssh root@10.1.1.10 -C "rm /log/updates/*"
   scp 3dr-solo.tar.gz root@10.1.1.10:/log/updates/
   ssh root@10.1.1.10 -C "cd /log/updates; md5sum 3dr-solo.tar.gz >3dr-solo.tar.gz.md5"
   ssh root@10.1.1.10 -C "touch /log/updates/UPDATE && /sbin/shutdown -r now"

note/s:

   If you have not SSH'd into your Solo before and changed things, you may be propted for a ssh password when doing 'ssh' or 'scp' commands. 
   The 'root' password to use is 'TjSDBkAu'.  more details here: https://dev.3dr.com/starting-network.html
   Two reboots of Solo may be required
   Your LED colours will be different that you are used to. Red cycling during first reboot, rainbow cycling during second reboot. maybe.  
   ssh can take some time to be available after a reflash, be patient, wait another minute.

After update, ensure the flash was successful:

   ::

      ls -l /log/updates/UPDATEFAILED
      cat /VERSION

The UPDATEFAILED file should NOT exist
The content of /VERSION should correspond to the build you made; in particular, ensure the date looks reasonable.

Update Artoo:

::

   cd /solo-build/build/tmp-eglibc/deploy/images/imx6solo-3dr-artoo
   ssh root@10.1.1.1 -C "rm /log/updates/*"
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

Load the default parameters file (which can be found here:  https://autotest.ardupilot.org/Tools/SoloBinaries/Solo_AC350_Params.param  )

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

