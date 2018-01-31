.. _supported-autopilot-controller-boards:

=====================================
Supported AutoPilot Controller Boards
=====================================

Currently ArduPilot supports the following autopilot boards.

Pixhawk
=======

Updated PX4, with more memory, improved sensors and a much
easier-to-use design. See the :ref:`Pixhawk Overview <copter:common-pixhawk-overview>` (wiki)
for more information.

+--------------------------------------+----------------------------------------------------------------------------------------------------+
| **Purchase**                         | various                                                                                            |
|                                      |                                                                                                    |
+--------------------------------------+----------------------------------------------------------------------------------------------------+
| **Setup**                            | `Pixhawk Overview <http://copter.ardupilot.com/common-pixhawk-overview/#specifications>`__,        |
|                                      | `Powering <http://copter.ardupilot.com/wiki/common-powering-the-pixhawk/>`__                       |
+--------------------------------------+----------------------------------------------------------------------------------------------------+
| **Design files**                     | `Schematic <http://firmware.ardupilot.org/downloads/wiki/pdf_guides/px4fmuv2.4.3_schematic.pdf>`__ |
|                                      |                                                                                                    |
|                                      | `Layout <http://firmware.ardupilot.org/downloads/wiki/pdf_guides/Pixhawk-Open-Hardware.zip>`__     |
+--------------------------------------+----------------------------------------------------------------------------------------------------+

APM2.x
======

`APM2 <http://store.jdrones.com/ArduPilot_MEGA_2_5_p/fcapm25side.htm>`__
is a popular but now obsolete AVR2560 8-bit autopilot.

.. note::

   APM 2.6 is not recommended for any new users, with many features introduced since 2014 not supported.

+--------------------------------------+--------------------------------------------+
| **Purchase**                         | |jdrones_amp2.x|                           |
+--------------------------------------+--------------------------------------------+
| **Setup**                            | :ref:`copter:common-apm25-and-26-overview` |
+--------------------------------------+--------------------------------------------+
| **Design files**                     | |APM_v25_schematic.pdf|                    |
|                                      |                                            |
|                                      | |APM_v252_RELEASE.zip|                     |
|                                      |                                            |
+--------------------------------------+--------------------------------------------+

.. |APM_v252_RELEASE.zip| replace:: `APM board layout <http://firmware.ardupilot.org/downloads/wiki/pdf_guides/APM_v252_RELEASE.zip>`__
.. |APM_v25_schematic.pdf| replace:: `APM schematic diagram <http://firmware.ardupilot.org/downloads/wiki/pdf_guides/APM_v25_schematic.pdf>`__
.. |jdrones_amp2.x| replace:: `store.jdrones.com <http://store.jdrones.com/ArduPilot_MEGA_2_5_p/fcapm25side.htm>`__  

PX4
===

A 32 bit ARM based autopilot, no longer produced, using the
`NuttX <http://nuttx.org/>`__ real-time operating system. See the :ref:`PX4 Overview <copter:common-px4fmu-overview>`
(wiki) for more information.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | Not available                        |
+--------------------------------------+--------------------------------------+
| **Setup**                            | :ref:`copter:common-px4fmu-overview` |
+--------------------------------------+--------------------------------------+
| **Design files**                     | |PX4_home_page|                      |
|                                      |                                      |
|                                      | |PX4_manual|                         |
|                                      |                                      |
|                                      | |PX4_schematic|                      |
|                                      |                                      |
|                                      | |PX4_eagle_1_6|                      |
|                                      |                                      |
|                                      | |PX4_eagle_1_7|                      |
+--------------------------------------+--------------------------------------+


.. |PX4_home_page| replace:: `Module homepage <https://pixhawk.ethz.ch/px4/modules/px4fmu>`__
.. |PX4_manual| replace:: `Manual <https://pixhawk.ethz.ch/px4/_media/modules/px4fmu-manual-v1.6.pdf>`__
.. |PX4_schematic| replace:: `Schematics download <https://pixhawk.ethz.ch/px4/_media/modules/px4fmu-schematic-v1.6.pdf>`__
.. |PX4_eagle_1_6| replace:: `Eagle files for version 1.6 download <http://stuff.storediydrones.com/PX4FMUv1.6.zip>`__
.. |PX4_eagle_1_7| replace:: `Eagle files for version 1.7 download <http://stuff.storediydrones.com/PX4FMUv1.7.zip>`__    



Arsov AUAV-X2
=============

`Arsov AUAV-X2 <http://www.auav.co/product-p/auavx2.htm>`__ is a high
quality, compact, light weight and cost effective alternative to the PX4
V2 or PixHawk autopilots. It is 100% compatible with the Ardupilot Pixhawk firmware.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | `www.auav.co <http://www.auav.co/pro |
|                                      | duct-p/auavx2.htm>`__                |
+--------------------------------------+--------------------------------------+
| **Setup**                            | `Manual <http://www.auav.co/v/Public |
|                                      | Downloads/AUAV_X2_Manual.pdf>`__     |
+--------------------------------------+--------------------------------------+
| **Design files**                     | `License <https://github.com/PX4/Har |
|                                      | dware/blob/master/README.md>`__      |
|                                      |                                      |
|                                      | `Main Board Design                   |
|                                      | Files <http://www.auav.co/v/PublicDo |
|                                      | wnloads/AUAV_X2_R01.zip>`__          |
|                                      |                                      |
|                                      | `mIMU Board Design                   |
|                                      | Files <http://www.auav.co/v/PublicDo |
|                                      | wnloads/AUAV_Micro_IMU_V2.zip>`__    |
+--------------------------------------+--------------------------------------+


.. _supported-autopilot-controller-boards_erle-brain3_autopilot:

Erle-Brain 3 autopilot
======================

:ref:`Erle-Brain 3 <copter:common-erle-brain-linux-autopilot>` An
autopilot for making drones and robots powered by Debian/Ubuntu and with
official support for the Robot Operating System (ROS). It has access to
an app store for drones and robots.

.. note::

   `Erle-Brain 3 <https://erlerobotics.com/blog/product/erle-brain-v3/>`__ is a
   commercial artificial robotic brain that runs Ardupilot. It combines
   a Raspberry Pi 3, a sensor cape and other components in order to achieve
   a complete embedded Linux board.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | `www.erlerobotics.com <https://erler |
|                                      | obotics.com/blog/product/erle-brain- |
|                                      | 3/>`__                               |
+--------------------------------------+--------------------------------------+
| **Setup**                            | `Documentation <http://erlerobotics. |
|                                      | com/docs/>`__                        |
+--------------------------------------+--------------------------------------+


.. |erlebrain_2_purchase| replace:: `Erle-Brain2 (store) <https://erlerobotics.com/blog/product/erle-brain-v2/>`__



.. _supported-autopilot-controller-boards_erle-brain_autopilot:

Erle-Brain 1 autopilot (discontinued)
=====================================

:ref:`Erle-Brain <copter:common-erle-brain-linux-autopilot>` An
autopilot for making drones powered by Ubuntu and with official support
for the Robot Operating System (ROS). It has access to the first app
store for drones and robots.

.. note::

   Erle-Brain is a commercial autopilot. It combines a BeagleBone
   Black, the :ref:`PixHawk Fire Cape <supported-autopilot-controller-boards_pixhawk_fire_cape_pxf>` (above) and
   other components.

+--------------------------------------+-----------------------------------------------------------------------------+
| **Purchase**                         | `erle-brain <http://erlerobotics.com/blog/product/erle-brain/>`__           |
+--------------------------------------+-----------------------------------------------------------------------------+
| **Setup**                            | `Updating software <http://erlerobotics.com/blog/updating-the-software/>`__ |
|                                      |                                                                             |
|                                      |                                                                             |
|                                      | :ref:`BeaglePilot                                                           |
|                                      | Project <beaglepilot>` (wiki)                                               |
|                                      |                                                                             |
|                                      |                                                                             |
|                                      | `Building ArduPilot for BeagleBone Black on                                 |
|                                      | Linux <building-for-beaglebone-black-on-linux/>`__  (wiki)                  |
|                                      |                                                                             |
+--------------------------------------+-----------------------------------------------------------------------------+
| **Design files**                     | `Design files <https://github.com/ArduPilot/PXF>`__                         |
|                                      |                                                                             |
|                                      |                                                                             |
|                                      | :ref:`copter:common-erle-brain-linux-autopilot`                             |
+--------------------------------------+-----------------------------------------------------------------------------+


.. _supported-autopilot-controller-boards_pixhawk_fire_cape_pxf:

PixHawk Fire Cape (PXF)
=======================

The PixHawk Fire Cape (PXF) is a daughter board for the :ref:`BeagleBone Black <beaglepilot>`
(BBB) development board that allows to create a fully functional Linux
autopilot for drones. The combination of BBB and PXF allows to a Linux
computer is a fully functional autopilot.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | No longer produced                   |
|                                      |                                      |
+--------------------------------------+--------------------------------------+
| **Setup**                            | |erlebrain_updating|                 |
|                                      |                                      |
|                                      |                                      |
|                                      | :ref:`BeaglePilot                    |
|                                      | Project <beaglepilot>` (wiki)        |
|                                      |                                      |
|                                      | `Building ArduPilot for BeagleBone   |
|                                      | Black on                             |
|                                      | Linux <building-for-beaglebone-black |
|                                      | -on-linux/>`__                       |
|                                      | (wiki)                               |
+--------------------------------------+--------------------------------------+
| **Design files**                     | https://github.com/ArduPilot/PXF     |
+--------------------------------------+--------------------------------------+

.. |erlebrain_updating| replace:: `Updating the software <http://erlerobotics.com/blog/updating-the-software/>`__

PixHawk Fire Mini Cape (PXFmini)
================================

The PixHawk Fire Mini Cape (PXFmini) is a daughter board designed for
the low cost `Raspberry Pi zero <https://www.raspberrypi.org/blog/raspberry-pi-zero/>`__ that
allows to create a fully functional Linux autopilot for drones. Inspired
in the PXF cape, provides a minimalist approach which allows having a
reduced size/lightweight and low-cost.

+--------------------------------------+----------------------------------------------------------------------------------------------------------------+
| **Purchase**                         | `pxfmini <https://erlerobotics.com/blog/product/pxfmini/>`__                                                   |
+-------------------------------------------------------------------------------------------------------------------------------------------------------+
| **Setup**                            | `Setup <http://erlerobotics.com/docs/Artificial_Brains_and_Autopilots/Autopilot_shields/PXFmini/Intro.html>`__ |
+--------------------------------------+----------------------------------------------------------------------------------------------------------------+
| **Design files**                     | To be delivered in February 2016                                                                               |
+--------------------------------------+----------------------------------------------------------------------------------------------------------------+

BBBMINI Cape
============

Low budget DIY Autopilot Cape for BeagleBone Black/Green running ArduPilot on
Linux.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | DIY                                  |
+--------------------------------------+--------------------------------------+
| **Setup**                            | https://github.com/mirkix/BBBMINI    |
+--------------------------------------+--------------------------------------+
| **Design files**                     | https://github.com/mirkix/BBBMINI    |
+--------------------------------------+--------------------------------------+

APM1 (discontinued)
===================

An AVR2560 based autopilot with separate sensor board (aka "oilpan"). As
with APM2 this is no longer supported by Copter. Not recommended for any
new users.

Closed boards
=============

The following boards are known to be closed (they do not publish their
design files).

Parrot Bebop and Bebop2 Drone
-----------------------------

The `Bebop Drone <http://www.parrot.com/usa/products/bebop-drone/>`__ is
a Wifi controlled quadrotor UAV that uses `this Linux autopilot <https://us.store.parrot.com/en/accessoires/247-main-board-3520410021619.html>`__
and which can run Copter firmware.

From Copter 3.3 the Bebop can run ArduPilot. Instructions for converting
a Bebop to run ardupilot are
:ref:`here <building-for-bebop-on-linux>`.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | `Parrot                              |
|                                      | Store <https://us.store.parrot.com/e |
|                                      | n/bebop-drone/297-bebop-drone-352041 |
|                                      | 0023996.html#/color-red>`__          |
+--------------------------------------+--------------------------------------+
| **Setup**                            | `Building for Bebop on               |
|                                      | Linux <building-for-bebop-on-linux>` |
|                                      | __                                   |
|                                      | (wiki)                               |
+--------------------------------------+--------------------------------------+
| **Design files**                     | NA                                   |
+--------------------------------------+--------------------------------------+

.. note::

   Some of this information was taken from the `Paparazzi UAV wiki
   page on the Bebop <http://wiki.paparazziuav.org/wiki/Bebop>`__.

NAVIO+
------

`NAVIO+ <http://www.emlid.com/>`__ is a sensor cape for the RaspberryPi2
from Emlid. Under rapid development.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | |navio_plus_shop|                    |
+--------------------------------------+--------------------------------------+


.. |navio_plus_shop| replace:: `www.emlid.com/shop/navio-plus <http://www.emlid.com/shop/navio-plus/>`__  
 

NAVIO2
-------

`NAVIO2 <http://www.emlid.com/>`__ is a new sensor cape for the RaspberryPi 3
from Emlid.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | |navio2_shop|                        |
+--------------------------------------+--------------------------------------+

.. |navio2_shop| replace:: `www.emlid.com/shop/navio2 <http://www.emlid.com/shop/navio2/>`__  
 

VRBrain
-------

`VRBrain <http://vrbrain.wordpress.com/>`__ is a multipurpose controller
board that comes loaded with a 32 bit version of Copter firmware. At
time of writing the latest version is `VR Brain 5 <http://www.virtualrobotix.it/index.php/en/shop/autopilot/vrbrain5-detail>`__.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | `vrbrain.wordpress.com/store/ <https |
|                                      | ://vrbrain.wordpress.com/store/>`__  |
+--------------------------------------+--------------------------------------+


Qualcomm Snapdragon Flight Kit
------------------------------

The `Qualcomm® Snapdragon Flight™ Kit (Developer’s Edition) <http://shop.intrinsyc.com/products/snapdragon-flight-dev-kit>`__
is small (58x40mm) but offers a lot of CPU power and two on-board
cameras. It contains 4 'Krait' ARM cores which run Linux (Ubuntu 14.04
Trusty, by default), and 3 'Hexagon' DSP cores which run the QURT RTOS.
In addition it includes Wi-Fi, Bluetooth connectivity, automotive-grade
GPS and many more features.

Information about using this board with ArduPilot can be found here:
:ref:`Building for Qualcomm Snapdragon Flight Kit <building-for-qualcomm-snapdragon-flight-kit>`, `QURT Port <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_QURT/README.md>`__
(Github) and `QFlight Port <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_Linux/qflight>`__
(Github).

.. warning::

   Due to some rather unusual licensing terms from Intrinsyc we
   cannot distribute binaries of ArduPilot (or any program built with the
   Qualcomm libraries). So you will have to build the firmware
   yourself.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | `shop.intrinsyc.com/products/snapdra |
|                                      | gon-flight-dev-kit <http://shop.intr |
|                                      | insyc.com/products/snapdragon-flight |
|                                      | -dev-kit>`__                         |
+--------------------------------------+--------------------------------------+


