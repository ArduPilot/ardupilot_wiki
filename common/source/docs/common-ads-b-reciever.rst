.. _common-ads-b-reciever:

==============
ADS-B Reciever
==============

This article shows the **experimental** integration of an `Automatic Dependent Surveillance Broadcast <https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast>`__
(ADS-B) receiver into Copter and Plane.

[copywiki destination="copter,plane"]

.. warning::

   The current implementation is under development for advanced sense and avoid behavior for both plane and copter.
   
   Ping ADS-B Receiver hardware available at uAvionix partnershops.

   USA: Mid-Atlantic Multirotor (http://www.midatlanticmultirotor.com/product/ping/)
   
   U.K.: Unmanned Tech (http://www.unmannedtech.co.uk/)
   
   Germany: UAV Store (http://www.uav-store.de/ads-b-receiver/#cc-m-product-10156920693)
   
   Asia: `jDrones pingRX <http://store.jdrones.com/ping_ads_b_receiver_p/adsbping01.htm>`__

   We're posting it here to encourage further discussion and
   contribution

Overview
========

ADS-B is an air traffic surveillance technology that enables aircraft to
be accurately tracked by air traffic controllers and other pilots
without the need for conventional radar.

This article explains how to attach and configure a MAVLink based
ADS-B receiver by `uAvionix <http://www.uavionix.com/>`__\  called
PING™. Please visit their website for technical specs including RF
characteristics and connector pinout.

PingRX setup video:
   https://www.youtube.com/watch?v=v1R23fp5PDI
   
Specific PING™ products can be found here http://www.uavionix.com/products/

.. image:: ../../../images/uAvionix.png
    :target: ../_images/uAvionix.png

.. image:: ../../../images/Penny.png
    :target: ../_images/Penny.png


This first implementation of sense and avoid supports Plane and Copter. Object avoidance is only
active in AUTO mode.

Connection to flight controller
===============================

The ADSB receiver comes with a DF13 serial cable that can be plugged
directly into a Pixhawk serial port.

The Ping sensor should be mounted so that the antenna is oriented
vertically.

Setup through the ground station
================================

The key parameters that control the ADS-B packet parsing and object
avoidance behavior options.

-  ADSB_ENABLE - set to "1" to enable ADSB
-  ADSB_BEHAVIOR (see below)

Once operational aircraft within about 50km should appear on the ground
station map.

.. image:: ../../../images/ADSB_MissionPlanner.jpg
    :target: ../_images/ADSB_MissionPlanner.jpg

To test the systems you can compare with flights shown on
`flighradar24.com <https://www.flightradar24.com/>`__.

The data is transmitted via the `ADSB_VEHICLE message <http://mavlink.org/messages/common#ADSB_VEHICLE>`__. When
received by ArduPilot, it is forwarded on to all MAVLink serial ports
meaning that if you have a GCS or companion computer connected it will
receive the forwarded packets automatically.

Serial Setup
------------

If you are using one of the UARTs on your board which default to MAVLink
then the default settings will work fine for the Ping. Alternatively you
can connect the Ping to one of the other UARTs, such as the GPS UART (if
it is unused) or the serial4/5 UART. In that case you will need to
configure the uart as MAVLink at a baudrate of 57600.

For example, if you wanted to use the port marked "serial4/5" on a
Pixhawk you would set:

-  SERIAL4_PROTOCOL to 1 (meaning MAVLink)
-  SERIAL4_BAUD 57 (meaning 57600)

You will need to reboot your board after making those changes.

Vehicle Database
================

When enabled, the ADS-B library will store information for up to 25
vehicles detected by the ADS-B receiver. Due to some experimental work
in other features, such as EKF2, available RAM may be limited. It is
important to note that when ADS-B is disabled (ADSB_ENABLE=0) then the
memory is released, effectively freeing up about 1KB of RAM. When
enabled, the detected vehicle list is checked once per second for
potential conflicts.

Vehicle detection and behavior
==============================

The current ADS-B library is not considered to be fully-featured, more
of a proof of concept. It is not intended to be used as-is for realistic
object avoidance.  More work is needed to make it a robust tool for
avoiding in a practical way.

Current behavior is to check the detected vehicle list once per second
and determine if any other aircraft is within 200m. The altitude is
ignored. At that point, a behavior is performed in an effort to avoid
it. The behavior persists until no vehicles are within 400m.

Object avoidance behavior is dictated by one of three different
ADSB_BEHAVIOR options:

-  ADSB_BEHAVIOR=0, NONE. Objects are detected and the GCS is notified
   but no action is taken.
-  ADSB_BEHAVIOR=1, LOITER. If another vehicle is nearby, switch from
   AUTO to LOITER mode. When the vehicle leaves, switch back to AUTO and
   resume.
-  ADSB_BEHAVIOR=2, LOITER_AND_DESCEND. Same as (1) but decrease the
   altitude 1 m/s. If the other vehicle location persists, you will
   eventually loiter into the ground.

Simulation
==========

This includes a new ADS-B simulation component in SITL where you can
have aircraft flying at you. Joy! To enable this you must have pymavlink
v1.1.70. If you have an older version, use:

::

    sudo pip install --upgrade pymavlink MAVProxy

When starting SITL use the following command:

::

    sim_vehicle.sh -A --adsb --console --map

This also supports plugging in a hardware ADS-B receiver to your
computer using a USB-to-Serial converter which will overlay real ADS-B
traffic into the simulation.

TODO
====

#. Altitude is ignored so if a plane flies over at 35000' but within
   200m horizontally, then yes you'll consider it a threat right now.
   Obviously that needs to change.
#. Automatically switching from LOITER to AUTO mode behavior can get
   goofy if you switch the modes externally while actively avoiding
   another vehicle. That is, AUTO -> LOITER -> AUTO works fine if only
   the ADS-B logic is doing the mode switching.
#. Add Copter vehicle avoidance behaviour (currently only reads data
   from sensor and forwards to ground station)
#. Add more complicated behaviors
#. Add realistic threat detection by calculating the vehicle's
   trajectory and computing a time-to-impact rather than a simple radius
   distance.
#. Add visualization to GCS
#. This implementation was intentionally simple to promote others to
   enhance it to suit their/everyones' needs. The limited number of
   params was simply to get the device driver rolling without committing
   to any particular param name.
