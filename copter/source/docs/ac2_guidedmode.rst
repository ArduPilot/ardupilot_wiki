.. _ac2_guidedmode:

===========
Guided Mode
===========

Guided mode is a capability of Copter to dynamically guide the copter to
a target location wirelessly using a telemetry radio module and ground
station application. This page provides instructions for using guided
mode.

.. note:: Guided Mode can also be used by LUA scripts and companion computers to command vehicle movement and navigation.

Overview
========

Guided mode is not a traditional flight mode that would be assigned to a
mode switch like other flight modes. The guided mode capability is
enabled using a ground station application (such as Mission Planner) and a
:ref:`telemetry radio <common-telemetry-landingpage>`. 
This capability allows the pilot to
interactively command the copter to travel to a target location by
clicking on a point on the Mission Planner Flight Data map. Once the
location is reached, the copter will hover at that location, waiting for
the next target. Follow Me mode also uses Guided Mode to make the copter
follow the pilot around the field.

.. image:: ../images/guided.jpg
    :target: ../_images/guided.jpg

What you'll need
================

To use guided mode, you'll need a :ref:`telemetry radio <common-telemetry-landingpage>` 
allowing your computer and
autopilot to communicate during flight, a ground station computer or
tablet, and a ground station application such as :ref:`Mission Planner <planner:home>`.

Instructions
============

-  Set up your copter at the field and establish a MAVLink connection
   over wireless telemetry between your copter and your laptop.
-  On your laptop, using the software that came with the telemetry
   module, make sure that it's working and that you have a GPS lock.
-  Take off in :ref:`Loiter Mode <loiter-mode>` and climb to a safe altitude
-  In the Mission Planner Flight Data screen map, try right-clicking on
   a nearby spot and select "Fly to Here".
-  You will be asked for a guided mode altitude. Enter an above home
   altitude in meters.

.. image:: ../images/FlightModes_Guide_FlyToHere.jpg
    :target: ../_images/FlightModes_Guide_FlyToHere.jpg

-  A "Guided" target should appear on the map and the orange line (which
   indicates the target heading) should point to this guided target.

.. image:: ../images/FlightModes_Guide_TargetEstablished.jpg
    :target: ../_images/FlightModes_Guide_TargetEstablished.jpg

-  The vehicle should fly to the target location and wait there until
   you enter another location or switch to another mode.

.. note::

   On *Mission Planner* there is no need to set up one of your flight
   modes as "Guided". This may not be the case for other Ground Control
   Stations.

Speed Control
=============

The maximum horizontal speed of the copter can be adjusted with the
**Speed** (:ref:`WPNAV_SPEED<WPNAV_SPEED>`) parameter from the Mission Planner's
Config/Tuning >> Copter Pids screen (see blue box above).  The default
is 1000 meaning 10m/s.  A typical copter can reach top speeds of 10m/s ~
13m/s (i.e. 1000 ~ 1300) before it becomes unable to both maintain
altitude and horizontal speed.

The vertical speeds up and down can similar be adjusted with the **Speed Up** (:ref:`WPNAV_SPEED_UP<WPNAV_SPEED_UP>`) and **Speed Dn** (:ref:`WPNAV_SPEED_DN<WPNAV_SPEED_DN>`) parameters. :ref:`WPNAV_ACCEL_Z<WPNAV_ACCEL_Z>` determines how fast the speed can change.

Speed can also be controlled while in GUIDED mode with :ref:`MAVLink commands <common-mavlink-mission-command-messages-mav_cmd>` like :ref:`DO_CHANGE_SPEED<mav_cmd_do_change_speed>`.

Guided Mode Options
===================

The :ref:`GUID_OPTIONS<GUID_OPTIONS>` parameter allows several guided mode behavior changes:

===    ==========
Bit 	Meaning
===    ==========
0 	   Allow Arming from Transmitter
2 	   Ignore pilot yaw input
3 	   SetAttitudeTarget interprets Thrust As Thrust
4      Do not stabilize PositionXY
5      Do not stabilize VelocityXY
6      Waypoint navigation used for position targets
7      Allow weathervaning
===    ==========

Bit 0 (e.g. "1") allowing arming in Guided mode from the RC transmitter

Bit 2 (e.g. "4") disables the pilot's ability to change the vehicle's heading using the RC transmitter

Bit 3 (e.g. "8") changes the interpretation of the `SET_ATTITUDE_TARGET MAVLink <https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET>`__ command's ``thrust`` field to be pure thrust expressed as a value between 0 and 1, instead of a climb rate. See :ref:`Copter Commands in Guided Mode <copter-commands-in-guided-mode>` for more details

Bit 4 (e.g. "16") disables the position controller's XY axis position error correction.  This may be useful if an external controller is providing high speed targets which already include position error correction

Bit 5 (e.g. "32") is the same as above but affects the position controller's velocity error correction

Bit 6 (e.g. "64") enables S-Curve path planning (the same as is used in :ref:`Auto mode <auto-mode>`) to reach the position target.  This may result a smoother acceleration and deceleration but the position target cannot be updated quickly.  This also allows :ref:`object avoidance path planning <common-object-avoidance-landing-page>` (e.g. :ref:`Bendy Ruler <common-oa-bendyruler>` and :ref:`Dijkstras<common-oa-dijkstras>`) to be used in Guided mode

Bit 7 (e.g. "128") enables :ref:`weathervaning <weathervaning>`

The :ref:`GUID_TIMEOUT<GUID_TIMEOUT>` parameter holds the timeout (in seconds) when the vehicle is being controlled using attitude, velocity and/or acceleration commands. If no commands are received from the companion computer for this many seconds, the vehicle will slow to a stop (if velocity and/or acceleration commands were being provided) or hold a level hover (if attitude commands were provided). The default setting is 3 seconds.

.. _guided_nogps:

Guided_NoGPS
============
This variation of Guided mode does not require a GPS but it only accepts `attitude targets <https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET>`__.  Because it does not accept position or velocity targets like regular Guided mode it is generally not useful for regular users.  This mode was created for use by companion computers that may want to fly the vehicle as if it was in AltHold mode.

.. note::

   Guided_NoGPS does not allow a vehicle to hold position without a GPS (i.e. non-GPS navigation).  For information on :ref:`non-GPS navigation see this wiki page <common-non-gps-navigation-landing-page>`
