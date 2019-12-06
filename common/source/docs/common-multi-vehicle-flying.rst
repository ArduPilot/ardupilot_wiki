.. _common-multi-vehicle-flying:

====================
Multi-Vehicle Flying
====================

This page shows how to monitor and control multiple vehicles using a
single ground station. Although this capability has been demonstrated
(see video below) it is somewhat
`experimental <https://diydrones.com/profiles/blogs/multi-vehicle-testing-with-apm-copter-tracker-and-mission-planner>`__.

.. note::

   This example has been greatly surpassed by `Michael Clement's 50 plane demonstration <https://diydrones.com/profiles/blogs/from-zero-to-fifty-planes-in-twenty-seven-minutes>`__\ 

..  youtube:: M4LxtYa94nk
    :width: 100%

Equipment you will need
=======================

-  Multiple `Planes <https://ardupilot.org/plane/index.html>`_ or
   :ref:`Copters <copter:home>` or
   :ref:`Rovers <rover:home>`
-  An :ref:`Antenna Tracker <antennatracker:home>`\ (a.k.a. Tracker)
   to combine the telemetry data from vehicles and send to the GCS.
-  Pairs of\ :ref:`telemetry radios <common-telemetry-landingpage>` for
   each vehicle (i.e. 2x number of vehicles) OR multi-point radios (like
   the :ref:`RFD900 <common-rfd900>`) in which case just 1 radio for
   each vehicle + 1 for the ground station.
-  GCS capable of displaying/controlling multiple vehicles (i.e. the
   :ref:`Mission Planner <planner:home>`).
-  RC transmitter for each vehicle (optional).

Pre-Flight Setup
================

-  If using SiK radios, :ref:`set the NetID <common-sik-telemetry-radio_configuring_using_the_mission_planner>`
   of each pair of radios to a unique number.  I.e. if controlling 3
   vehicles, set the ``NetID`` of one pair to "23", the 2nd pair to
   "24", the 3rd pair to "25".
-  Connect one of each radio pairs to the vehicle autopilots
   (i.e. Pixhawk) using Telem1 or Telem2.
-  Connect the other pairs to the Tracker's controller (i.e. Pixhawk)
   using Telem1, Telem2 and Serial 4/5 ports.
-  If using Serial 4/5 set the Tracker's ``SERIAL4_PROTOCOL`` to "1" to
   enable MAVLink communication on that port.
-  Connect to each vehicle's autopilot and the tracker and set
   the ``SYSID_THISMAV`` to a unique number (i.e. "1" for first vehicle,
   "2" for second vehicle, "3" for third, "9" for the tracker).
-  On the GCS, :ref:`reduce data rate <common-mission-planner-telemetry-logs_setting_the_datarate>`
   to the minimum acceptable value to reduce network traffic.
-  Plug in the tracker to the ground station PC using a USB cable.

Controlling the vehicles
========================

Plug in the battery of each vehicle and tracker.  After connecting to
the tracker from the *Mission Planner* (or other GCS) each vehicle
including the tracker should appear on the Map.

Controlling the vehicle depends upon the GCS used but if using the
*Mission Planner*, "Ctrl-X" pulls up a drop-down selector which allows
the user to choose which vehicle to control.

If using Copter, switch each vehicle into GUIDED mode.  On the MP's map,
right-mouse-button-click and select "Takeoff" and input 2m to get the
vehicle off the air.  Once in the air, each vehicle can be pushed around
in GUIDED mode (by using Ctrl-X to select the vehicle and then click on
the map) or the mode can be changed to AUTO etc.

If you wish to retake control of a vehicle, use the transmitter for that
vehicle to change the flight to LOITER, STABILIZE, etc and then fly
manually.  If only one transmitter is being used (not recommended) then
use the GCS to switch the single vehicle's mode to LOITER and then
control with the TX sticks (the other vehicles will ignore the TX's
roll, pitch and throttle input if they are in AUTO or GUIDED - they will
accept yaw input however).
