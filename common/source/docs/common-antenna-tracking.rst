.. _common-antenna-tracking:

================
Antenna Tracking
================

*Antenna Trackers* are systems that track your vehicle's location, and use this information to correctly align a directional antenna. This
approach significantly improves the range over which signals can be both sent and received from the ground station.

ArduPilot supports two methods for Antenna Tracking. One method uses GPS information from the vehicle and ground station to aim the antenna. The other uses an autopilot board and gets its GPS directly via telemetry from the vehicle so this :ref:`AntennaTracker <antennatracker:home>` is independent of any particular GCS.

:ref:`ArduPilot Autopilot based AntennaTracker <antennatracker:home>`
=====================================================================

-  The AntennaTracker firmware turns an ArduPilot-supported board (:ref:`common-autopilots`) nto the controller for an antenna tracker.
-  The board calculates the required antenna direction and can drive the antenna's servos directly.
-  Mission Planner (or any other GCS) may be used, but is not required.

TurnKey Systems
===============

.. toctree::
    :maxdepth: 1
    :hidden:

    Alpha Unmanned Systems <common-alphaunmannedsystems-vcs>
    MF18 Antenna Tracker <https://www.szmainlink.com/mf18-antenna/>
    SoarApex <https://soarapex.com/>

:ref:`Mission Planner-based GPS Tracking <common-mission-planner-gps-based-antenna-tracking>`
=============================================================================================

-  Uses the *Mission Planner* GCS to determine the direction to aim the antenna.
-  The PC running Mission Planner must have GPS.
-  You will need a special servo driver board to control the servos.

.. toctree::
    :maxdepth: 1
    :hidden:

    Mission Planner Antenna Tracking <common-mission-planner-gps-based-antenna-tracking>
    Antenna Design Overview <common-antenna-design>


[copywiki destination="plane,copter,rover"]
