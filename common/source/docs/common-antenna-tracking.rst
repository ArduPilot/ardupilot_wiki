.. _common-antenna-tracking:

================
Antenna Tracking
================

*Antenna Trackers* are systems that track your vehicle's location, and
use this information to correctly align a directional antenna. This
approach significantly improves the range over which signals can be both
sent and received from the ground station.

ArduPilot supports two methods for Antenna Tracking. Both methods use
GPS information from the vehicle and ground station to aim the antenna.
The main difference is that the
:ref:`AntennaTracker <antennatracker:home>` is independent
of any particular GCS.

:ref:`AntennaTracker <antennatracker:home>`

-  The AntennaTracker firmware turns an ArduPilot-supported board
   (Pixhawk, APM2, etc) into the controller for an antenna tracker.
-  The board calculates the required antenna direction and can drive the
   antenna's servos directly.
-  Mission Planner (or any other GCS) may be used, but is not required.

:ref:`Mission Planner-based GPS Tracking <common-mission-planner-gps-based-antenna-tracking>`

-  Uses the *Mission Planner* GCS to determine the direction to aim the
   antenna.
-  The PC running Mission Planner must have GPS.
-  You will need a special servo driver board to control the servo's.

[site wiki="rover"]

.. toctree::
    :maxdepth: 1

    Mission Planner Antenna Tracking <common-mission-planner-gps-based-antenna-tracking>
    Antenna Design Overview <common-antenna-design>
[/site]

[site wiki="planner"]

.. toctree::
    :maxdepth: 1

    Mission Planner Antenna Tracking <common-mission-planner-gps-based-antenna-tracking>
    Antenna Design Overview <common-antenna-design>
[/site]

[site wiki="copter"]

.. toctree::
    :maxdepth: 1

    Mission Planner Antenna Tracking <common-mission-planner-gps-based-antenna-tracking>
    Antenna Design Overview <common-antenna-design>
[/site]

[site wiki="plane"]

.. toctree::
    :maxdepth: 1

    Mission Planner Antenna Tracking <common-mission-planner-gps-based-antenna-tracking>
    Antenna Design Overview <common-antenna-design>
[/site]
