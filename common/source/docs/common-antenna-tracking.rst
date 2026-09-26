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

===========================================  ========  ================  ================================================================
Device                                       MAVLink?  Complete System?  URL
===========================================  ========  ================  ================================================================
Alpha Unmanned Systems GTRACK                Yes       Yes               :ref:`common-alphaunmannedsystems-vcs`
ARKBIRD AAT                                  No        Yes               https://www.arkbirdfpv.com/
Mainlink MF18                                Yes       No                https://www.szmainlink.com/mf18-antenna/
Motionew AAT v1 + Datalink Box               Yes       ?                 https://www.motionew.com/
Motionew CommuniNet MND-1410 AAT System      Yes       ?                 https://www.motionew.com/
Motionew Crossbow AAT                        Yes       No                https://www.motionew.com/shop/data-link-video-link/antenna/antenna-tracker/
Motionew Mini Crossbow AAT                   Yes       No                https://www.motionew.com/
MyFlyDream Crossbow AAT                      Yes       No                https://www.myflydream.com/
MyFlyDream miniCrossbow                      Yes       No                https://www.myflydream.com/
SoarApex ATS20                               Yes       No                https://soarapex.com/
===========================================  ========  ================  ================================================================

Complete System: includes necessary vehicle equipment

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
    common-alphaunmannedsystems-vcs


[copywiki destination="plane,copter,rover"]
