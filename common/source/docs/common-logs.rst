.. _common-logs:

====
Logs
====

Log Types (Dataflash vs Tlogs)
==============================

There are two ways to record your flight data. With some exceptions, the
two methods record very similar data but in different ways:

-  :ref:`Dataflash logs <common-downloading-and-analyzing-data-logs-in-mission-planner>` are recorded on the autopilot (often to the SD card) so they must be downloaded from the autopilot after a flight
-  :ref:`Telemetry logs <planner:mission-planner-telemetry-logs>` (also known as "Tlogs") are recorded by the ground station (i.e. Mission Planner) on the local PC when the autopilot is connected via a :ref:`telemetry link <common-telemetry-landingpage>`

If you are not yet familiar with the basics of these log files, first review the introductory pages to understand where these logs are stored and how you can download and view the information held within them.

Topics related to logging and analysis
--------------------------------------

.. toctree::
    :maxdepth: 1

[site wiki="plane,copter,dev,planner,rover"]
    Dataflash Logs <common-downloading-and-analyzing-data-logs-in-mission-planner>
    Telemetry Logs <https://ardupilot.org/planner/docs/mission-planner-telemetry-logs.html>
    Diagnosing problems using Logs <common-diagnosing-problems-using-logs>
[/site]
[site wiki="plane"]
    Plane Log Messages <logmessages>
[/site]
[site wiki="copter"]
    Copter Log Messages <logmessages>
[/site]
[site wiki="rover"]
    Rover Log Messages <logmessages>
[/site]
[site wiki="antennatracker"]
    Antenna Tracker Log Messages <logmessages>
[/site]
[site wiki="plane"]
    Log Analysis Case Study: Fly-by-Wire <case-study-fly-by-wire>
    Log Analysis Case Study: Turn Rate Adjustment <case-study-turn-rate>
[/site]
[site wiki="plane,copter,rover,dev,planner"]
    Measuring Vibration <common-measuring-vibration>
    Measuring Vibration with "Batch Sampling" <common-imu-batchsampling>
    Measuring Vibration with "Raw IMU Logging" (Preferred) <common-raw-imu-logging>
[/site]


Tools for Log Analysis
----------------------

.. toctree::
    :maxdepth: 1


    MAVExplorer <https://ardupilot.org/dev/docs/using-mavexplorer-for-log-analysis.html>
    ArduPilot WebTools <common-webtools>
    MissionPLanner <https://ardupilot.org/planner/>
    QGroundControl <http://qgroundcontrol.com/>
    Dronee Plotter <https://plot.dronee.aero/>


[copywiki destination="copter,plane,rover,dev,antennatracker,planner"]
