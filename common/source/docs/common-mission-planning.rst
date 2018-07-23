.. _common-mission-planning:

================
Mission Planning
================

This section contains articles about Mission Planning; creating
automated missions that will run when the ArduPilot is set to
:ref:`AUTO <auto-mode>` mode.

.. note::

   The examples demonstrate mission planning using :ref:`Mission Planner <planner:home>`, a fully-functioned
   Windows-based Ground Control Station (GCS) that you can :ref:`download and install from here <planner:install-mission-planner>`.

   .. figure:: ../../../images/mp_flight_plan_screen.jpg
      :target: ../_images/mp_flight_plan_screen.jpg

      Mission Planner: Flight Plan Screen

   The main concepts will also apply to other :ref:`Ground Control Stations <common-choosing-a-ground-station>` (although the details of
   the user interface will differ).

.. toctree::
    :maxdepth: 1
    
[site wiki="rover"]
    Learning a Mission <learning-a-mission>
[/site] 

    Planning a Mission with Waypoints and Events <common-planning-a-mission-with-waypoints-and-events>
[site wiki="copter"]
    mission-command-list
[/site]
    
    Mission Command List <common-mavlink-mission-command-messages-mav_cmd>
    Camera Control and Auto Missions <common-camera-control-and-auto-missions-in-mission-planner>
    Rally Points <common-rally-points>
    Geotagging Images with Mission Planner <common-geotagging-images-with-mission-planner>

[site wiki="plane"]
    Terrain Following <common-terrain-following>
[/site]
[site wiki="copter"]
    Terrain Following <terrain-following>
[/site]


[copywiki destination="copter,plane,rover,planner"]


