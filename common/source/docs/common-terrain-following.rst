.. _common-terrain-following:

=================
Terrain Following
=================

As of Plane 3.0.4 you can use automatic terrain following for fixed wing
aircraft if you have an autopilot board with local storage (such as the
Pixhawk). This page explains how terrain following works, how to enable
it and what its limitations are.

If using Copter, see :ref:`Copter specific terrain following instructions here <copter:terrain-following>`.

.. image:: ../../../images/Terrain_TitleImage.png
    :target: ../_images/Terrain_TitleImage.png

How it works
------------

Terrain following works by maintaining a terrain database on the microSD
card on the autopilot which gives the terrain height in meters above sea
level for a grid of geographic locations. On the Pixhawk this database
is stored in the APM\\TERRAIN directory on the microSD card.

The database is populated automatically by the autopilot requesting
terrain data from the ground station over a MAVLink telemetry link. This
can happen either during flight planning when the autopilot is connected
over USB, or during flight when connected over a radio link. Once the
terrain data is sent from the GCS to the autopilot it is stored on the
microSD card so that it is available even when the GCS is not connected.
This makes it possible for the autopilot to use terrain data to perform
a terrain following RTL (Return To Launch) even when it is not able to
talk to the ground station.

During flight the ArduPilot code automatically pages in the needed
terrain data from the microSD card into memory as the aircraft
approaches a new area. It maintains an area of about 7km by 8km in
memory if the default terrain grid spacing is used.

In addition to any terrain data for the immediate vicinity of the
aircraft, ArduPilot also asks the ground station for terrain data for
any mission waypoints which are loaded, and for any rally points which
are loaded. This ensures that terrain data is available on the microSD
card for a whole mission even if the GCS becomes unavailable.

Terrain Following Flight Modes
------------------------------

In Plane terrain following is available in the following flight modes:

-  RTL - Return to launch
-  LOITER - circle a point
-  CRUISE - long distance cruising
-  FBWB - speed/height maintenance
-  GUIDED - "fly to" waypoints
-  AUTO - fully autonomous missions

Use of terrain following in RTL, LOITER, CRUISE, FBWB and GUIDED modes
is controlled by the TERRAIN_FOLLOW parameter. That parameter defaults
to off, so no terrain following will be used in those modes. Set
TERRAIN_FOLLOW to 1 to enable terrain following in those modes.

Use of terrain following in AUTO missions is controlled on a waypoint by
waypoint basis using the reference frame of the waypoint. Normal (non
terrain following) waypoints have a "Relative" reference frame, and
altitudes are specified relative to the home location. Terrain following
waypoints have a "Terrain" reference frame, and altitudes are relative
to the ground level given in the terrain database.

Uses of Terrain Following
-------------------------

Terrain following is very useful when flying ArduPilot in areas where
the terrain may vary significantly. Key uses are:

-  **Safe RTL**. Being able to come over a hill rather than trying to
   fly through it when you enter RTL in a hilly area is very useful!
-  **Aerial Photography.** It is useful to be able to maintain a
   constant altitude over the ground when taking a sequence of aerial
   photos
-  **FPV flying.** When flying FPV in CRUISE mode it is useful to
   maintain constant height above the ground so you can spend more time
   enjoying the scenary and less time avoiding hills

Sources of terrain data
-----------------------

The ground station is responsible for providing the raw terrain data
which is sent to the aircraft via MAVLink. Right now only MissionPlanner
(version 1.3.9 or later) and MAVProxy support the required TERRAIN_DATA
and ``TERRAIN_REQUEST`` messages needed for terrain following support. If
you are using a different ground station then to load terrain data you
will need to connect using one of the two support ground stations to
allow ArduPilot to load terrain data onto your board. It typically takes
around 2 minutes to load all the terrain data for a mission. Once it is
loaded it is saved permanently on the microSD card.

Both MissionPlanner and MAVProxy support the global
`SRTM <https://en.wikipedia.org/wiki/SRTM>`__ database for terrain data.
That database has a global grid spacing of 3 arc-seconds (around 100
meters), but has a smaller grid spacing in some parts of the world
(around 30 meters in the US). Support for other terrain databases can be
added by extending the ground station code without changes to the
ArduPilot code.

Terrain Spacing
---------------

The ArduPilot terrain code has a user settable parameter called
TERRAIN_SPACING which controls the grid spacing which is used for
requests for terrain data from the aircraft to the ground station. The
default TERRAIN_SPACING is 100 meters, but users may set a different
grid spacing for specialist applications.

Note that the amount of terrain data kept in memory is directly related
to the grid spacing. If you decrease the ``TERRAIN_SPACING`` by a factor of
2 then the amount of terrain area kept in memory is reduced by a factor
of 4. It is recommended that you use a ``TERRAIN_SPACING`` of at least 30
meters to prevent the aircraft running off the side of a grid in flight
and not having data available.

If the ground station does not have terrain data available at the
resolution requested by the aircraft then the ground station will
interpolate as necessary to provide the requested grid size.

Terrain Accuracy
----------------

The accuracy of the SRTM database varies over the surface of the earth.
Typical accuracy is around 10 to 20 meters, although some areas are
worse. This makes terrain following suitable for aircraft that are
flying at altitudes of 60 meters or more. Using terrain data for low
flights is not recommended.

Setting up for terrain following
--------------------------------

To setup your fixed wing aircraft for terrain following follow these
steps

-  make sure you have Plane 3.0.4 or later loaded
-  make sure you have the latest MissionPlanner installed (version 1.3.9
   or later)
-  set TERRAIN_ENABLE to 1 and TERRAIN_FOLLOW to 1
-  connect to your vehicle over USB when you have GPS lock
-  check the FlightData->Status page in MissionPlanner and look for the
   terrain status data:

.. image:: ../../../images/MP-terrain.png
    :target: ../_images/MP-terrain.png

When the autopilot has finished loading terrain data you should see
"ter_pend" goes to zero and the current terrain altitude in meters
showing up in "ter_alt". The "ter_pend" value is the number of terrain
blocks that the autopilot is waiting to load from the ground station.

Terrain Lookahead
-----------------

The terrain following code "looks ahead" of the current position along
the flight path to try to ensure that the aircraft climbs soon enough to
avoid upcoming terrain. The amount of lookahead is controlled by the
``TERRAIN_LOOKAHD`` parameter, which defaults to 2000 meters. The lookahead
is also limited by the distance to the next waypoint in AUTO mode, so
you need to ensure that you don't have any legs of your mission which
include climb rates your aircraft cannot achieve.

The climb rate used in the terrain look ahead is based on the
``TECS_MAX_CLIMB`` parameter, combined with your current ground speed.


[copywiki destination="copter,plane"]