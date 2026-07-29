.. _common-object-avoidance-landing-page:

================
Object Avoidance
================

ArduPilot supports several kinds of object avoidance. Avoidance of Airborne Vehicles (ADSB), of manned boats (AIS) and Object Avoidance (Object/Ground/Ceiling).

Supported types vary with vehicle (Plane only supports ADSB). Some kinds of avoidance require external hardware, such as ADSB receivers, AIS receivers or  Rangefinders.

Avoidance Strategies
====================

Various strategies are employed and vary depending on vehicle, mode, and/or object to be avoided. Re-routing, slide, stop, or failsafe style actions are the most common ones. See the particular Avoidance Feature below for details.

[site wiki="copter,plane"]

ADSB Avoidance
===============

- :ref:`Airborne Vehicles (ADSB)<common-ads-b-receiver>`

[/site]

[site wiki="rover"]

AIS Avoidance
=============

A boat fitted with an AIS receiver feeds the positions of nearby manned vessels into the
object avoidance database, so the path planners listed below route around them in the same
way as any other obstacle. Note that AIS messages arrive infrequently, so a much longer
:ref:`OA_DB_EXPIRE<OA_DB_EXPIRE>` is required than for other obstacle sources.

- :ref:`Manned Boats (AIS)<common-ais>`

[/site]

[site wiki="copter,rover"]
Path Planning and Obstacle Avoidance Features
=============================================

These methods are used for avoiding proximity sensor detected obstacles as well as GCS set fences.

Hardware Setup
--------------

-  :ref:`Proximity Sensors<common-proximity-landingpage>`
-  :ref:`Rangefinders <common-rangefinder-landingpage>`
-  :ref:`Realsense Depth Camera <common-realsense-depth-camera>`


Avoidance Types
---------------

.. toctree::
    :maxdepth: 1

    Simple Object Avoidance <common-simple-object-avoidance>
    BendyRuler Path Planning Around Obstacles and Fences <common-oa-bendyruler>
    Dijkstra's Path Planning Around Fences<common-oa-dijkstras>
    Dijkstra's with BendyRuler Path Planning Around Obstacles and Fences<common-oa-dijkstrabendyruler>
[/site]

[copywiki destination="plane,copter,rover"]