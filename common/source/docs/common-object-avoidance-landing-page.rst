.. _common-object-avoidance-landing-page:

================
Object Avoidance
================

Ardupilot supports many kinds of object avoidance in vehicles. Airborne Vehicles (ADSB), Object Avoidance (Object/Ground/Ceiling), and Geo-Fences (Beacons, Exclusion zones, Fences).
Supported types vary with vehicle. Some kinds of avoidance require external hardware, such as ADSB receivers or  Rangefinders.

Avoidance Strategies
====================

Various strategies are employed and vary depending on vehicle, mode, and/or object to be avoided. Re-routing, slide, stop, or failsafe style actions are the most common ones. See the particular Avoidance Feature below for details.


Avoidance Features
==================

.. toctree::
    :maxdepth: 1

    Airborne Vehicles (ADSB) <common-ads-b-receiver>
[site wiki="copter,rover"]
    Simple Object Avoidance <common-simple-object-avoidance>
    Autonomous Mission Re-routing via Bendy Ruler <common-oa-bendyruler>
    Autonomous Mission Re-routing via Dijkstras <common-oa-dijkstras>
    Cylindrical Geo-Fencing Failsafe <common-ac2_simple_geofence>
    Polygon Geo-Fencing Failsafe <common-polygon_fence>
[/site]
[site wiki="plane"]
    Geo-Fencing Failsafes <geofencing>
[/site]

Hardware
========

-  :ref:`ADS-B Receiver <common-ads-b-receiver>`
[site wiki="copter,rover"]
-  :ref:`Rangefinders <common-rangefinder-landingpage>`
-  :ref:`Fence Beacons (Non GPS navigation) <common-non-gps-navigation-landing-page>` 
[/site]


