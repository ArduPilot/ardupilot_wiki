.. _common-object-avoidance-landing-page:

================
Object Avoidance
================

Ardupilot supports several kinds of object avoidance. Avoidance of Airborne Vehicles (ADSB) and Object Avoidance (Object/Ground/Ceiling).

Supported types vary with vehicle (Plane only supports ADSB). Some kinds of avoidance require external hardware, such as ADSB receivers or  Rangefinders.

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
    Bendy Path Planning Around Obstacles <common-oa-bendyruler>
    Dijkstras Path Planning Around Obstacles<common-oa-dijkstras>
[/site]




Hardware
========

-  :ref:`ADS-B Receiver <common-ads-b-receiver>`
-  :ref:`Rangefinders <common-rangefinder-landingpage>`
-  :ref:`Realsense Depth Camera <common-realsense-depth-camera>`




