.. _common-oa-bendyruler:

=================================
Object Avoidance with Bendy Ruler
=================================

..  youtube:: E0KYLpXnD1A
    :width: 100%

Copter and Rover 4.0 (and higher) support "BendyRuler" for path planning around obstacles and fences.  The BendyRuler algorithm probes around the vehicle in many directions looking for open spaces and then tries to pick the direction that is sufficiently open while also moving the vehicle towards the final destination.

.. note::

    This is only applicable in AUTO, GUIDED, and RTL flight modes.

.. image:: ../../../images/oa-bendy-ruler.png
    :width: 450px

Basic Configuration
-------------------

-  :ref:`OA_TYPE <OA_TYPE>` = 1 (BendyRuler).  You may need to refresh parameters after changing this to see the parameters below.
-  :ref:`OA_BR_LOOKAHEAD <OA_BR_LOOKAHEAD>`: This parameter is called "OA_LOOKAHEAD" before Copter and Rover 4.1. It is the distance (in meters) ahead of the vehicle that should be probed.  Obstacles further than this far away will be ignored.  This should be long enough that the path around obstacles can be "seen" but not too long or the vehicle will be overly cautious and not enter areas with a lot of obstacles. 5m is typical.
-  :ref:`OA_MARGIN_MAX <OA_MARGIN_MAX>`: the distance (in meters) that the vehicle should stay away from obstacles. 2m is a typical value.

.. note::

    Users often get confused between OA_BR_LOOKAHEAD and OA_MARGIN_MAX parameters. A smaller OA_BR_LOOKAHEAD will make BendyRuler look for obstacles in a shorter path towards the destination, and thus any object further than this many meters will not be used in the path planning process. A shorter OA_MARGIN_MAX parameter will let the vehicle come closer to an approaching obstacle.

BendyRuler Types
-----------------

Copter 4.1 onwards has support for two different types of BendyRuler Path Planning.

- 1. :ref:`OA_BR_TYPE <OA_BR_TYPE>` = 1: Horizontal BendyRuler: This is the only option available on Rover and older Copter versions. This searches for obstacle-free paths only in horizontal directions and therefore when it detects an obstacle in its path, it will only move in those directions.

..  youtube:: GRqjMf7kQxU
    :width: 100%



- 2. :ref:`OA_BR_TYPE <OA_BR_TYPE>` = 2: Vertical BendyRuler: This feature searches for path only in four directions: Straight towards the destination, vertically up, vertically down and backwards. This feature is typically helpful while avoiding short obstacles in a crowded space. Don't forget to setup an altitude fence if you do not want the Copter to climb above a certain height.

..  youtube:: cjv0ArVOCy0
    :width: 100%


Advanced Configuration
----------------------

If using a lidar or proximity sensor the following "obstacle database" parameters are available:

- :ref:`OA_DB_SIZE <OA_DB_SIZE>`: the maximum number of obstacles that can be tracked
- :ref:`OA_DB_EXPIRE <OA_DB_EXPIRE>`: the number of seconds after an obstacle disappears from view that it is removed from the database
- :ref:`OA_DB_QUEUE_SIZE <OA_DB_QUEUE_SIZE>`: the buffer size between the lidar and obstacle database.  Normally this can be left at the default value
- :ref:`OA_DB_OUTPUT <OA_DB_OUTPUT>`: controls whether tracked objects are visible on the GCS as small airplanes
- :ref:`OA_DB_ALT_MIN <OA_DB_ALT_MIN>`: OADatabase will reject obstacle's if vehicle's altitude above home is below this parameter, in a 3 meter radius around home. This can be useful if your sensor is picking up the ground as obstacles while taking off.


Videos
------

..  youtube:: SPu0a23FGKc
    :width: 100%

[copywiki destination="copter,rover"]
