.. _common-polygon_fence:

[copywiki destination="copter,rover"]
================
Polygon GeoFence
================

Overview
========

Copter and Rover includes support for a polygon geofence with up to 70 points.  The purpose of this fence is to attempt to stop your vehicle from flying out of the polygon by initiating an RTL or, if flying in Loiter mode, the vehicle will normally stop before breaching the fence.  This feature is an extension of the simpler :ref:`cylindrical fence <common-ac2_simple_geofence>`.


..  youtube:: U3Z8bO3KbyM
    :width: 100%

Enabling the Fence in Mission Planner
=====================================

..  youtube:: SEm4nVfbg00
    :width: 100%

-  Connect your Autopilot to the Mission Planner
-  Go to the **CONFIG\|GeoFence** screen
-  Click "Enable" and set Type to "All" or another option that includes "Polygon"

   .. image:: ../../../images/polygon_fence_enable.png
       :target: ../_images/polygon_fence_enable.png
       :width: 500px

-  Select the Action . "RTL or Land" is recommended.
-  Go to the **PLAN** screen and select FENCE from the upper-rigth drop-down box.
-  Right-mouse-button click on the map and select "Draw Polygon" >> "Add Polygon Point"
-  Click on other points on the map to define the polygon

   .. image:: ../../../images/polygon_add_point.png
       :target: ../_images/polygon_add_point.png
       :width: 500px

-  After the polygon has been defined you must Right-mouse-button click and "Draw Polygon" >> "Set Return Location".  This location is not actually used by Copter or Rover, but it must be set because the same underlying library is used as Plane.

   .. image:: ../../../images/polygon_add_return_point.png
       :target: ../_images/polygon_add_return_point.png
       :width: 500px

-  Upload the polygon geofence to the vehicle using WRITE button on the bottom.

   .. image:: ../../../images/polygon_upload.png
       :target: ../_images/polygon_upload.png
       :width: 500px

Combining with the Cylindrical Fence
====================================

The polygon fence can be used in combination with the :ref:`cylindrical fences <common-ac2_simple_geofence>` and the :ref:`FENCE_ACTION<FENCE_ACTION>` (i.e. LAND, RTL,etc.) will trigger at whichever barrier the vehicle reaches first (i.e. the green line shown below)

.. image:: ../../../images/copter_polygon_circular_fence..png
    :target: ../_images/copter_polygon_circular_fence..png

Please see the :ref:`Cylindrical Fence <common-ac2_simple_geofence>` page for additional warnings and instructions including how to enable/disable the fence with the RC channel auxiliary switches.


