.. _common-oa-dijkstras:

================================
Object Avoidance with Dijkstra's
================================

..  youtube:: GAmNaDTzy3Q
    :width: 100%

Copter and Rover support `Dijkstra's <https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm>`__ for path planning around fences and stay-out zones in Auto, Guided and RTL modes.  This well known algorithm internally builds up a list of "safe areas" calculated from the fence and stay-out zones and then finds the shortest path to the destination.

.. image:: ../../../images/oa-dijkstras.png

.. warning::

   Dijkstra's does not support avoiding objects sensed with lidar or proximity sensors

.. warning::

   Dijkstra's does not support Spline Waypoints. 

Configuration
-------------

-  :ref:`OA_TYPE <OA_TYPE>` = 2 (Dijkstra).  You may need to refresh parameters after changing this to see the parameters below.
-  :ref:`OA_MARGIN_MAX <OA_MARGIN_MAX>`: the distance (in meters) that the vehicle should stay away from the fences and stay-out zones
-  :ref:`OA_OPTIONS<OA_OPTIONS>` bit 2 (+4 to the value) can be set to use S-Curves around fence corners in the planned path to speed up turns. Note that using S-Curves, instead of the normal "approach,stop, turn, proceed" method of path planning around sharp fence corners, could still result in a fence breach.To avoid this :ref:`WPNAV_RADIUS<WPNAV_RADIUS>` should be set smaller than :ref:`FENCE_MARGIN<FENCE_MARGIN>`. Also waypoints should also be placed at least 10m from fence boundaries.
[site wiki="copter"]
- For use in :ref:`Guided mode <ac2_guidedmode>`, set :ref:`GUID_OPTIONS<GUID_OPTIONS>` bit 6 (e.g. 64)
[/site]

Videos
------

..  youtube:: JKxQGt6XW60
    :width: 100%

[copywiki destination="copter,rover"]
