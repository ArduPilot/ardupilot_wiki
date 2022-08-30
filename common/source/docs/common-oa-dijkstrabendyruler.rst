.. _common-oa-dijkstrabendyruler:

=================================================
Object Avoidance using Dijkstra's with BendyRuler
=================================================

Copter and Rover 4.1 (and higher) support path planning with a fusion of Dijkstras and BendyRuler.
BendyRuler does not guarantee a shortest path, and can be called a local planner. Although Dijkstra's gives us the ability to navigate around complex fences with the shortest path, yet due to the computational complexity it cannot be used for to avoid Proximity based obstacles. Therefore, the advantages of both of these algorithms are used together.

.. note::

    This is only applicable in AUTO, GUIDED, and RTL flight modes.

This method starts with traditional Dijkstra's planning the shortest path around all the fences that are present between the flight path. In between this path, if any proximity based obstacle is detected, the navigation is switched to BendyRuler. If there are no obstacles in the nearby view of the sensor, the vehicle resumes normal Dijkstra's based navigation.

Basic Configuration
-------------------

:ref:`OA_TYPE <OA_TYPE>` = 3

Rest of the parameters can be set by looking at the respective documentation of :ref:`BendyRuler<common-oa-bendyruler>` and :ref:`Dijsktra's<common-oa-dijkstras>`


Videos
------

..  youtube:: s0z0b2U2fAk
    :width: 100%


[copywiki destination="copter,rover"]