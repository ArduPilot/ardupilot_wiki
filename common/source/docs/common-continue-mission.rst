.. _common-continue-mission:

==================================
Continuing a Mission After Landing
==================================

This video shows how to setup a mission in which a landing occurs, disarms, waits, re-arms and then re-takeoff and continue the mission.

.. youtube:: BK3OsNJoF8A



.. note:: in the above video, since the vehicle is disarmed at the first landing point and then re-armed, home is reset to that first landing point. Any subsequent RTL will be to that newly set "home" unless the mission uses a ``DO_SET_HOME`` mission command to change it.

[copywiki destination="plane,copter,rover,planner"]