.. _learning-a-mission:

==================
Learning a Mission
==================

:ref:`LEARNING <learning-mode>` mode allows you to *teach* the Rover a
mission by manually driving around recording the desired waypoints.

.. tip::

   This is the easiest way to create a mission on Rover. Alternatively
   you can :ref:`plan a mission using Mission Planner <common-planning-a-mission-with-waypoints-and-events>` or some
   other GCS.

To add waypoints in *Learning Mode*:

-  Make sure you have GPS lock (solid blue LED on APM).
-  Turn on *Learning mode* using the mode switch.
-  Drive around manually and flick the CH7 toggle when you want the
   autopilot to record a waypoint.

The waypoints will be added to the end of the current mission stored in
the rover. Once you're done, you can switch into :ref:`AUTO <auto-mode>`
mode and the Rover should retrace your steps, hitting all the waypoints
you recorded.

If you're connected to the *Mission Planner*, you can click **Read WPs**
on the *Flight Planner* screen and it will show the recorded waypoints
on a map.

.. note::

   You can erase the mission stored in the rover by toggling the CH7
   switch while in :ref:`Manual <manual-mode>` mode.
