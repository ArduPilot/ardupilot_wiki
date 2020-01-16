.. _learning-a-mission:

==================
Learning a Mission
==================

Missions can be created by driving the vehicle around intermittently recording the vehicle's position as a waypoint by toggling the channel 7 switch.

.. tip::

   This is just one way to create a mission on Rover. Alternatively you can :ref:`plan a mission using Mission Planner <common-planning-a-mission-with-waypoints-and-events>` or some other GCS.

If using version 3.2 (or higher) this can be accomplished by doing the following:

- set the :ref:`Auxiliary Function Switch <common-auxiliary-functions>` to "Save Waypoint" by setting and RCx_OPTION for a channel to 1. (Prior to version 4.0, CH7_OPTION was used)
- wait for a good position estimate (i.e. LED will turn green)
- drive the vehicle around in any mode except Auto.
- when the vehicle is at a position that you would like to record as a waypoint, toggle the auxiliary function switch high (and then return to low)
- after recording all points, check the mission by connecting with a ground station and then download all waypoints.  If using the Mission Planner go to the Flight Plan screen and press the "Read WPs" button.

To then drive the mission, switch the vehicle into :ref:`Auto <auto-mode>`.

Deprecated instructions for Ver 3.1 (and lower)
-----------------------------------------------
   
If using version 3.1 (or lower):

- setup :ref:`Learning <learning-mode>` as a mode on the  :ref:`RC Transmitter Mode Swithc <common-rc-transmitter-flight-mode-configuration>`
- wait for a good position estimate (LED will become green)
- drive the vehicle around in :ref:`Learning <learning-mode>` mode
- when the vehicle is at a position that you would like to record as a waypoint, toggle the auxiliary function switch high (and then return to low)
- after recording all points, check the mission by connecting with a ground station and then download all waypoints.  If using the Mission Planner go to the Flight Plan screen and press the "Read WPs" button.

To then drive the mission, switch the vehicle into ref:`Auto <auto-mode>`.

.. note::

   In Rover-3.1 (and earlier) you can erase the mission stored in the rover by toggling the CH7 switch while in :ref:`Manual <manual-mode>` mode.
