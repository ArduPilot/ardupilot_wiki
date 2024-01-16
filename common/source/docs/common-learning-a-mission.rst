.. _common-learning-a-mission:

=================================
Recording Waypoints for a Mission
=================================

Missions can be created by flying (Copter) or driving (Rover) the vehicle around, intermittently recording the vehicle's position as a waypoint by toggling an RC transmitter switch which toggles an RC channel that has been setup as having the :ref:`Auxiliary Function <common-auxiliary-functions>` of "Save Waypoint" (function 7).

.. tip::

   This is just one way to create a mission. Alternatively you can :ref:`plan a mission using Mission Planner <common-planning-a-mission-with-waypoints-and-events>` or some other GCS.

Setup
=====

- set the :ref:`Auxiliary Function Switch <common-auxiliary-functions>` to "Save Waypoint" by setting and RCx_OPTION for a channel to 7. (Prior to version 4.0, CH7_OPTION was used)
- wait for a good position estimate (i.e. LED will turn green)
- arm and drive/fly the vehicle around in any mode except Auto.
- when the vehicle is at a position that you would like to record as a waypoint, toggle the auxiliary function switch high (and then return to low). That position and alt.itude will be added as a waypoint to the end of any existing mission list (including an initially blank list)
- after recording all points, and adding any additional items like a takeoff or landing, check the mission by connecting with a ground station and then download all waypoints to save if needed in the future.  If using the Mission Planner go to the Flight Plan screen and press the "Read WPs" button.

To then execute the mission, switch the vehicle into :ref:`Auto <auto-mode>`.

[copywiki destination="rover,copter"]