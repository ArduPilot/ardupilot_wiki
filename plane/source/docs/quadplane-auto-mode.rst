.. _quadplane-auto-mode:

QuadPlane AUTO Missions
=======================

You can also ask the QuadPlane code to fly :ref:`AUTO <auto-mode>`
missions. To do that you plan an :ref:`AUTO <auto-mode>` mission as usual
and send a DO_VTOL_TRANSITION with parameter 1 equal to 3 to ask the
aircraft to switch to VTOL mode while flying the mission. When you do
that the fixed wing motor will stop and the aircraft will continue the
mission as a quadcopter. You can then send a DO_VTOL_TRANSITION with
parameter 1 equal to 4 to switch back to fixed wing flight.

The smooth transition rules apply to transitions in :ref:`AUTO <auto-mode>`
mode as they do for other modes, plus quad assistance applies in auto
fixed-wing mode if :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` is enabled.

In addition to DO_VTOL_TRANSITION the QuadPlane code supports two new
mission commands:

-  NAV_VTOL_TAKEOFF
-  NAV_VTOL_LAND

These mission commands can be used as part of a full auto mission to
give a vertical takeoff, followed by smooth transition to auto fixed
wing flight and then a vertical landing.
