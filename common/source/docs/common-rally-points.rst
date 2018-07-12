.. _common-rally-points:

============
Rally Points
============

Overview
========

Ordinarily when a plane or copter enters :ref:`Return to Launch (RTL) <plane:rtl-mode>` mode (typically
triggered by an
autopilot \ :ref:`failsafe <plane:apms-failsafe-function>`),
the default behaviour is to return to the \ :ref:`Home point <common-planning-a-mission-with-waypoints-and-events_setting_the_home_position>`,
but there are often cases when that can be undesirable. For example it
may be an area full of people or property and a system running in RTL
mode may very likely be in a state that merits extreme caution!  It is
also possible that the flight plan is large enough that should the
aircraft enter RTL mode it is undesirable to traverse all the way back
to the point of takeoff.

For this reason we now support the creation of multiple Rally Points.
Should an aircraft enter RTL and Rally Points have been defined then it
will proceed to the closest Rally Point, rather than proceeding to the
Home position. Plane will then loiter at that location, and Copter will
perform an automated landing there.

.. figure:: ../../../images/mp_flight_plan_with_three_rally_points.jpg
   :target: ../_images/mp_flight_plan_with_three_rally_points.jpg

   A flight plan with Rally Points. Rally Points are denoted with Purpleplace markers. 
   Mousing over a Rally Point will give its loiter altitude(as in the middle point above).

Setting Up Rally Points
=======================

The following steps are for specifying Rally Points in Mission Planner:

#. Rally Point latitude and longitude.To set a Rally Point's location,
   right click on the Flight Plan map, and select Rally Points > Set
   Rally Point on the resulting pop-up menu (note that this ONLY work in
   the Flight Plan screen, not the Flight Data screen):
   
   .. image:: ../../../images/mp_rally_point_dialog.jpg
       :target: ../_images/mp_rally_point_dialog.jpg
    
#. The Rally loiter altitudes need to be specified (note that the
   default altitude for Rally Points is the default waypoint altitude
   value):

   .. image:: ../../../images/mp_rally_altitude_dialog.jpg
       :target: ../_images/mp_rally_altitude_dialog.jpg
       
   .. image:: ../../../images/defaultAltCircled1.png
       :target: ../_images/defaultAltCircled1.png
    
#. Repeat for all desired Rally points!
#. Upload to the vehicle by selecting Rally Points > Upload from the
   right-click popup menu

The following should be considered when using Rally Points:

#. If using a :ref:`geofence <plane:geofencing>`:
   its HIGHLY recommended the Rally Points you intend to use at your event
   are inside the geofence.
#. Make sure Rally Point altitudes are high enough to clear terrain and
   buildings.
#. Because of the limited flash memory size on the APM2.x hardware the
   number of Rally Points is restricted to 10 on Plane and 6 on Copter
   -- this limit may be expanded on other platforms such as Pixhawk in the future.
#. On Plane, loiter radius for a Rally Point is the same as all other
   loiter points; determined by the :ref:`WP_LOITER_RAD <plane:WP_LOITER_RAD>`
   parameter.
#. The :ref:`ALT_HOLD_RTL <plane:ALT_HOLD_RTL>` 
   :ref:`RTL_ALT <copter:RTL_ALT>` parameters are NOT used with Rally Points! 
   The aircraft will transit to the Rally Point at the altitude 
   specified when adding that point.

The following MAVLink parameters control Rally Point behavior:

#. :ref:`RALLY_LIMIT_KM <plane:RALLY_LIMIT_KM>`
   is the maximum distance a Rally Point may be from the aircraft to be
   considered for an RTL event.  If all Rally Points are greater than
   this distance from the aircraft, then the Home location is used for
   RTL events (at altitude :ref:`ALT_HOLD_RTL <plane:ALT_HOLD_RTL>`)
   **unless** Home is farther away than the nearest Rally Point -- in
   which case the nearest Rally point is used.  This parameter is to
   prevent fly offs if Rally Points have been specified for multiple
   flying fields. This parameter can be disabled if set to 0.
#. :ref:`RALLY_TOTAL <plane:RALLY_TOTAL>` is
   the number of Rally Points currently specified. This parameter will
   be set for you by your ground control station (e.g., Mission Planner)
   when you add and remove Rally Points.  **IT IS HIGHLY UNLIKELY YOU
   WANT TO SET THIS PARAMETER MANUALLY AND IT IS PROBABLY UNSAFE TO DO
   SO**.  RALLY_TOTAL should be 0 if you have specified no Rally Points
   and in this case the Home location will be used for RTL events.

Example Flight
==============

.. figure:: ../../../images/mp_rally_demo.jpg
   :target: ../_images/mp_rally_demo.jpg

   Flight in which RTL was commanded nearwaypoint. Plane began loitering about the southernmost RallyPoint.

   
.. figure:: ../../../images/mp_rally_point_dialog.jpg
   :target: ../_images/mp_rally_point_dialog.jpg


[copywiki destination="copter,plane,rover,planner"]