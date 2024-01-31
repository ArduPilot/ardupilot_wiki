.. _auto-mode:

=========
AUTO Mode
=========

In AUTO mode Plane will follow a mission (a set of GPS waypoints and other
commands) set by your ground station. When re-entering AUTO
mode Plane will continue from whatever mission item it was last doing,
unless you have reset the mission. If the mission ends with an item that does not continue indefinitely (like LOITER UNLIMITED), an RTL will be executed.

When in AUTO Plane will by default allow the pilot to influence the
flight of the plane by using "stick mixing", which allows for aileron,
elevator and rudder input to steer the plane in a way that can override
the autopilot control. Whether this is enabled is determined by the
:ref:`STICK_MIXING <STICK_MIXING>`
option. By default stick mixing behaves the same as :ref:`FLY BY WIRE_A (FBWA) <fbwa-mode>` mode.

The speed during the mission is nominally at :ref:`AIRSPEED_CRUISE<AIRSPEED_CRUISE>` when using an airspeed sensor, or at whatever speed results from :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` without an airspeed sensor. Setting :ref:`THROTTLE_NUDGE<THROTTLE_NUDGE>` = 1 allows the speed to be increased if the throttle stick is above mid-stick up to :ref:`AIRSPEED_MAX<AIRSPEED_MAX>` or :ref:`THR_MAX<THR_MAX>`, when using or not using an airspeed sensor, respectively.

.. warning::

   "Home" position is always supposed to be your Plane's actual
   GPS takeoff location:

   #. It is very important to acquire GPS lock before arming in order for
      RTL, Loiter, Auto or any GPS dependent mode to work properly.
   #. For Plane the home position is initially established at the time the
      plane acquires its GPS lock. It is then continuously updated as long as
      the autopilot is disarmed.

      - This means if you execute an RTL in Plane, it will return to the
	location where it was when it was armed - assuming it had
	acquired GPS lock.
      - Consider the use of :ref:`Rally Points <common-rally-points>` to
	avoid returning directly to your arming point on RTL

MISSION INTERRUPTION
====================

Changing out of AUTO Mode leaves whatever mission item being executed in a "suspended" state. Re-entry into AUTO mode later will either resume execution of the mission where it was left or restart the mission depending on the value of the :ref:`MIS_RESTART<MIS_RESTART>` parameter. By default, it will resume. If you were on the way to Waypoint 3 when changed modes out of AUTO, if re-entered, it will immediately head for Waypoint 3 and continue the mission.

You can reset the mission back to the beginning using either the ``RCx_OPTION`` switch "24", or via a MAVLink command. Mission Planner has a button in its "Actions" tab of the DATA screen to "Restart Mission". If :ref:`MIS_RESTART<MIS_RESTART>` is set to "1", the the mission will be reset to the start every time AUTO mode is entered.
