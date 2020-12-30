.. _auto-mode:

=========
AUTO Mode
=========

In AUTO mode Plane will follow a mission (a set of GPS waypoints and other
commands) set by your ground station. When entering AUTO
mode Plane will continue from whatever mission item it was last doing,
unless you have reset the mission.

When in AUTO Plane will by default allow the pilot to influence the
flight of the plane by using "stick mixing", which allows for aileron,
elevator and rudder input to steer the plane in a way that can override
the autopilot control. Whether this is enabled is determined by the
:ref:`STICK_MIXING <STICK_MIXING>`
option. By default stick mixing behaves the same as :ref:`FLY BY WIRE_A (FBWA) <fbwa-mode>` mode.

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
