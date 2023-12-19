.. _cruise-mode:

===========
CRUISE Mode
===========

Cruise mode is a bit like :ref:`FLY BY WIRE_B (FBWB) <fbwb-mode>`, but it
has "heading lock". It is the ideal mode for longer distance FPV flight,
as you can point the plane at a distant object and it will accurately
track to that object, automatically controlling altitude, airspeed and
heading.

.. warning::

   CRUISE mode is not suitable for flight close to the ground. The
   time constants for height control in CRUISE mode are the ones from
   TECS, which are quite slow. When you are close to the ground CRUISE
   mode does not give you the agility needed in pitch control to avoid
   hitting the ground.

The way it works is this:

-  if you have any aileron or rudder input then it flies just like
   :ref:`FBWB <fbwb-mode>`. So it holds altitude until you use the elevator
   to change the target altitude (at the ``FBWB_CLIMB_RATE`` rate) and
   it adjusts airspeed based on throttle
-  when you let go of the aileron and rudder sticks for more than 0.5
   seconds it sets an internal waypoint at your current location, and
   projects a target waypoint one kilometre ahead (note that heading
   lock will only activate if you have GPS lock, and have a ground speed
   of at least 3 m/s)
-  as it flies along it heads for the target waypoint, and constantly
   updates that target to always be one kilometre ahead, leaving the
   previous waypoint as the position that you centred the aileron and
   rudder sticks
-  as long as you don't touch the aileron or rudder, it will run the
   same navigation system it uses for waypoints, including crabbing,
   cross-track etc, so it will very accurately hold that ground course
   even in the face of changing wind conditions

One of the nicer aspects of CRUISE mode is how it handles rudder. If you
give it some rudder then the roll controller will keep the wings level,
but the plane will yaw with the rudder. So you get a "wings level" turn,
allowing you to rotate your flight to point at whatever geographic
feature you want to head towards. Then when you let go of the rudder it
will head straight for that point.

Note that you can configure CRUISE mode to do terrain following on
autopilots with microSD storage available. See the
:ref:`terrain following documentation <common-terrain-following>`.

.. warning::

   Make sure you only fly FPV if it is allowed by your country's
   flight and airspace control rules. Many countries do not allow
   non-line-of-sight flight without a special operating license.

Advanced Configuration
======================

- :ref:`fly-by-wire-low-altitude-limit`
- :ref:`Min/Max Altitude Fences <common-geofencing-landing-page>`
