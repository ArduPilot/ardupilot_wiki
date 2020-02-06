.. _circle-mode:

===========
Circle Mode
===========

Circle will orbit a point located :ref:`CIRCLE_RADIUS<CIRCLE_RADIUS>` centimeters in front
of the vehicle with the nose of the vehicle pointed at the center.

.. note::

   The units are **centimeters** from AC 3.2 (previously metres).
   Mission Planner reports the units as cm for all versions of the
   code.

Setting the :ref:`CIRCLE_RADIUS<CIRCLE_RADIUS>` to zero will cause the copter to simply stay
in place and slowly rotate (useful for panorama shots).

The speed of the vehicle (in deg/second) can be modified by changing the
:ref:`CIRCLE_RATE<CIRCLE_RATE>` parameter.  A positive value means rotate clockwise, a
negative means counter clockwise.  The vehicle may not achieve the
desired rate if this requires the acceleration towards the center of the
circle to surpass the maximum acceleration held in the :ref:`WPNAV_ACCEL<WPNAV_ACCEL>`
parameter (units are cm/s/s).

The circle rate set above can be dynamically adjusted in flight by two methods. The first is the use of RC Channel 6 if the :ref:`TUNE<TUNE>` option is set to 39, allowing decreasing the rate 50% or increasing it by 100%, at the channel min and max. The other is by enabling the :ref:`CIRCLE_CONTROL<CIRCLE_CONTROL>` parameter to allow stick adjustment of radius and speed.

Circle Control Option
=====================

When enabled, the :ref:`CIRCLE_CONTROL<CIRCLE_CONTROL>` parameter allows the adjustment of the circle's radius and angular velocity.

- Pitch stick up (reducing RC pwm) reduces the radius until it reaches zero. Think moving forward from an FPV perspective.
- Pitch stick down (increasing RC pwm) increases the radius. Think moving back from an FPV perspective.
- Roll stick right (think clockwise) will increase the speed while moving clockwise, or decrease the speed while moving counterclockwise until reaching zero, at which point it will stop.
- Roll stick left (think counterclockwise) will increase the speed while moving counterclockwise, or decrease the speed while moving clockwise until reaching zero, at which point it will stop. Once stopped (rate 0), releasing the roll stick and pushing it again in either direction will begin moving again in the desired direction. So yes, this allows you to completely change the direction on the fly.
- Roll stick rate changes are inhibited when CH6 tuning knob is set for circle rate.
- All stick changes are inhibited in radio failsafe.(ie if loiter turns was part of a mission that continues when in failsafe)
- Does not actually change the stored circle_rate or circle_radius parameter. Upon rebooting, any stick changes to rate and radius are gone and it will use the parameter values. So users should not have any surprises with every new flight.

Other Notes
===========

The pilot does not have any control over the roll and pitch but can
change the altitude with the throttle stick as in AltHold or Loiter
mode.

The pilot can control the yaw of the copter, but the autopilot will not
retake control of the yaw until circle mode is re-engaged.

The mission command ``LOITER_TURNS`` invokes Circle mode during a mission.
