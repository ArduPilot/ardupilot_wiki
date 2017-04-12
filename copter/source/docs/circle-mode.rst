.. _circle-mode:

===========
Circle Mode
===========

Circle will orbit a point located CIRCLE_RADIUS centimeters in front
of the vehicle with the nose of the vehicle pointed at the center.

.. note::

   The units are **centimeters** from AC 3.2 (previously metres).
   Mission Planner reports the units as cm for all versions of the
   code.

Setting the CIRCLE_RADIUS to zero will cause the copter to simply stay
in place and slowly rotate (useful for panorama shots).

The speed of the vehicle (in deg/second) can be modified by changing the
CIRCLE_RATE parameter.  A positive value means rotate clockwise, a
negative means counter clockwise.  The vehicle may not achieve the
desired rate if this requires the acceleration towards the center of the
circle to surpass the maximum acceleration held in the WPNAV_ACCEL
parameter (units are cm/s/s).

The pilot does not have any control over the roll and pitch but can
change the altitude with the throttle stick as in AltHold or Loiter
mode.

The pilot can control the yaw of the copter, the autopilot will not
retake control of the yaw until circle mode is re-engaged.

The mission command LOITER_TURNS invokes Circle mode during a mission.
