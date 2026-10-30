.. _circle-mode:

===========
Circle Mode
===========

Circle will orbit a point located :ref:`CIRCLE_RADIUS_M<CIRCLE_RADIUS_M>` meters in front
of the vehicle with the nose of the vehicle pointed at the center.

.. note::

   :ref:`CIRCLE_RADIUS_M<CIRCLE_RADIUS_M>` is in **meters** (default 10, range 0 to 2000). In firmware 4.6 and earlier this parameter was ``CIRCLE_RADIUS`` and was in centimeters; existing values are converted automatically on upgrade. See :ref:`common-param-name-changes`.

Setting the :ref:`CIRCLE_RADIUS_M<CIRCLE_RADIUS_M>` to zero will cause the copter to simply stay
in place and slowly rotate (useful for panorama shots).

Changes to :ref:`CIRCLE_RADIUS_M<CIRCLE_RADIUS_M>` made while flying in Circle mode take effect immediately and replace any radius adjustment made with the sticks.

The speed of the vehicle (in deg/second) can be modified by changing the
:ref:`CIRCLE_RATE<CIRCLE_RATE>` parameter.  A positive value means rotate clockwise, a
negative means counter clockwise.  The vehicle may not achieve the
desired rate, since its horizontal speed around the circle is limited to the lower of :ref:`WP_SPD<WP_SPD>` and the speed at which the acceleration towards the center of the circle would exceed half of :ref:`WP_ACC<WP_ACC>` (units are m/s/s). Small radii therefore limit the achievable rate.

Climb and descent with the throttle stick are limited by :ref:`PILOT_SPD_UP<PILOT_SPD_UP>`, :ref:`PILOT_SPD_DN<PILOT_SPD_DN>` and :ref:`PILOT_ACC_Z<PILOT_ACC_Z>`, as in :ref:`altholdmode`.

The circle rate set above can be dynamically adjusted in flight by two methods. The first is the use of RC Channel 6 if the :ref:`TUNE<TUNE>` option is set to 39, with the minimum and maximum circle rates set by :ref:`TUNE_MIN<TUNE_MIN>` and :ref:`TUNE_MAX<TUNE_MAX>`. The other is by enabling bit 0 of the :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>` bitmask parameter (enabled by default) to allow stick adjustment of radius and speed.

Circle Control Option
=====================

The :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>` bitmask parameter controls what the pilot can adjust with the sticks and how Circle mode operates. Its default value is 1 (bit 0 set).

When bit 0 of the :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>` parameter is set, the pilot can adjust the circle's radius and angular velocity with the control sticks:

- Pitch stick up (reducing RC pwm) reduces the radius. Think moving forward from an FPV perspective. At full stick deflection the radius changes at :ref:`WP_SPD<WP_SPD>` m/s.
- Pitch stick down (increasing RC pwm) increases the radius. Think moving back from an FPV perspective.
- Roll stick right (think clockwise) will increase the speed while moving clockwise, or decrease the speed while moving counterclockwise until reaching zero, at which point it will stop.
- Roll stick left (think counterclockwise) will increase the speed while moving counterclockwise, or decrease the speed while moving clockwise until reaching zero, at which point it will stop. Once stopped (rate 0), releasing the roll stick and pushing it again in either direction will begin moving again in the desired direction. So yes, this allows you to completely change the direction on the fly.
- Roll stick rate changes are inhibited when CH6 tuning knob is set for circle rate.
- All stick changes are inhibited in radio failsafe.(ie if loiter turns was part of a mission that continues when in failsafe)
- The above does not actually change the stored :ref:`CIRCLE_RATE<CIRCLE_RATE>` or :ref:`CIRCLE_RADIUS_M<CIRCLE_RADIUS_M>` parameter. Upon rebooting, any stick changes to rate and radius are gone and it will use the parameter values. So users should not have any surprises with every new flight.

When bit 1 is set of the :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>` parameter the Copter will face the direction of travel as it circles, otherwise, the Copter will point its nose at the center of the circle as it orbits.
When bit 2 is set of the :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>` parameter the circle's center position will set upon mode entry at the current location, rather than on the perimeter with the center in front of the Copter at the start.
When bit 3 is set of the :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>` parameter the mount's (if used) ROI aka region of interest will be set on the circle center causing mount to face the circle's center all the times. 

Other Notes
===========

- When :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>` bit 0 is cleared (it is set by default), the pilot does not have any control over the roll and pitch but can change the altitude with the throttle stick as in :ref:`altholdmode` or :ref:`loiter-mode`.

- The pilot can control the yaw of the copter, but the autopilot will not retake control of the yaw until circle mode is re-engaged.

- If the Copter cannot maintain a track close to the desired circle it will automatically decrease speed until it can maintain the desired track.

- The mission command ``LOITER_TURNS`` flies an orbit during a mission using the radius given in the command. In firmware 4.7 and earlier the orbit is flown at the :ref:`CIRCLE_RATE<CIRCLE_RATE>` rate. In firmware 4.8 and later an orbit with a non-zero radius is flown at the :ref:`WP_SPD<WP_SPD>` and :ref:`WP_ACC<WP_ACC>` limits, and :ref:`CIRCLE_RATE<CIRCLE_RATE>` is only used for a zero-radius command (rotate in place).
