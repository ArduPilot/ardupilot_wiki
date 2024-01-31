.. _fbwb-mode:

=========================
FBWB Mode (FLY BY WIRE B)
=========================

The FBWB mode is similar to :ref:`FLY BY WIRE A (FBWA) <fbwa-mode>`, but
Plane will try to hold altitude as well. Roll control is the same as
FBWA, and altitude is controlled using the elevator. The target airspeed
is controlled using the throttle.

To control your altitude in FBWB mode you use the elevator stick to ask for a
change in altitude. If you leave the elevator centered then Plane will
try to hold the current altitude. As you move the elevator stick, Plane will
try to gain or lose altitude at a rate in proportion to how far you move the
elevator. How much altitude it tries to gain for full elevator
deflection depends on the :ref:`FBWB_CLIMB_RATE<FBWB_CLIMB_RATE>` parameter, which defaults
to 2 meters/second. Note that 2 m/s is quite a slow change, so many
users will want to raise :ref:`FBWB_CLIMB_RATE<FBWB_CLIMB_RATE>` to a higher value to make
the altitude change more responsive.

Whether you need to pull back on the elevator stick or push forward to
climb depends on the setting of the :ref:`FBWB_ELEV_REV<FBWB_ELEV_REV>` parameter. The
default is for pulling back on the elevator to cause the plane to climb.
This corresponds to the normal response direction for a RC model. If you
are more comfortable with the reverse you can set :ref:`FBWB_ELEV_REV<FBWB_ELEV_REV>` to 1
and the elevator will be reversed in FBWB mode.

Note that the elevator stick does not directly control pitch, it controls target
climb or descent rate. The amount of pitch that will be used to achieve this
rate depends on your TECS tuning settings, but in
general the autopilot primarily climb or descend by raising or lowering the pitch AND throttle.
This can be disconcerting for people used to flying in FBWA mode, where
you have much more direct control over pitch.

If you have an airspeed sensor then the throttle will control the target
airspeed in the range :ref:`AIRSPEED_MIN<AIRSPEED_MIN>` to :ref:`AIRSPEED_MAX<AIRSPEED_MAX>`. If
throttle is minimum then the plane will try to fly at :ref:`AIRSPEED_MIN<AIRSPEED_MIN>`.
If it is maximum it will try to fly at :ref:`AIRSPEED_MAX<AIRSPEED_MAX>`.

If you don't have an airspeed sensor then the throttle input will set the
target throttle of the plane, which is :ref:`TRIM_THROTTLE<TRIM_THROTTLE>`, at throttle midstick and lower positions, and Plane will adjust the throttle around
that value to achieve the desired altitude hold while trying to maintain the pitch generally at its calibrated LEVEL setting, although the autopilot will primarily use pitch for climbing and descending, and manage throttle to maintain airspeed. 

Increasing the throttle stick above mid-stick
can be used to push the target throttle up beyond what it calculates is
needed, to fly faster.

As with FBWA, the rudder is under a combination of manual control and
auto control for turn coordination.

You should also have a look at CRUISE mode, as it is generally better
than FBWB, especially if there is significant wind. In CRUISE mode the
aircraft will hold a ground track as opposed to just leveling the wings
when you don't input any roll with the aileron stick.

Advanced Configuration
======================

- :ref:`fly-by-wire-low-altitude-limit`
- :ref:`Min/Max Altitude Fences <common-geofencing-landing-page>`