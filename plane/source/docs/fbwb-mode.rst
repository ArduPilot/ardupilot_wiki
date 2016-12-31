.. _fbwb-mode:

=========================
FBWB Mode (FLY BY WIRE_B)
=========================

The FBWB mode is similar to :ref:`FLY BY WIRE_A (FBWA) <fbwa-mode>`, but
Plane will try to hold altitude as well. Roll control is the same as
FBWA, and altitude is controlled using the elevator. The target airspeed
is controlled using the throttle.

To control your altitude in FBWB mode you use the elevator to ask for a
change in altitude. If you leave the elevator centred then Plane will
try to hold the current altitude. As you move the elevator Plane will
try to gain or lose altitude in proportion to how far you move the
elevator. How much altitude it tries to gain for full elevator
deflection depends on the ``FBWB_CLIMB_RATE`` parameter, which defaults
to 2 meters/second. Note that 2 m/s is quite a slow change, so many
users will want to raise ``FBWB_CLIMB_RATE`` to a higher value to make
the altitude change more responsive.

Whether you need to pull back on the elevator stick or push forward to
climb depends on the setting of the ``FBWB_ELEV_REV`` parameter. The
default is for pulling back on the elevator to cause the plane to climb.
This corresponds to the normal response direction for a RC model. If you
are more comfortable with the reverse you can set FBWB_ELEV_REV to 1
and the elevator will be reversed in FBWB mode.

Note that the elevator stick does not control pitch, it controls target
altitude. The amount of pitch that will be used to achieve the requested
climb or descent rate depends on your TECS tuning settings, but in
general the autopilot will try to hold the plane fairly level in pitch,
and will primarily climb or descend by raising or lowering the throttle.
This can be disconcerting for people used to flying in FBWA mode, where
you have much more direct control over pitch.

If you have an airspeed sensor then the throttle will control the target
airspeed in the range ``ARSPD_FBW_MIN`` to ``ARSPD_FBW_MAX``. If
throttle is minimum then the plane will try to fly at ``ARSPD_FBW_MIN``.
If it is maximum it will try to fly at ``ARSPD_FBW_MAX``.

If you don't have an airspeed sensor then the throttle will set the
target throttle of the plane, and Plane will adjust the throttle around
that setting to achieve the desired altitude hold. The throttle stick
can be used to push the target throttle up beyond what it calculates is
needed, to fly faster.

As with FBWA, the rudder is under a combination of manual control and
auto control for turn coordination.

You should also have a look at CRUISE mode, as it is generally better
than FBWB, especially if there is significant wind. In CRUISE mode the
aircraft will hold a ground track as opposed to just levelling the wings
when you don't input any roll with the aileron stick.
