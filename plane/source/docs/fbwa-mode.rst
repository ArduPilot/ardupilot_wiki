.. _fbwa-mode:

=========================
FBWA Mode (FLY BY WIRE_A)
=========================

This is the most popular mode for assisted flying in Plane, and is the
best mode for inexperienced flyers. In this mode Plane will hold the
roll and pitch specified by the control sticks. So if you hold the
aileron stick hard right then the plane will hold its pitch level and
will bank right by the angle specified in the :ref:`ROLL_LIMIT_DEG<ROLL_LIMIT_DEG>` parameter (in
degrees). It is not possible to roll the plane past the roll limit
specified in :ref:`ROLL_LIMIT_DEG<ROLL_LIMIT_DEG>`, and it is not possible to pitch the plane
beyond the :ref:`PTCH_LIM_MIN_DEG<PTCH_LIM_MIN_DEG>` or :ref:`PTCH_LIM_MAX_DEG<PTCH_LIM_MAX_DEG>` settings.

Note that holding level pitch does not mean the plane will hold
altitude. How much altitude a plane gains or loses at a particular pitch
depends on its airspeed, which is primarily controlled by throttle. So
to gain altitude you should raise the throttle, and to lose altitude you
should lower the throttle. If you want Plane to take care of holding
altitude then you should look at the FlyByWireB mode.

In FBWA mode throttle is manually controlled, but is constrained by the
:ref:`THR_MIN<THR_MIN>` and :ref:`THR_MAX<THR_MAX>` settings.

In FBWA mode the rudder is under both manual control, plus whatever
rudder mixing for roll you have configured. Thus you can use the rudder
for ground steering, and still have it used for automatically
coordinating turns.
