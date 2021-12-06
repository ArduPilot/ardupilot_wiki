.. _stall-prevention:

================
Stall Prevention
================

Plane has logic to try to prevent stalls when in certain modes. This
page describes how it works and how to configure it.

Basic Idea
==========

One of the most common ways that users crash is through a stall. A stall
happens when the airflow over the wing is not sufficient to hold the
aircraft in the air. Stalls can happen at any speed, but the most common
type of stall is a low speed stall, where the airflow is too slow to
provide enough lift.

The amount of airflow you need over the wing to hold the aircraft in the
air depends (among other things) on the bank angle you are flying at. If
you are banked over hard then you need to fly faster to get enough lift
to stay in the air. This is because the lift is produced perpendicular
to the wing, so when the wing is rolled over it provides only part of
the lift to holding the aircraft up, and the rest of the lift goes into
making the aircraft turn.

When the :ref:`STALL_PREVENTION <STALL_PREVENTION>`
parameter is set to 1 then two things are done:

-  when in roll controlled modes the autopilot will monitor your
   demanded bank angle and airspeed and work out if you have sufficient
   margin above the stall speed to turn at the demanded bank angle. If
   you don't then the turn will be limited to the safe limit, but it
   will always allow a bank of at least 25 degrees (to ensure you can
   still manoeuvre if your airspeed estimate is badly off).
-  when in auto-throttle modes the autopilot will also raise the minimum
   airspeed in the :ref:`TECS system <tecs-total-energy-control-system-for-speed-height-tuning-guide>`
   to the level at which the current demanded bank angle can be safely
   achieved. So it will add more engine power or lower the nose to raise
   the airspeed so that the bank angle that the navigation controller is
   demanding can be achieved without a stall

Configuring stall prevention
============================

There are two key parameters that control stall prevention:

-  The :ref:`STALL_PREVENTION <STALL_PREVENTION>`
   parameter. If this is set to zero then no stall prevention is done.
   This may be useful if you have no airspeed sensor and the synthetic
   airspeed estimate is not good enough
-  The :ref:`ARSPD_FBW_MIN <ARSPD_FBW_MIN>`
   parameter, which is the configured minimum airspeed for level flight.
   It is this value that is scaled with the bank angle to calculate the
   safe airspeed for any demanded bank angle

Affected modes
==============

The following modes are affected by the bank angle limiting of the stall
prevention code:

-  FBWA
-  FBWB
-  AUTOTUNE
-  CIRCLE
-  RTL
-  AUTO
-  CRUISE
-  LOITER
-  GUIDED
-  TRAINING

In each of these modes the aircraft is either calculating a desired bank
angle for the navigation code, or the user is inputting a desired bank
angle via the aileron stick. In both cases the stall prevention code
will limit the bank angle based on the amount of margin between
``ARSPD_FBW_MIN`` and the current airspeed.

The following modes are affected by the automatic speed increase in TECS
when the bank angle is limited by stall prevention:

-  CIRCLE
-  RTL
-  AUTO
-  CRUISE
-  LOITER
-  GUIDED

These are all of the "auto throttle" modes, where the TECS controller
controls pitch and throttle.

Example Usage
=============

An example of how stall prevention works is in AUTO mode when making a
turn for an automated landing. The landing approach will aim for an
airspeed of :ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>`,
which is often quite low, and may be just above the stall speed. If the
final turn is sharp (it is often 90 degrees) the plane may be trying to
perform a sharp turn while dropping speed for landing. That could induce
a stall. The stall prevention code means two things happen:

-  the initial turn is kept shallow enough to prevent the stall
-  the target airspeed is kept high enough during the turn to allow the
   turn to complete, after which it is lowered

Nose Down in FBWA and AUTOTUNE
==============================

The FBWA mode has an additional special feature to help prevent straight
line stalls. It is often difficult for a pilot to judge the airspeed of
a plane when it is coming head on, and so many pilots find themselves
flying too slow when they are trying to fly gently in FBWA mode. This is
especially true if the ArduPilot pitch controller is well tuned and
manages to keep the nose up even when flying slowly.

To make a level flight low speed stall less likely some additional down
pitch is added in FBWA and AUTOTUNE modes based on the throttle
position. The amount of down pitch added is based on the
:ref:`STAB_PITCH_DOWN <STAB_PITCH_DOWN>`
parameter, which defaults to 2 degrees. At zero throttle this full down
pitch is added. If throttle is above ``TRIM_THROTTLE`` then no down
pitch is added. Between those two values the down pitch is added in
proportion to the throttle.

This has the effect of slightly lowering the nose when you lower
throttle in FBWA and AUTOTUNE modes, which makes the plane gain a bit of
speed, and thus makes it less likely to stall. The value you need for
``STAB_PITCH_DOWN`` depends on how much drag your plane has. A very
sleek model will need a smaller value. A high drag model will need a
larger value.
