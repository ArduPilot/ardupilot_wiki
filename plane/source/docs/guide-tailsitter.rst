.. _guide-tailsitter:

=================
Tailsitter Planes
=================

In ArduPilot tail-sitters are any VTOL aircraft type that rotates the
fuselage (and autopilot) when moving between forward flight and hover.

Despite the name, not all tails-sitters land on their tails. Some are
"belly landers", where they lie down flat for landing to improve
takeoff and landing stability in wind. Some may have an undercarriage
for wheeled takeoff and others may have a stand or other landing aid.

All tails-sitters are considered types of QuadPlanes in ArduPilot. You
should start off by reading the :ref:`QuadPlane documentation
<quadplane-support>` before moving onto this tailsitter specific
documentation.

Vectored and non-Vectored
=========================

ArduPilot sub-divides tail-sitters into two broad categories:

- vectored tailsitters can tilt their rotors independently of the
  movement of the fuselage, giving them vectored thrust
- non-vectored tailsitters have fixed rotor orientation relative to
  the fuselage, and rely on large control surfaces for hover authority

Tailsitter Config
=================

The key parameter to make a plane a tailsitter is to set
Q_FRAME_CLASS=10. That tells the QuadPlane code to use the tailsitter
VTOL backend.

The tailsitter backend is a bit unusual, as it is the only
Q_FRAME_CLASS setting that doesn't have any motors associated with
it. The way the backend works is that it provides roll, pitch, yaw and
thrust values to the fixed wing control code. These values then
control your ailerons, elevons, elevators, rudder and motors.

This has a nice benefit when setting up the tailsitter that you can
follow the normal fixed wing setup guide in MANUAL and FBWA modes, and
then when you switch to hover all of your control directions will be
correct.

It also means that you can fly any fixed wing aircraft that is capable
of 3D flight as a tailsitter, and fly it in modes like QSTABILIZE,
QHOVER and QLOITER.

The key differences between fixed wing flight and hovering for a
tailsitter are:

- when hovering the copter PID gains will be used (the ones starting
  with Q_A_RAT_*)
- when in fixed wing flight the fixed wing PID gains will be used (the
  PTCH2SRV_* and RLL2SRV_* gains)
- when hovering the nose of the aircraft will try to point up for
  "level" flight
- when in fixed wing flight the nose of the aircraft will try to point
  forward for "level" flight

Orientation
===========

The AHRS_ORIENTATION, the accelerometer calibration and AHRS trim
should all be done for fixed wing flight. Fixed wing flight is
considered "normal" orientation for a tailsitter.

Vectored Thrust
===============

If your tailsitter has vectored thrust then you should set the
SERVOn_FUNCTION values for your two tilt servos for the left and right
tilt servos.

For example, if your left tilt servo is channel 5 and your right tilt
servo is channel 6, then set:

- SERVO5_FUNCTION=75
- SERVO6_FUNCTION=76

you also need to set the right SERVOn_REVERSED values, and the right
SERVOn_TRIM, SERVOn_MIN and SERVOn_MAX values.

Vectored Gains
==============

There are two vectoring gains available. One controls the amount of
vectored thrust movement in hover, and the other controls the amount
of vectored thrust movement in forward flight.

The Q_TAILSIT_VHGAIN parameter controls vectored thrust in hover. A
typical value is around 0.8, which gives a lot of control to vectored
thrust in hover. This control is combined with control from your
elevon mixing gain (controlled by MIXING_GAIN).

The Q_TAILSIT_VFGAIN parameter controls vectored thrust in forward
flight. A typical value is around 0.2, which gives a small amount of
control to vectored thrust in forward flight. This control is combined
with control from your elevon mixing gain (controlled by MIXING_GAIN).

By adjusting the relative values of Q_TAILSIT_VHGAIN, Q_TAILSIT_VFGAIN
and MIXING_GAIN you can adjust how much control you have from elevons
and thrust vectoring in each flight mode.

Tailsitter Input
================

You can change how control inputs while hovering a tailsitter will be
interpreted using the Q_TAILSIT_INPUT parameter. The choices are:

- Q_TAILSIT_INPUT=0 means that in hover the aircraft responds like a
  multi-rotor, with the yaw stick controlling earth-frame yaw, and
  roll stick controls earth-frame roll. This is a good choice for
  pilots who are used to flying multi-rotor aircraft.

- Q_TAILSIT_INPUT=1 means that in hover the aircraft responds like a
  3D aircaft, with the yaw stick controlling body-frame yaw, and roll
  stick controls body-frame roll. This is a good choice for pilots who
  are used to flying 3D aircraft in prop-hang.

Tailsitter Input Mask
=====================

To support people flying 3D aircraft and wanting to learn how to
prop-hang manually, you can set the Q_TAILSIT_MASK to a mask of
channels that will have full manual input control while hovering.

The mask of manual channels is enabled using a transmitter input
channel, specified with the Q_TAILSIT_MASKCH parameter.

For example, if you are learning how to fly 3D aircraft, and you want
some assistance learning how to best control the rudder, then you can
set:

- Q_TAILSIT_MASK=8 (for rudder)
- Q_TAILSIT_MASKCH=7

then when channel 7 goes above 1700 the pilot will be given full
manual control of rudder when hovering. This provides good 3D piloting
practice on one or more axes at a time.
  
Center of Gravity
=================

The center of gravity for a tailsitter is important in an extra
dimension. When hovering it is important that there is not too much
weight in the belly of the plane or on its back, so that it leans
forward or back. This is particularly important for non-vectored
tail-sitters.
