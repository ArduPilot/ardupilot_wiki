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

Tailsitter Configuration
========================

The key parameter to make a plane a tailsitter is to set
:ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` =10. That tells the QuadPlane code to use the tailsitter
VTOL backend.

The tailsitter backend is a bit unusual, as it is the only
:ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` setting that doesn't have any motors associated with
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
  
:ref:`Q_TAILSIT_THSCMX<Q_TAILSIT_THSCMX>` defines the maximum throttle scaling that will be applied
to the control surfaces, this should be reduced if oscillations are seen 
at throttles bellow hover throttle.

:ref:`Q_TAILSIT_RLL_MX<Q_TAILSIT_RLL_MX>` allows the roll limit angle limit to be set differently from
:ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>`. If left at zero both pitch and roll are limited by :ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>`. If :ref:`Q_TAILSIT_RLL_MX<Q_TAILSIT_RLL_MX>` is nonzero roll angle will be limited and pitch max angle will still be :ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>`.
This should be set if your tailsitter can achieve much larger pitch angle than 
would be safe for roll (some airframes can't recover from high-speed knife-edge flight).

:ref:`Q_TAILSIT_ANGLE<Q_TAILSIT_ANGLE>` specifies how far the nose must pitch up or down before a transition is complete:
down for transition from VTOL mode to FW mode, and up for transition from FW to VTOL. 
So a value of e.g. 60 degrees results in switching from copter to plane controller (forward transition) when the nose reaches 30 degrees above the horizon (60 degrees down from vertical). For the back transition, the plane controller would be used until the nose reaches 60 degrees above the horizon. So the larger the value of 
:ref:`Q_TAILSIT_ANGLE<Q_TAILSIT_ANGLE>`, the later the switch from one controller to the other.

:ref:`Q_TRANSITION_MS<Q_TRANSITION_MS>` specifies a timeout for transition from VTOL to FW flight. Even if the angle specified by :ref:`Q_TAILSIT_ANGLE<Q_TAILSIT_ANGLE>` has not been reached before this interval has elapsed, the transition will be considered complete. The timeout for back transitions (from FW to VTOL flight) is hardcoded to 2 seconds.

Orientation
===========

The AHRS_ORIENTATION, the accelerometer calibration and AHRS trim
should all be done for fixed wing flight. Fixed wing flight is
considered "normal" orientation for a tailsitter.

Vectored Thrust
===============

If your tailsitter has vectored thrust then you should set the
SERVOn_FUNCTION values for your two tilt servos for the left and right
motors.

For example, if your left tilt servo is channel 5 and your right tilt
servo is channel 6, then set:

- :ref:`SERVO5_FUNCTION<SERVO5_FUNCTION>` =75
- :ref:`SERVO6_FUNCTION<SERVO6_FUNCTION>` =76

you also need to set the right SERVOn_REVERSED values, and the right
SERVOn_TRIM, SERVOn_MIN and SERVOn_MAX values.

:ref:`Q_A_ANGLE_BOOST<Q_A_ANGLE_BOOST>` should be disabled for vectored thrust tailsitters. Failure to disable this will cause the throttle to decrease as the nose dips, making the nose dip even further and resulting in a crash. 

Vectored Gains
==============

There are two vectoring gains available. One controls the amount of
vectored thrust movement in hover, and the other controls the amount
of vectored thrust movement in forward flight.

The :ref:`Q_TAILSIT_VHGAIN<Q_TAILSIT_VHGAIN>` parameter controls vectored thrust in hover. A
typical value is around 0.8, which gives a lot of control to vectored
thrust in hover. This control is combined with control from your
elevon mixing gain (controlled by :ref:`MIXING_GAIN<MIXING_GAIN>` ).

The :ref:`Q_TAILSIT_VFGAIN<Q_TAILSIT_VFGAIN>` parameter controls vectored thrust in forward
flight. A typical value is around 0.2, which gives a small amount of
control to vectored thrust in forward flight. This control is combined
with control from your elevon mixing gain (controlled by :ref:`MIXING_GAIN<MIXING_GAIN>`).

By adjusting the relative values of :ref:`Q_TAILSIT_VHGAIN<Q_TAILSIT_VHGAIN>`, :ref:`Q_TAILSIT_VFGAIN<Q_TAILSIT_VFGAIN>`
and MIXING_GAIN you can adjust how much control you have from elevons
and thrust vectoring in each flight mode.

Tailsitter Input
================

You can change how control inputs while hovering a tailsitter will be
interpreted using the :ref:`Q_TAILSIT_INPUT<Q_TAILSIT_INPUT>` parameter. The choices are:

- :ref:`Q_TAILSIT_INPUT<Q_TAILSIT_INPUT>` =0 means that in hover the aircraft responds like a
  multi-rotor, with the yaw stick controlling earth-frame yaw, and
  roll stick controlling earth-frame roll. This is a good choice for
  pilots who are used to flying multi-rotor aircraft.

- :ref:`Q_TAILSIT_INPUT<Q_TAILSIT_INPUT>` =1 means that in hover the aircraft responds like a
  3D aircaft, with the yaw stick controlling earth-frame roll, and roll
  stick controlling earth-frame yaw. This is a good choice for pilots who
  are used to flying 3D aircraft in prop-hang, but is not very useful
  when flying around, due to the earth-frame multicopter control inputs.

- :ref:`Q_TAILSIT_INPUT<Q_TAILSIT_INPUT>` =2 and 3 mean that the aircraft responds like a 3D aircraft
  with the yaw stick controlling earth-frame yaw and the roll stick controlling
  body-frame roll when flying level. When hovering, these options behave the same
  as types 0 and 1, respectively. This is accomplished by splitting the roll and
  yaw command inputs into bodyframe roll and yaw components as a function of Euler pitch.

**Note:** 
Due to the rotation of the tailsitter body frame with respect to the multicopter body frame, the roll limits are set by parameter :ref:`Q_YAW_RATE_MAX<Q_YAW_RATE_MAX>` (in degrees), and the yaw rate limits are set by parameter :ref:`Q_TAILSIT_RLL_MX<Q_TAILSIT_RLL_MX>` (in deg/sec).  The pitch limit is set by parameter :ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>` (in centidegrees), and this also serves as the yaw rate limit if :ref:`Q_TAILSIT_RLL_MX<Q_TAILSIT_RLL_MX>` is zero. If any rate limit is too high for the airframe, you may experience glitches in attitude control at high rates.

Tailsitter Input Mask
=====================

To support people flying 3D aircraft and wanting to learn how to
prop-hang manually, you can set the :ref:`Q_TAILSIT_MASK<Q_TAILSIT_MASK>` to a mask of
channels that will have full manual input control while hovering.

The mask of manual channels is enabled using a transmitter input
channel, specified with the :ref:`Q_TAILSIT_MASKCH<Q_TAILSIT_MASKCH>` parameter.

For example, if you are learning how to fly 3D aircraft, and you want
some assistance learning how to best control the rudder, then you can
set:

- :ref:`Q_TAILSIT_MASKCH<Q_TAILSIT_MASKCH>` =8 (for rudder)
- :ref:`Q_TAILSIT_MASKCH<Q_TAILSIT_MASKCH>` =7

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
