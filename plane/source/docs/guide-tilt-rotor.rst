.. _guide-tilt-rotor:

=================
Tilt Rotor Planes
=================

Tilt rotors are treated by ArduPilot as a special type of
QuadPlane. You should start off by reading the :ref:`QuadPlane
documentation <quadplane-support>` before moving onto this tilt-rotor
specific documentation.

In ArduPilot nomenclature, a tilt-rotor is a type of VTOL aircraft
where transition between hover and forward flight is accomplished by
tilting one or more rotors so that it provides forward thrust instead
of upward thrust.

This is distinct from :ref:`tailsitters <guide-tailsitter>` where the
autopilot and main fuselage change orientation when moving between
hover and forward flight.

Types of Tilt-Rotors
====================

ArduPilot supports a very wide range of tilt-rotor
configurations. Common configurations include:

- tilt-quadplanes with the front two motors tilting
- tilt-quadplanes with all four motors tilting
- tilt-tricopters with the front two motors tilting and rear tilt for
  yaw
- tilt-tricopters with the front two motors tilting and vectored yaw
- tilt-hexacopters with the front four motors tilting
- tilt-wings where the main wing tilts along with two motors
- binary-tiltrotors where the tilt mechanism can only be in one of two positions
- continuous-tiltrotors where the tilt mechanism can be controlled to
  any angle in a range from straight up to straight forward
- vectored tilt-rotors where the tilt of the rotors on the left can be
  controlled independently from the tilt of the right motors

Combined with these varients are versions that use ailerons, elevons,
vtails and other control surfaces for fixed wing flight. There are an
amazing number of combinations possible, and experimentation with VTOL
designs is common. ArduPilot aims to support a very wide range of
tilt-rotor configurations with a small number of parameters.

Setting Up A Tilt-Rotor
=======================

The first thing you need to do is enable QuadPlane support by setting
Q_ENABLE to 1, and then choose the right quadplane frame class and
frame type.

The quadplane frame class is in Q_FRAME_CLASS. The frame class is
chosen based on your vehicles rotor configuration while
hovering. Currently supported tilt-rotor frame classes are:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Frame Class</th><th>Q_FRAME_CLASS</th></th></tr>
   <tr><td>Quadcopter</td><td>1</td></tr>
   <tr><td>Hexacopter</td><td>2</td></tr>
   <tr><td>Octacopter</td><td>3</td></tr>
   <tr><td>Octaquad</td><td>4</td></tr>
   <tr><td>Y6</td><td>5</td></tr>
   <tr><td>Tricopter</td><td>7</td></tr>
   </table>

Once you have chosen your frame class you will need to get the
Q_FRAME_TYPE right. The Q_FRAME_TYPE is the sub-type of frame. For
example, for a quadcopter, a frame type of 1 is for a "X" frame and a
frame type of 3 is for a "H" frame.

Please see the ArduCopter setup guide for multi-copters for more
information on choosing your frame type.

After setting up Q_ENABLE, Q_FRAME_CLASS and Q_FRAME_TYPE you will
need to reboot.

The Tilt Mask
=============

The most important parameter for a tilt-rotor is the tilt-mask, in the
Q_TILT_MASK parameter.

The Q_TILT_MASK is a bitmask of what motors can tilt on your
vehicle. The bits you need to enable correspond to the motor ordering
of the standard ArduCopter motor map for your chosen frame class and
frame type.

For example, if you have a tilt-tricopter where the front two motors
tilt, then you should set Q_TILT_MASK to 3, which is 2+1.

If you have a tilt-quadplane where all 4 motors tilt, then you should
set Q_TILT_MASK to 15, which is 8+4+2+1.

The Tilt Type
=============

Most tilt-rotors use normal servos for tilting their rotors. This
allows the autopilot to control the angle of tilt continuously in a
range from straight up to straight forward.

Some tilt-rotors instead have a binary mechanism, typically using
retract servos, where the autopilot can command the servo into either
a fully up or fully forward position, but can't ask for the tilt to
stop at some angle in between.

Finally some tilt-rotors have vectored control of yaw, where they can
control yaw by tilting the left rotors independently of the right
rotors.

You need to set the type of tilt you have using the Q_TILT_TYPE
parameter. Valid values are:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Tilt Type</th><th>Q_TILT_TYPE</th></tr>
   <tr><td>Continuous</td><td>0</td></tr>
   <tr><td>Binary</td><td>1</td></tr>
   <tr><td>Vectored</td><td>2</td></tr>
   </table>

Tilt Servos
===========

Next you need to configure which servo outputs will control tilt of
the tiltable rotors.

You control that with 3 possible servo function values.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>SERVOn_FUNCTION</th><th>Value</th><th>Value</th></tr>
   <tr><td>41</td><td>Motor tilt</td></tr>
   <tr><td>75</td><td>Left Motor tilt</td></tr>
   <tr><td>76</td><td>Right Motor tilt</td></tr>
   </table>

You should choose normal motor tilt unless you are configuring a
vectored yaw aircraft and have set Q_TILT_TYPE to 2.

For example, if you have a single servo which tilts your rotors
attached to servo output 11, then you should set SERVO11_FUNCTION=41.

Tilt Reversal and Range
=======================

You will need to set the SERVOn_REVERSED parameter on your tilt servos
according to the direction of your servos. You should adjust so that
in MANUAL mode the rotors are tilted forward and in QSTABILIZE mode
they point straight up.

You will probably also need to adjust the SERVOn_MIN an SERVOn_MAX
values to adjust the range of movement and the exact angle of each
servo for forward flight and hover.

Tilt Angle
==========

The Q_TILT_MAX parameter controls the tilt angle during
transitions for continuous tilt vehicles. It is the angle in degrees
that the rotors will move to while waiting for the transition airspeed
to be reached.

The right value for Q_TILT_MAX depends on how much tilt you need to
achieve sufficient airspeed for the wings to provide most of the
lift. For most tilt-rotors the default of 45 degrees is good.

Tilt Rate
=========

A critical parameter for tilt rotors is how quickly they move the tilt
servos when transitioning between hover and forward flight.

The two parameters that control tilt rate are:

- Q_TILT_RATE_UP is the tilt rate upwards in degrees per second
- Q_TILT_RATE_DN is the tilt rate downwards in degrees per second

If Q_TILT_RATE_DN is zero then Q_TILT_RATE_UP is used for both
directions.

How fast you should move the tilt servos depends on a number of
factors, particularly on how well tuned your vehicle is for
multi-rotor flight. In general it is recommended to err on the side of
slow transitions for initial testing, then slowly speed it up as
needed.

A typical value would be 15 degrees per second for both up and down.

Note that there are some automatic exceptions to the tilt rate in the
ArduPilot tilt-rotor code:

- the tilt rate when changing to MANUAL mode is 90 degrees per
  second. This gives you rapid forward flight control in case MANUAL
  mode is needed.

- once a forward transition is completed then the motors will cover
  any remaining angle at 90 degrees per second.

Vectored Yaw
============

Vectored yaw aircraft tilt the left and right rotors separately to
control yaw in hover. This reduces mechanical complexity in
tilt-tricopters as it avoids the need for a tilt servo for the rear
motor for yaw control.

To setup a vectored yaw aircraft you need to set Q_TILT_TYPE=2, and
also set Q_TILT_YAW_ANGLE to the angle in degrees that the tilt motors
can go up past 90 degrees.

For example, if you have a tilt-tricopter with vectored yaw, and your
motors can tilt through a total of 110 degrees from forward flight,
then your Q_TILT_YAW_ANGLE would be 20, as that is the angle past 90
degrees that the tilt mechanism can go.

You also need to setup your two tilt servos with SERVOn_FUNCTION=75
for left tilt and SERVOn_FUNCTION=76 for right tilt.

Pre Flight Checks
=================

In addition to the normal pre-flight checks for a quadplane, you
should check your tilt-rotor transition by changing between MANUAL and
QSTABILIZE modes on the ground. Make sure that your tilt moves
smoothly and that the servos are trimmed correctly for the right rotor
angles.
