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

.. note:: This is distinct from :ref:`tailsitters <guide-tailsitter>` where the autopilot and main fuselage change orientation when moving between hover and forward flight. Do  not use the information below for a Tailsitter, some parameters are shared, but use the  instructions in the :ref:`guide-tailsitter`.


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

Combined with these variants are versions that use ailerons, elevons,
vtails and other control surfaces for fixed wing flight. There are an
amazing number of combinations possible, and experimentation with VTOL
designs is common. ArduPilot aims to support a very wide range of
tilt-rotor configurations with a small number of parameters.

Setting Up A Tilt-Rotor
=======================

The first thing you need to do is enable QuadPlane support by setting
:ref:`Q_ENABLE<Q_ENABLE>` to 1, and then choose the right quadplane frame class and
frame type.

The quadplane frame class is in :ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` . The frame class is
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

.. note: the BiCopter tilt-rotor QuadPlane is a special case. It requires that :ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` be set to 10 which is reserved for Tailsitter configurations only, normally. See :ref:`BiCopter <bicopter>`  below.

Once you have chosen your frame class you will need to get the
:ref:`Q_FRAME_TYPE<Q_FRAME_TYPE>` right. The :ref:`Q_FRAME_TYPE<Q_FRAME_TYPE>` is the sub-type of frame. For
example, for a Quadcopter, a frame type of 1 is for a "X" frame and a
frame type of 3 is for a "H" frame. For Tri and Y6, this parameter is ignored.

Please see the ArduCopter setup guide for multi-copters for more
information on choosing your frame type.

After setting up :ref:`Q_ENABLE<Q_ENABLE>`, :ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` and :ref:`Q_FRAME_TYPE<Q_FRAME_TYPE>` you will
need to reboot.

The Tilt Mask
=============

The most important parameter for a tilt-rotor is the tilt-mask, in the
:ref:`Q_TILT_MASK<Q_TILT_MASK>` parameter.

The :ref:`Q_TILT_MASK<Q_TILT_MASK>` is a bitmask of what motors can tilt on your
vehicle. The bits you need to enable correspond to the motor ordering
of the standard ArduCopter motor map for your chosen frame class and
frame type, ie. bit 0 corresponds to Motor 1.

For example, if you have a tilt-tricopter where the front two motors
tilt, then you should set :ref:`Q_TILT_MASK<Q_TILT_MASK>` to 3, which is 2+1.

If you have a tilt-quadplane where all 4 motors tilt, then you should
set :ref:`Q_TILT_MASK<Q_TILT_MASK>` to 15, which is 8+4+2+1.

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

You need to set the type of tilt you have using the :ref:`Q_TILT_TYPE<Q_TILT_TYPE>`
parameter. Valid values are:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Tilt Type</th><th>Q_TILT_TYPE</th></tr>
   <tr><td>Continuous</td><td>0</td></tr>
   <tr><td>Binary</td><td>1</td></tr>
   <tr><td>Vectored</td><td>2</td></tr>
   <tr><td>BiCopter</td><td>3</td></tr>
   </table>


Tilt Servos
===========

Next you need to configure which servo outputs will control tilt of
the tiltable rotors.

You control that with 3 possible servo function values.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Tilt Control</th><th>SERVOn_FUNCTION</th></tr>
   <tr><td>Tilt Front Motors</td><td>41</td></tr>
   <tr><td>Tilt Front Left Motor</td><td>75</td></tr>
   <tr><td>Tilt Front Right Motor</td><td>76</td></tr>
   </table>

You should choose normal ``Tilt Front Motors`` unless you are configuring a
vectored yaw aircraft and have set :ref:`Q_TILT_TYPE<Q_TILT_TYPE>` to 2.

For example, if you have a single servo which tilts your rotors
attached to servo output 11, then you should set :ref:`SERVO11_FUNCTION<SERVO11_FUNCTION>` =41.

Tilt Reversal and Range
=======================

You will need to set the ``SERVOn_REVERSED`` parameter on your tilt servos
according to the direction of your servos. You should adjust so that
in MANUAL mode the rotors are tilted forward and in QSTABILIZE mode
they point straight up.

You will probably also need to adjust the SERVOn_MIN an SERVOn_MAX
values to adjust the range of movement and the exact angle of each
servo for forward flight and hover.

Tilt Angle
==========

The :ref:`Q_TILT_MAX<Q_TILT_MAX>` parameter controls the tilt angle during
transitions for continuous tilt vehicles. It is the angle in degrees
that the rotors will move to while waiting for the transition airspeed
to be reached.

The right value for :ref:`Q_TILT_MAX<Q_TILT_MAX>` depends on how much tilt you need to
achieve sufficient airspeed for the wings to provide most of the
lift. For most tilt-rotors the default of 45 degrees is good.

Tilt Rate
=========

A critical parameter for tilt rotors is how quickly they move the tilt
servos when transitioning between hover and forward flight.

The two parameters that control tilt rate are:

- :ref:`Q_TILT_RATE_UP<Q_TILT_RATE_UP>` is the tilt rate upwards in degrees per second
- :ref:`Q_TILT_RATE_DN<Q_TILT_RATE_DN>` is the tilt rate downwards in degrees per second

If :ref:`Q_TILT_RATE_DN<Q_TILT_RATE_DN>` is zero then :ref:`Q_TILT_RATE_UP<Q_TILT_RATE_UP>` is used for both
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

.. note:: For Binary type tilt servos these rates should be set at the actual measured rate of the servo since it's independent of ArudPilot control.

Vectored Yaw
============

Vectored yaw aircraft tilt the left and right rotors separately to
control yaw in hover. This reduces mechanical complexity in
tilt-tricopters as it avoids the need for a tilt servo for the rear
motor for yaw control.

To setup a vectored yaw aircraft you need to set :ref:`Q_TILT_TYPE<Q_TILT_TYPE>` =2, and
also set :ref:`Q_TILT_YAW_ANGLE<Q_TILT_YAW_ANGLE>` to the angle in degrees that the tilt motors
can go up past 90 degrees.

For example, if you have a tilt-tricopter with vectored yaw, and your
motors can tilt through a total of 110 degrees from forward flight,
then your :ref:`Q_TILT_YAW_ANGLE<Q_TILT_YAW_ANGLE>` would be 20, as that is the angle past 90
degrees that the tilt mechanism can go.

You also need to setup your two tilt servos with ``SERVOn_FUNCTION`` =75
for left front tilt and ``SERVOn_FUNCTION`` =76 for right front tilt.

Non-Vectored Yaw
================

Non-Vectored yaw aircraft (:ref:`Q_TILT_TYPE<Q_TILT_TYPE>` = 0 or 1) needs a tilt servo for yaw control.

You need to setup your front tilt servos with ``SERVOn_FUNCTION=41`` and also your servo for yaw control with ``SERVOn_FUNCTION=39``, if the frame is a Tricopter. You should set up the yaw servo’s maximum lean angle in degrees with :ref:`Q_M_YAW_SV_ANGLE<Q_M_YAW_SV_ANGLE>`. This lean angle assumes that ``SERVOn_MIN`` and ``SERVOn_MAX``, represent +/- 90 degrees, with ``SERVOn_TRIM`` representing 0 degrees lean.

Note:
``SERVO_FUNCTION=39`` is normally the output function for motor 7, but in a non-vectored yaw tilt-rotor Tricopter, the yaw servo is controlled via ``SERVOn_FUNCTION`` = 39.

If you wish to setup BLEHeli esc telemetry, you need to set :ref:`Q_M_PWM_TYPE<Q_M_PWM_TYPE>` to 4 (DShot 150), connect the telemetry signal to a SERIAL port, and set its ``SERIALn_PROTOCOL`` to 23.

Note that if you want to use BLHeli passthru setup or telemetry in a non-vectored yaw Tricopter,
you must not set :ref:`SERVO_BLH_AUTO<SERVO_BLH_AUTO>` to 1. Instead, set :ref:`SERVO_BLH_MASK<SERVO_BLH_MASK>` to the output-bitmask
of the servo-channels actually connected BLHELI-ESCs.

For example if your motors are connected to servo 9,10,11 (the first three aux-outputs of a pixhawk1), set :ref:`SERVO_BLH_MASK<SERVO_BLH_MASK>` to 1792.

.. _bicopter:

BiCopter Tilt-Rotor
===================

This is a special case of tilt-rotor QuadPlane. Setup is a bit different, but the configuration is actually a normal QuadPlane and performs QuadPlane transitions. In order to setup this vehicle configuration:

- :ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` = 10 (Tailsitter, even though this is not a tailsitter!)
- :ref:`Q_TILT_TYPE<Q_TILT_TYPE>` = 3 (BiCopter)

Motor and Tilt Setup
--------------------

For motors and tilt servos,you should set the SERVOn_FUNCTION values for your two tilt servos for the left and right motors, and for the left and right motor throttles.

The tilt servo limits are setup a bit differently than other Tilt Rotors. To setup, a normal tilt rotor range, you would set the :ref:`Q_TILT_YAW_ANGLE<Q_TILT_YAW_ANGLE>`, then the tilt servo's MIN and MAX output range to get vertical in QSTABLIZE and horizontal in MANUAL on the bench.The TRIM value is ignored.

With this frame type, the tilt servo's MIN sets horizontal position, TRIM the vertical position, and MAX the full rearward (max  VTOL yaw). In this case, the user must set the :ref:`Q_TILT_YAW_ANGLE<Q_TILT_YAW_ANGLE>` for the amount of forward yaw from vertical (should match the rearward angle to prevent asymmetric yaw authority in one direction).

.. note:: MIN and MAX may be swapped if the tilt servo had to be reversed to get proper directions.

Otherwise, this frame type conforms to the normal vectored yaw tilt-rotor QuadPlane transitions, and parameters.

Tilt Rotor Movement Setup
=========================

.. toctree::
    :maxdepth: 1

    Tilt Rotor Setup Tips<tilt-rotor-tips>

Also:
    :ref:`Tilt Rotor Servo Setup<tilt-rotor-setup>`



Pre Flight Checks
=================

In addition to the normal pre-flight checks for a QuadPlane, you
should check your tilt-rotor transition by changing between MANUAL and
QSTABILIZE modes on the ground. Make sure that your tilt moves
smoothly and that the servos are trimmed correctly for the right rotor
angles.
