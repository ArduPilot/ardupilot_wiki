.. _guide-tailsitter:

=================
Tailsitter Planes
=================

In ArduPilot tailsitters are any VTOL aircraft type that rotates the
fuselage (and autopilot) when moving between forward flight and hover.

Despite the name, not all tailsitters land on their tails. Some are
"belly landers", where they lie down flat for landing to improve
takeoff and landing stability in wind. Some may have an undercarriage
for wheeled takeoff and others may have a stand or other landing aid.

All tailsitters are considered types of QuadPlanes in ArduPilot. You
should start off by reading the :ref:`QuadPlane documentation
<quadplane-support>` before moving onto this tailsitter specific
documentation.

Vectored and non-Vectored
=========================

ArduPilot sub-divides tailsitters into two broad categories:

- Vectored tailsitters can tilt their rotors independently of the
  movement of the fuselage, giving them vectored thrust
- Non-vectored tailsitters have fixed rotor orientation relative to
  the fuselage, and rely on large control surfaces for hover authority (although dual motor versions can use differential thrust for body frame yaw control also)

  - Within Non-vectored are two sub-categories: Single/Dual Motor and CopterMotor. The first uses one or two motors and can employ only differential thrust if dual motor , while the second uses three, four, or more motors and operates in a more copter-like fashion. Single motor tailsitters are similar to normal 3D planes that can hover using large control surfaces exposed to the single motor prop wash for control, but with ArduPilot providing the control stability for hovering, loitering, and VTOL mission navigation. Dual Motor add differential thrust to assist in body frame yaw control, while copter motor tailsitters are almost the same as a multicopter when in VTOL stance with flying surfaces, if present, adding to the control.

Tailsitter Configuration
========================

The key parameter to make a  single or dual motor QuadPlane a tailsitter is to set
:ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` =10. That tells the QuadPlane code to use the tailsitter
VTOL backend.

The tailsitter backend is a bit unusual, as it is the only
:ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` setting that may not have any multicopter-style motors associated with it. If :ref:`Q_TAILSIT_MOTMX<Q_TAILSIT_MOTMX>` is zero (the default), meaning no multicopter-like motors, it provides roll, pitch, yaw and thrust (Throttle, Throttle Left, Throttle Right) values to the fixed wing control code. These values then control your ailerons, elevons, elevators, rudder and forward motors.

This has a nice benefit when setting up the tailsitter that you can
follow the normal fixed wing setup guide in MANUAL and FBWA modes, and
then when you switch to hover all of your control directions will be
correct.

It also means that you can fly any fixed wing aircraft that is capable
of 3D flight as a single or dual motor tailsitter, and fly it in modes like QSTABILIZE,
QHOVER and QLOITER.

.. youtube:: bMsfjwUAfkM
    :width: 450px


However, it can also have copter-like motors, like a conventional QuadPlane if :ref:`Q_TAILSIT_MOTMX<Q_TAILSIT_MOTMX>` is non-zero. Then this parameter determines which motors remain active in normal forward flight (plane modes). If non-zero, then use the :ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` and :ref:`Q_FRAME_TYPE<Q_FRAME_TYPE>` parameter to configure the multicopter motor style, and the appropriate MOTORx outputs will be activated.

All copter motor layouts are supported as tailsitters.

Motor Rotation and Layout
=========================

(looking down on nose from above)

Single/Dual Motor Tailsitter
----------------------------

Motors are controlled by the Throttle, Throttle Left, Throttle Right outputs:

.. image:: ../images/tailsit-motors.jpg
  :width: 450px

.. youtube:: cfqP9-2IWtQ

Orientation
===========

The :ref:`AHRS_ORIENTATION<AHRS_ORIENTATION>`, the accelerometer calibration and AHRS trim
should all be done for fixed wing flight. Fixed wing flight is
considered "normal" orientation for a tailsitter.

Key parameters
==============

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
  
:ref:`Q_TAILSIT_RLL_MX<Q_TAILSIT_RLL_MX>` allows the roll limit angle limit to be set differently from
:ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>`. If left at zero both pitch and roll are limited by :ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>`. If :ref:`Q_TAILSIT_RLL_MX<Q_TAILSIT_RLL_MX>` is nonzero roll angle will be limited and pitch max angle will still be :ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>`.
This should be set if your tailsitter can achieve much larger pitch angle than 
would be safe for roll (some airframes can't recover from high-speed knife-edge flight).

:ref:`Q_TRIM_PITCH<Q_TRIM_PITCH>` can be used to account for any offset in hover pitch angle due to the center of gravity being closer to the belly or top of the fuselage. Set this such that the the vehicle does not drift forwards or backwards in QSTABILIZE or QHOVER with no pilot inputs.

Vectored Thrust
===============

If your tailsitter has vectored thrust then you should set the
SERVOn_FUNCTION values for your two tilt servos for the left and right
motors and for the left and right motor throttles.

.. note:: All tailsitters ignore the :ref:`Q_TILT_TYPE<Q_TILT_TYPE>` parameter, requiring continuous output servos, and will drive the tilt servos appropriately.

For example, if your left tilt servo is channel 5 and your right tilt
servo is channel 6, then set:

- :ref:`SERVO5_FUNCTION<SERVO5_FUNCTION>` =75
- :ref:`SERVO6_FUNCTION<SERVO6_FUNCTION>` =76

and you need to assign left throttle to the left motor and right throttle to the right motor, for example using the SERVO 7 and SERVO 8 outputs, for left and right motor escs, respectively:

- :ref:`SERVO7_FUNCTION<SERVO7_FUNCTION>` =73
- :ref:`SERVO8_FUNCTION<SERVO8_FUNCTION>` =74

you also need to set the right SERVOn_REVERSED values, and the right
SERVOn_TRIM, SERVOn_MIN and SERVOn_MAX values, as appropriate.

:ref:`Q_A_ANGLE_BOOST<Q_A_ANGLE_BOOST>` should be disabled for vectored thrust tailsitters. Failure to disable this will cause the throttle to decrease as the nose dips, making the nose dip even further and resulting in a crash. 

.. caution:: When disarmed, switching to QHOVER or QLOITER will force the motors forward into fixed wing orientation. If armed in this position, a prop strike could occur for Belly Sitter configurations. Tilt will be raised to VTOL position when throttle is raised above idle, but the strike will have already occurred. The solution is to momentarily raise the throttle above idle, allowing the tilts to raise, return throttle stick to idle, then arm. This needs to be done also for AUTO mode takeoffs, which should be started from QSTABILIZE with motors raised, armed, and then change to AUTO for the takeoff.

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
and :ref:`MIXING_GAIN<MIXING_GAIN>` you can adjust how much control you have from elevons
and thrust vectoring in each flight mode.

.. youtube:: s2KLOAdS_HY
    :width: 100%

PID gain scheduling
===================

There are a number of options for scaling control surface movement. Control surface effectiveness is dependent on airspeed they see, in tailsitter configurations this is heavily driven by the motor layout and prop wash.
A copter tailsitter with lots of control authority from thrust and small control surfaces will be much less sensitive to these gain scheduling parameters. For a vehicle with large control surfaces care must be taken to setup 
gain scaling to best suit the configuration.

The gain scheduling scheme is selected with :ref:`Q_TAILSIT_GSCMSK<Q_TAILSIT_GSCMSK>`, it is a bitmask, some options can be used in combination, others must be used in isolation.

The maximum and minimum scaling that can be applied by any scheme is set by :ref:`Q_TAILSIT_GSCMIN<Q_TAILSIT_GSCMIN>` and :ref:`Q_TAILSIT_GSCMAX<Q_TAILSIT_GSCMAX>`. If a scheme is working well at all but the extremes these endpoints can be adjusted.

.. tip:: Scaling is done relative the the hover throttle point, ensure this is set correctly before proceeding, see: :ref:`Flight Modes<quadplane-flight-modes>`

Disk theory gain scheduling is the most advanced method available and should result in the best results if setup correctly.

Throttle scaling (Bit 0)
------------------------

Control surfaces will be scaled directly with throttle. High throttle will result in less control surface movement, low throttle will result in more. This method is always used for thrust vectoring scaling independent of :ref:`Q_TAILSIT_GSCMSK<Q_TAILSIT_GSCMSK>`.

reduce gain at high throttle/tilt (Bit 1)
-----------------------------------------

Attitude/throttle based gain attenuation, with this option control surface deflection is reduced at high tilt angles and high throttle levels to prevent oscillation at high airspeeds. This can be used in combination with throttle scaling.

Disk theory (Bit 2)
--------------------
Disk theory gain scheduling attempts to calculate the airspeed seen on the control surfaces as a result of both prop wash and forward airspeed. In order for this calculation to be done :ref:`Q_TAILSIT_DSKLD<Q_TAILSIT_DSKLD>` must be set. This is the aircraft weight in KG divided by the total disk area of the propellers.
The disk area for each propeller is calculated from the radius, the disk area of all propellers should then be summed. This allows ArduPilot to calculate the airspeed directly behind the propeller, however on a real vehicle 100% of the control surface is not in the direct prop wash. For example if half of the control surfaces are in the prop wash the calculated disk loading value 
should also be halved. Some tuning will be required for best performance, If oscillations are seen at high throttle :ref:`Q_TAILSIT_DSKLD<Q_TAILSIT_DSKLD>` should be reduced.

For best results an airspeed sensor should be fitted.

Note that Disk theory scheduling is only used if both Bit 0 and Bit 1 are disabled.

Altitude correction (Bit 3)
---------------------------

Gain is scaled with altitude, this should be considered when operating over a wide range of altitudes, this method can be enabled in combination with any other method.

Transitions
===========

Tailsitter transitions are a little different than other QuadPlane transitions. 

:ref:`Q_TAILSIT_ANGLE<Q_TAILSIT_ANGLE>` specifies how far the nose must pitch down in a VTOL mode before transition to forward flight is complete. So a value of e.g. 60 degrees results in switching from copter to plane controller (forward transition) when the nose reaches 30 degrees above the horizon (60 degrees down from vertical).
The pitch rate used when pitching down to forward flight is given by :ref:`Q_TAILSIT_RAT_FW<Q_TAILSIT_RAT_FW>`, this rate will be held until :ref:`Q_TAILSIT_ANGLE<Q_TAILSIT_ANGLE>` is reached.


For the back transition from forward flight to VTOL, the plane controller will be used until the nose reaches :ref:`Q_TAILSIT_ANG_VT<Q_TAILSIT_ANG_VT>` above the horizon. If :ref:`Q_TAILSIT_ANG_VT<Q_TAILSIT_ANG_VT>` is 0 :ref:`Q_TAILSIT_ANGLE<Q_TAILSIT_ANGLE>` will be used for both forward and back transitions.
The pitch rate used when pitching up to VTOL flight is given by :ref:`Q_TAILSIT_RAT_VT<Q_TAILSIT_RAT_VT>`, this rate will be held until :ref:`Q_TAILSIT_ANG_VT<Q_TAILSIT_ANG_VT>` is reached.

.. note:: During transitions, pilot input is disabled and vehicle attitude is controlled totally by the autopilot.

.. tip:: ArduPilot will send as message to the GCS when transition is complete, these can be found in the Mission Planner messages tab. If the transition does not complete normally the transition parameters and vehicle tune should be checked. A example message is ```Transition FW done, timeout```, the timeout time is one and a half times the expected transition time as calculated from the angle and rate parameters.

Control Surfaces
================

Although not recommended it is possible to fly a tailsitter aircraft with no control surfaces. Care should be taken to get the vehicle flying well in the hover modes first. :ref:`Q_OPTIONS<Q_OPTIONS>` bit 7:Force Qassist will force the vehicle to use the copter controller in all flight modes. :ref:`Q_OPTIONS<Q_OPTIONS>` bit 8:Mtrs_Only_Qassist 
allows use of the copter controller for the motors but leave any controls surfaces under plane control, this allows the control surfaces to act as 'trim tabs' for the motors. See :ref:`Assisted Fixed-Wing Flight<assisted_fixed_wing_flight>` for more details on how Qassist can be automatically enabled and disabled with airspeed. 

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

.. note:: Due to the rotation of the tailsitter body frame with respect to the multicopter body frame, the roll limits are set by parameter :ref:`Q_YAW_RATE_MAX<Q_YAW_RATE_MAX>` (in degrees), and the yaw rate limits are set by parameter :ref:`Q_TAILSIT_RLL_MX<Q_TAILSIT_RLL_MX>` (in deg/sec).  The pitch limit is set by parameter :ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>` (in centidegrees), and this also serves as the yaw rate limit if :ref:`Q_TAILSIT_RLL_MX<Q_TAILSIT_RLL_MX>` is zero. If any rate limit is too high for the airframe, you may experience glitches in attitude control at high rates.

.. note:: :ref:`Q_TAILSIT_INPUT<Q_TAILSIT_INPUT>` is ignored in QACRO modes. All inputs are body-frame referenced.

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

- :ref:`Q_TAILSIT_MASK<Q_TAILSIT_MASK>` =8 (for rudder)
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
tailsitters.

Pre-Arm Issues
==============

Due to an issue in DCM related to compass fusion for yaw when pointing straight up, sometimes the AHRS subsystems will disagree when powering up, nose up. Slight errors in compass calibration, while resulting in a successful calibration, may worsen this effect.

The result is that some setups will give a pre-arm failure. Typically it is "Pre-Arm:DCM roll/pitch inconsistent by "x" degrees" or similar. If this happens consistently, then one of two solutions can be used:

- Power up horizontally, and allow the autopilot to begin initialization in this position. After the IMUs tilt initialization is completed (usually in the first ten to fifteen seconds or so), the Tailsitter can be set vertically for the remainder of the initialization (ie after GPS lock and EKF is using the GPS) and then armed.
- Or, if you get the Pre-Arm failure above, lay the Tailsitter down horizontally for 10-30 seconds to allow the various AHRS subsystems to synchronize. After that it can be raised and arming should proceed normally.
