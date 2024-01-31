.. _traditional-helicopter-tailrotor-setup:

=========================================
Traditional Helicopter – Tailrotor Setup
=========================================

There are several ways for controlling the tailrotor to maintain yaw stabilization and provide yaw control, and each have a unique setup.  The :ref:`H_TAIL_TYPE <H_TAIL_TYPE>` parameter is used to specify method for controlling the tailrotor.  A list of available tail types is given below:

- Servo Only: ArduPilot will supply the tail rotor stabilization like a tail rotor gyro and control the pitch of the tail rotor blades.
- Servo with External Gyro: ArduPilot will output yaw demands without direct yaw attitude stabilization which is provided via an external gyro.
- Direct Drive Variable Pitch (DDVP): Instead of the tail rotor being driven from the main rotor via mechanical coupling as in the above cases, an electric motor is used with motor controlled(ramp up/down, operating point) by ArduPilot. Main yaw control is via tail blade pitch servo, as above.
- Direct Drive Fixed Pitch Clockwise (DDFP CW): Tail rotor is driven by a motor whose ESC is controlled by ArduPilot to maintain yaw stability and yaw direction. Used with clockwise rotating main rotors, when viewed from above.
- Direct Drive Fixed Pitch Counter-Clockwise (DDFP CCW): Tail rotor driven by a motor whose ESC is controlled by ArudPilot to maintain yaw stability and yaw direction. Used with counter-clockwise rotating main rotors, when viewed from above.


Tailrotor Types
===============

Servo Only :ref:`H_TAIL_TYPE <H_TAIL_TYPE>` = 0
+++++++++++++++++++++++++++++++++++++++++++++++

The Servo Only tail type uses a servo connected to an autopilot output whose ``SERVOx_FUNCTION`` is selected as ``Motor4`` to control the tailrotor pitch slider.  The tailrotor speed is controlled by a physical connection (tailrotor shaft or belt) to the engine and main shaft. Depending on your servo setup and mechanical throws, set the ``SERVOx_MIN``, ``SERVOx_MAX``, and ``SERVOx_REVERSED`` parameter values.  Setting the ``SERVOx_TRIM`` is discussed below.

Be sure to check the direction of operation of the Tail Servo. Move the rudder stick and notice the change in tail rotor pitch. Be sure that its increase or decrease of pitch is such that the change in thrust will result in the desired direction of movement. If not, reverse the servo direction with the ``SERVOx_REVERSED`` parameter.

Servo with External Gyro :ref:`H_TAIL_TYPE <H_TAIL_TYPE>` = 1
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Instead of ArduPilot controlling both the desired yaw rate and stability, this option relies on an external gyro for stabilization. A servo is still used to input yaw rate demands to the tail and its setup as above, for the Servo Only case.

The external gyro gain can be set by the :ref:`H_GYR_GAIN<H_GYR_GAIN>` parameter using the autopilot's "Motor7" output function,``SERVOx_FUNCTION``.

In ACRO mode, this gain can be changed to the value of the :ref:`H_GYR_GAIN_ACRO<H_GYR_GAIN_ACRO>` parameter, if non-zero.

Direct Drive Variable Pitch :ref:`H_TAIL_TYPE <H_TAIL_TYPE>` = 2
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

The Direct Drive Variable Pitch (DDVP) tail type uses a separate tail motor with a variable pitch propeller.  A separate servo is used to control the tailrotor pitch.  The setup of the servo is similar to the servo only tail type. ``SERVOx_FUNCTION`` "Motor4" is assigned to the output number that corresponds to the servo channel the tailrotor servo is physically connected (defaults to output 4).  For the tail motor, the "HeliTailRSC" ("32") is assigned to the motor/servo channel to which the tail motor ESC is physically connected (defaults to output 7).

The range of the motor ESC is controlled by its output's ``SERVOx_MIN/MAX`` parameters and ArduPilot will ramp it output up or down, as appropriate, to its normal operating value set by the :ref:`H_TAIL_SPEED<H_TAIL_SPEED>` parameter.

Direct Drive Fixed Pitch :ref:`H_TAIL_TYPE <H_TAIL_TYPE>` = 3 or 4
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

The Direct Drive Fixed Pitch (DDFP) tail type uses a fixed pitch tail rotor and separate motor. The stabilization and yaw is controlled by the speed of this motor. It has two options: one where the main rotor rotates clockwise when viewed from above and the other where the main rotor rotates counter-clockwise when viewed from above.  Be sure to select the DDFP tail type for the main rotor rotation.  In this case, the control of tailrotor thrust is accomplished through tailrotor speed since it is a fixed pitch propeller. The ``SERVOx_FUNCTION`` of "32" (HeliTailRSC) should be assigned to the servo channel (default is output 7) to which the tailrotor ESC is physically connected.

There are several parameters that provide the ability to linearize the thrust produced by the tail rotor motor and therefore provide better control:

- :ref:`H_DDFP_THST_EXPO<H_DDFP_THST_EXPO>` - Tail rotor DDFP motor thrust curve exponent (0.0 for linear to 1.0 for second order curve). Default = 0.55
- :ref:`H_DDFP_SPIN_MIN<H_DDFP_SPIN_MIN>` - Point at which the DDFP motor thrust starts expressed as a number from 0 to 1 in the entire output range.  Default = 0.95
- :ref:`H_DDFP_SPIN_MAX<H_DDFP_SPIN_MAX>` - Point at which the DDFP motor thrust saturates expressed as a number from 0 to 1 in the entire output range. Default = 0.15
- :ref:`H_DDFP_BAT_IDX<H_DDFP_BAT_IDX>` - Index of battery to be used for voltage compensation. Default = 0.
- :ref:`H_DDFP_BAT_V_MAX<H_DDFP_BAT_V_MAX>` - Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust). Recommend 4.2 * cell count, 0 = Disabled. Default = 0.
- :ref:`H_DDFP_BAT_V_MIN<H_DDFP_BAT_V_MIN>` - Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust). Recommend 3.3 * cell count, 0 = Disabled. Default = 0.

These parameters should be set similarly to how Copter's motor scaling parameterss are setup. See :ref:`Copters Motor Thrust Scaling <motor-thrust-scaling>` document for more information.

These Tail Type Connections are summarized below:

==============================  ============   ===============    =============
Type                            H_TAIL_TYPE    TailPitch Servo    TailMotor ESC
==============================  ============   ===============    =============
Servo only                      0                 Motor4          none
Servo with Gyro                 1                 Motor4          none
DirectDriveVariablePitch        2                 Motor4          HeliTailRSC
DirectDriveFixedPitch(CW)       3                 na              Motor4
DirectDriveFixesPitch(CCW)      4                 na              Motor4
==============================  ============   ===============    =============

Setting Tail Trim
=================

Setting the trim value of the Servo output is important to ensuring that the integrator offset of the tail rotor control loop is minimized, to maximize control range.  Collective to tailrotor compensation can also help with this and is discussed below.

For DDFP tails using V4.5 or later
++++++++++++++++++++++++++++++++++

If no :ref:`collective-to-tailrotor-compensation` is used, then it is recommended that the :ref:`H_YAW_TRIM<H_YAW_TRIM>` parameter is set to minimize the yaw I term in the hover.  To determine this:

- Ensure PID logging is switched on in the :ref:`LOG_BITMASK<LOG_BITMASK>` parameter.
- Hover the aircraft, maintaining altitude.  For best results, this is preferentially done on a light wind day. If possible, leave the aircraft drift with the wind and mintain a fixed height (ALT HOLD is very useful for this).
- After the flight, download the log and find the ``PIDY.I`` message.  Zoom in the on that portion of the flight with the hovering (should be a relatively flat line). Determine the average value of the I term. Enter this average value in the :ref:`H_YAW_TRIM<H_YAW_TRIM>` parameter.
- To confirm this has been done correctly repeat the flight.  Now, in the hovering portion of the flight, the yaw I Term should be close to zero.

If :ref:`collective-to-tailrotor-compensation` is used, then set the :ref:`H_YAW_TRIM<H_YAW_TRIM>` sufficient to compensate for the main rotor zero blade pitch drag.

Other Tail Types and DDFP using firmware before V4.5
++++++++++++++++++++++++++++++++++++++++++++++++++++

If no :ref:`collective-to-tailrotor-compensation` is used,  then it is recommended that the ``SERVOx_TRIM`` for the tailrotor servo is set to the PWM that corresponds to the tailrotor pitch required for hover, or the motor speed for DDFP. To determine this, hover the aircraft.  After the flight, pull the log and determine the average PWM value for the servo for hovering flight.  Either set that as the ``SERVOx_TRIM`` or mechanically adjust the tail pitch (non-DDFP tail types) for the tail pitch corresponding to the PWM.  Then set the ``SERVOx_TRIM`` to the servo midpoint. The latter approach is usually preferable.

If If no :ref:`collective-to-tailrotor-compensation` is used,  the set the ``SERVOx_TRIM`` for the PWM that corresponds to zero tailrotor pitch.  Or, the tailrotor pitch can be mechanically adjusted to zero pitch for the servo midpoint.

.. _collective-to-tailrotor-compensation:

Collective to Tailrotor Compensation
====================================

Collective to tailrotor compensation is used to remove the effects of the aircraft momentarily yawing when the collective pitch is changed rapidly and to minimize integrator offsets. 

In versions 4.3 and earlier, the parameter ``H_COLYAW`` was used.  This implementation assumed the tailrotor changed linearly with collective blade pitch.  In versions 4.4 and later, the parameter :ref:`H_COL2YAW<H_COL2YAW>` is used.  This implementation uses an accepted helicopter performance relationship between helicopter power required and weight.  Setting this parameter will only be valid for one rotor speed.  Set the tailrotor pitch so it is zero deg blade pitch at the ``SERVOx_TRIM`` value. If the rotor speed is changed then the parameter might require retuning.  The relationship uses collective to the 1.5 power to determine the tailrotor correction.  It is recommended to start at 0.5 and increase the parameter until there is little to no yawing when changing the collective pitch.  One other way would be to determine the yaw required for hovering as well as the collective and then calculate the value.

See also: :ref:`traditional-helicopter-aerobatic-setup`
