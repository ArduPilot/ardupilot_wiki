.. _traditional-helicopter-tailrotor-setup:

=========================================
Traditional Helicopter – Tailrotor Setup
=========================================

There are several ways for controlling the speed and/or pitch of the tailrotor and each have a unique setup.  The :ref:`H_TAIL_TYPE <H_TAIL_TYPE>` parameter is used to specify method for controlling the tailrotor.  A list of available tail types is given below:

- Servo Only (ArduPilot will supply the tail rotor stabilization like a tail rotor gyro)
- Direct Drive Fixed Pitch Clockwise (DDFP CW) (tail rotor driven by a motor whose ESC is controlled by ArudPilot)
- Direct Drive Fixed Pitch Counter-Clockwise (DDFP CCW) (tail rotor driven by a motor whose ESC is controlled by ArudPilot)
- Direct Drive Variable Pitch (DDVP) (both a tail rotor motor ESC and a servo controlled pitch link are used)

Tailrotor setups
================

Servo Only
++++++++++

The Servo Only tail type uses a servo to control the tailrotor pitch slider.  The tailrotor speed is controlled by a physical connection (tailrotor shaft or belt) to the engine and main shaft. In the Heli Page of the Mission Planner Setup Tab, motor 4 is assigned to the servo number that corresponds to the servo channel the tail rotor servo is physically connected.  Depending on your servo type, set the minimum, maximum and trim PWM.  Setting the trim PWM is discussed below.

Direct Drive Fixed Pitch
++++++++++++++++++++++++

The Direct Drive Fixed Pitch (DDFP) tail type accommodates two options: one where the main rotor rotates clockwise when viewed from above and the other where the main rotor rotates counter-clockwise when viewed from above.  Be sure to select the DDFP tail type for the main rotor rotation.  In this case, the control of tailrotor thrust is accomplished through tailrotor speed since it is a fixed pitch propeller. In the Heli Page of the Mission Planner Setup Tab, motor 4 is assigned to the servo number that corresponds to the servo channel the tailrotor ESC is physically connected.

There are several parameters that provide the ability to linearize the thrust produced by the tail rotor motor and therefore provide better control:

- :ref:`H_DDFP_THST_EXPO<H_DDFP_THST_EXPO>` - Tail rotor DDFP motor thrust curve exponent (0.0 for linear to 1.0 for second order curve). Default = 0.55
- :ref:`H_DDFP_SPIN_MIN<H_DDFP_SPIN_MIN>` - Point at which the DDFP motor thrust starts expressed as a number from 0 to 1 in the entire output range.  Default = 0.95
- :ref:`H_DDFP_SPIN_MAX<H_DDFP_SPIN_MAX>` - Point at which the DDFP motor thrust saturates expressed as a number from 0 to 1 in the entire output range. Default = 0.15
- :ref:`H_DDFP_BAT_IDX<H_DDFP_BAT_IDX>` - Index of battery to be used for voltage compensation. Default = 0.
- :ref:`H_DDFP_BAT_V_MAX<H_DDFP_BAT_V_MAX>` - Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust). Recommend 4.2 * cell count, 0 = Disabled. Default = 0.
- :ref:`H_DDFP_BAT_V_MIN<H_DDFP_BAT_V_MIN>` - Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust). Recommend 3.3 * cell count, 0 = Disabled. Default = 0.

These parameters should be set similarly to how MultiCopters motor scaling params are setup. See :ref:`Copters Motor Thrust Scaling <motor-thrust-scaling>` document for more information.

Direct Drive Variable Pitch
+++++++++++++++++++++++++++

The Direct Drive Variable Pitch (DDVP) tail type uses a tail motor with a variable pitch propeller.  A separate servo is used to control the tailrotor pitch.  The setup of the servo is similar to the servo only tail type. In the Heli Page of the Mission Planner Setup Tab, motor 4 is assigned to the servo number that corresponds to the servo channel the tailrotor servo is physically connected.  For the tail motor, the HeliTailRSC is assigned to the servo number that corresponds to the servo channel the tail motor ESC is physically connected.

Setting Tail Trim
=================

Setting the trim value of the Servo output is important to ensuring that the integrator offset of the tail rotor control loop is minimized, to maximize control range.  Collective to tailrotor compensation can also help with this and is discussed in the next section.

If no collective to tailrotor compensation is used, then it is recommended that the ``SERVOx_TRIM`` for the tailrotor servo is set to the PWM that corresponds to the tailrotor pitch required for hover, or the motor speed for DDFP. To determine this, hover the aircraft.  After the flight, pull the log and determine the average PWM value for the servo for hovering flight.  Either set that as the ``SERVOx_TRIM`` or mechanically adjust the tail pitch (non-DDFP tail types) for the tail pitch corresponding to the PWM.  Then set the ``SERVOx_TRIM`` to the servo midpoint. The latter approach is usually preferable.

If the collective to tail rotor compensation is used (see below) the set the ``SERVOx_TRIM`` for the PWM that corresponds to zero tailrotor pitch.  Or, the tailrotor pitch can be mechanically adjusted to zero pitch for the servo midpoint.

Collective to Tailrotor Compensation
====================================

Collective to tailrotor compensation is used to remove the effects of the aircraft momentarily yawing when the collective pitch is changed rapidly and to minimize integrator offsets. 

In versions 4.3 and earlier, the parameter ``H_COLYAW`` was used.  This implementation assumed the tailrotor changed linearly with collective blade pitch.  In versions 4.4 and later, the parameter :ref:`H_COL2YAW<H_COL2YAW>` is used.  This implementation uses an accepted helicopter performance relationship between helicopter power required and weight.  Setting this parameter will only be valid for one rotor speed.  Set the tailrotor pitch so it is zero deg blade pitch at the ``SERVOx_TRIM`` value. If the rotor speed is changed then the parameter might require retuning.  The relationship uses collective to the 1.5 power to determine the tailrotor correction.  It is recommended to start at 0.5 and increase the parameter until there is little to no yawing when changing the collective pitch.  One other way would be to determine the yaw required for hovering as well as the collective and then calculate the value.

See also: :ref:`traditional-helicopter-aerobatic-setup`
