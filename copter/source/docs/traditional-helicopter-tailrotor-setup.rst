.. _traditional-helicopter-tailrotor-setup:

=========================================
Traditional Helicopter – Tailrotor Setup
=========================================

There are several ways for controlling the speed and/or pitch of the tailrotor and each have a unique setup.  The :ref:`H_TAIL_TYPE <H_TAIL_TYPE>` parameter is used to specify method for controlling the tailrotor.  A list of available tail types is given below:

- Servo Only (ArduPilot will supply the tail rotor stabilization like a tail rotor gyro)
- Direct Drive Fixed Pitch Clockwise (DDFP CW) (tail rotor driven by a motor whose ESC is controlled by ArduPilot)
- Direct Drive Fixed Pitch Counter-Clockwise (DDFP CCW) (tail rotor driven by a motor whose ESC is controlled by ArduPilot)
- Direct Drive Variable Pitch (DDVP) (both a tail rotor motor ESC and a servo controlled pitch link are used)

Tailrotor setups
================

Servo Only
++++++++++

The Servo Only tail type uses a servo to control the tailrotor pitch slider.  The tailrotor speed is controlled by a physical connection (tailrotor shaft or belt) to the engine and main shaft. In the Heli Page of the Mission Planner Setup Tab, motor 4 is assigned to the servo number that corresponds to the servo channel the tail rotor servo is physically connected.  Depending on your servo type, set the minimum, maximum and trim PWM.  Setting the trim PWM is discussed below.

Direct Drive Fixed Pitch
++++++++++++++++++++++++

The Direct Drive Fixed Pitch (DDFP) tail type accommodates two options: one where the main rotor rotates clockwise when viewed from above and the other where the main rotor rotates counter-clockwise when viewed from above.  Be sure to select the DDFP tail type for the main rotor rotation.  In this case, the control of tailrotor thrust is accomplished through tailrotor speed since it is a fixed pitch propeller. In the Heli Page of the Mission Planner Setup Tab, motor 4 is assigned to the servo number that corresponds to the servo channel the tailrotor ESC is physically connected.

From version 4.5 and onward, DDFP tails now have access to thrust linearisation and voltage scaling. These features are off by default. The parameters used to setup these features can be found with the prefix  ``H_DDRP_x``.  Configuration of these features is exactly the same as thrust linearisation for multicopter, details for which can be found: ref:`here<motor-thrust-scaling>`.

Direct Drive Variable Pitch
+++++++++++++++++++++++++++

The Direct Drive Variable Pitch (DDVP) tail type uses a tail motor with a variable pitch propeller.  A separate servo is used to control the tailrotor pitch.  The setup of the servo is similar to the servo only tail type. In the Heli Page of the Mission Planner Setup Tab, motor 4 is assigned to the servo number that corresponds to the servo channel the tailrotor servo is physically connected.  For the tail motor, the HeliTailRSC is assigned to the servo number that corresponds to the servo channel the tail motor ESC is physically connected.

Setting Tail Trim
=================

Setting the trim value of the Servo output is important to ensuring that the integrator offset of the tail rotor control loop is minimized, to maximize control range.  Collective to tailrotor compensation can also help with this and is discussed in the next section.

For DDFP tails on Version 4.5 and Onward
++++++++++++++++++++++++++++++++++++++++

If no collective to tailrotor compensation is used, then it is recommended that the :ref:`H_YAW_TRIM<H_YAW_TRIM>` parameter is set to minimise the yaw I term in the hover.  To determine this:

- Ensure PID logging is switched on in the :ref:`LOG_BITMASK<LOG_BITMASK>` parameter.
- Hover the aircraft, maintaining altitude.  For best results, this is preferentially done on a light wind day. If possible, leave the aircraft drift with the wind and mintain a fixed height (ALT HOLD is very useful for this).
- After the flight, download the log and find the ``PIDY.I`` message.  Zoom in the on that portion of the flight with the hovering (should be a relatively flat line). Determine the average value of the I term. Enter this average value in the :ref:`H_YAW_TRIM<H_YAW_TRIM>` parameter.
- To confirm this has been done correctly repeat the flight.  Now, in the hovering portion of the flight, the yaw I Term should be close to zero. 

If the collective to tail rotor compensation is used (see below) then set the :ref:`H_YAW_TRIM<H_YAW_TRIM>` for the I term that is sufficient to compensate for the main rotor zero blade pitch drag.

Other Tail Types and DDFP using Version 4.4.2 and Older
+++++++++++++++++++++++++++++++++++++++++++++++++++++++

If no collective to tailrotor compensation is used, then it is recommended that the ``SERVOx_TRIM`` for the tailrotor servo is set to the PWM that corresponds to the tailrotor pitch required for hover, or the motor speed for DDFP. To determine this, hover the aircraft.  After the flight, pull the log and determine the average PWM value for the servo for hovering flight.  Either set that as the ``SERVOx_TRIM`` or mechanically adjust the tail pitch (non-DDFP tail types) for the tail pitch corresponding to the PWM.  Then set the ``SERVOx_TRIM`` to the servo midpoint. The latter approach is usually preferable.

If the collective to tail rotor compensation is used (see below) the set the ``SERVOx_TRIM`` for the PWM that corresponds to zero tailrotor pitch.  Or, the tailrotor pitch can be mechanically adjusted to zero pitch for the servo midpoint.

Collective to Tailrotor Compensation
====================================

Collective to tailrotor compensation is used to remove the effects of the aircraft momentarily yawing when the collective pitch is changed rapidly and to minimize integrator offsets. 

In versions 4.3 and earlier, the parameter ``H_COLYAW`` was used.  This implementation assumed the tailrotor changed linearly with collective blade pitch.  In versions 4.4 and later, the parameter :ref:`H_COL2YAW<H_COL2YAW>` is used.  This implementation uses an accepted helicopter performance relationship between helicopter power required and weight.  Setting this parameter will only be valid for one rotor speed.  Set the tailrotor pitch so it is zero deg blade pitch at the ``SERVOx_TRIM`` value. If the rotor speed is changed then the parameter might require retuning.  The relationship uses collective to the 1.5 power to determine the tailrotor correction.  It is recommended to start at 0.5 and increase the parameter until there is little to no yawing when changing the collective pitch.  One other way would be to determine the yaw required for hovering as well as the collective and then calculate the value.

See also: :ref:`traditional-helicopter-aerobatic-setup`
