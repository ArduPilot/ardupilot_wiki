.. _quadplane-parameters:

=========================
QuadPlane Parameter setup
=========================

All QuadPlane specific parameters start with a "Q\_" prefix. The
parameters are very similar to the equivalent Copter parameters so if
you are familiar with those you should find setting up a QuadPlane is
easy.

Key Parameters
==============

-  To enable QuadPlane functionality you need to set the :ref:`Q_ENABLE<Q_ENABLE>`
   parameter to 1 and then refresh the parameter list
-  The :ref:`Q_M_PWM_MIN<Q_M_PWM_MIN>` and :ref:`Q_M_PWM_MAX<Q_M_PWM_MAX>` parameters used to set the
   PWM range of the VTOL motors (MOTORx) and the SERVOx_MIN/MAX for the outputs driving these motors is ignored. These need to be set to the range
   your ESCs expect.
-  The most critical tuning parameters are :ref:`Q_A_RAT_RLL_P<Q_A_RAT_RLL_P>` and
   :ref:`Q_A_RAT_PIT_P<Q_A_RAT_PIT_P>`. These default to 0.25 but you may
   find significantly higher values are needed for a QuadPlane.
-  The :ref:`Q_M_SPIN_ARM<Q_M_SPIN_ARM>` parameter is important for getting the right
   level of motor output when armed in a quad mode
-  It is recommended that you set :ref:`ARMING_RUDDER<ARMING_RUDDER>` to 2 to allow for
   rudder disarm. Alternatively you could have :ref:`MANUAL <manual-mode>`
   as one of your available flight modes (as that will shut down the
   quad motors). Please be careful not to use hard left rudder and zero
   throttle while flying or you risk disarming your motors.
-  The default :ref:`SCHED_LOOP_RATE<SCHED_LOOP_RATE>` for a QuadPlane is to 300 (Hz). Most QuadPlanes do not need this to be rasied. Some very small vehicles (< 1Kg) might benefit from setting it to 400. In heavier vehicles, their higher inertia results in lower effective control response rates, so they do not benefit from a higher loop rate. Raising above 300 only leads to larger log files in these vehicles.

.. _return_behavior_setup:

Return Behavior Setup Guide:
============================

While there are many parameters setting distances and heights for the various home return modes/behaviors (read about failsafes in :ref:`quadplane-flying` and :ref:`QRTL and RTL modes <quadplane-flight-modes>`), this is a quick setup guide for basic behaviors.

RTL mode
--------

Is a fixed wing return mode which normally flys back to the home point and loiters, but can optionally do an automatic mission sequence, usually set by the user to land the vehicle at home in fixed wing mode.

If entered from VTOL flight several other behavior options can be selected by the :ref:`Q_RTL_MODE<Q_RTL_MODE>` parameter.

QRTL mode
---------

When operating close to home is a VTOL return and the land at home, but further away will switch to fixed wing flight until back closer to home, and then transition back to VTOL and land at home. QRTL mode always results in a VTOL landing at home unless the pilot interrupts it.

RC Failsafe
-----------

Loss of RC link can switch to a return flight mode or several other behaviors.

Setup
-----

1.If you lose RC link for greater than :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` in fixed wing flight set :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` value below to obtain various behavior options:

- "0" : do nothing if in AUTO mode, otherwise switch to RTL mode (see #3,4 below)
- "1" : switch to RTL mode (see #3,4 below)
- "2" : cut throttle and glide in FWBA mode
- "3" : deploy parachute (assuming you have one setup)
- "4" : switch to AUTO mode and execute mission at current mission sequence pointer

2. If you lose RC link for greater than :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` in VTOL flight, you will immediately QLAND, unless you select the following optional behavior:

- set :ref:`Q_OPTIONS<Q_OPTIONS>` bit 5 to switch to QRTL mode instead. (see #5 below)
- set :ref:`Q_OPTIONS<Q_OPTIONS>` bit 20 to switch to RTL mode instead (see #3,4 below). If bit 5 is set above, it will be ignored in lieu of this option bit.

3. Anytime you switch to RTL (due either to manual mode change or failsafe action), do you want to execute an autoland sequence (does not need to actually have a land command, if some other action is desired), rather than just return and loiter around home?

- If yes,then set up a DO_LAND_START mission sequence and enable the :ref:`RTL_AUTOLAND<RTL_AUTOLAND>` parameter. See :ref:`do_land_start` for details of setup.

4. If in a VTOL mode, and you switch to RTL (due either to manual mode change or failsafe action), then set the :ref:`Q_RTL_MODE<Q_RTL_MODE>` parameter value as follows to determine the behavior:

- "0" : Switch to normal RTL mode, transitioning to fixed wing (see #3 above)
- "1" : Transition to fixed wing, fly towards home, transition back to VTOL mode when close to home, move to over home, switch to QLAND and land at home in VTOL.
- "2" : Transition to fixed wing, fly towards home, loiter down to altitude around home, turn into the wind, transition back to VTOL mode and move to over home, switch to QLAND and land at home in VTOL
- "3" : Switch to QRTL :Transition to fixed wing, and do a special approach to home including "airbraking",  transition back to VTOL mode, move to over home, switch to QLAND and land at home in VTOL.

5. When switching to QRTL  default behavior is to transition to fixed wing if in VTOL (assuming you are not close to home already in VTOL Flight), flying back to home, then switching back to VTOL as you approach home, switching to QLAND over home, and landing at home (see :ref:`qrtl-mode` for more information). You can disable the fixed wing approach, and return home and land only using VTOL mode if :ref:`Q_OPTIONS<Q_OPTIONS>` bit 16 set.


Q_OPTIONS
=========
In addition, the behavior of QuadPlane can be modified by the setting of the :ref:`Q_OPTIONS<Q_OPTIONS>` bitmask parameter (no bits are set, by default):

- bit 0 (+1), if set, will force the transition from VTOL to Plane mode to keep the wings level and not begin climbing with the VTOL motors (as in a mission to a higher waypoint after VTOL takeoff) during the transition.
- bit 1 (+2), if set, will use a fixed wing takeoff instead of a VTOL takeoff for ground stations that can only send TAKEOFF instead of a separate VTOL_TAKEOFF mission command. Otherwise, QuadPlane will use VTOL takeoffs for a TAKEOFF mission command.
-  bit 2 (+4), if set, will use a fixed wing landing instead of a VTOL landing for ground stations that can only send LAND instead of a separate VTOL_LAND mission command. Otherwise, QuadPlane will use VTOL_LAND for a LAND mission command.
-  bit 3 (+8), if set, will interpret the takeoff altitude of a mission VTOL_TAKEOFF as specified when setup in Mission Planner (ie Relative to Home/Absolute {ASL}/Terrain {AGL}). Otherwise, it is relative to the takeoff point's altitude (AGL).
-  bit 4(+16), if set, for “Always use FW spiral approach”  then during a VTOL_LAND mission command,instead of transitioning to VTOL flight and doing a VTOL landing, it will remain in plane mode, and proceed to the landing position, climbing or descending to the altitude set in the VTOL_LAND waypoint. When it reaches within :ref:`Q_FW_LND_APR_RAD<Q_FW_LND_APR_RAD>` of the landing location, it will perform a LOITER_TO_ALT to finish the climb or descent to that altitude set in the waypoint, then, turning into the wind, transition to VTOL mode and proceed to the landing location and land. Otherwise, a standard VTOL_LAND will be executed. See :ref:`quadplane-auto-mode` for more information.
-  bit 5(+32), if set,  it will replace QLAND with QRTL for failsafe actions when in VTOL modes. See the Radio and Throttle Failsafe section of :ref:`quadplane-flying` for more information.
-  bit 6(+64), if set, will enforce the ICE idle governor even in MANUAL mode.
-  bit 7(+128), if set, will force QASSIST to be active at all times in VTOL modes. See :ref:`Assisted Fixed-Wing Flight<assisted_fixed_wing_flight>`.
-  bit 8(+256), if set, QASSIST will only affect VTOL motors. If not set, QAssist will also use flying surfaces to stabilize(:ref:`Assisted Fixed-Wing Flight<assisted_fixed_wing_flight>` ).
-  bit 9(+512), if set, will enable AirMode (:ref:`airmode`) if armed via an RC switch. See :ref:`Auxiliary Functions<common-auxiliary-functions>` option value 41. This function has been deprecated as of Version 4.2. See new arming switch options in :ref:`common-auxiliary-functions`
-  bit 10(+1024), if set, will allow the tilt servos to move with rudder input in vectored tilt setups while disarmed to determine range of motion.
-  bit 11(+2048), if set, will delay VTOL motor spin up until 2 seconds after arming.
-  bit 12(+4096), if set, disable speed based Qassist when using synthetic airspeed
-  bit 13(+8192), if set, will disable Ground Effect Compensation of baro due to ground effect pressures
-  bit 14(+16384), if set, ignore forward flight angle limits in Qmodes, otherwise :ref:`PTCH_LIM_MAX_DEG<PTCH_LIM_MAX_DEG>`, :ref:`PTCH_LIM_MIN_DEG<PTCH_LIM_MIN_DEG>`, and :ref:` ROLL_LIMIT_DEG<ROLL_LIMIT_DEG>` can constrain :ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>` in VTOL modes.
-  bit 15(+32768), if set, will allow pilot to control descent during VTOL AUTO-LAND phases, similar to throttle stick action during QHOVER or QLOITER. However, this will not become active until the throttle stick is raised above 70% during the descent at least once.
-  bit 16(+65536), if set, will disable the fixed wing approach in QRTL mode and VTOL_LANDING mission items, see Hybrid RTL modes section of :ref:`quadplane-flying` for details of this hybrid landing approach.
-  bit 17(+131072), if set, will enable pilot horizontal re-positioning during VTOL auto LAND phases, momentarily pausing the descent while doing so.
-  bit 18(+262144), if set, will only allow arming in VTOL and AUTO modes. This can be used for tailsitters to prevent arming in a fixed wing mode when sitting in VTOL stance to prevent tip-overs. For AUTO mode, WP must be a VTOL takeoff in order to arm with this option.
-  bit 19(+524288), if set, will allow the forcing of VTOL to Fixed Wing transitions if :ref:`Q_TRANS_FAIL<Q_TRANS_FAIL>` is not zero and exceeded, and if the airspeed is greater than 1/2 of :ref:`AIRSPEED_MIN<AIRSPEED_MIN>`, then the transition to fixed wing will immediately complete, rather than taking the :ref:`Q_TRANS_FAIL_ACT<Q_TRANS_FAIL_ACT>` action. See :ref:`quadplane-transitions`.
-  bit 20(+1048576), if set overrides bit 5, if set, and forces an RTL on RC failsafe while in a VTOL mode. This is useful in over-water operations where either an QLAND or QRTL is undesirable.
-  bit 21(+2097152), if set tilts tilt motors up when disarmed in FW modes (except manual) to prevent ground strikes.

Behavior can be modified as well as by the :ref:`Q_RTL_MODE<Q_RTL_MODE>` and :ref:`Q_GUIDED_MODE<Q_GUIDED_MODE>` parameters.

.. warning:: If you set :ref:`INITIAL_MODE<INITIAL_MODE>` to a VTOL mode, then switch to a fixed wing (other than MANUAL/ACRO/TRAINING) before arming, you will effectively be in a VTOL transition and when armed, the motors will spin up, and move, if tilted, to vertical.

.. note::

   The QuadPlane code requires GPS lock for proper operation. This is
   inherited from the plane code, which disables inertial estimation of
   attitude and position if GPS lock is not available. Do not try to fly a
   QuadPlane indoors. It will not fly well!!!!

