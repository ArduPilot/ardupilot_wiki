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
   PWM range of the quad motors (this allows them to be different from
   the range for the forward motor). These need to be set to the range
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
-  bit 14(+16384), if set, ignore forward flight angle limits in Qmodes, otherwise LIM_PITCH_CD and LIM_ROLL_CD can constrain Q_ANG_MAX in VTOL modes.
-  bit 15(+32768), if set, will allow pilot to control descent during VTOL AUTO-LAND phases, similar to throttle stick action during QHOVER or QLOITER. However, this will not become active until the throttle stick is raised above 70% during the descent at least once.
-  bit 16(+65536), if set, will disable the fixed wing approach in QRTL mode and VTOL_LANDING mission items, see Hybrid RTL modes section of :ref:`quadplane-flying` for details of this hybrid landing approach.
-  bit 17(+131072), if set, will enable pilot horizontal re-positioning during VTOL auto LAND phases, momentarily pausing the descent while doing so.
-  bit 18(+262144), if set, will only allow arming in VTOL and AUTO modes. This can be used for tailsitters to prevent arming in a fixed wing mode when sitting in VTOL stance to prevent tip-overs. For AUTO mode, WP must be a VTOL takeoff in order to arm with this option.
-  bit 19(+524288), if set, will allow the forcing of VTOL to Fixed Wing transitions if :ref:`Q_TRANS_FAIL<Q_TRANS_FAIL>` is not zero and exceeded, and if the airspeed is greater than 1/2 of :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>`, then the transition to fixed wing will immediately complete, rather than taking the :ref:`Q_TRANS_FAIL_ACT<Q_TRANS_FAIL_ACT>` action. See :ref:`quadplane-transitions`.
-  bit 20(+1048576), if set overrides bit 5, if set, and forces an RTL on RC failsafe while in a VTOL mode. This is useful in over-water operations where either an QLAND or QRTL is undesirable.
-  bit 21(+2097152), if set tilts tilt motors up when disarmed in FW modes (except manual) to prevent ground strikes.

Behavior can be modified as well as by the :ref:`Q_RTL_MODE<Q_RTL_MODE>` and :ref:`Q_GUIDED_MODE<Q_GUIDED_MODE>` parameters.

.. warning:: If you set :ref:`INITIAL_MODE<INITIAL_MODE>` to a VTOL mode, then switch to a fixed wing (other than MANUAL/ACRO/TRAINING) before arming, you will effectively be in a VTOL transition and when armed, the motors will spin up, and move, if tilted, to vertical.

.. note::

   The QuadPlane code requires GPS lock for proper operation. This is
   inherited from the plane code, which disables inertial estimation of
   attitude and position if GPS lock is not available. Do not try to fly a
   QuadPlane indoors. It will not fly well

