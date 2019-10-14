.. _quadplane-parameters:

QuadPlane Parameter setup
=========================

All QuadPlane specific parameters start with a "Q\_" prefix. The
parameters are very similar to the equivalent Copter parameters so if
you are familiar with those you should find setting up a QuadPlane is
easy.

Key parameters are:

-  To enable QuadPlane functionality you need to set the :ref:`Q_ENABLE<Q_ENABLE>`
   parameter to 1 and then refresh the parameter list
-  The :ref:`Q_THR_MIN_PWM<Q_THR_MIN_PWM>` and :ref:`Q_THR_MAX_PWM<Q_THR_MAX_PWM>` parameters used to set the
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

In addition, the behavior of Quadplane can be modified by the setting of the :ref:`Q_OPTIONS<Q_OPTIONS>` bitmask parameter (no bits are set, by default):

- bit 0, if set, will force the transition from VTOL to Plane mode to keep the wings level and not begin climbing with the VTOL motors (as in a mission to a higher waypoint after VTOL takeoff) during the transition.
- bit 1, if set, will use a fixed wing takeoff instead of a VTOL takeoff for ground stations that can only send TAKEOFF instead of a separate VTOL_TAKEOFF mission command. Otherwise, QuadPlane will use VTOL takeoffs for a TAKEOFF mission command.
-  bit 2, if set, will use a fixed wing landing instead of a VTOL landing for ground stations that can only send LAND instead of a separate VTOL_LAND mission command. Otherwise, QuadPlane will use VTOL_LAND for a LAND mission command.
-  bit 3, if set, will interpret the takeoff altitude of a mission VTOL_TAKEOFF as specified when setup in Mission Planner (ie Relative to Home/Absolute {ASL}/Terrain {AGL}). Otherwise, it is relative to the takeoff point's altitude (AGL).
-  bit 4, if set, for “Use a fixed wing approach”  then during a VTOL_LAND mission command,instead of transitioning to VTOL flight and doing a VTOL landing, it will remain in plane mode, and proceed to the landing position, climbing or descending to the altitude set in the VTOL_LAND waypoint. When it reaches within :ref:`Q_FW_LND_APR_RAD<Q_FW_LND_APR_RAD>` of the landing location, it will perform a LOITER_TO_ALT to finish the climb or descent to that altitude set in the waypoint, then, turning into the wind, transition to VTOL mode and proceed to the landing location and land. Otherwise, a standard VTOL_LAND will be executed. See :ref:`quadplane-auto-mode` for more information.
-  bit 5, if set,  it will replace QLAND with QRTL for failsafe actions when in VTOL modes. See the Radio and Throttle Failsafe section of :ref:`quadplane-flying` for more information.

Behavior can be modified as well as by the :ref:`Q_RTL_MODE<Q_RTL_MODE>` and :ref:`Q_GUIDED_MODE<Q_GUIDED_MODE>` parameters.

.. note::

   The QuadPlane code requires GPS lock for proper operation. This is
   inherited from the plane code, which disables inertial estimation of
   attitude and position if GPS lock is not available. Do not try to fly a
   QuadPlane indoors. It will not fly well

