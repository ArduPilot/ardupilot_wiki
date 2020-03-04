.. _tuning-cruise:

Tuning Cruise Configuration
===========================


Overview of Constant Altitude, Level Flight Operation
=====================================================

"Hands-off" constant altitude cruising occurs in FBWB, CRUISE, and the automatic throttle controlled modes like AUTO, GUIDED, CIRCLE, LOITER, RTL, etc.

In FBWB and CRUISE modes, without an airspeed sensor, the autopilot will set the target throttle at :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` with the throttle at mid-stick, and adjust pitch to hold altitude. Speed will be whatever results from this. Raising the throttle stick will increase throttle and therefore, speed.

Using an airspeed sensor, the autopilot will use throttle position to set the target airspeed as a linear interpolation between :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>` and :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>`. And pitch will be adjusted for constant altitude flight. 

In the automatic throttle controlled modes, :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` is used for the target airspeed if an airspeed sensor is being used, while :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` will be set for the throttle value if no sensor is used. The :ref:`THROTTLE_NUDGE<THROTTLE_NUDGE>` option allows the pilot to tweak this value while in flight with the throttle, if desired in these modes.

.. note:: While :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` is not used when using an airspeed sensor directly, it is important to set it at a working value, since it will be used in case of an airspeed sensor failure.

AHRS LEVEL Attitude
===================

During ACC Calibration, a "level" attitude position is set. Usually this is set to 2-6 degrees positive by blocking up the plane appropriately, such that the plane will have its wings at that small angle ("angle of attack" or AOA) to produce lift at the cruising speed to hold altitude.

This may not be what actually results in the above modes when cruising since the autopilot's priority to obtain the correct combination of pitch and throttle and/or speed to maintain level altitude flight.

While the autopilot will operate correctly with minor offsets to the calibrated "level" attitude, optimum performance will occur if this adjusted to match that pitch attitude. This can be done by examining the average ``ATT.pitch`` value during level altitude cruise from dataflash logs, or visually if using an OSD or GCS, to adjust the horizon line level using slight adjustments to the :ref:`AHRS_TRIM_Y<AHRS_TRIM_Y>` parameter value. This also will allow STABILIZE and FBWA modes to hold altitude at the same throttle level as used in the other modes.

.. note:: It is important that the :ref:`STAB_PITCH_DOWN<STAB_PITCH_DOWN>` parameter be set correctly. This parameter will add "nose-down" trim when the throttle is lowered in pilot throttle controlled, stablized modes, such as FBWA, to prevent the autopilot from holding the nose up as the plane slows down and potentially stalling. This can be tested at altitude in FBWA mode by moving the throttle to idle and checking that there is sufficient airspeed in a turn to avoid stalling (be prepared to do a stall recovery which might occur). Increase the value of :ref:`STAB_PITCH_DOWN<STAB_PITCH_DOWN>` , if necessary.

Adjusting CRUISE Mode Speed
===========================

When using an airspeed sensor, cruise speed can be directly controlled with the throttle stick position. Mid throttle will set the speed as halfway between :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>` (high stick) and :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>` (low stick).

Without an airspeed sensor, the :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` parameter would need to be changed appropriately for the desired mid-stick cruise speed. 

