.. _tuning-cruise:

===========================
Tuning Cruise Configuration
===========================


Overview of Constant Altitude, Level Flight Operation
=====================================================

"Hands-off" constant altitude hold cruising occurs in automatic throttle modes such as FBWB, CRUISE, AUTO, GUIDED, CIRCLE, LOITER, RTL, etc. See :ref:`flight-modes` for the full list of automatic throttle modes.

In FBWB and CRUISE modes, without an airspeed sensor, the autopilot will set the target throttle at :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` with the throttle at mid-stick, and adjust pitch to hold altitude. The airspeed will be whatever results from the change in thrust. Raising the throttle stick will increase throttle and thereby airspeed. When using an airspeed sensor, the autopilot will use throttle position to set the target airspeed as a linear interpolation between :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>` and :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>`. And pitch will be adjusted for constant altitude flight. 

In the automatic throttle controlled modes, :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` is used for the target airspeed if an airspeed sensor is being used, while :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` will be set for the throttle value if no sensor is used. The :ref:`THROTTLE_NUDGE<THROTTLE_NUDGE>` option allows the pilot to tweak this value while in flight with the throttle, if desired in these modes.

.. warning:: While :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` is not used when using an airspeed sensor directly, it is important to set it at a working value, since it will be used in case of an airspeed sensor failure.

AHRS Level Attitude
===================

During :ref:`common-accelerometer-calibration`, a "level" attitude position is set with the wings perfectly level. Often for planes, the trim "level" position has the nose raised a few degrees. This is explained in the diagrams below and is known as the trim condition. Most planes fly with a few degrees of Angle of Attack (AOA) and while many planes will have some Angle of Incidence (i.e., the cord of the wing is at a positive angle to the fuselage cord) built-in, some do not, and some need a slightly higher AOA to fly at lower cruise speeds. If the level step of calibration is done with the plane's fuselage line level, initial flights will be safe, but the aircraft may lose altitude due to not having enough lift. There are a few options to produce more lift and improve the trim condition for FBWA mode:

- Add a few extra degrees nose up when during the Level step of 3-axis calibration.
- Add a few extra degrees nose up and use the  ``Calibrate Level`` button on the Mission Planner page to adjust the AHRS_TRIM parameters. AHRS_TRIM parameters can only change the difference between the autopilot's plane and "level" by 10 degrees maximum. If more is needed, (e.g., the autopilot is mounted slightly downward), then you can use :ref:`TRIM_PITCH_CD<TRIM_PITCH_CD>` to alter the AOA manually.
- Add a few degrees nose up to :ref:`TRIM_PITCH_CD<TRIM_PITCH_CD>`. This will also add an offset to the artificial horizon on a GCS or an OSD.
- Increase :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>`. (Increase :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` if an airspeed sensor or synthetic airspeed is not used)

While the autopilot will operate correctly with minor offsets to the calibrated "level" attitude, optimum performance will occur if this is adjusted to match the pitch attitude that the autopilot tends to use in automatic throttle modes. This can be done by examining the average ``ATT.pitch`` value during level altitude flight from dataflash logs or in-flight on an artificial horizon (e.g., GCS or OSD HUD). Make slight adjustments to the :ref:`TRIM_PITCH_CD<TRIM_PITCH_CD>` parameter value based on the value seen on the artificial horizon. Making these adjustments also will allow STABILIZE and FBWA modes to hold altitude at the same throttle level as used in the other modes.

.. image:: ../../../images/AOA.jpg

The autopilot's priority is to obtain the correct combination of elevator and throttle to maintain constant altitude flight. How the autopilot does this is detailed in :ref:`tecs-total-energy-control-system-for-speed-height-tuning-guide`.

The :ref:`STAB_PITCH_DOWN<STAB_PITCH_DOWN>` parameter will add "nose-down" trim when the throttle stick is lowered in pilot throttle controlled and stabilized modes, such as FBWA, to prevent the autopilot from holding the nose up as the plane slows down and potentially causing a stall. This can be tested, at altitude, in FBWA mode by moving the throttle to idle and checking that there is sufficient airspeed in a turn to avoid stalling. Be prepared to recover from a stall! Increase the value of :ref:`STAB_PITCH_DOWN<STAB_PITCH_DOWN>`, if necessary.

.. tip:: For small planes or gliders, it is important that :ref:`STAB_PITCH_DOWN<STAB_PITCH_DOWN>` be set correctly. It might be desireable to increase STAB_PITCH_DOWN more than the default value of 2 degrees.

Adjusting FBWB or CRUISE Mode Airspeed
======================================

When using an airspeed sensor in FBWB or CRUISE, the target airspeed can be directly controlled with the throttle stick position. Mid throttle will set the speed as halfway between :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>` (high stick) and :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>` (low stick). Without an airspeed sensor, the :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` parameter would need to be changed appropriately for the desired mid-stick cruise speed. 

