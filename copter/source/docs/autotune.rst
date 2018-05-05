.. _autotune:

========
AutoTune
========

This article explains how to use AutoTune on Copter.

Overview
========

AutoTune attempts to automatically tune the Stabilize P, Rate P and D, and maximum rotational accelerations to provide the highest response without significant overshoot. Copter needs to be "basically" flyable in :ref:`AltHold mode <altholdmode>` before attempting to use AutoTune as the feature needs to be able to "twitch" the copter in the roll and pitch axis.

.. note::

   Position Hold during AutoTune is available in Copter 3.5 (and higher).

..  youtube:: js2GzeRysAc
    :width: 100%

Setup before flying
===================
#. Set up one flight mode switch position to be AltHold.
#. Set an :ref:`Auxiliary Function Switch <channel-7-and-8-options>`
   to Autotune to allow you to turn the auto tuning on/off with the a
   switch.\ |AutoTuneCh7Switch|
#. Remove the camera gimbal or any other parts of the frame that could wobble in flight
#. Select which combination of axis (roll, pitch, yaw) you wish to tune using the :ref:`AUTOTUNE_AXES <AUTOTUNE_AXES>` parameter
#. Set the autotune's aggressiveness using the :ref:`AUTOTUNE_AGGR <AUTOTUNE_AGGR>` parameter (0.1=agressive, 0.075=medium, 0.050=weak), normally start with the default 0.1.
#. For large copters (with props at least 13inch or 33cm diameter) set the Rate Roll and Pitch filters to 10hz (in Copter-3.3 these are RATE_RLL_FILT_HZ and RATE_PIT_FILT_HZ, in Copter-3.4 they are :ref:`ATC_RAT_RLL_FILT <ATC_RAT_RLL_FILT>`, :ref:`ATC_RAT_PIT_FILT <ATC_RAT_PIT_FILT>`)
#. It is recommended to enable :ref:`battery voltage scaling of PID gains <current-limiting-and-voltage-scaling>`

How to invoke AutoTune
======================
#. Wait for a calm day and go to a large open area.
#. Ensure the ch7 or ch8 switch is in the LOW position.
#. Take off and put the copter into AltHold mode at a comfortable
   altitude.
#. Face the vehicle so that it will twitch at 90degrees from the direction the wind is blowing (i.e. if tuning Roll first, point the vehicle into the wind)

   .. image:: ../images/autotune_copterwind.png
       :target: ../_images/autotune_copterwind.png
       :width: 500px
#. Set the ch7/ch8 switch to the HIGH position to engage auto tuning:

   -  You will see it twitch about 20 degrees left and right for a few
      minutes, then it will repeat forward and back.
   -  Use the roll and pitch stick at any time to reposition the copter
      if it drifts away (it will use the original PID gains during
      repositioning and between tests).  When you release the sticks it
      will continue auto tuning where it left off.
   -  Move the ch7/ch8 switch into the LOW position at any time to
      abandon the autotuning and return to the origin PIDs.
   -  Make sure that you do not have any trim set on your transmitter or
      the autotune may not get the signal that the sticks are centered.

#. When the tune completes the copter will change back to the original
   PID gains.
#. Put the ch7/ch8 switch into the LOW position then back to the HIGH
   position to test the tuned PID gains.
#. Put the ch7/ch8 switch into the LOW position to fly using the
   original PID gains.
#. If you are happy with the autotuned PID gains, leave the ch7/ch8
   switch in the HIGH position, land and disarm to save the PIDs
   permanently.

   If you DO NOT like the new PIDS, switch ch7/ch8 LOW to return to the
   original PIDs. The gains will not be saved when you disarm.

If you find after performing an AutoTune that the vehicle feels overly twitchy when flying Stabilize, AltHold or PosHold (but ok in more
autonomous modes like Loiter, RTL, Auto) try reducing the RC_FEEL parameter to 0.25.  This smooths out the pilot's input.
Alternatively try reducing the :ref:`AUTOTUNE_AGGR <AUTOTUNE_AGGR>` parameter (it should always be in the range 0.05 to 0.10) and try again.

If the vehicle feels sloppy after the AutoTune, try increasing the :ref:`AUTOTUNE_AGGR <AUTOTUNE_AGGR>` parameter as high as 0.10 and attempt the autotune again.

Invoke AutoTune with Position Hold
==================================

In Copter-3.5 (and higher) AutoTune performs a weak position hold if invoked from Loiter or PosHold flight modes (as opposed to AltHold).

   .. image:: ../images/autotune_from_loiter.png
       :target: ../_images/autotune_from_loiter.png
       :width: 400px

- The vehicle will gently lean (up to 10 degrees) towards a "target point" which is initially set to the vehicle's location at the moment AutoTune was invoked.
- The pilot can reposition the vehicle using the roll, pitch, yaw or throttle sticks.  The target position will be reset to the vehicle's location at the moment the pilot releases the roll and pitch sticks.
- In order to twitch perpendicular to the wind direction, the vehicle may suddenly rotate in either direction up to 90 degrees as it drifts 5m (or more) from the target location.
- If there is little or no wind, the vehicle's gentle position control may mean it moves back and forth, ping ponging around the target point changing yaw each time it strays more than 5m from the target.  In these cases it may be more comfortable to revert the simpler AltHold based AutoTune. 

Additional Notes
================

-  In Copter-3.3 (and higher) AutoTune can be setup as a flight-mode.  Switching into or out of the AutoTune flight mode responds in the same way as raising or lowering a ch7/ch8 aux switch high assigned the AutoTune function.
-   :ref:`AUTOTUNE_AXES <AUTOTUNE_AXES>` allows control of which axis are to be tuned.  This is useful if the vehicle's battery life is not long enough to complete all 3-axis).  "1" = tune roll, "2" = tune pitch, "4" = tune yaw.  Add these numbers together to tune multiple axis in a single session (i.e. "7" = tune all axis)
-   :ref:`AUTOTUNE_AGGR <AUTOTUNE_AGGR>` : Should be in the range of 0.05 to 0.10.  Higher values will produce a more aggressive tune but sometimes results in gains that are too high.  More specifically this parameter controls the threshold for D-term bounce back and P-term overshoot. This affects the tuning noise immunity (a higher value is more tolerant to flex in the frame or other disturbances that could trick the tuning algorithm).  High values also leads to a tune that rejects external disturbances better.  Lower values result in a tune that is more responsive to pilot input.

-   The full list of parameters that may be updated by AutoTune

        - Roll angular P gain :ref:`ATC_ANG_RLL_P <ATC_ANG_RLL_P>` (in AC3.3: STB_RLL_P)
        - Roll rate P, I and D gains :ref:`ATC_RAT_RLL_P <ATC_RAT_RLL_P>`, :ref:`ATC_RAT_RLL_I <ATC_RAT_RLL_I>`, :ref:`ATC_RAT_RLL_D <ATC_RAT_RLL_D>`  (in AC3.3: RATE_RLL_P, RATE_RLL_I, RATE_RLL_D)
        - Roll max acceleration :ref:`ATC_ACCEL_R_MAX <ATC_ACCEL_R_MAX>`
        - Pitch angular P gain :ref:`ATC_ANG_PIT_P <ATC_ANG_PIT_P>` (in AC3.3: STB_PIT_P)
        - Pitch rate P, I and D gains :ref:`ATC_RAT_PIT_P <ATC_RAT_PIT_P>`, :ref:`ATC_RAT_PIT_I <ATC_RAT_PIT_I>`, :ref:`ATC_RAT_PIT_D <ATC_RAT_PIT_D>`  (in AC3.3: RATE_PIT_P, RATE_PIT_I, RATE_PIT_D)
        - Pitch max acceleration :ref:`ATC_ACCEL_P_MAX <ATC_ACCEL_P_MAX>`
        - Yaw angular P gain :ref:`ATC_ANG_YAW_P <ATC_ANG_YAW_P>` (in AC3.3: STB_YAW_P)
        - Yaw rate P, I gain :ref:`ATC_RAT_YAW_P <ATC_RAT_YAW_P>`, :ref:`ATC_RAT_YAW_I <ATC_RAT_YAW_I>`, :ref:`ATC_RAT_YAW_D <ATC_RAT_YAW_D>` (in AC3.3: RATE_YAW_P, RATE_YAW_I, RATE_YAW_D)
        - Yaw rate filter :ref:`ATC_RAT_YAW_FILT <ATC_RAT_YAW_FILT>` (in AC3.3: RATE_YAW_FILT_HZ)
        - Yaw max acceleration :ref:`ATC_ACCEL_Y_MAX <ATC_ACCEL_Y_MAX>`
        - Roll and pitch axis rate feed-forward is enabled (:ref:`ATC_RATE_FF_ENABLE <ATC_RATE_FF_ENABLE>`)
-   After you have a good tune, you may wish to increase :ref:`ATC_THR_MIX_MAX <ATC_THR_MIX_MAX>` (or MOT_THR_MIX_MAX in Copter-3.3) to 0.9 (default is 0.5) to increase prioritisation of attitude control over throttle.  This can reduce the pitch overshoot sometimes seen (especially on copters with large propellers) in AltHold if the vehicle suddenly slows after performing fast forward flight.  In this situation wind catches under the propellers providing lift but also disturbs the vehicle's attitude leading to a conflict between throttle and attitude control.  The danger in increasing this parameter's value is that if the rate gains are later raised so high that the vehicle oscillates badly it may be difficult for the vehicle to descend (because it will prioritise trying to correct the attitude oscillations and never reduce throttle sufficiently).
-   AutoTune can **request very large and fast changes in output**\ s to the motors which can cause ESC sync issues especially when using SimonK firmware and/or low KV motors (under 500KV). See this `video showing a test <https://www.youtube.com/watch?v=hBUBbeyLe0Q>`__ which recreates a sync problem.
-   AutoTune is sometimes unable to find a good tune for frames with very soft vibration dampening of the flight controller or very flexible arms.
-   For best results the copter shouldn't be allowed to build up too much horizontal speed. This can be prevented by applying a quick correction between tests (twitches) to stop the vehicle from flying too fast.
-   Be advised that AutoTune will engage from Stabilize, so don't accidentally flip your AutoTune switch until you are in AltHold and ready to begin the procedure.

Common Problems
===============

- If the vehicle will not start tuning (i.e. it won't twitch) even though it is in AutoTune mode then the problem is likely that the roll, pitch, yaw or throttle sticks are not exactly in the middle. It may help to increase the deadzone on the RC input by increasing :ref:`RC1_DZ <RC1_DZ>`, :ref:`RC2_DZ <RC2_DZ>`, :ref:`RC3_DZ <RC3_DZ>` and :ref:`RC4_DZ <RC4_DZ>` to 50 (or higher).
- If the AutoTune produces an overly twitchy vehicle try reducing the :ref:`AUTOTUNE_AGGR <AUTOTUNE_AGGR>` parameter (should never be below 0.05) and perform the AutoTune again.
- If the AutoTune produces a sloppy vehicle, try increasing the :ref:`AUTOTUNE_AGGR <AUTOTUNE_AGGR>` parameter (should never be above 0.1) and perform the AutoTune again.

.. tip::

   When reporting issues with AutoTune please include a description of your frame and a dataflash log of the flight.

Dataflash logging
=================

ATUN (auto tune overview) and ATDE (auto tune details) messages are
written to the dataflash logs. Some details of the contents of those
messages can be found on the :ref:`Downloading and Analyzing Data Logs in Mission Planner <common-downloading-and-analyzing-data-logs-in-mission-planner_message_details_copter_specific>` wiki page.

.. |AutoTuneCh7Switch| image:: ../images/AutoTuneCh7Switch.png
    :target: ../_images/AutoTuneCh7Switch.png

