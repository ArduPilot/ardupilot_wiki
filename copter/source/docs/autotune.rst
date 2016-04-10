.. _autotune:

========
AutoTune
========

This article explains how to use AutoTune on Copter.

Overview
========

AutoTune attempts to automatically tune the Stabilize P and Rate P and D
terms in order to provide the highest response without significant
overshoot. Copter needs to be "basically" flyable in :ref:`AltHold mode <altholdmode>` before attempting to use AutoTune as the feature
needs to be able to "twitch" the copter in the roll and pitch axis.

.. note::

   The AutoTune feature was introduced in Copter 3.1.

..  youtube:: js2GzeRysAc
    :width: 100%

How to invoke AutoTune
======================

#. Set up one flight mode switch position to be AltHold.
#. Set an :ref:`Auxiliary Function Switch <channel-7-and-8-options>`
   to Autotune to allow you to turn the auto tuning on/off with the a
   switch.\ |AutoTuneCh7Switch|
#. Ensure the ch7 or ch8 switch is in the LOW position.
#. Wait for a calm day and go to a large open area.
#. Take off and put the copter into AltHold mode at a comfortable
   altitude.
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

If you find after performing an AutoTune that the vehicle feels overly
twitchy when flying Stabilize, AltHold or PosHold (but ok in more
autonomous modes like Loiter, RTL, Auto) try reducing the RC_FEEL
parameter to 0.25.  This smooths out the pilot's input.

Copter-3.3 changes
==================

Copter 3.3 adds some additional features:

-  AutoTune can be setup as a flight-mode.  Switching into or out of the
   AutoTune flight mode responds in the same way as raising or lowering
   a ch7/ch8 aux switch high assigned the AutoTune function.
-  Yaw axis is also autotuned
-  AUTOTUNE_BITMASK allows control of which axis are to be tuned
   (useful if the vehicle's battery life is not long enough to complete
   all 3-axis).  "1" = tune roll, "2" = tune pitch, "4" = tune yaw.  Add
   these numbers together to tune multiple axis in a single session
   (i.e. "7" = tune all axis)
-  AUTOTUNE_AGGR : Should be in the range of 0.05 to 0.10. Controls the
   threshold for D-term bounce back and P-term overshoot. This affects
   the tuning noise immunity (a higher value is more tolerant to flex in
   the frame or other disturbances that could trick the tuning
   algorithm).  High values also leads to a tune that rejects external
   disturbances better.  Lower values result in a tune that is more
   responsive to pilot input.
-  Upon a succesful tune these additional values are saved:

   -  roll and pitch axis rate feed-forward is enabled
      (ATC_RATE_FF_ENABLE)
   -  roll, pitch and yaw acceleration limits are saved (ACCEL_R\_MAX,
      ACCEL_P\_MAX, ACCEL_Y\_MAX)

AutoTuning notes
================

-  AutoTune can r\ **equest very large and fast changes in output**\ s
   to the motors which can cause ESC sync issues especially when using
   SimonK firmware and/or low KV motors (under 500KV). See this
   `video showing a test <https://www.youtube.com/watch?v=hBUBbeyLe0Q>`_
   which recreates a sync problem.
-  AutoTune is sometimes unable to find a good tune for frames with very
   soft dampening on the APM or very flexible arms.
-  For best results the copter shouldn't be allowed to build up too much
   speed. This can be prevented by applying a quick correction between
   tests (twitches).
-  Be advised that AutoTune will engage in Stabilize, so don't
   accidentally flip your AutoTune switch until you are in AltHold and
   ready to begin the procedure.

.. tip::

   When reporting issues with AutoTune please include a description of
   your frame and a dataflash log of the flight.

Dataflash logging
=================

ATUN (auto tune overview) and ATDE (auto tune details) messages are
written to the dataflash logs. Some details of the contents of those
messages can be found on the :ref:`Downloading and Analyzing Data Logs in Mission Planner <common-downloading-and-analyzing-data-logs-in-mission-planner_message_details_copter_specific>`
wiki page.

.. |AutoTuneCh7Switch| image:: ../images/AutoTuneCh7Switch.png
    :target: ../_images/AutoTuneCh7Switch.png