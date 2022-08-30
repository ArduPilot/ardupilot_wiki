.. _drift-mode:

==========
Drift Mode
==========

This page provides tips for flying in Drift Mode and methods for tuning
your copter to fly optimally in Drift Mode.

Overview
========

-  Drift Mode allows the user to fly a multi-copter as if it were a
   plane with built in automatic coordinated turns.
-  The user has direct control of Yaw and Pitch, but Roll is controlled
   by the autopilot.  This allows the copter to be controlled very
   intuitively with a single control stick if using a Mode 2 transmitter
-  The user has completely manual control over the throttle as in
   :ref:`Stabilize mode <stabilize-mode>`.
-  **Drift Mode is available as of release 3.1 of the Copter firmware.**

..  youtube:: 0mdk2-sNXmg
    :width: 100%

How Drift Mode works
====================

-  You "fly" the MultiCopter with the right stick (on Mode 2
   controllers) controls Pitch and Yaw.
-  You use the left stick primarily for altitude control but not for yaw
   directly.
-  When you push the right stick forward or back the copter will pitch
   (and accelerate) in the appropriate direction.
-  When you push the right stick towards one side or the other the right
   or to the left the copter will turn in the direction specified.
-  The copter will also bank at the same time so as to make a
   coordinated turn in that direction.
-  When turning with the right stick yaw is automatically applied and
   sufficient roll is added to cancel the copters velocity in the roll
   axis.
-  This allows you to maintain a coordinated (non-skidding) turn.
-  Letting go of the sticks effectively turns on a speed brake in the
   Pitch axis that slows the copter to a stop over a two second period.
-  A copter in Drift Mode with the right stick in the center will
   loosely hold horizontal position (It will slowly drift in the wind.)
-  Pilot’s throttle input controls the average motor speed meaning that
   constant adjustment of the throttle is required to maintain
   altitude.  If the pilot puts the throttle completely down the motors
   will go to their minimum rate (MOT_SPIN_ARMED) and if the vehicle
   is flying it will lose attitude control and tumble.
-  Drift Mode relies on your GPS for control.
-  If you lose your GPS signal in flight while in Drift Mode, your
   copter will either land or enter altitude hold based on your
   failsafe_gps_enabled setting.
-  You should also be prepared to switch back to Stabilize Mode for
   manual recovery if necessary.

Whats it Useful For
===================

-  FPV flyers who are looking for a dynamic, plane like flight as well
   as loiter-like position hold.
-  New flyers who want to try a more intuitive and easy to learn flight
   mode.
-  Anybody who would like to try an easy to fly and easy to learn and
   very fun mode.
-  Photographers and especially videographers who want a smoother and
   more coordinated filming result.

Setting up Drift Mode
=====================

-  In the Mission Planner Configuration Section under flight modes
   select Drift Mode to apply to an appropriate switch setting.
-  More information on tuning Drift mode is expected to be forthcoming
   shortly as are additional enhancements to Drift Mode itself.
