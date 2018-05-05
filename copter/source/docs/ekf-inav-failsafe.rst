.. _ekf-inav-failsafe:

========================
EKF / DCM Check Failsafe
========================

Copter 3.2 adds a DCM heading check and an :ref:`EKF (Extended Kalman Filter - Pixhawk only) <common-apm-navigation-extended-kalman-filter-overview>` check
to catch flyaways caused by a bad heading estimate.

.. note::

   Starting in Copter 3.3 the EKF failsafe replaces the :ref:`GPS Failsafe <archived-gps-failsafe>`. 

When will it trigger?
=====================

The DCM check runs by default on all boards and will trigger when the
GPS implied heading and DCM's estimated heading disagree by at least 60
degrees (configurable with the DCM_CHECK_THRESH parameter) for a full
second.

The EKF check runs only on the Pixhawk and only when the EKF is being
used as the primary source for attitude and position estimates (i.e.
AHRS_EKF_USE = 1).  This check will trigger when the EKF's compass and
velocity "variance" are higher than 0.8 (configurable with
EKF_CHECK_THRESH parameter) for one second.  This "variance" increases
as the estimates become untrustworthy.  0 = very trustworthy, >1.0 =
very untrustworthy.  If both variances climb above the
EKF_CHECK_THRESH parameter (default is 0.8) the EKF/Inav failsafe
triggers.

What will happen when the failsafe triggers?
============================================

The Pixhawk's `LED will flash red-yellow, the tone-alarm will sound <https://www.youtube.com/watch?v=j-CMLrAwlco&feature=player_detailpage#t=60>`__.

If telemetry is attached "EKF variance" will appear on the HUD.

And EKF/DCM error will be written to the dataflash logs

If flying in a flight mode that does not require GPS nothing further
will happen but you will be unable to switch into an autopilot flight
mode (Loiter, PosHold, RTL, Guided, Auto) until the failure clears.

If flying in a mode that requires GPS (Loiter, PosHold, RTL, Guided,
Auto) the vehicle will switch to "pilot controlled" LAND.  Meaning the
pilot will have control of the roll and pitch angle but the vehicle will
descend, land and finally disarm it's motors.  The pilot can, like
always switch into a manual flight mode including Stabilize or AltHold
to bring the vehicle home.

Adjusting sensitivity or disabling the check
============================================

The DCM and EKF check and failsafe can be disabled by setting the
DCM_CHECK_THRESH or EKF_CHECK_THRESH to "0" through the Mission
Planner's Config/Tuning >> Full Parameter List.  Alternatively it can be
made less sensitive by increasing this parameter from 0.8 to 0.9 or
1.0.  The downside of increasing this parameter's value is that during a
flyaway caused by a bad compass or GPS glitching, the vehicle will fly
further away before the vehicle is automatically switched to LAND mode.

.. image:: ../images/ekfcheck_setupThroughMP.png
    :target: ../_images/ekfcheck_setupThroughMP.png

Video
=====

..  youtube:: zJbephAEFWQ
    :width: 100%

.. toctree::
    :maxdepth: 1

    GPS Failsafe and Glitch Protection <gps-failsafe-glitch-protection>
