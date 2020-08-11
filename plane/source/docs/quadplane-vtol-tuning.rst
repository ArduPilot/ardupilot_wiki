.. _quadplane-vtol-tuning:

=====================
QuadPlane VTOL Tuning
=====================

This section provides an overview of how to tune various QuadPlane VTOL parameters.

Overview
========

The default parameters controlling the VTOL motors PID loops should allow most frames to initially hover uncontrollably,  if the motors' mechanics are setup and aligned correctly and escs calibrated. 

The most important parameters controlling stability are the Roll/Pitch/Yaw PIDS. For altitude control, the vertical position controller's parameters and Motor Thrust Scaling parameters, and for navigation/loiter the Loiter controllers's parameters

Normally, it's best to start by tuning the Rate Roll/Pitch P in QSTABILIZE
mode then move onto tuning altitude hold in QHOVER mode, then QLOITER
(which often needs no tuning) and finally the waypoint navigation
performance in Auto mode.

Filter tuning
=============

QuadPlanes are often affected by vibration and tuning the various software filters available is critical to achieving an optimum overall tune. A guide on tuning the various notch filters available can be found on the :ref:`Notch Filtering wiki page <common-imu-notch-filtering>`. A optimum tune will be obtained on the D terms, below, if the filtering has been optimized first. However, in many circumstances reasonable performance can be obtained with just PID tuning the P terms. 


Roll/Pitch tuning
=================

The ``Q_A_RAT_RLL_x`` and ``Q_A_RAT_PIT_x`` Roll/Pitch Rate parameters which convert the desired rotation rate into a motor output are the most important. 

The :ref:`Q_A_ANG_RLL_P<Q_A_ANG_RLL_P>` and :ref:`Q_A_ANG_PIT_P<Q_A_ANG_PIT_P>` Roll/Pitch P converts the desired angle into a desired rotation rate which is then fed to the Rate controller.

-  A higher value will make the QuadPlane more responsive to roll/pitch
   inputs, a lower value will make it smoother
-  If set too high, the QuadPlane will oscillate on the roll and/or pitch
   axis
-  If set too low the QuadPlane will become sluggish to inputs

An objective view of the overall Roll and Pitch performance can be seen
by graphing the :ref:`dataflash log's <common-downloading-and-analyzing-data-logs-in-mission-planner>` ATT message's DesRoll vs Roll and DesPit vs Pitch. The "Roll" (i.e.
actual roll) should closely follow the "DesRoll" while in stabilized modes. Pitch should similarly closely follow DesPit.

Alternatively you may wish to try tuning both the rate and angle
parameters using the :ref:`QAUTOTUNE mode <qautotune-mode>`.

Yaw Tuning
==========

The Angle Yaw and Rate Yaw parameters control the yaw response. With QuadPlanes, these often need tuning to get the desired YAW response, since configurations vary widely.

Similar to roll and pitch, if either :ref:`Q_A_RAT_YAW_P<Q_A_RAT_YAW_P>` or :ref:`Q_A_ANG_YAW_P<Q_A_RAT_YAW_P>` is too high the QuadPlane's heading will oscillate. If they are too low, the QuadPlane may be unable to maintain its heading.

The :ref:`Q_A_ANG_YAW_P<Q_A_ANG_YAW_P>` is the gain on the error between the autopilots desired heading and actual heading which is fed into the Rate controller to demand a rotation rate. The :ref:`Q_A_RAT_YAW_P<Q_A_RAT_YAW_P>` is the gain applied to the difference between demanded rotation rate and actual.

The :ref:`Q_YAW_RATE_MAX<Q_YAW_RATE_MAX>` parameter controls how quickly QuadPlane rotates based on a pilot’s yaw input in stabilized modes. 

Altitude Tuning
===============

The QHOVER (altitude hold) related tuning parameters are related to the vertical position controller and the motor thrust scaling, which linearizes the throttle to motor thrust response to improve the position controllers response.

The :ref:`Q_P_POSZ_P<Q_P_POSZ_P>` parameter is used to convert the altitude error (the difference between the desired altitude and the actual altitude) to a desired climb or descent rate. A higher rate will make it more aggressively attempt to maintain it’s altitude but if set too high leads to a jerky throttle response.

The :ref:`Q_P_VELZ_P<Q_P_VELZ_P>` (which normally requires no tuning) converts the desired climb or descent rate into a desired acceleration up or down.

The :ref:`Q_P_ACCZ_P<Q_P_ACCZ_P>` ,:ref:`Q_P_ACCZ_I<Q_P_ACCZ_I>` ,:ref:`Q_P_ACCZ_D<Q_P_ACCZ_D>`   PID gains convert the acceleration error (i.e the difference between the desired acceleration and the actual acceleration) into a motor output. The 1:2 ratio of P to I (i.e. I is twice the size of P) should be maintained if you modify these parameters. These values should never be increased but for very powerful QuadPlane VTOL motors you may get better response by reducing both by 50% (i.e P to 0.5, I to 1.0).

Loiter Tuning
=============

Generally if Roll and Pitch are tuned correctly,  the
:ref:`GPS <common-diagnosing-problems-using-logs_gps_glitches>`
and :ref:`compass <common-diagnosing-problems-using-logs_compass_interference>`
are set-up and performing well and :ref:`vibration levels <common-diagnosing-problems-using-logs_vibrations>`
are acceptable, Loiter does not require much tuning but please see the
:ref:`Loiter Mode <qloiter-mode>` page for more details on tunable
parameters including the horizontal speed.

In-flight Tuning
================

Many parameters can be tuned while in flight, see :ref:`common-transmitter-tuning`


Video introduction to PIDs
==========================

PIDs (Proportional - Integral - Derivative) are the method used by our
firmware to continuously stabilize the vehicle

-  Proportional = Immediate Correction: The further off you are the
   bigger the correction you make.
-  Integral = Over time or steady state correction: If we are failing to
   make progress add additional correction.
-  Derivative = Take it Easy correction: Is the correction going to
   fast? if it is slow it down (dampen) it a bit to avoid overshoot.

..  youtube:: l03SioQ9ySg
    :width: 100%

..  youtube:: sDd4VOpOnnA
    :width: 100%

-----

.. image:: ../../../images/banner-freespace.png
   :target: https://freespace.solutions/
