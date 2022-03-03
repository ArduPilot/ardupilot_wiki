.. _traditional-helicopter-tuning-preparing:

====================
Preparing for Tuning
====================

It is important to conduct the steps in this wiki to prepare for manual or autotune.  The following video covers the steps described on this wiki page.

..  youtube:: 3O5Y5L3damU

Reduce Noisy Control Signals
============================

Prior to starting to manually tune or AutoTune, make sure that the noise in the control signals is reduced as low as possible.  The best way to do this is to use the harmonic notch filter.  Follow the instructions in :ref:`Helicopter Dynamic Notch Filter Setup<common-imu-notch-filtering-helicopter-setup>`.  

A good way to check the control signals is to set the LOG_BITMASK parameter so that the FAST ATTITUDE and PID messages are selected in addition to the default selections.  Use a GCS software like Mission Planner to view the PIDR.Act, PIDP.Act, and PIDY.Act.  The noise in these signals should be low.  The example below shows the target roll rate (red) and actual roll rate (green).  The rates are given in rad/s.  Note the vibration (noise) in the actual roll rate with peaks reaching 0.2 to -0.2 rad/s. This would be a lot of noise and could cause the autotune to fail or cause a poor tune.  Ultimately, the deviation in the actual rates should be small (< 0.05 rad/s).  

 .. image:: ../images/tradheli-vibration-example.png

.. Note::  Before tuning, it is recommended to check that high frequency vibrations are not causing the "leans" (see :ref:`traditional-helicopter-tips`)

Ensure Helicopter does not drift in a Hover 
===========================================

Follow the procedures in the Hover Trim section of the :ref:`First Flight Tests<traditional-helicopter-first-flight-tests>` wiki page to ensure the heli maintains a near driftless hover (< 1 m/s).
  

Initial Setup of Parameters
===========================
Pitch and Roll Axes
-------------------

Below are the initial parameters values that should be used to start the tuning
of your helicopter. The helicopter will be easily controllable with just the FF set to
0.15 on pitch and roll in the event that you need to modify the tail settings
from the defaults.  

+----------------------------------------------------------------------+---------+
| :ref:`ATC_ACCEL_P_MAX<ATC_ACCEL_P_MAX>`                              | 110000  |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_ACCEL_R_MAX<ATC_ACCEL_R_MAX>`                              | 110000  |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_ANG_PIT_P<ATC_ANG_PIT_P>`                                  | 4.5     |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_ANG_RLL_P<ATC_ANG_RLL_P>`                                  | 4.5     |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_PIT_D<ATC_RAT_PIT_D__AC_AttitudeControl_Heli>`         | 0       |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_PIT_FLTD<ATC_RAT_PIT_FLTD__AC_AttitudeControl_Heli>`   | 0       |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_PIT_FLTE<ATC_RAT_PIT_FLTE__AC_AttitudeControl_Heli>`   | 20      |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_PIT_FLTT<ATC_RAT_PIT_FLTT__AC_AttitudeControl_Heli>`   | 20      |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_PIT_I<ATC_RAT_PIT_I__AC_AttitudeControl_Heli>`         | 0.1     |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_PIT_ILMI<ATC_RAT_PIT_ILMI>`                            | 0.05    |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_PIT_IMAX<ATC_RAT_PIT_IMAX__AC_AttitudeControl_Heli>`   | 0.40    |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_PIT_P<ATC_RAT_PIT_P__AC_AttitudeControl_Heli>`         | 0       |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_PIT_FF<ATC_RAT_PIT_FF__AC_AttitudeControl_Heli>`       | 0.15    |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_RLL_D<ATC_RAT_RLL_D__AC_AttitudeControl_Heli>`         | 0       |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_RLL_FLTD<ATC_RAT_RLL_FLTD__AC_AttitudeControl_Heli>`   | 0       |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_RLL_FLTE<ATC_RAT_RLL_FLTE__AC_AttitudeControl_Heli>`   | 20      |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_RLL_FLTT<ATC_RAT_RLL_FLTT__AC_AttitudeControl_Heli>`   | 20      |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_RLL_I<ATC_RAT_RLL_I__AC_AttitudeControl_Heli>`         | 0.1     |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_RLL_ILMI<ATC_RAT_RLL_ILMI>`                            | 0.05    |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_RLL_IMAX<ATC_RAT_RLL_IMAX__AC_AttitudeControl_Heli>`   | 0.40    |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_RLL_P<ATC_RAT_RLL_P__AC_AttitudeControl_Heli>`         | 0       |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_RAT_RLL_FF<ATC_RAT_RLL_FF__AC_AttitudeControl_Heli>`       | 0.15    |
+----------------------------------------------------------------------+---------+
| :ref:`ATC_INPUT_TC<ATC_INPUT_TC>`                                    | 0.15    |
+----------------------------------------------------------------------+---------+

Yaw Axis (Rudder)
-----------------

It is recommended to make sure the tail functions properly before proceeding
with tuning pitch and roll. Below are the suggested settings for yaw. 

.. Note::  UAV helicopters, as opposed to sport helicopters, will usually be running low headspeed and higher disc loading. With a mechanically driven tail this also means lower than normal tail speed and reduced tail authority. If your helicopter meets this description, it is recommended to set :ref:`ATC_RAT_YAW_FF<ATC_RAT_YAW_FF__AC_AttitudeControl_Heli>` to 0.05 before the first test hover.

+----------------------------------------------------------------------+----------+
| :ref:`ATC_ACCEL_Y_MAX<ATC_ACCEL_Y_MAX>`                              | 80000    |
+----------------------------------------------------------------------+----------+
| :ref:`ATC_ANG_YAW_P<ATC_ANG_YAW_P>`                                  | 4.5      |
+----------------------------------------------------------------------+----------+
| :ref:`ATC_RAT_YAW_D<ATC_RAT_YAW_D__AC_AttitudeControl_Heli>`         | 0.003    |
+----------------------------------------------------------------------+----------+
| :ref:`ATC_RAT_YAW_FLTD<ATC_RAT_YAW_FLTD__AC_AttitudeControl_Heli>`   | 0        |
+----------------------------------------------------------------------+----------+
| :ref:`ATC_RAT_YAW_FLTE<ATC_RAT_YAW_FLTE__AC_AttitudeControl_Heli>`   | 20       |
+----------------------------------------------------------------------+----------+
| :ref:`ATC_RAT_YAW_FLTT<ATC_RAT_YAW_FLTT__AC_AttitudeControl_Heli>`   | 20       |
+----------------------------------------------------------------------+----------+
| :ref:`ATC_RAT_YAW_I<ATC_RAT_YAW_I__AC_AttitudeControl_Heli>`         | 0.12     |
+----------------------------------------------------------------------+----------+
| :ref:`ATC_RAT_YAW_ILMI<ATC_RAT_YAW_ILMI>`                            | 0.0      |
+----------------------------------------------------------------------+----------+
| :ref:`ATC_RAT_YAW_IMAX<ATC_RAT_YAW_IMAX__AC_AttitudeControl_Heli>`   | 0.33     |
+----------------------------------------------------------------------+----------+
| :ref:`ATC_RAT_YAW_P<ATC_RAT_YAW_P__AC_AttitudeControl_Heli>`         | 0.18     |
+----------------------------------------------------------------------+----------+
| :ref:`ATC_RAT_YAW_FF<ATC_RAT_YAW_FF__AC_AttitudeControl_Heli>`       | 0.0      |
+----------------------------------------------------------------------+----------+
