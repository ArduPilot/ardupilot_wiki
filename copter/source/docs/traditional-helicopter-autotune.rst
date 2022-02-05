.. _traditional-helicopter-autotune:

===============================
Autotune
===============================
The autotune for tradheli is completely different from multicopter autotune.  It can tune any combination of feedforward (ATC_RAT_XXX_VFF), the rate gains (ATC_RAT_XXX_P and ATC_RAT_XXX_D), or angle P gain (ATC_ANG_XXX_P).  The tuning for rate gains begins with finding the maximum allowable value for the rate gains and then tunes them.  Knowing the maximum value enables the autotune feature to keep from creating an instability.

Before you start autotune, you must:

#. Reduce the vibrations in the control signals as low as possible
#. On a calm day, the heli will maintain a near driftless hover (< 1 m/s)

This will greatly improve your chances at a successful autotune.

Parameter Descriptions
======================
:ref:`Tune Sequence Bitmask<AUTOTUNE_SEQ>`
------------------------------------------

User can specify the tuning desired.  Individual gain tuning or combination of tuning several gains can be specified using the bitmask.  Rate P and Rate D are conducted together and always preceeded with determining max gains allowable.

+-----------------------------+------------------------------------+
| Bits                        | Values                             |
+=============================+====================================+
| +-------+-----------------+ | +---------+----------------------+ |
| | Bits  | Tune Type       | | | Values  | Tune Type            | |
| +=======+=================+ | +=========+======================+ |
| | 0     | Feedforward     | | | 1       | Feedforward          | |
| +-------+-----------------+ | +---------+----------------------+ |
| | 1     | Rate D & Rate P | | | 2       | Rate D* & Rate P     | |
| +-------+-----------------+ | +---------+----------------------+ |
| | 2     | Angle P         | | | 3       | Feedforward, Rate D*,| |
| +-------+-----------------+ | |         | & Rate P             | |
| | 3     | Max Gain        | | +---------+----------------------+ |
| +-------+-----------------+ | | 4       | Angle P              | |
|                             | +---------+----------------------+ |
|                             | | 5       | Feedforward & Angle P| |
|                             | +---------+----------------------+ |
|                             | | 6       | Rate D*, Rate P,     | |
|                             | |         | & Angle P            | |
|                             | +---------+----------------------+ |
|                             | | 7       | Feedforward, Rate D*,| |
|                             | |         | Rate P, & Angle P    | |
|                             | +---------+----------------------+ |
|                             | | 8       | Max Gain             | |
|                             | +---------+----------------------+ |
+-----------------------------+------------------------------------+

Max gain determination is done before tuning Rate D and Rate P gains.

:ref:`Axes Bitmask<AUTOTUNE_AXES>`
----------------------------------

Specifies one or more axes to be tuned.

+----------------------+---------------------------------+
| Bits                 | Values                          |
+======================+=================================+
| +-------+----------+ | +---------+-------------------+ |
| | Bits  | Axis     | | | Values  | Axes tested       | |
| +=======+==========+ | +=========+===================+ |
| | 0     | Roll     | | | 1       | Roll              | |
| +-------+----------+ | +---------+-------------------+ |
| | 1     | Pitch    | | | 2       | Pitch             | |
| +-------+----------+ | +---------+-------------------+ |
| | 2     | Yaw      | | | 3       | Roll & Pitch      | |
| +-------+----------+ | +---------+-------------------+ |
|                      | | 4       | Yaw               | |
|                      | +---------+-------------------+ |
|                      | | 5       | Roll & Yaw        | |
|                      | +---------+-------------------+ |
|                      | | 6       | Pitch & Yaw       | |
|                      | +---------+-------------------+ |
|                      | | 7       | Roll, Pitch & Yaw | |
|                      | +---------+-------------------+ |
+----------------------+---------------------------------+

:ref:`Maximum Response Gain<AUTOTUNE_GN_MAX>`
---------------------------------------------

Specifies the maximum response gain to be used to tune the Rate D, Rate P and Angle P gains.  The response gain is the output of the response divided by the input.  It is recommended that 1.8 to 2.0 be used for pitch and roll axes as these axes are typically more lightly damped.  It is recommended that 1.0 to 1.4 be used for the yaw axis.


:ref:`Minimum Test Frequency<AUTOTUNE_FRQ_MIN>`
-----------------------------------------------

Specifies the minimum frequency in radians per second used during the dwell or frequency sweeps.  For frequency sweeps, this will be the starting frequency.


:ref:`Maximum Test Frequency<AUTOTUNE_FRQ_MAX>`
-----------------------------------------------

Specifies the maximum frequency in radians per second used during the dwell or frequency sweeps.  For frequency sweeps, this will be the ending frequency.


:ref:`Velocity P Gain<AUTOTUNE_VELXY_P>`
----------------------------------------

Specifies P gain for velocity feedback.  This aids the autotune in maintaining aircraft position during the frequency sweeps and dwells.  Keep this at 0.1 unless the aircraft is drifting more than 10 meters during the dwell and frequency sweeps.  It only affects position holding while the aircraft is oscillating during these tests.  In between the oscillations, it may drift.  This gain will not help with that.


Preparing for Autotune
======================
Noisy Signals
-------------

Prior to starting the autotune, make sure that the noise in the control signals is reduced as low as possible.  The best way to do this is to use the harmonic notch filter.  Follow the instructions in :ref:`Helicopter Dynamic Notch Filter Setup<common-imu-notch-filtering-helicopter-setup>`.  

A good way to check the control signals is to set the LOG_BITMASK parameter so that the FAST ATTITUDE and PID messages are selected in addition to the default selections.  Use a GCS software like Mission Planner to view the PIDR.Act, PIDP.Act, and PIDY.Act.  The noise in these signals should be low.

ADD figure showing example of low noise 

Transmitter Setup
-----------------

Be sure to put the Autotune flight mode as one of the flight modes on your transmitter flight mode switch.  You don’t want to be reaching for the GCS to switch out of the autotune if your heli is not behaving properly.  You want to be able to switch modes instantly.

Initial Setup Tuning Parameters
-------------------------------

Below are the initial parameters values that should be used to start the tuning
of your helicopter. The helicopter will be easily controllable with just the FF set to
0.15 on pitch and roll in the event that you need to modify the tail settings
from the defaults.

+---------------------------------------------------------------------+-------+
| :ref:`ATC_ACCEL_P_MAX<ATC_ACCEL_P_MAX>`                             | 110000|
+---------------------------------------------------------------------+-------+
| :ref:`ATC_ACCEL_R_MAX<ATC_ACCEL_R_MAX>`                             | 110000|
+---------------------------------------------------------------------+-------+
| :ref:`ATC_ANG_PIT_P<ATC_ANG_PIT_P>`                                 | 4.5   |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_ANG_RLL_P<ATC_ANG_RLL_P>`                                 | 4.5   |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_PIT_D<ATC_RAT_PIT_D__AC_AttitudeControl_Heli>`        | 0     |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_PIT_FLTD<ATC_RAT_PIT_FLTD__AC_AttitudeControl_Heli>`  | 0     |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_PIT_FLTE<ATC_RAT_PIT_FLTE__AC_AttitudeControl_Heli>`  | 0     |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_PIT_FLTT<ATC_RAT_PIT_FLTT__AC_AttitudeControl_Heli>`  | 20    |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_PIT_I<ATC_RAT_PIT_I__AC_AttitudeControl_Heli>`        | 0.1   |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_PIT_ILMI<ATC_RAT_PIT_ILMI>`                           | 0.05  |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_PIT_IMAX<ATC_RAT_PIT_IMAX__AC_AttitudeControl_Heli>`  | 0.40  |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_PIT_P<ATC_RAT_PIT_P__AC_AttitudeControl_Heli>`        | 0     |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_PIT_VFF<ATC_RAT_PIT_VFF>`                             | 0.15  |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_RLL_D<ATC_RAT_RLL_D__AC_AttitudeControl_Heli>`        | 0     |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_RLL_FLTD<ATC_RAT_RLL_FLTD__AC_AttitudeControl_Heli>`  | 0     |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_RLL_FLTE<ATC_RAT_RLL_FLTE__AC_AttitudeControl_Heli>`  | 0     |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_RLL_FLTT<ATC_RAT_RLL_FLTT__AC_AttitudeControl_Heli>`  | 20    |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_RLL_I<ATC_RAT_RLL_I__AC_AttitudeControl_Heli>`        | 0.1   |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_RLL_ILMI<ATC_RAT_RLL_ILMI>`                           | 0.05  |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_RLL_IMAX<ATC_RAT_RLL_IMAX__AC_AttitudeControl_Heli>`  | 0.40  |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_RLL_P<ATC_RAT_RLL_P__AC_AttitudeControl_Heli>`        | 0     |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_RLL_VFF<ATC_RAT_RLL_VFF>`                             | 0.15  |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_INPUT_TC<ATC_INPUT_TC>`                                   | 0.15  |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_ACCEL_Y_MAX<ATC_ACCEL_Y_MAX>`                             | 80000 |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_ANG_YAW_P<ATC_ANG_YAW_P>`                                 | 4.5   |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_YAW_D<ATC_RAT_YAW_D__AC_AttitudeControl_Heli>`        | 0.003 |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_YAW_FLTD<ATC_RAT_YAW_FLTD__AC_AttitudeControl_Heli>`  | 0     |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_YAW_FLTE<ATC_RAT_YAW_FLTE__AC_AttitudeControl_Heli>`  | 0     |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_YAW_FLTT<ATC_RAT_YAW_FLTT__AC_AttitudeControl_Heli>`  | 20    |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_YAW_I<ATC_RAT_YAW_I__AC_AttitudeControl_Heli>`        | 0.12  |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_YAW_ILMI<ATC_RAT_YAW_ILMI>`                           | 0     |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_YAW_IMAX<ATC_RAT_YAW_IMAX__AC_AttitudeControl_Heli>`  | 0.33  |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_YAW_P<ATC_RAT_YAW_P__AC_AttitudeControl_Heli>`        | 0.18  |
+---------------------------------------------------------------------+-------+
| :ref:`ATC_RAT_YAW_VFF<ATC_RAT_YAW_VFF>`                             | 0.0   |
+---------------------------------------------------------------------+-------+

Autotune Flights
================
Suggested Tuning Sequence
-------------------------

In any axis, it is recommended to conduct the feedforward gain tuning first, then the Rate D and Rate P gain tuning, and lastly the Angle P tuning.  Don't try to tune every axis in one flight, it could take up to 10-15 min to tune one axis.  Here is a suggested flights and tuning sequence. Yaw feedforward tuning is not needed for conventional helicopters.  The yaw feedforward gain can be set to zero.  It may be needed for dual helicopters though.

+--------+-----------------------------------------+-----------------------+
| Flight | Parameter                               | Value                 |
+========+=========================================+=======================+
|    1   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES>`     | 3 (Roll and Pitch)    |
|        +-----------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`       | 1 (feedforward)       |
+--------+-----------------------------------------+-----------------------+
|    2   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES>`     | 2 (Pitch)             |
|        +-----------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`       | 2 (Rate D and Rate P) |
|        +-----------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>` | 1.8                   |
+--------+-----------------------------------------+-----------------------+
|    3   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES>`     | 1 (Roll)              |
|        +-----------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`       | 2 (Rate D and Rate P) |
|        +-----------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>` | 1.8                   |
+--------+-----------------------------------------+-----------------------+
|    4   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES>`     | 3 (Roll and Pitch)    |
|        +-----------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`       | 4 (Angle P)           |
|        +-----------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>` | 2.0                   |
+--------+-----------------------------------------+-----------------------+
|    5   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES>`     | 4 (Yaw)               |
|        +-----------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`       | 2 (Rate D and Rate P) |
|        +-----------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>` | 1.0                   |
+--------+-----------------------------------------+-----------------------+
|    6   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES>`     | 4 (Yaw)               |
|        +-----------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`       | 4 (Angle P)           |
|        +-----------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>` | 1.2                   |
+--------+-----------------------------------------+-----------------------+

Tuning Flight Procedures
------------------------

When conducting an autotune flight, be sure to have at least at 50 meter by 50 meter area to fly.  During the feedforward test, the aircraft will travel 5-10 meters.  You may have to bring it back after each iteration of the tuning test.  Do not let the aircraft get too far away.

#. Power up the controller
#. Set the flight mode to either stabilize or AltHold (Althold recommended)
#. Enable motor interlock and allow rotors to complete runup
#. Lift off and establish stable hover approximately 3-5 meters above the ground
#. Switch into Autotune and center all sticks
#. Autotune will start conducting the maneuvers

.. note::

   If you don’t see anything happening, then your sticks are not centered

#. After the tuning is complete, a message will appear in the GCS saying Autotune complete
#. To test the settings, switch out of autotune and then back into autotune and you will be 
   able to test the settings that were tuned.
#. Once you are finished testing, descend and land in AutoTune.  Once the aircraft has landed, the 
   engine will shutdown on its own.  At that point flip your motor interlock switch to disabled
   and disarm the aircraft.

.. note::

   Aircraft must be disarmed in the autotune flight mode to save the gain settings.

Tuning Maneuver Descriptions
----------------------------

VFF Tuning
++++++++++

        During VFF tuning the aircraft may drift, reposition the aircraft as needed to keep it from drifting.  Making any inputs during this test will stop the tuning and won’t begin again unless the sticks are centered.

Rate D and Rate P Tuning
++++++++++++++++++++++++

        During this tuning, you can’t make any inputs to hold position during the tuning.  If you make any inputs, then it will stop the tuning and wait until you center the sticks before it begins again.  The aircraft will drift some but shouldn’t drift too far (< 50 m).  The sweeps are 23 seconds.  

Angle P Tuning
++++++++++++++

        During Angle P tuning, you may make small inputs in the pitch and roll axes only to keep the aircraft from drifting while it is oscillating.  Try to just bias the stick in one direction (slow inputs) to keep the aircraft from drifting.  Don’t make inputs to counter the oscillations.
