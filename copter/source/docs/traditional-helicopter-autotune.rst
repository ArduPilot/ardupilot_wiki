.. _traditional-helicopter-autotune:

========
AutoTune
========
The AutoTune for tradheli is completely different from multicopter AutoTune.  It can tune any combination of feedforward (``ATC_RAT_xxx_FF``), 
the rate gains (``ATC_RAT_xxx_P`` and ``ATC_RAT_xxx_D``), or angle P gain (``ATC_ANG_xxx_P``).  The tuning for rate gains begins with finding the maximum allowable value for the rate gains and then tunes them.  Knowing the maximum value enables the AutoTune feature to keep from creating an instability.

Before you start AutoTune, you must:

#. Reduce the vibrations in the control signals as low as possible
#. On a calm day, the heli will maintain a near driftless hover (< 1 m/s)

This will greatly improve your chances at a successful AutoTune.

The following video covers the steps described in this wiki page.

..  youtube:: 5960K8EV13A

Parameter Descriptions
======================
:ref:`Tune Sequence Bitmask<AUTOTUNE_SEQ>`
------------------------------------------

User can specify the tuning desired using the :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>` parameter.  Individual gain tuning or combination of tuning several gains can be specified using the bitmask.  Rate P and Rate D are conducted together and always preceded with determining max gains allowable.

+-----------------------------------------------------+-----------------------------------------------------+
| Bits                                                | Values                                              |
+=====================================================+=====================================================+
| +-------+------------------------------------------+| +---------+----------------------------------------+|
| | Bits  | Tune Type                                || | Values  | Tune Type                              ||
| +=======+==========================================+| +=========+========================================+|
| | 0     | ``ATC_RAT_xxx_FF``                       || | 1       | ``ATC_RAT_xxx_FF``                     ||
| +-------+------------------------------------------+| +---------+----------------------------------------+|
| | 1     | ``ATC_RAT_xxx_D`` & ``ATC_RAT_xxx_P`` ** || | 2       | ``ATC_RAT_xxx_D`` & ``ATC_RAT_xxx_P``  ||
| +-------+------------------------------------------+| +---------+----------------------------------------+|
| | 2     | ``ATC_ANG_xxx_P``                        || | 3       | ``ATC_RAT_xxx_FF``, ``ATC_RAT_xxx_D``, ||
| +-------+------------------------------------------+| |         |  & ``ATC_RAT_xxx_P``                   ||
| | 3     | Max Gain Determination                   || +---------+----------------------------------------+|
| +-------+------------------------------------------+| | 4       | ``ATC_ANG_xxx_P``                      ||
|                                                     | +---------+----------------------------------------+|
|                                                     | | 5       | ``ATC_RAT_xxx_FF`` & ``ATC_ANG_xxx_P`` ||
|                                                     | +---------+----------------------------------------+|
|                                                     | | 6       | ``ATC_RAT_xxx_D``, ``ATC_RAT_xxx_P``,  ||
|                                                     | |         | & ``ATC_ANG_xxx_P``                    ||
|                                                     | +---------+----------------------------------------+|
|                                                     | | 7       | ``ATC_RAT_xxx_FF``, ``ATC_RAT_xxx_D``, ||
|                                                     | |         | ``ATC_RAT_xxx_P``, & ``ATC_ANG_xxx_P`` ||
|                                                     | +---------+----------------------------------------+|
|                                                     | | 8       | Max Gain Determination                 ||
|                                                     | +---------+----------------------------------------+|
+-----------------------------------------------------+-----------------------------------------------------+

** Max gain determination is always done before tuning Rate D and Rate P gains. This test determines the maximum allow values of ``ATC_RAT_xxx_D`` & ``ATC_RAT_xxx_P`` so that the helicopter never experiences servere oscillations due to raising these gains too high.  Since the maximum allowable values of ``ATC_RAT_xxx_D`` & ``ATC_RAT_xxx_P`` are found, tuning is done by incrementing these gains by 5% of their maximum allowable value.

:ref:`Axes Bitmask<AUTOTUNE_AXES__AC_AutoTune_Heli>`
-----------------------------------------------------------

The :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES__AC_AutoTune_Heli>` parameter specifies one or more axes to be tuned.

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

The :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>` parameter specifies the maximum response gain to be used to tune the Rate D, Rate P and Angle P gains.  The response gain is the output of the response divided by the input.  It is recommended that 1.8 to 2.0 be used for pitch and roll axes as these axes are typically more lightly damped.  It is recommended that 1.0 to 1.4 be used for the yaw axis.


:ref:`Minimum Test Frequency<AUTOTUNE_FRQ_MIN>`
-----------------------------------------------

The :ref:`AUTOTUNE_FRQ_MIN<AUTOTUNE_FRQ_MIN>` parameter specifies the minimum frequency in radians per second used during the dwell or frequency sweeps.  For frequency sweeps, this will be the starting frequency.  The default value is good for helicopters with rotor diameters less than 1.4 meters.


:ref:`Maximum Test Frequency<AUTOTUNE_FRQ_MAX>`
-----------------------------------------------

The :ref:`AUTOTUNE_FRQ_MAX<AUTOTUNE_FRQ_MAX>` parameter specifies the maximum frequency in radians per second used during the dwell or frequency sweeps.  For frequency sweeps, this will be the ending frequency.  The default value is fine for helicopters with rotor diameters greater than 1.2 meters.  Consider raising to 100 radians/sec for helicopters with rotor diameters less then 1.2 meters.


:ref:`Velocity P Gain<AUTOTUNE_VELXY_P>`
----------------------------------------

The :ref:`AUTOTUNE_VELXY_P<AUTOTUNE_VELXY_P>` parameter specifies P gain for velocity feedback.  This aids the AutoTune in maintaining aircraft position during the frequency sweeps and dwells.  Keep this at 0.1 unless the aircraft is drifting more than 10 meters during the dwell and frequency sweeps.  It only affects position holding while the aircraft is oscillating during these tests.  In between the oscillations, it may drift.  This gain will not help with that.


Preparing for AutoTune
======================

Ensure you complete all of the items in the :ref:`Preparing for Tuning<traditional-helicopter-tuning-preparing>` wiki page.

Transmitter Setup
-----------------

Be sure to put the AutoTune flight mode as one of the flight modes on your transmitter flight mode switch.  You don’t want to be reaching for the GCS to switch out of the AutoTune if your heli is not behaving properly.  You want to be able to switch modes instantly.


AutoTune Flights
================
Suggested Tuning Sequence
-------------------------

In any axis, it is recommended to conduct the feedforward gain tuning first, then the Rate D and Rate P gain tuning, and lastly the Angle P tuning.  Don't try to tune every axis in one flight, it could take up to 10-15 min to tune one axis.  Here is a suggested flights and tuning sequence. Conventional helicopters do not need Yaw feedforward tuning, therefore the gain can be set to zero.  Dual helicopters may require Yaw feedforward tuning.

+--------+-------------------------------------------------------------+-----------------------+
| Flight | Parameter                                                   | Value                 |
+========+=============================================================+=======================+
|    1   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES__AC_AutoTune_Heli>`       | 3 (Roll and Pitch)    |
|        +-------------------------------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`                           | 1 (feedforward)       |
+--------+-------------------------------------------------------------+-----------------------+
|    2   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES__AC_AutoTune_Heli>`       | 2 (Pitch)             |
|        +-------------------------------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`                           | 2 (Rate D and Rate P) |
|        +-------------------------------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>`                     | 1.8                   |
+--------+-------------------------------------------------------------+-----------------------+
|    3   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES__AC_AutoTune_Heli>`       | 1 (Roll)              |
|        +-------------------------------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`                           | 2 (Rate D and Rate P) |
|        +-------------------------------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>`                     | 1.8                   |
+--------+-------------------------------------------------------------+-----------------------+
|    4   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES__AC_AutoTune_Heli>`       | 3 (Roll and Pitch)    |
|        +-------------------------------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`                           | 4 (Angle P)           |
|        +-------------------------------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>`                     | 2.0                   |
+--------+-------------------------------------------------------------+-----------------------+
|    5   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES__AC_AutoTune_Heli>`       | 4 (Yaw)               |
|        +-------------------------------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`                           | 2 (Rate D and Rate P) |
|        +-------------------------------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>`                     | 1.0                   |
+--------+-------------------------------------------------------------+-----------------------+
|    6   | :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES__AC_AutoTune_Heli>`       | 4 (Yaw)               |
|        +-------------------------------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_SEQ<AUTOTUNE_SEQ>`                           | 4 (Angle P)           |
|        +-------------------------------------------------------------+-----------------------+
|        | :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>`                     | 1.2                   |
+--------+-------------------------------------------------------------+-----------------------+

Tuning Flight Procedures
------------------------

When conducting an AutoTune flight, be sure to have at least at 50 meter by 50 meter area to fly. The aircraft will move during the autotuning process.  Do not let the aircraft get too far away.

.. caution::  During the feedforward test, the aircraft may travel 5-10 meters, and perhaps even further with larger helicopters. You may have to bring it back after each iteration of the tuning test.  

#. Power up the controller.
#. Set the flight mode to either stabilize or AltHold (Althold recommended).
#. Enable motor interlock and allow rotors to complete runup.
#. Lift off and establish stable hover approximately 3-5 meters above the ground.
#. Switch into AutoTune and center all sticks.
#. AutoTune will start conducting the maneuvers.

.. note::  If you don’t see anything happening, verify your sticks are centered.

7. After the tuning is complete, a message will appear in the GCS saying "AutoTune complete".
8. To test the settings, switch out of AutoTune and then back to AutoTune and you will be 
   able to test the settings that were tuned.
9. Once you are finished testing, descend and land in AutoTune.  Once the aircraft has landed, the 
   engine will shutdown on its own.  At that point flip your motor interlock switch to disabled
   and disarm the aircraft.

.. note::  Aircraft must be disarmed in the AutoTune flight mode to save the gain settings.

Tuning Maneuver Descriptions
----------------------------

``ATC_RAT_xxx_FF`` Tuning
+++++++++++++++++++++++++

        The ``ATC_RAT_xxx_FF`` tuning is accomplished by achieving a constant angular rate of 50 deg/s and determining the steady state command required to maintain the 50 deg/s.  The maneuver to achieve the constant angular rate consists of changing attitude by 15 deg in one direction then reversing direction to achieve a constant rate of 50 deg/s before reaching 15 deg in the opposite direction.  Finally it returns to the starting attitude.   During ``ATC_RAT_xxx_FF`` tuning the aircraft may drift, reposition the aircraft as needed to keep it from drifting.  Making any inputs during this test will stop the tuning and won’t begin again unless the sticks are centered.  The following video demonstrates the ``ATC_RAT_xxx_FF`` tuning.

..  youtube:: 2XLBIycPiq0

``ATC_RAT_xxx_D`` and ``ATC_RAT_xxx_P`` Tuning
++++++++++++++++++++++++++++++++++++++++++++++

        ``ATC_RAT_xxx_D`` and ``ATC_RAT_xxx_P`` tuning starts with determining the maximum ``ATC_RAT_xxx_D`` and ``ATC_RAT_xxx_P`` gains that can be safely tuned.  A frequency sweep is conducted from the :ref:`AUTOTUNE_FRQ_MIN<AUTOTUNE_FRQ_MIN>` to :ref:`AUTOTUNE_FRQ_MAX<AUTOTUNE_FRQ_MAX>`.  This determines the approximate frequency required for calculating the maximum allowable gains.  A series of dwells (oscillations at one frequency) are completed to more accurately determine the data required to calculate the maximum allowable ``ATC_RAT_xxx_D`` and ``ATC_RAT_xxx_P`` gains. Next another frequency sweep is conducted to approximate the frequency for tuning ``ATC_RAT_xxx_D`` gain.  Then the ``ATC_RAT_xxx_D`` gain is raised until the response gain stops decreasing. Next the ``ATC_RAT_xxx_P`` gain is increased until the response gain exceeds the :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>`.  During this tuning, you can’t make any inputs to hold position during the tuning.  If you make any inputs, then it will stop the tuning and wait until you center the sticks before it begins again.  The aircraft will drift some but shouldn’t drift too far (< 50 m).  The tuning sweeps are 23 seconds in duration.  The following video demonstrates the ``ATC_RAT_xxx_D`` and ``ATC_RAT_xxx_P`` tuning.

..  youtube:: IOOIG_z1Cwc

``ATC_ANG_xxx_P`` Tuning
++++++++++++++++++++++++

        ``ATC_ANG_xxx_P`` tuning starts with conducting a frequency sweep from from the :ref:`AUTOTUNE_FRQ_MIN<AUTOTUNE_FRQ_MIN>` to :ref:`AUTOTUNE_FRQ_MAX<AUTOTUNE_FRQ_MAX>`.  This determines the approximate frequency for the maximum response gain.  Then dwells (oscillations at one frequency) are conducted to tune the ``ATC_ANG_xxx_P`` gain. The gain is raised or lowered to determine the ``ATC_ANG_xxx_P`` gain that corresponds to a response gain (output angle/input angle request) that matches :ref:`AUTOTUNE_GN_MAX<AUTOTUNE_GN_MAX>`. During this tuning, you can’t make any inputs to hold position during the tuning.  If you make any inputs, then it will stop the tuning and wait until you center the sticks before it begins again.  The aircraft will drift some but shouldn’t drift too far (< 50 m).  The tuning sweeps are 23 seconds in duration.  

..  youtube:: aI-uJuQAh-0

