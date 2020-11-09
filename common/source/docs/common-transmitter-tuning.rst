.. _common-transmitter-tuning:

[copywiki destination="plane,copter"]
========================
Transmitter Based Tuning
========================

You can perform extensive parameter tuning in flight using your R/C
transmitter. This is meant for advanced users who are unable to use
the Autotune features or wish to do fine tuning with full manual
tuning control of each parameter


Overview
========

Transmitter based tuning allows you to tune a single parameter or a
set of parameters while flying. The basic idea is to link the tuning
value of a parameter to a knob or slider on your transmitter then to
adjust the parameter in flight by moving the knob.

[site wiki="plane"]

Key features of transmitter based tuning are:

- Tune either a single parameter or a whole set of related parameters
  in a single flight

- Re-scale the range of the parameter value linked to the tuning knob
  using a "selector switch"

- Switch between parameters to tune while flying using a selector
  switch

- Save the new parameters in flight

- Receive audible feedback from the flight board buzzer and the GCS on
  tuning progress


Concepts
========

The two key controls for transmitter based tuning are:

- A "tuning knob", setup on your transmitter to a convenient knob or
  slider and linked to a RC channel that your board receives

- An optional "selector switch" for controlling advanced features of
  the tune. This should be linked to a two position switch mapped to
  a RC channel that your board receives. A spring loaded switch (such
  as the trainer switch) works particularly well.

In addition to those input controls the following concepts are useful
in understanding the tuning process:

- A "parameter set" is a group of related parameters that can be
  tuned one after the other in a single flight

- The "mid-point" is point on the tuning knobs range where it
  produces a PWM value half way between minimum and maximum


Setting up for tuning
=====================

To setup your vehicle for tuning you need to set the following
parameters:

- :ref:`TUNE_CHAN<TUNE_CHAN>` : the RC input channel associated with your chosen tuning
  knob
- :ref:`TUNE_CHAN_MIN<TUNE_CHAN_MIN>` : the minimum PWM values produced on :ref:`TUNE_CHAN<TUNE_CHAN>`
- :ref:`TUNE_CHAN_MAX<TUNE_CHAN_MAX>` : the maximum PWM values produced on :ref:`TUNE_CHAN<TUNE_CHAN>`
- :ref:`TUNE_PARAM<TUNE_PARAM>` : the parameter or set of parameters you will be tuning
- :ref:`TUNE_SELECTOR<TUNE_SELECTOR>` : the RC input channel associated with your chosen
  selector switch (optional)
- :ref:`TUNE_RANGE<TUNE_RANGE>` : the scaling range which the tuning knob covers

The :ref:`TUNE_PARAM<TUNE_PARAM>` parameter selects the parameter or set of parameters you
will be tuning. Values of :ref:`TUNE_PARAM<TUNE_PARAM>` less than 100 correspond to
individual tunable parameters whereas values of 101 or higher
correspond to sets of related parameters that can be tuned one after
the other in a flight.

.. note:: Most of the tuneable parameters apply to the VTOL operation of QuadPlanes. Only :ref:`TUNE_PARAM<TUNE_PARAM>` values in the 50-57 range are for fixed wing operation.

Use your ground stations parameter interface to see see the full list
of tunable parameters and parameter sets available for :ref:`TUNE_PARAM<TUNE_PARAM>` . For
Plane most of the parameters are associated with tuning the
QuadPlane VTOL motors as those are the most difficult to tune. You can
also tune some fixed wing parameters, although most people find the
automatic tuning with AUTOTUNE mode is the best option for fixed wing
flight.

If you want to tune a set of parameters (by choosing a :ref:`TUNE_PARAM<TUNE_PARAM>` value
over 100) you must have a selector switch configured with the
:ref:`TUNE_SELECTOR<TUNE_SELECTOR>` parameter.

Using the tuning knob
=====================

The basic operation of the tuning knob is very simple. If the tuning
knob is at the bottom of its range then the parameter being tuned is
set to the initial value divided by the :ref:`TUNE_RANGE<TUNE_RANGE>` . If the tuning knob
is at the top of its range then the parameter being tuned will be set
to the initial value multiplied by :ref:`TUNE_RANGE<TUNE_RANGE>` .

So with a default value for :ref:`TUNE_RANGE<TUNE_RANGE>` of 2 you will be able to change
the parameter in a range from half its initial value to 2x the initial
value. This is a good range for many tuning tasks.

Activating the tuning knob
--------------------------

When you first start tuning a parameter you will find the tuning knob
is not yet active. This is because the knob does not activate until it
passes the "mid-point value", defined as half way between
:ref:`TUNE_CHAN_MIN<TUNE_CHAN_MIN>` and :ref:`TUNE_CHAN_MAX<TUNE_CHAN_MAX>` . Activating the tuning knob in this
way ensures that you don't accidentally take off with a large change
in tuning value. You are guaranteed to start the tune with a value
very close to your current value for the parameter.

When the tuning knob activates by reaching the mid-point the buzzer on
the flight board will give a quick "bup-bip" sound to indicate that
tuning has been activated.

Re-centering the tuning knob
----------------------------

It is quite common to find that the :ref:`TUNE_RANGE<TUNE_RANGE>` is not wide enough to
move the tuning value to the ideal point for your vehicle. For
example, you may have started the tune with a P gain for some axis of
0.7, and when you move the tuning knob up all the way the vehicle
still hasn't started oscillating. In that case the tuning value will
have reached 1.4 and you need some more range.

To get more range you can use the selector switch to re-center the
tuning knob around the current value. Toggle the selector switch
briefly high then low and the center-value will change to whatever the
tubing knob is set to. When you re-center the tuning knob will
de-activate again until you move it to the mid-point position. This
prevents you getting a jump in the tuning value when you re-center.

Tuning multiple parameters
==========================

You can tune multiple parameters in one flight by setting :ref:`TUNE_PARAM<TUNE_PARAM>` to
one of the "tuning set" parameters. For example, if you set :ref:`TUNE_PARAM<TUNE_PARAM>`
to 101 then you will have 4 different stages to your tune:

- ``Roll: Rate D``
- ``Roll: Rate P and Rate I``
- ``Pitch: Rate D``
- ``Pitch: Rate P and Rate I``

notice that ``Roll: Rate P and Rate I`` is actually two parameters in one, controlling
both the P gain and the I gain for roll. This follows the normal
advice for MultiCopters that you should keep the P and I values equal
when doing a manual tune.

When you choose a tuning set with :ref:`TUNE_PARAM<TUNE_PARAM>` then you will initially be
tuning the first parameter in the set. Once you have adjusted that
parameter as much as you need to you can move to the next parameter in
the set by holding the selector switch for more than 2 seconds. It is
suggested that you count to 3 to ensure you are over 2 seconds.

Holding the selector switch for more than 2 seconds will switch you to
the next parameter and will also change the tuning knob back to its
"wait for mid-point" state on the new parameter. The buzzer on the
board will give a loud BEEP sequence to indicate which parameter in
the set you have changed to. For the first parameter in the set you
will get one loud BEEP. For the second parameter you will get two loud
BEEPs and so on.

When you have cycled through all of the parameters in the tuning set
you have chosen it will wrap back around to the first parameter in the
set.

+-----------------------------+-------------------+---------------------------------------------------------------------------------------------------------------------+
+:ref:`TUNE_PARAM<TUNE_PARAM>`+  Set Name         +       Params Tuned                                                                                                  +
+-----------------------------+-------------------+---------------------------------------------------------------------------------------------------------------------+
+    101                      + Set_RateRollPitch +     :ref:`Q_A_RAT_RLL_D<Q_A_RAT_RLL_D>`/:ref:`Q_A_RAT_RLL_P<Q_A_RAT_RLL_P>` & :ref:`Q_A_RAT_RLL_I<Q_A_RAT_RLL_I>`/  +
+                             +                   +     :ref:`Q_A_RAT_PIT_D<Q_A_RAT_PIT_D>`/:ref:`Q_A_RAT_PIT_P<Q_A_RAT_PIT_P>` & :ref:`Q_A_RAT_PIT_I<Q_A_RAT_PIT_I>`   +
+-----------------------------+-------------------+---------------------------------------------------------------------------------------------------------------------+
+    102                      + Set_RateRoll      +     :ref:`Q_A_RAT_RLL_D<Q_A_RAT_RLL_D>` / :ref:`Q_A_RAT_RLL_P<Q_A_RAT_RLL_P>` & :ref:`Q_A_RAT_RLL_I<Q_A_RAT_RLL_I>` +
+-----------------------------+-------------------+---------------------------------------------------------------------------------------------------------------------+
+    103                      + Set_RatePitch     +     :ref:`Q_A_RAT_PIT_D<Q_A_RAT_PIT_D>` / :ref:`Q_A_RAT_PIT_P<Q_A_RAT_PIT_P>` & :ref:`Q_A_RAT_PIT_I<Q_A_RAT_PIT_I>` +
+-----------------------------+-------------------+---------------------------------------------------------------------------------------------------------------------+
+    104                      + Set_RateYaw       +     :ref:`Q_A_RAT_YAW_P<Q_A_RAT_YAW_P>` / :ref:`Q_A_RAT_YAW_I<Q_A_RAT_YAW_I>` / :ref:`Q_A_RAT_YAW_D<Q_A_RAT_YAW_D>` +
+-----------------------------+-------------------+---------------------------------------------------------------------------------------------------------------------+
+    105                      + Set_AngleRollPitch+     :ref:`Q_A_ANG_RLL_P<Q_A_ANG_RLL_P>` / :ref:`Q_A_ANG_PIT_P<Q_A_ANG_PIT_P>`                                       +
+-----------------------------+-------------------+---------------------------------------------------------------------------------------------------------------------+
+    106                      + Set_VelXY         +     :ref:`Q_P_VELXY_P<Q_P_VELXY_P>`  / :ref:`Q_P_VELXY_I<Q_P_VELXY_I>`                                              +
+-----------------------------+-------------------+---------------------------------------------------------------------------------------------------------------------+
+    107                      + Set_AccelZ        +     :ref:`Q_P_ACCZ_P<Q_P_ACCZ_P>` /  :ref:`Q_P_ACCZ_I<Q_P_ACCZ_I>` / :ref:`Q_P_ACCZ_D<Q_P_ACCZ_D>`                  +
+-----------------------------+-------------------+---------------------------------------------------------------------------------------------------------------------+


Saving the tuning results
=========================

When you are happy with the tune you can save the result by holding
the selector switch for more than 5 seconds. After 5 seconds the board
will make a rapid bup-bip-bup-bip sound to indicate that the save is
complete. If you leave the selector switch in the high position then
tuning will remain disabled after the save.

Reverting the tune
==================

If you are not happy with your tuning results or the vehicle becomes
unstable you should change flight mode. Any change of flight mode will
immediately revert all of the parameters you are tuning to the last
saved value. However, you can prevent this reversion upon flight mode change by setting 
the :ref:`TUNE_MODE_REVERT<TUNE_MODE_REVERT>` parameter to 0.

The tuning process
==================

The tuning system is designed to make it easy to quickly get a
reasonable manual tune on a vehicle in one flight. The most common use
for this type of tuning will be in adjusting the rate roll and pitch
PID gains. The tuning procedure outlined below is for that particular
case.

Setting up
----------

To setup for tuning your rate PIDs you should set :ref:`TUNE_CHAN<TUNE_CHAN>` to your
tuning channel, :ref:`TUNE_SELECTOR<TUNE_SELECTOR>` to your selector switch and :ref:`TUNE_PARAM<TUNE_PARAM>`
to 101 (which is the "rate roll and pitch PIDs tuning set").

Then takeoff and switch the vehicle to a comfortable flight mode for
rate tuning. For a QuadPlane
QHOVER or QLOITER are the best choices.

The first parameter you will be tuning will be RateRollD. To tune that
parameter (and the other parameters in the rate roll/pitch set) you
should follow this process:

- move the tuning knob to the mid-point to active the knob. You will
  hear a rapid bup-bip from the board to indicate the tuning knob is
  activated.
- start raising the tuning knob slowly, stopping immediately if the
  vehicle starts to oscillate. While you raise the gain you should
  give some small roll inputs on the sticks.
- if you get to the top of the tuning knob range and the vehicle has
  not yet started to oscillate then use the selector switch to
  re-center the range, then move the tuning knob to the mid-point to
  reactivate the tuning knob
- as soon as you see oscillation you should immediately lower the
  tuning knob to the point where the oscillation just stops
- once the oscillation stops then re-center the tuning knob using the
  selector switch, then move it to the mid-point to activate it, and
  then lower the tuning knob all the way down. That will move the
  tuning value to half of the value that just stopped the
  oscillation (assuming a :ref:`TUNE_RANGE<TUNE_RANGE>` of 2). Moving to half of that value will give you enough
  margin in your tune to ensure your vehicle can handle a wide range
  of flight conditions.

Once you have completed the above process for the first parameter then
you can move to the 2nd parameter by holding the selector switch for a
count of 3. You will hear a BEEP BEEP sound from the vehicle
indicating that you have moved to parameter 2, which is the RateRollPI
parameters. You should then repeat exactly the same tuning process
with that parameter.

Keep tuning each parameter in turn using the above process until you
are happy with all of them and then save your new tuning parameters by
holding the selector switch for more than five seconds. You will know
the 5 seconds is up when you hear the distinctive rapid
bup-bip-bup-bip sound from the buzzer.

At that point you can land the vehicle, or just enjoy flying it.

The first time you do a full tune in this way it will probably take
about five minutes of flight time to do a tune. With some practice you
can do a full tune in a bit over a minute.

[/site]
[site wiki="copter"]

With transmitter based tuning you can tune a single or multiple parameters in flight using Channel 6 of the transmitter.

The :ref:`TUNE<TUNE>` parameter determines which parameter is being tuned.

The :ref:`TUNE_MAX<TUNE_MAX>` parameter determines the maximum value of the parameter when the channel is at :ref:`RC6_MAX<RC6_MAX>`, while the :ref:`TUNE_MIN<TUNE_MIN>` parameter determines the value when RC channle 6 is at :ref:`RC6_MIN<RC6_MIN>`.

:ref:`TUNE<TUNE>` Values
========================

+--------+-------------------------+----------------------------------------------------------------------+
|Value	 |Meaning                  | Parameter                                                            |
+========+=========================+======================================================================+
|0       |         None            |                                                                      |
+--------+-------------------------+----------------------------------------------------------------------+
|1       |Stab Roll/Pitch kP       |  :ref:`ATC_ANG_RLL_P<ATC_ANG_RLL_P>` ,                               |
|        |                         |  :ref:`ATC_ANG_PIT_P<ATC_ANG_PIT_P>`                                 |
+--------+-------------------------+----------------------------------------------------------------------+
|4       |Rate Roll/Pitch kP       |  :ref:`ATC_RAT_RLL_P<ATC_RAT_RLL_P__AC_AttitudeControl_Multi>` ,     |
|        |                         |  :ref:`ATC_RAT_PIT_P<ATC_RAT_PIT_P__AC_AttitudeControl_Multi>`       |
+--------+-------------------------+----------------------------------------------------------------------+
|5       |Rate Roll/Pitch kI       |  :ref:`ATC_RAT_RLL_I<ATC_RAT_RLL_I__AC_AttitudeControl_Multi>` ,     |
|        |                         |  :ref:`ATC_RAT_PIT_I<ATC_RAT_PIT_I__AC_AttitudeControl_Multi>`       |
+--------+-------------------------+----------------------------------------------------------------------+
|21      |Rate Roll/Pitch kD       |  :ref:`ATC_RAT_RLL_D<ATC_RAT_RLL_D__AC_AttitudeControl_Multi>` ,     |
|        |                         |  :ref:`ATC_RAT_PIT_D<ATC_RAT_PIT_D__AC_AttitudeControl_Multi>`       |
+--------+-------------------------+----------------------------------------------------------------------+
|3       |Stab Yaw kP              |  :ref:`ATC_ANG_YAW_P<ATC_ANG_YAW_P>`                                 |
+--------+-------------------------+----------------------------------------------------------------------+
|6       |Rate Yaw kP              |  :ref:`ATC_RAT_YAW_P<ATC_RAT_YAW_P__AC_AttitudeControl_Multi>`       |
+--------+-------------------------+----------------------------------------------------------------------+
|26      |Rate Yaw kD              |  :ref:`ATC_RAT_YAW_D<ATC_RAT_YAW_D__AC_AttitudeControl_Multi>`       |
+--------+-------------------------+----------------------------------------------------------------------+
|56      |Rate Yaw Filter          |  :ref:`ATC_RAT_YAW_FLTE<ATC_RAT_YAW_FLTE__AC_AttitudeControl_Multi>` |
+--------+-------------------------+----------------------------------------------------------------------+
|55      |Motor Yaw Headroom       |  :ref:`MOT_YAW_HEADROOM<MOT_YAW_HEADROOM>`                           |
+--------+-------------------------+----------------------------------------------------------------------+
|14      |AltHold kP               |  :ref:`PSC_POSZ_P<PSC_POSZ_P>`                                       |
+--------+-------------------------+----------------------------------------------------------------------+
|7       |Throttle Rate kP         |  :ref:`PSC_VELZ_P<PSC_VELZ_P>`                                       |
+--------+-------------------------+----------------------------------------------------------------------+
|34      |Throttle Accel kP        |  :ref:`PSC_ACCZ_P<PSC_ACCZ_P>`                                       |
+--------+-------------------------+----------------------------------------------------------------------+
|3       |Throttle Accel kI        |  :ref:`PSC_ACCZ_I<PSC_ACCZ_I>`                                       |
+--------+-------------------------+----------------------------------------------------------------------+
|36      |Throttle Accel kD        |  :ref:`PSC_ACCZ_I<PSC_ACCZ_I>`                                       |
+--------+-------------------------+----------------------------------------------------------------------+
|12      |Loiter Pos kP            |  :ref:`PSC_POSXY_P<PSC_POSXY_P>`                                     |
+--------+-------------------------+----------------------------------------------------------------------+
|22      |Velocity XY kP           |  :ref:`PSC_POSXY_P<PSC_POSXY_P>`                                     |
+--------+-------------------------+----------------------------------------------------------------------+
|28      |Velocity XY kI           |  :ref:`PSC_VELXY_I<PSC_VELXY_I>`                                     |
+--------+-------------------------+----------------------------------------------------------------------+
|10      |WP Speed                 |  :ref:`WPNAV_SPEED<WPNAV_SPEED>`                                     |
+--------+-------------------------+----------------------------------------------------------------------+
|25      |Acro RollPitch kP        | :ref:`ACRO_RP_P<ACRO_RP_P>`                                          |
+--------+-------------------------+----------------------------------------------------------------------+
|40      |Acro Yaw kP              | :ref:`ACRO_YAW_P<ACRO_YAW_P>`                                        |
+--------+-------------------------+----------------------------------------------------------------------+
|45      |RC Feel                  | :ref:`ATC_INPUT_TC<ATC_INPUT_TC>`                                    |
+--------+-------------------------+----------------------------------------------------------------------+
|13      |Heli Ext Gyro            | :ref:`H_GYR_GAIN<H_GYR_GAIN>`                                        |
+--------+-------------------------+----------------------------------------------------------------------+
|38      |Declination              | :ref:`COMPASS_DEC<COMPASS_DEC>`                                      |
+--------+-------------------------+----------------------------------------------------------------------+
|39      |Circle Rate              | :ref:`CIRCLE_RATE<CIRCLE_RATE>`                                      |
+--------+-------------------------+----------------------------------------------------------------------+
|41      |RangeFinder Gain         |  :ref:`RNGFND_GAIN<RNGFND_GAIN>`                                     |
+--------+-------------------------+----------------------------------------------------------------------+
|46      |Rate Pitch kP            | :ref:`ATC_RAT_PIT_P<ATC_RAT_PIT_P__AC_AttitudeControl_Multi>`        |
+--------+-------------------------+----------------------------------------------------------------------+
|47      |Rate Pitch kI            | :ref:`ATC_RAT_PIT_I<ATC_RAT_PIT_I__AC_AttitudeControl_Multi>`        |
+--------+-------------------------+----------------------------------------------------------------------+
|48      |Rate Pitch kD            | :ref:`ATC_RAT_PIT_D<ATC_RAT_PIT_D__AC_AttitudeControl_Multi>`        |
+--------+-------------------------+----------------------------------------------------------------------+
|49      |Rate Roll kP             | :ref:`ATC_RAT_RLL_P<ATC_RAT_RLL_P__AC_AttitudeControl_Multi>`        |
+--------+-------------------------+----------------------------------------------------------------------+
|50      |Rate Roll kI             | :ref:`ATC_RAT_RLL_I<ATC_RAT_RLL_I__AC_AttitudeControl_Multi>`        |
+--------+-------------------------+----------------------------------------------------------------------+
|51      |Rate Roll kD             | :ref:`ATC_RAT_RLL_D<ATC_RAT_RLL_D__AC_AttitudeControl_Multi>`        |
+--------+-------------------------+----------------------------------------------------------------------+
|52      |Rate Pitch FF            | :ref:`ATC_RAT_PIT_FF<ATC_RAT_PIT_FF>` (heli only)                    |
+--------+-------------------------+----------------------------------------------------------------------+
|53      |Rate Roll FF             | :ref:`ATC_RAT_RLL_FF<ATC_RAT_RLL_FF>` (heli only)                    |
+--------+-------------------------+----------------------------------------------------------------------+
|54      |Rate Yaw FF              | :ref:`ATC_RAT_YAW_FF<ATC_RAT_YAW_FF>` (heli only)                    |
+--------+-------------------------+----------------------------------------------------------------------+
|57      |Winch                    | :ref:`WINCH_RATE_MAX<WINCH_RATE_MAX>`                                |
+--------+-------------------------+----------------------------------------------------------------------+
|58      |SysID Magnitude          | :ref:`SIDS_MAGNITUDE<SID_MAGNITUDE>`                                 |
+--------+-------------------------+----------------------------------------------------------------------+


These values can be either set manually or using Mission Planner


Setting with Mission Planner
============================

Rate Roll P and Rate Pitch P will be used in the following example procedure

.. image:: ../images/RollPitchTuning.png
    :target: ../_images/RollPitchTuning.png

#. Connect your autopilot to Mission Planner
#. On Mission Planner, select CONFIG>>Extended Tuning
#. Set the TUNE drop down box option to "Rate Roll/Pitch kP"
#. Set Min to 0.08, Max to 0.20 (most copters ideal gain is within this
   range although from a small number of copter the Max can be as high
   as 0.25)
#. Push the "Write Params" button
#. Turn your transmitter's CH6 tuning knob to the minimum position,
   press the "Refresh Params" button and ensure that the Rate Roll P and
   Rate Pitch P values become 0.08 (or something very close)
#. Turn the CH6 knob to it's maximum position, press "Refresh Params"
   and ensure the Rate Roll P moves to 0.20
#. Move the CH6 knob back to the middle
#. Arm and fly your copter in Stabilize mode adjusting the ch6 knob
   until you get a copter that is responsive but not wobbly
#. After the flight, disconnect your LiPo battery and reconnect the autopilot to the mission planner
#. With the CH6 knob in the position that gave the best performance,
   return to the Copter Pids screen and push the "Refresh Params" button
#. In the Rate Roll P and Rate Pitch P fields re-type the value that you
   see but just slightly modified so that the mission planner recongises
   that it's changed and resends to the autopilot (Note: if you re-type
   exactly the same number as what appears in Rate Roll P it won't be
   updated).  So for example if the Rate Roll P appears as "0.1213" make
   it "0.1200"
#. Set Ch6 Opt back to "None" and push "Write Params"
#. Push the Disconnect button on the top right, and the Connect
#. Ensure that the Rate Roll P value is the value that you retyped in
   step #12

Note: while you are moving the tuning knob the values update at 3 times
per second.  The need to press the Refresh button in the mission planner
in steps #6 and #7 above is just because the Copter is not sending the
updates to the mission planner in real-time.

[/site]
