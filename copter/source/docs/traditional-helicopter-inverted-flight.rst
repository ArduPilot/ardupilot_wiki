.. _traditional-helicopter-inverted-flight:

===============
Inverted Flight
===============

Since helicopters, including dual rotor and heli-quads, are capable of generating negative thrust (negative collective) they have the capability of flying, even hovering, inverted. This, of course, requires that the vehicle is capable of generating sufficient thrust while inverted and is setup to do so (see :ref:`traditional-helicopter-aerobatic-setup`).

Inverted flight can be commanded in two different ways:

- Under pilot attitude control in ACRO mode which requires the pilot to push the collective stick down to climb while inverted and the pitch control is oriented to the inverted aircraft as well, or
- Under ArduPilot attitude stabilization control, in STABILIZE, ALTHOLD, LOITER, and AUTO modes using the ``RCx_OPTION`` invert switch function "43". This merely flips the effective orientation of the IMU to be inverted and the vehicle follows. It will stabilise the vehicle with a roll of 180 degrees from normal whenever inverted flight is enabled. While inverted, collective is used normally in STABILIZE, ALTHOLD and LOITER where pushing up on the collective causes the inverted helicopter to climb away from the ground.  Pitch control is also used as though the aircraft is upright.

Notes
=====

- GPS required modes such as LOITER and AUTO require good GPS performance which may be impacted adversely if the GPS is facing down for long periods, especially hover precision. Dual blended GPSes oriented such that one has a "sky-view" while inverted can ameliorate this. In extreme cases, EKF failsafes might occur with undesirable effects.
- Negative collective range is not impacted using the invert switch, ie it becomes the effective climb control so be sure there is sufficient range.

Setup of the inverted flight feature
====================================

The inverted flight feature is initiated by setting an RC channel via RCx_OPTION parameter.  The RCx_OPTION parameter allows you to configure a transmitter switch to enable inverted flight for helicopters. To enable it, choose a RC input channel which is not being used for anything else, and setup your transmitter to give a PWM value larger than 1750 when a previously unused 2 position switch is enabled, and a value below 1200 when it is not enabled.  For example, if channel 7 is currently unused, and you set :ref:`RC7_OPTION<RC7_OPTION>` to InvertedFlight (which has value 43) then if your transmitter sends 1900 on channel 7 the helicopter will flip upside down and continue flying in whatever mode it is in, but with the helicopter inverted.

Be sure to setup pitch and roll rate limits.  This will keep the attitude controller from commanding unachievable roll or pitch rates.  It is recommended in STABILIZE or ACRO that you command max roll and pitch rates.  Pull the flight logs and determine what is the maximum pitch and roll rate the aircraft can achieve.  Then set :ref:`ATC_RATE_R_MAX<ATC_RATE_R_MAX>` for max roll rate and :ref:`ATC_RATE_P_MAX<ATC_RATE_P_MAX>` for max pitch rate.  If you are unable to determine the max rates then be conservative and set them to 120 deg/s initially.

Tips
====

Here are some general tips about inverted flight

- Try it out in simulation first to get used to it, and ensure your
  transmitter is setup right.
- make sure you have plenty of height before engaging or disengaging inverted flight for the first time. The aircraft should not lose much altitude during the transition but it is worth having the extra altitude incase something unexpected happens.  As you get more comfortable in the aircraft's ability to transition from upright to inverted and vice-versa, then you can lower the altitude where the transition is made.