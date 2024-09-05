.. _traditional-helicopter-inverted-flight:

===============
Inverted Flight
===============

Since helicopters, including dual rotor and heli-quads, are capable of generating negative thrust (negative collective) they have the capability of flying, even hovering, inverted. This, of course, requires that the vehicle is capable of generating sufficient thrust while inverted and is setup to do so (see :ref:`traditional-helicopter-aerobatic-setup`).

Inverted flight can be commanded in two different ways:

- Under pilot attitude control in ACRO mode, or
- Under ArduPilot attitude stabilization control, in STABILIZE, ALTHOLD, LOITER, and AUTO modes using the ``RCx_OPTION`` invert switch function "43". This merely flips the effective orientation of the IMU to be inverted and the vehicle follows. It will stabilise the vehicle with a roll of 180 degrees from normal whenever inverted flight is enabled.

Notes
=====

- GPS required modes such as LOITER and AUTO require good GPS performance which may be impacted adversely if the GPS is facing down for long periods, especially hover precision. Dual blended GPSes oriented such that one has a "sky-view" while inverted can ameliorate this. In extreme cases, EKF failsafes might occur with undesirable effects.
- Negative collective range is not impacted using the invert switch, ie it becomes the effective climb control so be sure there is sufficient range.