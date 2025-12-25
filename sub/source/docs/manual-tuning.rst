.. _manual-tuning:

====================
Manual Tuning of Sub
====================

Sub has several control mechanisms for axis stabilization in certain modes, manuvering speeds, angle limits, etc.

Axis Stabilization
==================

Manual tuning may be required to provide maximum stabilization without oscillation in the Roll,Pitch, and Yaw axes. Depending on frame configuration and mode, Sub can provide attitude/heading stabilization in those axes, but tuning of their attitude controllers may be required to obtain the optimum stabilization, ie. rapid approach to rate/angle targets with minimal overshoot or oscillation).

This can be accomplished by manually adjusting each axis' attitude rate and angle controllers' parameter values.

The following rate controller parameters are adjusted:

===================================   ===================================  ===================================
Roll                                  Pitch                                YAW
===================================   ===================================  ===================================
:ref:`ATC_RAT_RLL_P<ATC_RAT_RLL_P>`   :ref:`ATC_RAT_PIT_P<ATC_RAT_PIT_P>`  :ref:`ATC_RAT_YAW_P<ATC_RAT_YAW_P>`
:ref:`ATC_RAT_RLL_I<ATC_RAT_RLL_I>`   :ref:`ATC_RAT_PIT_I<ATC_RAT_PIT_I>`  :ref:`ATC_RAT_YAW_I<ATC_RAT_YAW_I>`
:ref:`ATC_RAT_RLL_D<ATC_RAT_RLL_D>`   :ref:`ATC_RAT_PIT_D<ATC_RAT_PIT_D>`  :ref:`ATC_RAT_YAW_D<ATC_RAT_YAW_D>`
===================================   ===================================  ===================================

.. note:: depending on frame type, roll and pitch axis control may not be implemented, so tuning is obviously unnecessary. Yaw control is always implemented in Sub frames.

For each axis:

If the vehicle already oscillates in an axis, first lower the P, D, and I terms in 50% steps until stable, before starting manual tuning.

When oscillations start do not make large or sudden stick inputs, and reduce the parameter increased as soon as possible.

1. Increase the D term in steps of 50% until oscillation is observed
2. Reduce the D term in steps of 10% until the oscillation disappears
3. Reduce the D term by a further 25%
4. Increase the P term in steps of 50% until oscillation is observed
5. Reduce the P term in steps of 10% until the oscillation disappears
6. Reduce the P term by a further 25%

Each time the P term is changed set the I term to approximately 3/4 the P term. Those parameters can be changed on the surface, and preferably when disarmed.

The other ``ATC_`` rarely need changing from the defaults.

Maximum Attitude Angle Limits
=============================
These can be changed with:

- :ref:`ANGLE_MAX<ANGLE_MAX>`
- :ref:`XTRACK_ANG_LIM<XTRACK_ANG_LIM>`

Speeds
======
Several parameters impact speed or speed limits:

- :ref:`WPNAV_SPEED<WPNAV_SPEED>`
- :ref:`WPNAV_SPEED_UP<WPNAV_SPEED_UP>`
- :ref:`WPNAV_SPEED_DN<WPNAV_SPEED_DN>`

- :ref:`PILOT_SPEED<PILOT_SPEED>`
- :ref:`PILOT_SPEED_UP<PILOT_SPEED_UP>`
- :ref:`PILOT_SPEED_DN<PILOT_SPEED_DN>`

Depth Control
=============

Vertical position holding modes (ALT_HOLD,etc.) have these primary tuning parameters, which can be lowered if depth oscillation occurs:

- ``PSC_POSZ_P``
- ``PSC_VELZ_P``
