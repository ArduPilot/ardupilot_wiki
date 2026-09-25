.. _plane-input-shaping:

============================
Fixed-Wing Input Shaping
============================

ArduPlane 4.8 adds input shaping for roll and pitch, this smooths the response to pilot stick inputs and autopilot attitude demands. Rather than instantly demanding a new attitude or rate, the controller constrains the acceleration (and jerk) of the attitude target, producing smoother transitions that are easier on airframes and more predictable to fly.

The following video gives an overview of the new functionality:

.. youtube:: qmSYjWMwUWM

This `web tool <https://firmware.ardupilot.org/Tools/WebTools/KinematicTool/plane/>`__ gives a feel for how the parameters affect the input shaping.

How It Works
============

In angle-stabilised modes (e.g. :ref:`FBWA <fbwa-mode>`, :ref:`FBWB <fbwb-mode>`, :ref:`CRUISE <cruise-mode>`, :ref:`AUTO <auto-mode>`), the roll and pitch angle targets pass through an acceleration-limited shaping function that also limits jerk (rate of change of acceleration).

In :ref:`ACRO mode <acro-mode>`, the demanded rates are shaped in the same way using the same acceleration limits.

The shaped targets for angle, angular velocity, and angular acceleration are logged in the :ref:`ATIS<ATIS>` message.

Conditions for Activation
==========================

Input shaping is active on an axis when all of the following are true:

- The axis acceleration limit is greater than zero (:ref:`RLL2SRV_ACCEL<RLL2SRV_ACCEL>` for roll, :ref:`PTCH2SRV_ACCEL<PTCH2SRV_ACCEL>` for pitch).
- The vehicle is in a mode that applies rate limiting (all stabilised modes; pitch shaping is additionally suppressed when flying inverted).
- :ref:`AUTOTUNE mode <automatic-tuning-with-autotune>` is not running.

Parameters
==========

There are two new parameters for each axis, the accel limit and an angle gain, the pre-existing rate limit parameters are also used.

Roll
----

- :ref:`RLL2SRV_ACCEL<RLL2SRV_ACCEL>` — Maximum roll acceleration in deg/s². Default 500. Set to 0 to disable roll input shaping.
- :ref:`RLL2SRV_RMAX<RLL2SRV_RMAX>` — Maximum roll rate in deg/s. Default 0. Used as the velocity limit inside the shaping function. Setting to 0 removes the rate constraint.
- :ref:`RLL_ANGLE_P<RLL_ANGLE_P>` — Roll angle P gain used within the shaping loop. If zero (default), a gain of 1 / :ref:`RLL2SRV_TCONST<RLL2SRV_TCONST>` is used instead.

Pitch
-----

- :ref:`PTCH2SRV_ACCEL<PTCH2SRV_ACCEL>` — Maximum pitch acceleration in deg/s². Default 500. Set to 0 to disable pitch input shaping.
- :ref:`PTCH2SRV_RMAX_UP<PTCH2SRV_RMAX_UP>` — Maximum nose-up pitch rate in deg/s. Default 0 for disabled.
- :ref:`PTCH2SRV_RMAX_DN<PTCH2SRV_RMAX_DN>` — Maximum nose-down pitch rate in deg/s. Default 0 for disabled.
- :ref:`PTCH_ANGLE_P<PTCH_ANGLE_P>` — Pitch angle P gain used within the shaping loop. If zero (default), a gain of 1 / :ref:`PTCH2SRV_TCONST<PTCH2SRV_TCONST>` is used instead.

Tuning
======

The recommended workflow is:

1. **Run AutoTune first.** Input shaping is automatically disabled during :ref:`AutoTune <automatic-tuning-with-autotune>` so it does not interfere with the tuning process. AutoTune will set the time constants (:ref:`RLL2SRV_TCONST<RLL2SRV_TCONST>` and :ref:`PTCH2SRV_TCONST<PTCH2SRV_TCONST>`) based on the selected aggressiveness.

2. **Lock in the autotuned angle P gain.** After AutoTune completes, set the angle P gains to match the autotuned time constants:

   - :ref:`RLL_ANGLE_P<RLL_ANGLE_P>` = 1 / :ref:`RLL2SRV_TCONST<RLL2SRV_TCONST>`
   - :ref:`PTCH_ANGLE_P<PTCH_ANGLE_P>` = 1 / :ref:`PTCH2SRV_TCONST<PTCH2SRV_TCONST>`

   This preserves the autotuned angle tracking performance inside the shaping loop regardless of what the time constant is set to next.

3. **Adjust the time constant for pilot feel.** With the angle P gains fixed, the time constants can now be freely adjusted to tune how quickly the aircraft responds to stick inputs without affecting the underlying stabilisation tune. Increasing the time constant gives a slower, more docile response; decreasing it makes the aircraft feel more immediate (provided the vehicle can keep up with the faster demand).

The acceleration limits (:ref:`RLL2SRV_ACCEL<RLL2SRV_ACCEL>` and :ref:`PTCH2SRV_ACCEL<PTCH2SRV_ACCEL>`) control how quickly the shaped trajectory builds up to the rate limits. The defaults of 500 deg/s² are suitable for most aircraft. Higher values may be needed for small aircraft and lower values for large aircraft. It should never be set such that the vehicle cannot keep up with the demanded acceleration.

Logging
=======

The :ref:`ATIS<ATIS>` log message records the shaped roll and pitch targets on every loop:

- ``rAng`` / ``pAng`` — Shaped angle target (deg).
- ``rVel`` / ``pVel`` — Shaped angular velocity target (deg/s).
- ``rAcc`` / ``pAcc`` — Shaped angular acceleration target (deg/s²).
- ``rErr`` / ``pErr`` — Angle error between shaped target and measured attitude (deg).

When evaluating angle performance the input shaping controls how the shaped angle target follows the raw angle target, plot ATT.DesRoll vs ATIS.rAng for roll and ATT.DesPitch vs ATIS.pAng for pitch. To evaluate the performance of the closed loop angle control compare the shaped targets to the measurements, ATIS.rAng vs ATT.Roll and ATIS.pAng vs ATT.Pitch.
