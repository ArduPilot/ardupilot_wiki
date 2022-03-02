.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


Plane
=====



:ref:`flight-options` page:
---------------------------

- add to table

=====================================   ======================
:ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>`   Function
=====================================   ======================
7                                       Enable default airspeed EKF fusion for takeoff (Advanced users only)
8                                       Remove :ref:`TRIM_PITCH_CD<TRIM_PITCH_CD>` offset on the GCS horizon to show pitch relative to AHRS trim (ie the attitude at which the flight controller was calibrated,unless manually changed)
9                                       Remove :ref:`TRIM_PITCH_CD<TRIM_PITCH_CD>` on the OSD horizon to show pitch relative to AHRS trim (ie the attitude at which the flight controller was calibrated,unless manually changed)
10                                      Adjust mid-throttle to be :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` in non-auto throttle modes except MANUAL,instead of midway between MAX and MIN stick values (note that the RCx_TRIM value for the throttle channel (x) MUST BE set to center stick value)
=====================================   ======================

.. note:: Normally, TRIM_PITCH_CD is subtracted from the AHRS pitch so that the artificial horizon shows pitch as if the flight controller was calibrated with aircraft level position set at TRIM_PITCH_CD instead of flat.  This normally results in the artificial horizon indicating 0 pitch when in cruise at desired cruise speed. TRIM_PITCH_CD is the pitch trim that would be required in stabilized modes to maintain altitude at nominal cruise airspeed and throttle, and for most planes is 1-3 degrees positive, depending on the aircraft design (see :ref:`tuning-cruise`).

:ref:`tuning-cruise` page: 
--------------------------

- add in appropriate place

Using :ref:`TRIM_PITCH_CD<TRIM_PITCH_CD>` to adjust cruise attitude will also add an offset to the artificial horizon on a GCS or an OSD, but this can be disabled using the :ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>` bitmask.

:ref:`guide-tailsitter` page:
-----------------------------

- add under Transitions section:

Depending on the entry speed and time required to transition, the vehicle may gain altitude, sometimes significantly, since the throttle is set to the current :ref:`Q_M_THRST_HOVER<Q_M_THST_HOVER>` hover thrust value throughout the transition to VTOL. This can be overridden with a lower value by setting :ref:`Q_TAILSIT_THR_VT<Q_TAILSIT_THR_VT>`. With experimentation, changing the rates, angle, and this parameter for fixed wing to VTOL transitions, it is possible to obtain almost level altitude transitions. Especially with copter style tailsitters with no control surfaces using Q_TAILSIT_ENABLE = 2, keeping attitude control active even at low or zero throttle values.

:ref:`soaring-4_1` page:
-------------------------

Add content from :ref:`soaring-speed-to-fly`

:ref:`guide-tilt-rotor` page:
-----------------------------

Under Setting Up a Tilt Rotor replace first sentence with:

The first thing you need to do is enable QuadPlane support by setting
:ref:`Q_ENABLE<Q_ENABLE>` to 1 and Tilt Rotor support by setting :ref:`Q_TILT_ENABLE<Q_TILT_ENABLE>` = "1", and then choose the right quadplane frame class and
frame type.

:ref:`apms-failsafe-function` page:
-----------------------------------

add to Battery Failsafes section:

Battery Failsafe Actions
------------------------

The following is a description of the actions that can be taken for battery failsafes:

+-----+------------------+-----------------------------------------------------------------------------+
+Value| Action           |     Description                                                             +
+=====+==================+=============================================================================+
+ 0   | None             | Do nothing except warn                                                      +
+-----+------------------+-----------------------------------------------------------------------------+
+ 1   | RTL              | Switch to :ref:`RTL<rtl-mode>` mode                                         +
+-----+------------------+-----------------------------------------------------------------------------+
+ 2   | Land             | Switch to AUTO mode and execute nearest DO_LAND sequence, if in mission     +
+-----+------------------+-----------------------------------------------------------------------------+
+ 3   | Terminate        |  Disarm                                                                     +
+-----+------------------+-----------------------------------------------------------------------------+
+ 4   | QLAND            | If QuadPlane, switch to :ref:`qland-mode`, otherwise do nothing             +
+-----+------------------+-----------------------------------------------------------------------------+
+ 5   | Parachute        |  Trigger Parachute (Critical action only)                                   +
+-----+------------------+-----------------------------------------------------------------------------+
+ 6   | LOITER_TO_QLAND  | If QuadPlane, switch to LOITER_TO_QLAND mode,                               +
+     |                  | otherwise do nothing                                                        +
+-----+------------------+-----------------------------------------------------------------------------+



[copywiki destination="plane,copter,rover,dev"]