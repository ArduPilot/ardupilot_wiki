.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================

Common
======

:ref:`common-frsky-telemetry` page
----------------------------------

add to end of Configuration Section:

Set :ref:`FRSKY_OPTIONS<FRSKY_OPTIONS>` bit 0 to "1" to enable sending alternating airspeed and groundspeed data to the display script. The :ref:`Yaapu FrSky Telemetry Script for OpenTX <common-frsky-yaapu>` can use this but it may cause other display apps/scripts to alternate the speed value readout.


Plane
=====

:ref:`quadplane-parameters` page
--------------------------------

- add:

bit 18, if set,will allow arming only if in a VTOL mode. Primarily used for tailsitters to prevent accidental immediate tip-over if armed in a fixed wing mode with assistance active.

- change bit 15 to:

bit 15, if set, will allow pilot to control descent during VTOL AUTO-LAND phases, similar to throttle stick action during QHOVER or QLOITER. However, this will not become active until the throttle stick is raised above 70% during the descent at least once.

:ref:`guide-tailsitter` page:
-----------------------------

- at top of  page:

Tailsitters and their parameters are enabled by setting :ref:`Q_TAILSIT_ENABLE<Q_TAILSIT_ENABLE>` to either "1" ,for most tailsitters, or "2" for the special case of Copter Motor Only Tailsitters (those without control surfaces like elevons or ailerons/elevators).

- in Vectored and non-Vectored change:

 " CopterMotor tailsitters without them (ie. only have a lifting wing with no control surfaces) must use QASSIST (discussed below) to provide control while in fixed wing flight modes." to:**"                CopterMotor tailsitters without them (ie. only have a lifting wing with no control surfaces) must use always use their motors to provide control while in fixed wing flight modes. Setting** :ref:`Q_TAILSIT_ENABLE<Q_TAILSIT_ENABLE>` = 2 **automatically does this."**

- under Tailsitter Configuration change:

 "The key to make a QuadPlane a tailsitter is to either set Q_FRAME_CLASS =10 or Q_TAILSIT_MOTMX non-zero. That tells the QuadPlane code to use the tailsitter VTOL backend." to:**"The key to make a QuadPlane a tailsitter is to either set** :ref:`Q_TAILSIT_ENABLE<Q_TAILSIT_ENABLE>` **to "1" or "2" to tell the QuadPlane code to use the tailsitter VTOL backend."**

:ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` **determines the number and layout of VTOL motors and** :ref:`Q_TAILSIT_MOTMX<Q_TAILSIT_MOTMX>` **determines which motors are active when in fixed wing modes, except in the special case of the Copter Motor Only Tailsitter which keeps running the motors like a Copter mode even when flying in a fixed wing mode for control surface-less Copter tailsitters (ie always running the motors to provide attitude control, even at low throttle).**

- Add table at end of the Tailsitter Configuration section:

+-------------------+------+----------------+-------------+--------------+-----------------------+
|Tailsitter Style   |ENABLE| CLASS          |  TYPE       |  MOTORMASK   | Motor Output Functions+
+===================+======+================+=============+==============+=======================+
|3D Single Motor    |  1   | 10(Single/Dual)|  NA         | 0            | Throttle              |
+-------------------+------+----------------+-------------+--------------+-----------------------+
|Twin Motor and Twin|  1   | 10(Single/Dual)|  NA         | 0            | Left Throttle,        |
|Motor Vectored     |      |                |             |              | Right Throttle        |
+-------------------+------+----------------+-------------+--------------+-----------------------+
|Copter Tailsitters |  1   |to match number | to match    |active motors |   Motor 1- Motor x    |
|with fixed wing    |      |of VTOL motors  | motor mixing|in fixed wing |                       |
|control surfaces   |      |                |             |modes         |                       |
+-------------------+------+----------------+-------------+--------------+-----------------------+
|Copter Tailsitters |  2   |to match number | to match    |active motors |   Motor 1- Motor x    |
|with no fixed wing |      |of VTOL motors  | motor mixing|in fixed wing |                       |
|control surfaces   |      |                |             |modes         |                       |
+-------------------+------+----------------+-------------+--------------+-----------------------+

The ENABLE column refers to the :ref:`Q_TAILSIT_ENABLE<Q_TAILSIT_ENABLE>` parameter, while CLASS,TYPE, and MOTORMASK refer to :ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>`, :ref:`Q_FRAME_TYPE<Q_FRAME_TYPE>`, and :ref:`Q_TAILSIT_MOTMX<Q_TAILSIT_MOTMX>`, respectively.

- under Copter Tailsitters change the note to read:

.. note:: It is possible to have a CopterMotor Tailsitter using no fixed wing control surfaces, ie basically a quadcopter with a wing. For that configuration, use :ref:`Q_TAILSIT_ENABLE<Q_TAILSIT_ENABLE>` = 2. :ref:`Q_TAILSIT_MOTMX<Q_TAILSIT_MOTMX>` is ignored in that case.

- Remove Tailsitter Input Mask section entirely

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

:ref:`quadplane-flight-modes` page:

add to mode TOC tree:

    :LOITER_TO_QLAND mode page

[copywiki destination="plane,copter,rover,dev"]